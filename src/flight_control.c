/**
 * @file flight_control.c
 * @author Alberto Vazquez
 * @brief Cascaded attitude/rate controller with normalized X-frame outputs.
 * @version 1.4.4
 * @date 2026-09-29
 */

#include <math.h>
#include <string.h>

#include "flight_control.h"

#define RAD_TO_DEG                     57.2957795f
#define DEG_TO_RAD                     0.0174532925f
#define STANDARD_GRAVITY_MPS2          9.80665f
#define MIN_ACCEL_NORM_MPS2            (0.90f * STANDARD_GRAVITY_MPS2)
#define MAX_ACCEL_NORM_MPS2            (1.10f * STANDARD_GRAVITY_MPS2)
#define MIN_LEVEL_NORM_MPS2            (0.85f * STANDARD_GRAVITY_MPS2)
#define MAX_LEVEL_NORM_MPS2            (1.15f * STANDARD_GRAVITY_MPS2)
#define MAX_LEVEL_TILT_RAD              (15.0f * DEG_TO_RAD)
#define MAX_ESTIMATED_PITCH_RAD        (80.0f * DEG_TO_RAD)
#define TWO_PI                         (2.0f * M_PI)
#define DEFAULT_MAX_TILT_DEG            12.0f

/* Convert the installed IMU axes into right-handed airframe body rates. */
#define AIRFRAME_PITCH_ACCEL_SIGN       (-1.0f)
#define AIRFRAME_PITCH_GYRO_SIGN        (-1.0f)
#define AIRFRAME_YAW_GYRO_SIGN          (1.0f)

/* Match transmitter stick directions to positive airframe rotations. */
#define RECEIVER_ROLL_COMMAND_SIGN      (-1.0f)
#define RECEIVER_PITCH_COMMAND_SIGN     (-1.0f)

static const FlightControl_Config_t g_default_config = {
    DEFAULT_MAX_TILT_DEG,
    180.0f,
    150.0f,
    3.5f,
    0.03f,
    0.05f,
    0.35f,
    5.0f,
    30.0f,
    1.00f,
    { 0.0025f, 0.0008f, 0.000010f, 0.08f, 0.28f, 30.0f },
    { 0.0030f, 0.0015f, 0.000012f, 0.14f, 0.32f, 30.0f },
    { 0.0030f, 0.0015f, 0.000000f, 0.10f, 0.20f, 30.0f }
};

static float32_t clampf(float32_t value,
                        float32_t minimum,
                        float32_t maximum);
static float32_t wrap_angle(float32_t angle_rad);
static float32_t apply_deadband(float32_t value, float32_t deadband);
static bool pid_config_is_valid(const FlightControl_PIDConfig_t *config);
static bool config_is_valid(const FlightControl_Config_t *config);
static bool level_reference_is_valid(
    const LSM6DS_Calibration_t *imu_calibration);
static void reset_pid(FlightControl_PIDState_t *state);
static void reset_controllers(FlightControl_Data_t *data);
static void zero_actuation(FlightControl_Data_t *data);
static void zero_commands_and_actuation(FlightControl_Data_t *data);
static FlightControl_Status_t disable_control(
    FlightControl_Data_t *data,
    FlightControl_Status_t status);
static void update_accel_filter(FlightControl_Data_t *data,
                                const LSM6DS_Sample_t *imu);
static void update_rate_filter(FlightControl_Data_t *data,
                               const LSM6DS_Sample_t *imu);
static bool update_attitude(FlightControl_Data_t *data,
                            const LSM6DS_Sample_t *imu);
static float32_t update_pid(const FlightControl_PIDConfig_t *config,
                            FlightControl_PIDState_t *state,
                            float32_t setpoint_deg_s,
                            float32_t measurement_deg_s,
                            float32_t dt_s,
                            bool integrator_enabled,
                            FlightControl_AxisOutput_t *output);
static void mix_x_frame(FlightControl_Data_t *data);

FlightControl_Status_t FlightControl_Init(
    FlightControl_Data_t *data,
    const FlightControl_Config_t *config,
    const LSM6DS_Calibration_t *imu_calibration)
{
    const FlightControl_Config_t *selected_config;
    const LSM6DS_Vector3f_t *level_accel_mps2;
    float32_t horizontal_norm;
    float32_t level_roll_rad;
    float32_t level_pitch_rad;
    float32_t upright_accel_z_sign;

    if(data == NULL)
    {
        return FLIGHT_CONTROL_STATUS_INVALID_ARGUMENT;
    }

    selected_config = (config == NULL) ? &g_default_config : config;
    if(!config_is_valid(selected_config))
    {
        return FLIGHT_CONTROL_STATUS_INVALID_CONFIG;
    }
    if(!level_reference_is_valid(imu_calibration))
    {
        return FLIGHT_CONTROL_STATUS_INVALID_LEVEL_REFERENCE;
    }

    level_accel_mps2 = &imu_calibration->level_accel_mps2;
    upright_accel_z_sign =
        (level_accel_mps2->z >= 0.0f) ? 1.0f : -1.0f;
    horizontal_norm = sqrtf(
        (level_accel_mps2->y * level_accel_mps2->y) +
        (level_accel_mps2->z * level_accel_mps2->z));
    level_roll_rad = atan2f(
        upright_accel_z_sign * level_accel_mps2->y,
        upright_accel_z_sign * level_accel_mps2->z);
    level_pitch_rad = AIRFRAME_PITCH_ACCEL_SIGN *
        atan2f(-upright_accel_z_sign * level_accel_mps2->x,
               horizontal_norm);
    if((fabsf(level_roll_rad) > MAX_LEVEL_TILT_RAD) ||
       (fabsf(level_pitch_rad) > MAX_LEVEL_TILT_RAD))
    {
        return FLIGHT_CONTROL_STATUS_INVALID_LEVEL_REFERENCE;
    }

    memset(data, 0, sizeof(*data));
    data->config = *selected_config;
    data->upright_accel_z_sign = upright_accel_z_sign;
    data->filtered_accel_mps2 = *level_accel_mps2;
    data->accel_filter_initialized = true;
    data->level_roll_rad = level_roll_rad;
    data->level_pitch_rad = level_pitch_rad;
    data->level_reference.roll_deg =
        data->level_roll_rad * RAD_TO_DEG;
    data->level_reference.pitch_deg =
        data->level_pitch_rad * RAD_TO_DEG;
    data->level_reference.valid = true;
    data->initialized = true;
    data->last_status = FLIGHT_CONTROL_STATUS_OK;
    return data->last_status;
}

FlightControl_Status_t FlightControl_Update(
    FlightControl_Data_t *data,
    LSM6DS_Status_t imu_status,
    const LSM6DS_Sample_t *imu,
    const RP4TDM_Controls_t *receiver)
{
    float32_t roll_command;
    float32_t pitch_command;
    float32_t yaw_command;
    float32_t roll_rate_deg_s;
    float32_t pitch_rate_deg_s;
    float32_t yaw_rate_deg_s;

    if((data == NULL) || (imu == NULL) || (receiver == NULL))
    {
        return FLIGHT_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = FLIGHT_CONTROL_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }

    /* Polling faster than the IMU output rate is normal; retain the output. */
    if(imu_status == LSM6DS_STATUS_NO_NEW_DATA)
    {
        return data->last_status;
    }
    if((imu_status != LSM6DS_STATUS_OK) || !imu->fresh || !imu->valid ||
       !imu->timing_valid || imu->saturated)
    {
        return disable_control(data, FLIGHT_CONTROL_STATUS_INVALID_IMU);
    }

    data->output.timestamp_us = imu->timestamp_us;
    data->output.sequence++;
    update_accel_filter(data, imu);
    update_rate_filter(data, imu);
    if(!update_attitude(data, imu))
    {
        return disable_control(data, FLIGHT_CONTROL_STATUS_INVALID_IMU);
    }

    if(!receiver->valid)
    {
        return disable_control(
            data, FLIGHT_CONTROL_STATUS_RECEIVER_UNAVAILABLE);
    }

    roll_command = RECEIVER_ROLL_COMMAND_SIGN *
        apply_deadband(receiver->roll, data->config.stick_deadband);
    pitch_command = RECEIVER_PITCH_COMMAND_SIGN *
        apply_deadband(receiver->pitch, data->config.stick_deadband);
    yaw_command = apply_deadband(receiver->yaw,
                                 data->config.stick_deadband);

    data->output.setpoint.throttle =
        clampf(receiver->throttle, 0.0f, 1.0f);
    data->output.setpoint.roll_angle_deg =
        roll_command * data->config.max_tilt_deg;
    data->output.setpoint.pitch_angle_deg =
        pitch_command * data->config.max_tilt_deg;
    data->output.setpoint.roll_rate_deg_s = clampf(
        data->config.angle_kp *
        (data->output.setpoint.roll_angle_deg -
         data->output.attitude.roll_deg),
        -data->config.max_roll_pitch_rate_deg_s,
        data->config.max_roll_pitch_rate_deg_s);
    data->output.setpoint.pitch_rate_deg_s = clampf(
        data->config.angle_kp *
        (data->output.setpoint.pitch_angle_deg -
         data->output.attitude.pitch_deg),
        -data->config.max_roll_pitch_rate_deg_s,
        data->config.max_roll_pitch_rate_deg_s);
    data->output.setpoint.yaw_rate_deg_s =
        yaw_command * data->config.max_yaw_rate_deg_s;
    data->output.valid = true;

    if(data->output.setpoint.throttle <=
       data->config.minimum_control_throttle)
    {
        reset_controllers(data);
        zero_actuation(data);
        data->last_status = FLIGHT_CONTROL_STATUS_IDLE;
        return data->last_status;
    }

    data->output.integrator_enabled =
        data->output.setpoint.throttle >=
        data->config.integrator_enable_throttle;

    roll_rate_deg_s = data->filtered_gyro_rad_s.x * RAD_TO_DEG;
    pitch_rate_deg_s = AIRFRAME_PITCH_GYRO_SIGN *
                       data->filtered_gyro_rad_s.y * RAD_TO_DEG;
    yaw_rate_deg_s = AIRFRAME_YAW_GYRO_SIGN *
                     data->filtered_gyro_rad_s.z * RAD_TO_DEG;

    (void)update_pid(&data->config.roll_rate,
                     &data->roll_pid,
                     data->output.setpoint.roll_rate_deg_s,
                     roll_rate_deg_s,
                     imu->dt_s,
                     data->output.integrator_enabled,
                     &data->output.roll);
    (void)update_pid(&data->config.pitch_rate,
                     &data->pitch_pid,
                     data->output.setpoint.pitch_rate_deg_s,
                     pitch_rate_deg_s,
                     imu->dt_s,
                     data->output.integrator_enabled,
                     &data->output.pitch);
    (void)update_pid(&data->config.yaw_rate,
                     &data->yaw_pid,
                     data->output.setpoint.yaw_rate_deg_s,
                     yaw_rate_deg_s,
                     imu->dt_s,
                     data->output.integrator_enabled,
                     &data->output.yaw);

    mix_x_frame(data);
    data->output.active = true;
    data->last_status = FLIGHT_CONTROL_STATUS_OK;
    return data->last_status;
}

static float32_t clampf(float32_t value,
                        float32_t minimum,
                        float32_t maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

static float32_t wrap_angle(float32_t angle_rad)
{
    while(angle_rad > M_PI)
    {
        angle_rad -= TWO_PI;
    }
    while(angle_rad < -M_PI)
    {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

static float32_t apply_deadband(float32_t value, float32_t deadband)
{
    float32_t magnitude;
    float32_t scaled;

    value = clampf(value, -1.0f, 1.0f);
    magnitude = fabsf(value);
    if(magnitude <= deadband)
    {
        return 0.0f;
    }

    scaled = (magnitude - deadband) / (1.0f - deadband);
    return (value < 0.0f) ? -scaled : scaled;
}

static bool pid_config_is_valid(const FlightControl_PIDConfig_t *config)
{
    return isfinite(config->kp) && (config->kp >= 0.0f) &&
           isfinite(config->ki) && (config->ki >= 0.0f) &&
           isfinite(config->kd) && (config->kd >= 0.0f) &&
           isfinite(config->integrator_limit) &&
           (config->integrator_limit >= 0.0f) &&
           isfinite(config->output_limit) &&
           (config->output_limit > 0.0f) &&
           isfinite(config->derivative_cutoff_hz) &&
           (config->derivative_cutoff_hz > 0.0f);
}

static bool config_is_valid(const FlightControl_Config_t *config)
{
    return isfinite(config->max_tilt_deg) &&
           (config->max_tilt_deg > 0.0f) &&
           (config->max_tilt_deg < 80.0f) &&
           isfinite(config->max_roll_pitch_rate_deg_s) &&
           (config->max_roll_pitch_rate_deg_s > 0.0f) &&
           isfinite(config->max_yaw_rate_deg_s) &&
           (config->max_yaw_rate_deg_s > 0.0f) &&
           isfinite(config->angle_kp) && (config->angle_kp > 0.0f) &&
           isfinite(config->stick_deadband) &&
           (config->stick_deadband >= 0.0f) &&
           (config->stick_deadband < 0.25f) &&
           isfinite(config->minimum_control_throttle) &&
           (config->minimum_control_throttle >= 0.0f) &&
           (config->minimum_control_throttle < 1.0f) &&
           isfinite(config->integrator_enable_throttle) &&
           (config->integrator_enable_throttle >
            config->minimum_control_throttle) &&
           (config->integrator_enable_throttle < 1.0f) &&
           isfinite(config->accel_filter_cutoff_hz) &&
           (config->accel_filter_cutoff_hz > 0.0f) &&
           isfinite(config->rate_filter_cutoff_hz) &&
           (config->rate_filter_cutoff_hz > 0.0f) &&
           isfinite(config->attitude_correction_time_s) &&
           (config->attitude_correction_time_s > 0.0f) &&
           pid_config_is_valid(&config->roll_rate) &&
           pid_config_is_valid(&config->pitch_rate) &&
           pid_config_is_valid(&config->yaw_rate);
}

static bool level_reference_is_valid(
    const LSM6DS_Calibration_t *imu_calibration)
{
    const LSM6DS_Vector3f_t *level_accel_mps2;
    float32_t norm;

    if((imu_calibration == NULL) || !imu_calibration->level_valid)
    {
        return false;
    }

    level_accel_mps2 = &imu_calibration->level_accel_mps2;
    norm = sqrtf(
        (level_accel_mps2->x * level_accel_mps2->x) +
        (level_accel_mps2->y * level_accel_mps2->y) +
        (level_accel_mps2->z * level_accel_mps2->z));
    return isfinite(norm) &&
           (norm >= MIN_LEVEL_NORM_MPS2) &&
           (norm <= MAX_LEVEL_NORM_MPS2);
}

static void reset_pid(FlightControl_PIDState_t *state)
{
    memset(state, 0, sizeof(*state));
}

static void reset_controllers(FlightControl_Data_t *data)
{
    reset_pid(&data->roll_pid);
    reset_pid(&data->pitch_pid);
    reset_pid(&data->yaw_pid);
}

static void zero_actuation(FlightControl_Data_t *data)
{
    memset(&data->output.roll, 0, sizeof(data->output.roll));
    memset(&data->output.pitch, 0, sizeof(data->output.pitch));
    memset(&data->output.yaw, 0, sizeof(data->output.yaw));
    memset(&data->output.motors, 0, sizeof(data->output.motors));
    data->output.mixer_scale = 0.0f;
    data->output.integrator_enabled = false;
    data->output.active = false;
}

static void zero_commands_and_actuation(FlightControl_Data_t *data)
{
    memset(&data->output.setpoint, 0, sizeof(data->output.setpoint));
    zero_actuation(data);
    data->output.valid = false;
}

static FlightControl_Status_t disable_control(
    FlightControl_Data_t *data,
    FlightControl_Status_t status)
{
    reset_controllers(data);
    zero_commands_and_actuation(data);
    data->last_status = status;
    return status;
}

/** Filters motor vibration before acceleration is qualified as gravity. */
static void update_accel_filter(FlightControl_Data_t *data,
                                const LSM6DS_Sample_t *imu)
{
    float32_t filter_time_s;
    float32_t alpha;

    if(!data->accel_filter_initialized)
    {
        data->filtered_accel_mps2 = imu->accel_mps2;
        data->accel_filter_initialized = true;
        return;
    }

    filter_time_s = 1.0f /
        (TWO_PI * data->config.accel_filter_cutoff_hz);
    alpha = imu->dt_s / (filter_time_s + imu->dt_s);
    data->filtered_accel_mps2.x += alpha *
        (imu->accel_mps2.x - data->filtered_accel_mps2.x);
    data->filtered_accel_mps2.y += alpha *
        (imu->accel_mps2.y - data->filtered_accel_mps2.y);
    data->filtered_accel_mps2.z += alpha *
        (imu->accel_mps2.z - data->filtered_accel_mps2.z);
}

/**
 * Filters the measured body rates before they enter either the estimator or
 * the rate PID. This prevents propeller vibration from directly commanding
 * alternating full-scale motor corrections.
 */
static void update_rate_filter(FlightControl_Data_t *data,
                               const LSM6DS_Sample_t *imu)
{
    float32_t filter_time_s;
    float32_t alpha;

    if(!data->rate_filter_initialized)
    {
        data->filtered_gyro_rad_s = imu->gyro_rad_s;
        data->rate_filter_initialized = true;
        return;
    }

    filter_time_s = 1.0f /
        (TWO_PI * data->config.rate_filter_cutoff_hz);
    alpha = imu->dt_s / (filter_time_s + imu->dt_s);
    data->filtered_gyro_rad_s.x += alpha *
        (imu->gyro_rad_s.x - data->filtered_gyro_rad_s.x);
    data->filtered_gyro_rad_s.y += alpha *
        (imu->gyro_rad_s.y - data->filtered_gyro_rad_s.y);
    data->filtered_gyro_rad_s.z += alpha *
        (imu->gyro_rad_s.z - data->filtered_gyro_rad_s.z);
}

static bool update_attitude(FlightControl_Data_t *data,
                            const LSM6DS_Sample_t *imu)
{
    float32_t ax;
    float32_t ay;
    float32_t az;
    float32_t accel_norm;
    float32_t accel_roll;
    float32_t accel_pitch;
    float32_t predicted_roll;
    float32_t predicted_pitch;
    float32_t roll_rate;
    float32_t body_pitch_rate;
    float32_t body_yaw_rate;
    float32_t pitch_rate;
    float32_t correction;
    bool accel_trusted;

    ax = data->filtered_accel_mps2.x;
    ay = data->filtered_accel_mps2.y;
    az = data->filtered_accel_mps2.z;
    accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    data->accel_trusted = isfinite(accel_norm) &&
                          (accel_norm >= MIN_ACCEL_NORM_MPS2) &&
                          (accel_norm <= MAX_ACCEL_NORM_MPS2);
    accel_trusted = data->accel_trusted;

    if(accel_trusted)
    {
        accel_roll = wrap_angle(
            atan2f(data->upright_accel_z_sign * ay,
                   data->upright_accel_z_sign * az) -
            data->level_roll_rad);
        accel_pitch = (AIRFRAME_PITCH_ACCEL_SIGN *
            atan2f(-data->upright_accel_z_sign * ax,
                   sqrtf((ay * ay) + (az * az)))) -
            data->level_pitch_rad;
    }
    else
    {
        accel_roll = 0.0f;
        accel_pitch = 0.0f;
    }

    if(!data->attitude_initialized)
    {
        if(!accel_trusted)
        {
            data->output.attitude.valid = false;
            return false;
        }
        data->estimated_roll_rad = accel_roll;
        data->estimated_pitch_rad = accel_pitch;
        data->attitude_initialized = true;
    }
    else
    {
        body_pitch_rate = AIRFRAME_PITCH_GYRO_SIGN *
                          data->filtered_gyro_rad_s.y;
        body_yaw_rate = AIRFRAME_YAW_GYRO_SIGN *
                        data->filtered_gyro_rad_s.z;
        roll_rate = data->filtered_gyro_rad_s.x +
            sinf(data->estimated_roll_rad) *
            tanf(data->estimated_pitch_rad) * body_pitch_rate +
            cosf(data->estimated_roll_rad) *
            tanf(data->estimated_pitch_rad) *
            body_yaw_rate;
        pitch_rate =
            cosf(data->estimated_roll_rad) * body_pitch_rate -
            sinf(data->estimated_roll_rad) *
            body_yaw_rate;
        predicted_roll = wrap_angle(
            data->estimated_roll_rad + (roll_rate * imu->dt_s));
        predicted_pitch = clampf(
            data->estimated_pitch_rad + (pitch_rate * imu->dt_s),
            -MAX_ESTIMATED_PITCH_RAD,
            MAX_ESTIMATED_PITCH_RAD);

        if(accel_trusted)
        {
            correction = imu->dt_s /
                (data->config.attitude_correction_time_s + imu->dt_s);
            predicted_roll = wrap_angle(
                predicted_roll +
                (correction * wrap_angle(accel_roll - predicted_roll)));
            predicted_pitch += correction *
                (accel_pitch - predicted_pitch);
        }

        data->estimated_roll_rad = predicted_roll;
        data->estimated_pitch_rad = clampf(
            predicted_pitch,
            -MAX_ESTIMATED_PITCH_RAD,
            MAX_ESTIMATED_PITCH_RAD);
    }

    data->output.attitude.roll_deg =
        data->estimated_roll_rad * RAD_TO_DEG;
    data->output.attitude.pitch_deg =
        data->estimated_pitch_rad * RAD_TO_DEG;
    data->output.attitude.valid = true;
    return true;
}

static float32_t update_pid(const FlightControl_PIDConfig_t *config,
                            FlightControl_PIDState_t *state,
                            float32_t setpoint_deg_s,
                            float32_t measurement_deg_s,
                            float32_t dt_s,
                            bool integrator_enabled,
                            FlightControl_AxisOutput_t *output)
{
    float32_t raw_derivative;
    float32_t filter_time_s;
    float32_t filter_alpha;
    float32_t candidate_integrator;
    float32_t unsaturated;

    output->measured_deg_s = measurement_deg_s;
    output->error_deg_s = setpoint_deg_s - measurement_deg_s;
    output->proportional = config->kp * output->error_deg_s;

    if(!state->initialized)
    {
        state->previous_measurement = measurement_deg_s;
        state->filtered_derivative = 0.0f;
        state->initialized = true;
    }

    raw_derivative =
        (measurement_deg_s - state->previous_measurement) / dt_s;
    filter_time_s = 1.0f /
        (TWO_PI * config->derivative_cutoff_hz);
    filter_alpha = dt_s / (filter_time_s + dt_s);
    state->filtered_derivative += filter_alpha *
        (raw_derivative - state->filtered_derivative);
    state->previous_measurement = measurement_deg_s;

    if(integrator_enabled)
    {
        candidate_integrator = clampf(
            state->integrator +
            (config->ki * output->error_deg_s * dt_s),
            -config->integrator_limit,
            config->integrator_limit);
    }
    else
    {
        state->integrator = 0.0f;
        candidate_integrator = 0.0f;
    }
    output->derivative = -config->kd * state->filtered_derivative;
    unsaturated = output->proportional + candidate_integrator +
                  output->derivative;

    if(((unsaturated > config->output_limit) &&
        (output->error_deg_s > 0.0f)) ||
       ((unsaturated < -config->output_limit) &&
        (output->error_deg_s < 0.0f)))
    {
        candidate_integrator = state->integrator;
        unsaturated = output->proportional + candidate_integrator +
                      output->derivative;
    }

    state->integrator = candidate_integrator;
    output->integral = state->integrator;
    output->output = clampf(unsaturated,
                            -config->output_limit,
                            config->output_limit);
    return output->output;
}

static void mix_x_frame(FlightControl_Data_t *data)
{
    float32_t correction[4];
    float32_t maximum_correction;
    float32_t headroom;
    float32_t scale;
    float32_t throttle;
    uint32_t index;

    correction[0] = data->output.roll.output +
                    data->output.pitch.output +
                    data->output.yaw.output;
    correction[1] = -data->output.roll.output +
                     data->output.pitch.output -
                     data->output.yaw.output;
    correction[2] = -data->output.roll.output -
                     data->output.pitch.output +
                     data->output.yaw.output;
    correction[3] = data->output.roll.output -
                    data->output.pitch.output -
                    data->output.yaw.output;

    maximum_correction = 0.0f;
    for(index = 0U; index < 4U; index++)
    {
        if(fabsf(correction[index]) > maximum_correction)
        {
            maximum_correction = fabsf(correction[index]);
        }
    }

    throttle = data->output.setpoint.throttle;
    headroom = (throttle < (1.0f - throttle)) ?
        throttle : (1.0f - throttle);
    scale = ((maximum_correction > headroom) &&
             (maximum_correction > 0.0f)) ?
        headroom / maximum_correction : 1.0f;
    data->output.mixer_scale = scale;

    data->output.motors.front_left =
        clampf(throttle + (correction[0] * scale), 0.0f, 1.0f);
    data->output.motors.front_right =
        clampf(throttle + (correction[1] * scale), 0.0f, 1.0f);
    data->output.motors.rear_right =
        clampf(throttle + (correction[2] * scale), 0.0f, 1.0f);
    data->output.motors.rear_left =
        clampf(throttle + (correction[3] * scale), 0.0f, 1.0f);
}
