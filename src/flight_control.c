/**
 * @file flight_control.c
 * @author Alberto Vazquez
 * @brief Cascaded attitude/rate controller with normalized X-frame outputs.
 *
 * @details Beginner's reading guide:
 * - FlightControl_Update() receives one calibrated IMU sample, the latest
 *   receiver controls, the latest magnetometer sample, and elapsed time.
 * - Roll and pitch sticks become desired angles in degrees. Flight Trim and an
 *   optional horizontal-drift correction are added to those desired angles.
 * - The outer attitude loop converts angle error into desired angular rate in
 *   degrees per second. Three inner PID controllers compare desired rate with
 *   measured gyroscope rate and create normalized roll, pitch, and yaw effort.
 * - mix_x_frame() combines throttle and the three efforts into four values in
 *   the dimensionless range 0.0 to 1.0. MotorOutput later converts these values
 *   to microseconds; this module never writes PWM registers directly.
 * - Every angle with suffix _deg is in degrees, every angular rate with suffix
 *   _deg_s is degrees/second, _rad_s is radians/second, and _s is seconds.
 * - All failure exits clear controller state before publishing zero actuation,
 *   so stale integral or derivative terms cannot reappear unexpectedly.
 * @version 1.6.0
 * @date 2026-09-29
 */

#include <math.h>
#include <string.h>

#include "flight_control.h"

/* Angular conversion constants at degree-based command/telemetry boundaries. */
#define RAD_TO_DEG                     57.2957795f
#define DEG_TO_RAD                     0.0174532925f
/* Standard gravity used to qualify accelerometer data as a tilt reference. */
#define STANDARD_GRAVITY_MPS2          9.80665f
/* In-flight acceleration is trusted only when its norm stays within +/-10% g. */
#define MIN_ACCEL_NORM_MPS2            (0.90f * STANDARD_GRAVITY_MPS2)
#define MAX_ACCEL_NORM_MPS2            (1.10f * STANDARD_GRAVITY_MPS2)
/* Startup level calibration accepts a wider +/-15% gravity magnitude range. */
#define MIN_LEVEL_NORM_MPS2            (0.85f * STANDARD_GRAVITY_MPS2)
#define MAX_LEVEL_NORM_MPS2            (1.15f * STANDARD_GRAVITY_MPS2)
/* Reject startup calibration if the stationary airframe is tilted over 15 deg. */
#define MAX_LEVEL_TILT_RAD              (15.0f * DEG_TO_RAD)
/* Euler kinematics become singular near +/-90 deg pitch; clamp at +/-80 deg. */
#define MAX_ESTIMATED_PITCH_RAD        (80.0f * DEG_TO_RAD)
#define TWO_PI                         (2.0f * M_PI)
/* Maximum commanded roll or pitch angle from full stick. */
#define DEFAULT_MAX_TILT_DEG            12.0f
/* Independent flight trims shift only the roll/pitch attitude setpoints. */
#define DEFAULT_FLIGHT_ROLL_TRIM_DEG     0.0f
#define DEFAULT_FLIGHT_PITCH_TRIM_DEG    0.0f
/* Prevent a configuration error from requesting a large permanent tilt. */
#define MAX_ABSOLUTE_FLIGHT_TRIM_DEG     2.0f

/* Convert the installed IMU axes into right-handed airframe body rates. */
#define AIRFRAME_PITCH_ACCEL_SIGN       (-1.0f)
#define AIRFRAME_PITCH_GYRO_SIGN        (-1.0f)
#define AIRFRAME_YAW_GYRO_SIGN          (1.0f)

/* Match transmitter stick directions to positive airframe rotations. */
#define RECEIVER_ROLL_COMMAND_SIGN      (-1.0f)
#define RECEIVER_PITCH_COMMAND_SIGN     (-1.0f)

/* Magnetic heading hold is enabled after validating the inner yaw-rate loop. */
#define MAGNETIC_HEADING_HOLD_ENABLED   1U
/* Magnetic heading increases clockwise; the validated yaw-rate loop uses
 * the opposite sign at this cascade boundary. */
#define HEADING_ERROR_TO_YAW_RATE_SIGN  (-1.0f)

/* Flight-tested cascaded-controller configuration and PID gains. */
static const FlightControl_Config_t g_default_config = {
    DEFAULT_MAX_TILT_DEG, /* max_tilt_deg */
    DEFAULT_FLIGHT_ROLL_TRIM_DEG,  /* flight_roll_trim_deg */
    DEFAULT_FLIGHT_PITCH_TRIM_DEG, /* flight_pitch_trim_deg */
    180.0f, /* max_roll_pitch_rate_deg_s */
    150.0f, /* max_yaw_rate_deg_s */
    3.0f,   /* angle_kp: deg/s of rate request per degree of angle error */
    2.0f,   /* yaw_heading_kp: deg/s per degree of magnetic heading error */
    45.0f,  /* max_yaw_heading_correction_deg_s */
    0.03f,  /* stick_deadband, normalized */
    0.05f,  /* minimum_control_throttle, normalized */
    0.35f,  /* roll/pitch integrator enable throttle, normalized */
    0.15f,  /* yaw integrator enable throttle, normalized */
    3.0f,   /* accel_filter_cutoff_hz */
    15.0f,  /* rate_filter_cutoff_hz */
    1.00f,  /* attitude_correction_time_s */
    /* kp, ki, kd, integrator limit, output limit, derivative cutoff Hz */
    { 0.00125f, 0.0008f, 0.000010f, 0.08f, 0.28f, 12.0f }, /* roll  */
    { 0.00170f, 0.0015f, 0.000012f, 0.14f, 0.32f, 12.0f }, /* pitch */
    { 0.00500f, 0.0100f, 0.000006f, 0.20f, 0.30f, 15.0f }  /* yaw   */
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
static void update_yaw_setpoint(FlightControl_Data_t *data,
                                float32_t yaw_command);
static float32_t update_pid(const FlightControl_PIDConfig_t *config,
                            FlightControl_PIDState_t *state,
                            float32_t setpoint_deg_s,
                            float32_t measurement_deg_s,
                            float32_t dt_s,
                            bool integrator_enabled,
                            FlightControl_AxisOutput_t *output);
static void mix_x_frame(FlightControl_Data_t *data);

/**
 * @brief Initialize one flight-controller instance and clear all runtime state.
 * @param data Destination instance that will retain configuration and outputs.
 * @param config Optional configuration; NULL selects the documented defaults.
 * @param imu_calibration Startup IMU calibration containing the level reference
 *        in m/s^2 and the gyroscope bias in rad/s.
 * @return FLIGHT_CONTROL_STATUS_OK on success, otherwise an argument or
 *         configuration status. No control output is enabled on failure.
 * @note This function stores data by value; the caller may release config after
 *       the call, but data and imu_calibration must be valid during the call.
 */
FlightControl_Status_t FlightControl_Init(
    FlightControl_Data_t *data,
    const FlightControl_Config_t *config,
    const LSM6DS_Calibration_t *imu_calibration)
{
    const FlightControl_Config_t *selected_config; /* Effective controller settings. */
    const LSM6DS_Vector3f_t *level_accel_mps2; /* Stationary gravity vector, m/s^2. */
    float32_t horizontal_norm; /* Y/Z gravity-plane magnitude, m/s^2. */
    float32_t level_roll_rad;  /* Mechanical startup roll offset, radians. */
    float32_t level_pitch_rad; /* Mechanical startup pitch offset, radians. */
    float32_t upright_accel_z_sign; /* +1 or -1 according to gravity Z polarity. */
    MagHeading_Status_t heading_status; /* Heading-estimator initialization result. */
    HorizontalDrift_Status_t drift_status; /* Drift-module initialization result. */

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

    /* Derive the installed airframe's zero-angle reference from calibration. */
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

    /* Initialize submodules before exposing a valid controller instance. */
    memset(data, 0, sizeof(*data));
    data->config = *selected_config;
    heading_status = MagHeading_Init(&data->heading_estimator, NULL);
    if(heading_status != MAG_HEADING_STATUS_OK)
    {
        return FLIGHT_CONTROL_STATUS_INVALID_CONFIG;
    }
    drift_status = HorizontalDrift_Init(
        &data->horizontal_drift, NULL, level_accel_mps2);
    if(drift_status != HORIZONTAL_DRIFT_STATUS_OK)
    {
        return FLIGHT_CONTROL_STATUS_INVALID_CONFIG;
    }
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

/**
 * @brief Execute one complete attitude/rate-control iteration for a fresh IMU sample.
 * @param data Initialized controller instance and destination for all new state.
 * @param imu_status Result returned by the IMU driver for this scheduler visit.
 * @param imu Latest calibrated acceleration [m/s^2], angular rate [rad/s], and
 *        sample timing. It is read only and is never modified here.
 * @param magnetometer Latest LIS2MDL instance used for optional heading hold.
 * @param receiver Latest normalized controls: throttle [0,1], axes [-1,1].
 * @return OK when actuation was calculated, IDLE when throttle is low, or a
 *         precise gating/error status when control was intentionally disabled.
 * @note A NO_NEW_DATA IMU status returns without recalculating outputs because
 *       the control period is defined by fresh IMU samples, approximately 416 Hz.
 */
FlightControl_Status_t FlightControl_Update(
    FlightControl_Data_t *data,
    LSM6DS_Status_t imu_status,
    const LSM6DS_Sample_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const RP4TDM_Controls_t *receiver)
{
    float32_t roll_command;      /* Deadbanded signed roll stick, [-1,+1]. */
    float32_t pitch_command;     /* Deadbanded signed pitch stick, [-1,+1]. */
    float32_t yaw_command;       /* Deadbanded signed yaw stick, [-1,+1]. */
    float32_t roll_rate_deg_s;   /* Filtered measured body roll rate, deg/s. */
    float32_t pitch_rate_deg_s;  /* Filtered measured body pitch rate, deg/s. */
    float32_t yaw_rate_deg_s;    /* Filtered measured body yaw rate, deg/s. */
    bool command_centered;       /* True only when all attitude sticks are neutral. */

    if((data == NULL) || (imu == NULL) || (magnetometer == NULL) ||
       (receiver == NULL))
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

    /* A valid fresh IMU sample is the clock and trigger for one control update. */
    data->output.timestamp_us = imu->timestamp_us;
    data->output.sequence++;
    update_accel_filter(data, imu);
    update_rate_filter(data, imu);
    if(!update_attitude(data, imu))
    {
        return disable_control(data, FLIGHT_CONTROL_STATUS_INVALID_IMU);
    }

    /* The heading estimator may reject a sample without disabling rate control. */
    (void)MagHeading_Update(
        &data->heading_estimator,
        magnetometer,
        data->output.attitude.roll_deg,
        data->output.attitude.pitch_deg,
        imu->timestamp_us);
    data->output.heading.heading_deg =
        data->heading_estimator.output.heading_deg;
    data->output.heading.valid =
        data->heading_estimator.output.valid;

    if(!receiver->valid)
    {
        return disable_control(
            data, FLIGHT_CONTROL_STATUS_RECEIVER_UNAVAILABLE);
    }

    /* Apply validated transmitter sign conventions and remove stick-center noise. */
    roll_command = RECEIVER_ROLL_COMMAND_SIGN *
        apply_deadband(receiver->roll, data->config.stick_deadband);
    pitch_command = RECEIVER_PITCH_COMMAND_SIGN *
        apply_deadband(receiver->pitch, data->config.stick_deadband);
    yaw_command = apply_deadband(receiver->yaw,
                                 data->config.stick_deadband);

    data->output.setpoint.throttle =
        clampf(receiver->throttle, 0.0f, 1.0f);
    update_yaw_setpoint(data, yaw_command);

    /* Drift braking is allowed only when the pilot is not requesting motion. */
    command_centered = (roll_command == 0.0f) &&
                       (pitch_command == 0.0f) &&
                       (yaw_command == 0.0f);
    (void)HorizontalDrift_Update(
        &data->horizontal_drift,
        &imu->accel_mps2,
        data->output.attitude.roll_deg,
        data->output.attitude.pitch_deg,
        imu->dt_s,
        data->output.setpoint.throttle,
        command_centered,
        data->output.heading.hold_active,
        data->accel_trusted);
    /* Combine pilot command, fixed Flight Trim, and optional drift correction
     * without changing the measured attitude or the IMU calibration. */
    data->output.setpoint.roll_angle_deg = clampf(
        (roll_command * data->config.max_tilt_deg) +
        data->config.flight_roll_trim_deg +
        data->horizontal_drift.output.roll_correction_deg,
        -data->config.max_tilt_deg,
        data->config.max_tilt_deg);
    data->output.setpoint.pitch_angle_deg = clampf(
        (pitch_command * data->config.max_tilt_deg) +
        data->config.flight_pitch_trim_deg +
        data->horizontal_drift.output.pitch_correction_deg,
        -data->config.max_tilt_deg,
        data->config.max_tilt_deg);
    /* Outer P angle loops request bounded body rates for the inner PID loops. */
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
    data->output.valid = true;

    /* Below idle threshold, clear all PID memory and command zero motor torque. */
    if(data->output.setpoint.throttle <=
       data->config.minimum_control_throttle)
    {
        reset_controllers(data);
        zero_actuation(data);
        data->last_status = FLIGHT_CONTROL_STATUS_IDLE;
        return data->last_status;
    }

    /* Roll/pitch I terms wait for likely flight; yaw starts earlier because the
     * counter-torque bias appears while the propellers are still spooling up. */
    data->output.integrator_enabled =
        data->output.setpoint.throttle >=
        data->config.integrator_enable_throttle;
    data->output.yaw_integrator_enabled =
        data->output.setpoint.throttle >=
        data->config.yaw_integrator_enable_throttle;

    roll_rate_deg_s = data->filtered_gyro_rad_s.x * RAD_TO_DEG;
    pitch_rate_deg_s = AIRFRAME_PITCH_GYRO_SIGN *
                       data->filtered_gyro_rad_s.y * RAD_TO_DEG;
    yaw_rate_deg_s = AIRFRAME_YAW_GYRO_SIGN *
                     data->filtered_gyro_rad_s.z * RAD_TO_DEG;

    /* Three independent inner rate PIDs produce normalized torque corrections. */
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
                     data->output.yaw_integrator_enabled,
                     &data->output.yaw);

    /* Convert collective throttle plus axis torques to four X-frame commands. */
    mix_x_frame(data);
    data->output.active = true;
    data->last_status = FLIGHT_CONTROL_STATUS_OK;
    return data->last_status;
}

/**
 * @brief Limit one scalar so it cannot leave an inclusive interval.
 * @param value Input value in the same unit as both limits.
 * @param minimum Smallest permitted result.
 * @param maximum Largest permitted result.
 * @return minimum, maximum, or value when already inside the interval.
 */
static float32_t clampf(float32_t value,
                        float32_t minimum,
                        float32_t maximum)
{
    /* Return value constrained to the closed interval [minimum, maximum]. */
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

/**
 * @brief Wrap an angle to the half-open interval [-pi, +pi) radians.
 * @param angle_rad Arbitrary signed angle [rad].
 * @return Equivalent shortest signed angle [rad].
 */
static float32_t wrap_angle(float32_t angle_rad)
{
    /* Normalize a finite angle to [-pi,+pi] for shortest-path errors. */
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

/**
 * @brief Remove small centered-stick noise while preserving full-scale output.
 * @param value Normalized stick request in [-1, +1].
 * @param deadband Symmetric zero region, normalized and nonnegative.
 * @return Zero inside the deadband; rescaled normalized value outside it.
 */
static float32_t apply_deadband(float32_t value, float32_t deadband)
{
    float32_t magnitude; /* Absolute normalized stick displacement. */
    float32_t scaled;    /* Remaining command rescaled to recover full range. */

    value = clampf(value, -1.0f, 1.0f);
    magnitude = fabsf(value);
    if(magnitude <= deadband)
    {
        return 0.0f;
    }

    scaled = (magnitude - deadband) / (1.0f - deadband);
    return (value < 0.0f) ? -scaled : scaled;
}

/**
 * @brief Check that one rate-PID configuration is finite and physically usable.
 * @param config Candidate gains, filter frequency, integral limit, and output limit.
 * @return true only when every field meets the controller's numeric constraints.
 */
static bool pid_config_is_valid(const FlightControl_PIDConfig_t *config)
{
    /* Gains may be zero, while output and filter limits must be strictly positive. */
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

/**
 * @brief Validate the complete controller configuration before it becomes active.
 * @param config Candidate attitude, yaw, PID, filter, trim, and gating settings.
 * @return true when every scalar is finite and every range is internally consistent.
 */
static bool config_is_valid(const FlightControl_Config_t *config)
{
    /* Validate every numerical input before it can influence live motor commands. */
    return isfinite(config->max_tilt_deg) &&
           (config->max_tilt_deg > 0.0f) &&
           (config->max_tilt_deg < 80.0f) &&
           isfinite(config->flight_roll_trim_deg) &&
           (fabsf(config->flight_roll_trim_deg) <=
            MAX_ABSOLUTE_FLIGHT_TRIM_DEG) &&
           isfinite(config->flight_pitch_trim_deg) &&
           (fabsf(config->flight_pitch_trim_deg) <=
            MAX_ABSOLUTE_FLIGHT_TRIM_DEG) &&
           isfinite(config->max_roll_pitch_rate_deg_s) &&
           (config->max_roll_pitch_rate_deg_s > 0.0f) &&
           isfinite(config->max_yaw_rate_deg_s) &&
           (config->max_yaw_rate_deg_s > 0.0f) &&
           isfinite(config->angle_kp) && (config->angle_kp > 0.0f) &&
           isfinite(config->yaw_heading_kp) &&
           (config->yaw_heading_kp > 0.0f) &&
           isfinite(config->max_yaw_heading_correction_deg_s) &&
           (config->max_yaw_heading_correction_deg_s > 0.0f) &&
           (config->max_yaw_heading_correction_deg_s <=
            config->max_yaw_rate_deg_s) &&
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
           isfinite(config->yaw_integrator_enable_throttle) &&
           (config->yaw_integrator_enable_throttle >
            config->minimum_control_throttle) &&
           (config->yaw_integrator_enable_throttle <=
            config->integrator_enable_throttle) &&
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

/**
 * @brief Validate the stationary gravity vector used as the aircraft level reference.
 * @param imu_calibration Calibration produced during stationary IMU startup.
 * @return true when calibration is marked valid and gravity magnitude is plausible.
 */
static bool level_reference_is_valid(
    const LSM6DS_Calibration_t *imu_calibration)
{
    const LSM6DS_Vector3f_t *level_accel_mps2; /* Calibrated gravity vector. */
    float32_t norm; /* Gravity-vector magnitude in m/s^2. */

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

/**
 * @brief Clear the memory of one PID controller.
 * @param state PID integral, prior error, derivative filter, and initialization flag.
 * @return Nothing; the pointed structure is overwritten with zeros.
 */
static void reset_pid(FlightControl_PIDState_t *state)
{
    /* Clear integral, derivative history, and first-update initialization flag. */
    memset(state, 0, sizeof(*state));
}

/**
 * @brief Clear roll, pitch, yaw, and magnetic-heading controller memory together.
 * @param data Controller instance whose dynamic states will be reset.
 * @return Nothing; configuration and measured attitude remain unchanged.
 */
static void reset_controllers(FlightControl_Data_t *data)
{
    reset_pid(&data->roll_pid);
    reset_pid(&data->pitch_pid);
    reset_pid(&data->yaw_pid);
}

/**
 * @brief Publish zero PID effort and zero normalized motor commands.
 * @param data Controller instance whose output fields will be made non-actuating.
 * @return Nothing; attitude and setpoint diagnostic fields are preserved.
 */
static void zero_actuation(FlightControl_Data_t *data)
{
    /* Clear only actuator-producing fields while retaining valid attitude data. */
    memset(&data->output.roll, 0, sizeof(data->output.roll));
    memset(&data->output.pitch, 0, sizeof(data->output.pitch));
    memset(&data->output.yaw, 0, sizeof(data->output.yaw));
    memset(&data->output.motors, 0, sizeof(data->output.motors));
    data->output.mixer_scale = 0.0f;
    data->output.integrator_enabled = false;
    data->output.yaw_integrator_enabled = false;
    data->output.active = false;
}

/**
 * @brief Clear setpoints as well as all actuation when no valid command exists.
 * @param data Controller instance to place in a neutral diagnostic state.
 * @return Nothing; measured attitude remains available for debugging.
 */
static void zero_commands_and_actuation(FlightControl_Data_t *data)
{
    /* Receiver/IMU loss invalidates commands and every dependent controller state. */
    memset(&data->output.setpoint, 0, sizeof(data->output.setpoint));
    HorizontalDrift_Reset(&data->horizontal_drift);
    data->output.heading.error_deg = 0.0f;
    data->output.heading.hold_active = false;
    data->heading_setpoint_initialized = false;
    zero_actuation(data);
    data->output.valid = false;
}

/**
 * @brief Centralize a safe controller exit caused by invalid input or timing.
 * @param data Controller instance to reset and mark inactive.
 * @param status Reason that control is being disabled.
 * @return The same status after resetting state and zeroing commands/actuation.
 */
static FlightControl_Status_t disable_control(
    FlightControl_Data_t *data,
    FlightControl_Status_t status)
{
    /* Centralized fail-safe path guarantees identical state clearing for all faults. */
    reset_controllers(data);
    zero_commands_and_actuation(data);
    data->last_status = status;
    return status;
}

/** Filters motor vibration before acceleration is qualified as gravity. */
/**
 * @brief Low-pass filter the three calibrated acceleration axes.
 * @param data Instance holding prior filter state and new filtered output [m/s^2].
 * @param imu Fresh source acceleration and sample period [s].
 * @return Nothing; data->filtered_accel_mps2 is updated in place.
 */
static void update_accel_filter(FlightControl_Data_t *data,
                                const LSM6DS_Sample_t *imu)
{
    float32_t filter_time_s; /* LPF time constant 1/(2*pi*fc), seconds. */
    float32_t alpha;         /* Discrete LPF coefficient dt/(tau+dt). */

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
/**
 * @brief Map gyroscope axes and low-pass filter body angular rates.
 * @param data Instance holding prior filter state and output [degrees/second].
 * @param imu Fresh calibrated gyroscope sample [rad/s] and sample period [s].
 * @return Nothing; data->filtered_rate_deg_s is updated in place.
 */
static void update_rate_filter(FlightControl_Data_t *data,
                               const LSM6DS_Sample_t *imu)
{
    float32_t filter_time_s; /* Rate LPF time constant, seconds. */
    float32_t alpha;         /* Discrete rate LPF coefficient in [0,1]. */

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

/**
 * @brief Update roll and pitch with a complementary gyro/accelerometer filter.
 * @param data Instance containing level reference, filter state, and attitude output.
 * @param imu Fresh sample providing acceleration [m/s^2] and dt [s].
 * @return true when finite roll/pitch estimates were produced; false on bad data.
 * @note Gyroscope integration supplies fast motion. Trusted gravity supplies the
 *       slow absolute tilt correction and is rejected during strong acceleration.
 */
static bool update_attitude(FlightControl_Data_t *data,
                            const LSM6DS_Sample_t *imu)
{
    float32_t ax;              /* Filtered X acceleration, m/s^2. */
    float32_t ay;              /* Filtered Y acceleration, m/s^2. */
    float32_t az;              /* Filtered Z acceleration, m/s^2. */
    float32_t accel_norm;      /* Acceleration-vector magnitude, m/s^2. */
    float32_t accel_roll;      /* Roll inferred from gravity, radians. */
    float32_t accel_pitch;     /* Pitch inferred from gravity, radians. */
    float32_t predicted_roll;  /* Gyroscope-propagated roll, radians. */
    float32_t predicted_pitch; /* Gyroscope-propagated pitch, radians. */
    float32_t roll_rate;       /* Euler roll derivative, rad/s. */
    float32_t body_pitch_rate; /* Sign-corrected body q rate, rad/s. */
    float32_t body_yaw_rate;   /* Sign-corrected body r rate, rad/s. */
    float32_t pitch_rate;      /* Euler pitch derivative, rad/s. */
    float32_t correction;      /* Complementary-filter acceleration gain. */
    bool accel_trusted;        /* True when magnitude is close enough to 1 g. */

    ax = data->filtered_accel_mps2.x;
    ay = data->filtered_accel_mps2.y;
    az = data->filtered_accel_mps2.z;
    accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    data->accel_trusted = isfinite(accel_norm) &&
                          (accel_norm >= MIN_ACCEL_NORM_MPS2) &&
                          (accel_norm <= MAX_ACCEL_NORM_MPS2);
    accel_trusted = data->accel_trusted;

    /* Gravity supplies absolute roll/pitch only during low linear acceleration. */
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
        /* Convert body p/q/r to Euler roll/pitch derivatives before integration. */
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

        /* Complement gyro prediction toward the low-frequency gravity angle. */
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

/**
 * Preserve the proven inner yaw-rate PID and add magnetic heading as an outer
 * loop. Moving the yaw stick keeps direct rate control; releasing it captures
 * and holds the current heading. Invalid magnetic data falls back to rate mode.
 */
/**
 * @brief Select pilot yaw-rate control or magnetic heading hold.
 * @param data Instance containing heading, yaw configuration, and rate setpoint.
 * @param yaw_command Deadbanded dimensionless yaw stick in [-1,+1].
 * @return Nothing; yaw setpoint [degrees/second] and hold flags are updated.
 */
static void update_yaw_setpoint(FlightControl_Data_t *data,
                                float32_t yaw_command)
{
    float32_t current_heading_rad; /* Filtered magnetic heading, radians. */
    float32_t heading_error_rad;   /* Shortest held-heading error, radians. */
    float32_t heading_error_deg;   /* Same error in degrees for the outer P loop. */

    /* Without a valid compass, keep the direct yaw-rate behavior operational. */
    if((MAGNETIC_HEADING_HOLD_ENABLED == 0U) ||
       !data->output.heading.valid)
    {
        data->heading_setpoint_initialized = false;
        data->output.heading.setpoint_deg =
            data->output.heading.heading_deg;
        data->output.heading.error_deg = 0.0f;
        data->output.heading.hold_active = false;
        data->output.setpoint.yaw_rate_deg_s =
            yaw_command * data->config.max_yaw_rate_deg_s;
        return;
    }

    current_heading_rad =
        data->output.heading.heading_deg * DEG_TO_RAD;
    /* Capture heading while idle, during pilot yaw, or at first valid sample. */
    if((data->output.setpoint.throttle <=
        data->config.minimum_control_throttle) ||
       (yaw_command != 0.0f) ||
       !data->heading_setpoint_initialized)
    {
        data->heading_setpoint_rad = current_heading_rad;
        data->heading_setpoint_initialized = true;
        data->output.heading.setpoint_deg =
            data->output.heading.heading_deg;
        data->output.heading.error_deg = 0.0f;
        data->output.heading.hold_active = false;
        data->output.setpoint.yaw_rate_deg_s =
            yaw_command * data->config.max_yaw_rate_deg_s;
        return;
    }

    /* Neutral yaw stick closes the outer heading-to-yaw-rate proportional loop. */
    heading_error_rad = wrap_angle(
        data->heading_setpoint_rad - current_heading_rad);
    heading_error_deg = heading_error_rad * RAD_TO_DEG;
    data->output.heading.setpoint_deg =
        data->heading_setpoint_rad * RAD_TO_DEG;
    data->output.heading.error_deg = heading_error_deg;
    data->output.heading.hold_active = true;
    data->output.setpoint.yaw_rate_deg_s = clampf(
        HEADING_ERROR_TO_YAW_RATE_SIGN *
        data->config.yaw_heading_kp * heading_error_deg,
        -data->config.max_yaw_heading_correction_deg_s,
        data->config.max_yaw_heading_correction_deg_s);
}

/**
 * @brief Execute one filtered, bounded rate-PID equation.
 * @param config Gains and limits for this axis. kp, ki, and kd convert the
 *        degree-based error terms into dimensionless motor-mixer effort.
 * @param state Persistent integral, prior-error, and derivative-filter memory.
 * @param setpoint_deg_s Requested angular rate [degrees/second].
 * @param measurement_deg_s Measured angular rate [degrees/second].
 * @param dt_s Elapsed sample time [seconds].
 * @param integrator_enabled true when the I term may accumulate.
 * @param output Destination diagnostic terms, errors, and total effort.
 * @return Bounded dimensionless PID effort for the mixer.
 */
static float32_t update_pid(const FlightControl_PIDConfig_t *config,
                            FlightControl_PIDState_t *state,
                            float32_t setpoint_deg_s,
                            float32_t measurement_deg_s,
                            float32_t dt_s,
                            bool integrator_enabled,
                            FlightControl_AxisOutput_t *output)
{
    float32_t raw_derivative;      /* Measurement derivative, deg/s^2. */
    float32_t filter_time_s;       /* Derivative LPF time constant, seconds. */
    float32_t filter_alpha;        /* Discrete derivative LPF coefficient. */
    float32_t candidate_integrator;/* Proposed bounded I contribution. */
    float32_t unsaturated;         /* P+I+D before final output limiting. */

    output->measured_deg_s = measurement_deg_s;
    output->error_deg_s = setpoint_deg_s - measurement_deg_s;
    output->proportional = config->kp * output->error_deg_s;

    if(!state->initialized)
    {
        state->previous_measurement = measurement_deg_s;
        state->filtered_derivative = 0.0f;
        state->initialized = true;
    }

    /* Derivative-on-measurement avoids a D kick when the setpoint changes. */
    raw_derivative =
        (measurement_deg_s - state->previous_measurement) / dt_s;
    filter_time_s = 1.0f /
        (TWO_PI * config->derivative_cutoff_hz);
    filter_alpha = dt_s / (filter_time_s + dt_s);
    state->filtered_derivative += filter_alpha *
        (raw_derivative - state->filtered_derivative);
    state->previous_measurement = measurement_deg_s;

    /* Integrate rate error only after the throttle indicates likely flight. */
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

    /* Conditional integration stops windup when error drives deeper saturation. */
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

/**
 * @brief Combine throttle and axis efforts into four X-frame motor commands.
 * @param data Instance supplying throttle/PID effort and receiving motor values.
 * @return Nothing; data->output.motors receives four dimensionless [0,1] demands.
 * @note A common shift and optional scale preserve torque differences while
 *       keeping every motor command within the representable interval.
 */
static void mix_x_frame(FlightControl_Data_t *data)
{
    float32_t correction[4]; /* FL, FR, RR, RL differential motor commands. */
    float32_t maximum_correction; /* Largest absolute differential request. */
    float32_t headroom;      /* Symmetric distance to normalized rail 0 or 1. */
    float32_t scale;         /* Common torque scale used to avoid clipping. */
    float32_t throttle;      /* Collective normalized motor command [0,1]. */
    uint32_t index;          /* Motor correction array index. */

    /* X-frame signs assume the documented motor positions and spin directions. */
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

    /* Scale all axes together so attitude direction survives output saturation. */
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
