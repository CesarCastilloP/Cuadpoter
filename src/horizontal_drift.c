/**
 * @file horizontal_drift.c
 * @author Alberto Vazquez
 * @brief Bounded IMU-only horizontal drift estimation and braking.
 * @version 1.0.0
 * @date 2026-09-29
 */

#include <math.h>
#include <string.h>

#include "horizontal_drift.h"

#define DEG_TO_RAD             0.0174532925f
#define STANDARD_GRAVITY_MPS2  9.80665f

static const HorizontalDrift_Config_t g_default_config = {
    HORIZONTAL_DRIFT_COMPENSATION_ENABLED != 0U,
    2.0f,
    0.04f,
    6.0f,
    4.0f,
    1.5f,
    2.5f,
    2.0f,
    0.35f,
    0.12f,
    0.05f,
    0.08f,
    0.04f,
    0.35f,
    1.50f,
    0.75f
};

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

static float32_t vector_norm(HorizontalDrift_Vector2f_t vector)
{
    return sqrtf((vector.x * vector.x) + (vector.y * vector.y));
}

static float32_t apply_deadband(float32_t value, float32_t deadband)
{
    float32_t magnitude = fabsf(value);

    if(magnitude <= deadband)
    {
        return 0.0f;
    }
    return (value < 0.0f) ?
        -(magnitude - deadband) : (magnitude - deadband);
}

static bool config_is_valid(const HorizontalDrift_Config_t *config)
{
    return (config != NULL) &&
           isfinite(config->acceleration_filter_cutoff_hz) &&
           (config->acceleration_filter_cutoff_hz > 0.0f) &&
           isfinite(config->acceleration_deadband_mps2) &&
           (config->acceleration_deadband_mps2 >= 0.0f) &&
           isfinite(config->velocity_leak_time_s) &&
           (config->velocity_leak_time_s > 0.0f) &&
           isfinite(config->displacement_leak_time_s) &&
           (config->displacement_leak_time_s > 0.0f) &&
           isfinite(config->displacement_gain_deg_per_m) &&
           (config->displacement_gain_deg_per_m >= 0.0f) &&
           isfinite(config->velocity_gain_deg_per_mps) &&
           (config->velocity_gain_deg_per_mps >= 0.0f) &&
           isfinite(config->maximum_correction_deg) &&
           (config->maximum_correction_deg > 0.0f) &&
           (config->maximum_correction_deg <= 5.0f) &&
           isfinite(config->minimum_throttle) &&
           (config->minimum_throttle >= 0.0f) &&
           (config->minimum_throttle < 1.0f) &&
           isfinite(config->motion_acceleration_threshold_mps2) &&
           (config->motion_acceleration_threshold_mps2 > 0.0f) &&
           isfinite(config->motion_velocity_threshold_mps) &&
           (config->motion_velocity_threshold_mps > 0.0f) &&
           isfinite(config->settled_acceleration_threshold_mps2) &&
           (config->settled_acceleration_threshold_mps2 >= 0.0f) &&
           (config->settled_acceleration_threshold_mps2 <
            config->motion_acceleration_threshold_mps2) &&
           isfinite(config->settled_velocity_threshold_mps) &&
           (config->settled_velocity_threshold_mps >= 0.0f) &&
           (config->settled_velocity_threshold_mps <
            config->motion_velocity_threshold_mps) &&
           isfinite(config->settled_time_s) &&
           (config->settled_time_s > 0.0f) &&
           isfinite(config->maximum_velocity_mps) &&
           (config->maximum_velocity_mps > 0.0f) &&
           isfinite(config->maximum_displacement_m) &&
           (config->maximum_displacement_m > 0.0f);
}

static void reset_runtime(HorizontalDrift_Data_t *data,
                          bool increment_counter)
{
    uint32_t reset_count = data->output.reset_count;

    if(increment_counter && (reset_count < 0xFFFFFFFFU))
    {
        reset_count++;
    }
    memset(&data->output, 0, sizeof(data->output));
    data->output.reset_count = reset_count;
    memset(&data->filtered_acceleration_mps2, 0,
           sizeof(data->filtered_acceleration_mps2));
    memset(&data->previous_acceleration_mps2, 0,
           sizeof(data->previous_acceleration_mps2));
    data->settled_elapsed_s = 0.0f;
    data->filter_initialized = false;
    data->integration_initialized = false;
}

HorizontalDrift_Status_t HorizontalDrift_Init(
    HorizontalDrift_Data_t *data,
    const HorizontalDrift_Config_t *config,
    const LSM6DS_Vector3f_t *level_acceleration_mps2)
{
    const HorizontalDrift_Config_t *selected_config;

    if((data == NULL) || (level_acceleration_mps2 == NULL))
    {
        return HORIZONTAL_DRIFT_STATUS_INVALID_ARGUMENT;
    }
    selected_config = (config == NULL) ? &g_default_config : config;
    if(!config_is_valid(selected_config) ||
       !isfinite(level_acceleration_mps2->x) ||
       !isfinite(level_acceleration_mps2->y) ||
       !isfinite(level_acceleration_mps2->z))
    {
        return HORIZONTAL_DRIFT_STATUS_INVALID_CONFIG;
    }

    memset(data, 0, sizeof(*data));
    data->config = *selected_config;
    data->level_acceleration_mps2 = *level_acceleration_mps2;
    data->initialized = true;
    data->last_status = data->config.enabled ?
        HORIZONTAL_DRIFT_STATUS_INACTIVE :
        HORIZONTAL_DRIFT_STATUS_DISABLED;
    return HORIZONTAL_DRIFT_STATUS_OK;
}

void HorizontalDrift_Reset(HorizontalDrift_Data_t *data)
{
    if((data == NULL) || !data->initialized)
    {
        return;
    }
    reset_runtime(data, false);
    data->last_status = data->config.enabled ?
        HORIZONTAL_DRIFT_STATUS_INACTIVE :
        HORIZONTAL_DRIFT_STATUS_DISABLED;
}

HorizontalDrift_Status_t HorizontalDrift_Update(
    HorizontalDrift_Data_t *data,
    const LSM6DS_Vector3f_t *acceleration_mps2,
    float32_t roll_deg,
    float32_t pitch_deg,
    float32_t dt_s,
    float32_t throttle,
    bool command_centered,
    bool heading_stable,
    bool acceleration_trusted)
{
    HorizontalDrift_Vector2f_t raw_acceleration;
    HorizontalDrift_Vector2f_t previous_velocity;
    float32_t acceleration_filter_time_s;
    float32_t acceleration_alpha;
    float32_t velocity_decay;
    float32_t displacement_decay;
    float32_t acceleration_norm;
    float32_t velocity_norm;
    float32_t pitch_correction;
    float32_t roll_correction;

    if((data == NULL) || (acceleration_mps2 == NULL))
    {
        return HORIZONTAL_DRIFT_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = HORIZONTAL_DRIFT_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }
    if(!data->config.enabled)
    {
        reset_runtime(data, false);
        data->last_status = HORIZONTAL_DRIFT_STATUS_DISABLED;
        return data->last_status;
    }
    if(!isfinite(acceleration_mps2->x) ||
       !isfinite(acceleration_mps2->y) ||
       !isfinite(acceleration_mps2->z) ||
       !isfinite(roll_deg) || !isfinite(pitch_deg) ||
       !isfinite(dt_s) || (dt_s <= 0.0f) || (dt_s > 0.02f) ||
       !isfinite(throttle))
    {
        reset_runtime(data, false);
        data->last_status = HORIZONTAL_DRIFT_STATUS_INVALID_ARGUMENT;
        return data->last_status;
    }
    if(!command_centered || !heading_stable || !acceleration_trusted ||
       (throttle < data->config.minimum_throttle))
    {
        reset_runtime(data, false);
        data->last_status = HORIZONTAL_DRIFT_STATUS_INACTIVE;
        return data->last_status;
    }

    data->output.active = true;

    /*
     * This reproduces the validated short-table experiment: subtract the
     * startup stationary baseline and the horizontal gravity projection
     * predicted by the current roll/pitch estimate. X is forward motion;
     * positive Y follows the empirically validated left-motion convention.
     */
    raw_acceleration.x =
        (acceleration_mps2->x - data->level_acceleration_mps2.x) -
        (STANDARD_GRAVITY_MPS2 * sinf(pitch_deg * DEG_TO_RAD));
    raw_acceleration.y =
        (acceleration_mps2->y - data->level_acceleration_mps2.y) -
        (STANDARD_GRAVITY_MPS2 * sinf(roll_deg * DEG_TO_RAD));

    acceleration_filter_time_s = 1.0f /
        (2.0f * M_PI * data->config.acceleration_filter_cutoff_hz);
    acceleration_alpha = dt_s /
        (acceleration_filter_time_s + dt_s);
    if(!data->filter_initialized)
    {
        data->filtered_acceleration_mps2 = raw_acceleration;
        data->filter_initialized = true;
    }
    else
    {
        data->filtered_acceleration_mps2.x += acceleration_alpha *
            (raw_acceleration.x - data->filtered_acceleration_mps2.x);
        data->filtered_acceleration_mps2.y += acceleration_alpha *
            (raw_acceleration.y - data->filtered_acceleration_mps2.y);
    }

    data->output.acceleration_mps2.x = apply_deadband(
        data->filtered_acceleration_mps2.x,
        data->config.acceleration_deadband_mps2);
    data->output.acceleration_mps2.y = apply_deadband(
        data->filtered_acceleration_mps2.y,
        data->config.acceleration_deadband_mps2);

    previous_velocity = data->output.velocity_mps;
    velocity_decay = data->config.velocity_leak_time_s /
        (data->config.velocity_leak_time_s + dt_s);
    displacement_decay = data->config.displacement_leak_time_s /
        (data->config.displacement_leak_time_s + dt_s);

    if(!data->integration_initialized)
    {
        data->previous_acceleration_mps2 =
            data->output.acceleration_mps2;
        data->integration_initialized = true;
    }
    data->output.velocity_mps.x = clampf(
        (velocity_decay * data->output.velocity_mps.x) +
        (0.5f * (data->previous_acceleration_mps2.x +
                 data->output.acceleration_mps2.x) * dt_s),
        -data->config.maximum_velocity_mps,
        data->config.maximum_velocity_mps);
    data->output.velocity_mps.y = clampf(
        (velocity_decay * data->output.velocity_mps.y) +
        (0.5f * (data->previous_acceleration_mps2.y +
                 data->output.acceleration_mps2.y) * dt_s),
        -data->config.maximum_velocity_mps,
        data->config.maximum_velocity_mps);
    data->output.displacement_m.x = clampf(
        (displacement_decay * data->output.displacement_m.x) +
        (0.5f * (previous_velocity.x +
                 data->output.velocity_mps.x) * dt_s),
        -data->config.maximum_displacement_m,
        data->config.maximum_displacement_m);
    data->output.displacement_m.y = clampf(
        (displacement_decay * data->output.displacement_m.y) +
        (0.5f * (previous_velocity.y +
                 data->output.velocity_mps.y) * dt_s),
        -data->config.maximum_displacement_m,
        data->config.maximum_displacement_m);
    data->previous_acceleration_mps2 =
        data->output.acceleration_mps2;

    acceleration_norm = vector_norm(data->output.acceleration_mps2);
    velocity_norm = vector_norm(data->output.velocity_mps);
    if((acceleration_norm >=
        data->config.motion_acceleration_threshold_mps2) ||
       (velocity_norm >= data->config.motion_velocity_threshold_mps))
    {
        data->output.motion_detected = true;
    }

    pitch_correction =
        (data->config.displacement_gain_deg_per_m *
         data->output.displacement_m.x) +
        (data->config.velocity_gain_deg_per_mps *
         data->output.velocity_mps.x);
    roll_correction =
        (data->config.displacement_gain_deg_per_m *
         data->output.displacement_m.y) +
        (data->config.velocity_gain_deg_per_mps *
         data->output.velocity_mps.y);
    data->output.pitch_correction_deg = clampf(
        pitch_correction,
        -data->config.maximum_correction_deg,
        data->config.maximum_correction_deg);
    data->output.roll_correction_deg = clampf(
        roll_correction,
        -data->config.maximum_correction_deg,
        data->config.maximum_correction_deg);

    if(data->output.motion_detected &&
       (acceleration_norm <=
        data->config.settled_acceleration_threshold_mps2) &&
       (velocity_norm <= data->config.settled_velocity_threshold_mps))
    {
        data->settled_elapsed_s += dt_s;
    }
    else
    {
        data->settled_elapsed_s = 0.0f;
    }

    if(data->settled_elapsed_s >= data->config.settled_time_s)
    {
        reset_runtime(data, true);
        data->output.active = true;
        data->last_status = HORIZONTAL_DRIFT_STATUS_RESET;
        return data->last_status;
    }

    data->last_status = HORIZONTAL_DRIFT_STATUS_OK;
    return data->last_status;
}
