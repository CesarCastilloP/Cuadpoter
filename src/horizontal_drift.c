/**
 * @file horizontal_drift.c
 * @author Alberto Vazquez
 * @brief Bounded IMU-only horizontal drift estimation and braking.
 *
 * @details Beginner's reading guide:
 * - This optional module removes the gravity component from accelerometer data,
 *   rotates the remaining horizontal acceleration into a heading-fixed frame,
 *   and integrates it over short intervals into velocity and displacement.
 * - The estimates use m/s^2, m/s, and m respectively. They intentionally leak,
 *   clamp, and reset because an IMU alone cannot provide absolute XY position.
 * - The outputs are small roll/pitch corrections in degrees. They are added to
 *   attitude setpoints only when the feature and all gating conditions are true.
 * - Centered sticks request braking; deliberate pilot commands immediately reset
 *   the local estimate and return zero correction.
 * @version 1.1.0
 * @date 2026-09-29
 */

#include <math.h>
#include <string.h>

#include "horizontal_drift.h"

/* Convert attitude angles at the public degree-based interface to radians. */
#define DEG_TO_RAD             0.0174532925f
/* Standard gravitational acceleration used to remove horizontal gravity. */
#define STANDARD_GRAVITY_MPS2  9.80665f

/* Flight-tested defaults; every entry is named and dimensioned below. */
static const HorizontalDrift_Config_t g_default_config = {
    HORIZONTAL_DRIFT_COMPENSATION_ENABLED != 0U, /* enabled */
    2.0f,  /* acceleration_filter_cutoff_hz */
    0.04f, /* acceleration_deadband_mps2 */
    10.0f, /* velocity_leak_time_s */
    8.0f,  /* displacement_leak_time_s */
    1.0f,  /* displacement_gain_deg_per_m */
    4.0f,  /* velocity_gain_deg_per_mps */
    3.0f,  /* maximum_correction_deg */
    0.35f, /* minimum_throttle, normalized */
    0.12f, /* motion_acceleration_threshold_mps2 */
    0.05f, /* motion_velocity_threshold_mps */
    0.08f, /* settled_acceleration_threshold_mps2 */
    0.04f, /* settled_velocity_threshold_mps */
    0.35f, /* settled_time_s */
    1.50f, /* maximum_velocity_mps */
    0.75f  /* maximum_displacement_m */
};

/** Limit a scalar to [minimum, maximum]; all three arguments share one unit. */
static float32_t clampf(float32_t value,
                        float32_t minimum,
                        float32_t maximum)
{
    /* Saturation bounds estimator states and final correction commands. */
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
 * @brief Calculate the Euclidean length of an XY vector.
 * @param vector Two components carrying the same physical unit.
 * @return sqrt(x*x+y*y), in the component's original unit.
 */
static float32_t vector_norm(HorizontalDrift_Vector2f_t vector)
{
    /* Euclidean magnitude of a horizontal X/Y vector. */
    return sqrtf((vector.x * vector.x) + (vector.y * vector.y));
}

/**
 * @brief Remove signed sensor noise whose magnitude does not exceed deadband.
 * @param value Measurement to condition.
 * @param deadband Symmetric threshold in the same unit as value.
 * @return Zero near the origin; otherwise value with the deadband removed.
 */
static float32_t apply_deadband(float32_t value, float32_t deadband)
{
    /* Absolute input magnitude, in the same units as value and deadband. */
    float32_t magnitude = fabsf(value);

    if(magnitude <= deadband)
    {
        return 0.0f;
    }
    /* Subtract the threshold to keep the transfer function continuous. */
    return (value < 0.0f) ?
        -(magnitude - deadband) : (magnitude - deadband);
}

/** Validate every drift-estimator threshold, gain, limit, and time constant. */
static bool config_is_valid(const HorizontalDrift_Config_t *config)
{
    /* Enforce finite, physical values and the intended threshold ordering. */
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

/**
 * @brief Clear local acceleration, velocity, displacement, and angle correction.
 * @param data Estimator instance to reset.
 * @param increment_counter true to record this operational reset in telemetry.
 * @return Nothing; configuration and level reference are retained.
 */
static void reset_runtime(HorizontalDrift_Data_t *data,
                          bool increment_counter)
{
    /* Preserve the lifetime reset diagnostic while clearing estimator states. */
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

/**
 * @brief Initialize the bounded inertial estimator and store its gravity reference.
 * @param data Destination estimator instance.
 * @param config Optional configuration; NULL selects defaults.
 * @param level_acceleration_mps2 Stationary body acceleration vector [m/s^2].
 * @return OK or a precise argument/configuration/reference error.
 */
HorizontalDrift_Status_t HorizontalDrift_Init(
    HorizontalDrift_Data_t *data,
    const HorizontalDrift_Config_t *config,
    const LSM6DS_Vector3f_t *level_acceleration_mps2)
{
    /* Effective configuration: caller supplied or flight-tested defaults. */
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

    /* The stationary calibration acceleration becomes the zero-motion baseline. */
    memset(data, 0, sizeof(*data));
    data->config = *selected_config;
    data->level_acceleration_mps2 = *level_acceleration_mps2;
    data->initialized = true;
    data->last_status = data->config.enabled ?
        HORIZONTAL_DRIFT_STATUS_INACTIVE :
        HORIZONTAL_DRIFT_STATUS_DISABLED;
    return HORIZONTAL_DRIFT_STATUS_OK;
}

/**
 * @brief Explicitly discard the current local motion estimate.
 * @param data Estimator instance; NULL is accepted and ignored.
 * @return Nothing.
 */
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

/**
 * @brief Update short-term horizontal motion and attitude-braking corrections.
 * @param data Initialized estimator and destination for new state.
 * @param acceleration_mps2 Calibrated body acceleration [m/s^2].
 * @param roll_deg Current aircraft roll [degrees].
 * @param pitch_deg Current aircraft pitch [degrees].
 * @param dt_s Time since the previous fresh IMU sample [seconds].
 * @param throttle Dimensionless collective command [0,1].
 * @param command_centered true when pilot roll and pitch sticks are centered.
 * @param heading_stable true when yaw reference is usable for earth-frame rotation.
 * @param acceleration_trusted true when acceleration magnitude is physically plausible.
 * @return OK while estimating, or DISABLED/INACTIVE/RESET/error status otherwise.
 */
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
    HorizontalDrift_Vector2f_t raw_acceleration; /* Gravity-removed body X/Y, m/s^2. */
    HorizontalDrift_Vector2f_t previous_velocity; /* Previous integration state, m/s. */
    float32_t acceleration_filter_time_s; /* Acceleration LPF time constant, seconds. */
    float32_t acceleration_alpha; /* Discrete first-order acceleration LPF gain. */
    float32_t velocity_decay;     /* Leaky velocity memory factor in [0,1]. */
    float32_t displacement_decay; /* Leaky displacement memory factor in [0,1]. */
    float32_t acceleration_norm;  /* Horizontal acceleration magnitude, m/s^2. */
    float32_t velocity_norm;      /* Estimated horizontal speed, m/s. */
    float32_t pitch_correction;   /* Unsaturated forward braking angle, degrees. */
    float32_t roll_correction;    /* Unsaturated lateral braking angle, degrees. */

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
    /* Estimation is meaningful only in hands-off, airborne, stable conditions. */
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

    /* First-order low-pass: alpha = dt / (tau + dt), tau = 1/(2*pi*fc). */
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

    /* Leaks make unobservable IMU bias decay instead of growing without bound. */
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
    /* Trapezoidal acceleration integration estimates horizontal velocity. */
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
    /* A second trapezoidal integration estimates short-term displacement. */
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

    /* Latch motion after either measured acceleration or estimated speed is clear. */
    acceleration_norm = vector_norm(data->output.acceleration_mps2);
    velocity_norm = vector_norm(data->output.velocity_mps);
    if((acceleration_norm >=
        data->config.motion_acceleration_threshold_mps2) ||
       (velocity_norm >= data->config.motion_velocity_threshold_mps))
    {
        data->output.motion_detected = true;
    }

    /* PD-like braking: position creates restoring tilt; velocity opposes motion. */
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

    /* Require sustained low motion before clearing the completed correction event. */
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
