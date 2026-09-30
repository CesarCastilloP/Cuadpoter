/**
 * @file horizontal_drift.h
 * @author Alberto Vazquez
 * @brief Short-horizon inertial brake for uncommanded horizontal drift.
 * @version 1.0.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_HORIZONTAL_DRIFT_H_
#define INCLUDE_HORIZONTAL_DRIFT_H_

#include "lsm6ds.h"

/** Change to 0 to compile the controller with drift compensation disabled. */
#define HORIZONTAL_DRIFT_COMPENSATION_ENABLED  1U

typedef enum
{
    HORIZONTAL_DRIFT_STATUS_OK = 0,
    HORIZONTAL_DRIFT_STATUS_RESET,
    HORIZONTAL_DRIFT_STATUS_INACTIVE,
    HORIZONTAL_DRIFT_STATUS_DISABLED,
    HORIZONTAL_DRIFT_STATUS_INVALID_ARGUMENT,
    HORIZONTAL_DRIFT_STATUS_INVALID_CONFIG,
    HORIZONTAL_DRIFT_STATUS_NOT_INITIALIZED
} HorizontalDrift_Status_t;

typedef struct
{
    float32_t x;
    float32_t y;
} HorizontalDrift_Vector2f_t;

typedef struct
{
    bool enabled;
    float32_t acceleration_filter_cutoff_hz;
    float32_t acceleration_deadband_mps2;
    float32_t velocity_leak_time_s;
    float32_t displacement_leak_time_s;
    float32_t displacement_gain_deg_per_m;
    float32_t velocity_gain_deg_per_mps;
    float32_t maximum_correction_deg;
    float32_t minimum_throttle;
    float32_t motion_acceleration_threshold_mps2;
    float32_t motion_velocity_threshold_mps;
    float32_t settled_acceleration_threshold_mps2;
    float32_t settled_velocity_threshold_mps;
    float32_t settled_time_s;
    float32_t maximum_velocity_mps;
    float32_t maximum_displacement_m;
} HorizontalDrift_Config_t;

typedef struct
{
    HorizontalDrift_Vector2f_t acceleration_mps2;
    HorizontalDrift_Vector2f_t velocity_mps;
    HorizontalDrift_Vector2f_t displacement_m;
    float32_t roll_correction_deg;
    float32_t pitch_correction_deg;
    bool active;
    bool motion_detected;
    uint32_t reset_count;
} HorizontalDrift_Output_t;

/** One independent short-horizon estimator/controller instance. */
typedef struct
{
    bool initialized;
    HorizontalDrift_Config_t config;
    HorizontalDrift_Output_t output;
    HorizontalDrift_Status_t last_status;

    /* Private runtime state. */
    LSM6DS_Vector3f_t level_acceleration_mps2;
    HorizontalDrift_Vector2f_t filtered_acceleration_mps2;
    HorizontalDrift_Vector2f_t previous_acceleration_mps2;
    float32_t settled_elapsed_s;
    bool filter_initialized;
    bool integration_initialized;
} HorizontalDrift_Data_t;

/**
 * Initialize an instance. The level acceleration is the stationary mean
 * captured during the normal IMU startup calibration.
 */
HorizontalDrift_Status_t HorizontalDrift_Init(
    HorizontalDrift_Data_t *data,
    const HorizontalDrift_Config_t *config,
    const LSM6DS_Vector3f_t *level_acceleration_mps2);

/** Clear the local velocity/displacement origin and commanded correction. */
void HorizontalDrift_Reset(HorizontalDrift_Data_t *data);

/**
 * Update the short-horizon estimator. command_centered must be true only when
 * the pilot is not requesting roll, pitch, or yaw. heading_stable indicates
 * that magnetic heading hold is active, so the body-local horizontal axes do
 * not rotate during an estimation episode.
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
    bool acceleration_trusted);

#endif /* INCLUDE_HORIZONTAL_DRIFT_H_ */
