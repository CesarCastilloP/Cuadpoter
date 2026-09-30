/**
 * @file flight_control.h
 * @author Alberto Vazquez
 * @brief Cascaded attitude and angular-rate control for an X quadcopter.
 * @version 1.6.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_FLIGHT_CONTROL_H_
#define INCLUDE_FLIGHT_CONTROL_H_

#include "horizontal_drift.h"
#include "lsm6ds.h"
#include "mag_heading.h"
#include "rp4tdm.h"

typedef enum
{
    FLIGHT_CONTROL_STATUS_OK = 0,
    FLIGHT_CONTROL_STATUS_IDLE,
    FLIGHT_CONTROL_STATUS_RECEIVER_UNAVAILABLE,
    FLIGHT_CONTROL_STATUS_INVALID_IMU,
    FLIGHT_CONTROL_STATUS_INVALID_ARGUMENT,
    FLIGHT_CONTROL_STATUS_INVALID_CONFIG,
    FLIGHT_CONTROL_STATUS_INVALID_LEVEL_REFERENCE,
    FLIGHT_CONTROL_STATUS_NOT_INITIALIZED
} FlightControl_Status_t;

/** PID gains produce a normalized torque command from errors in deg/s. */
typedef struct
{
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t integrator_limit;
    float32_t output_limit;
    float32_t derivative_cutoff_hz;
} FlightControl_PIDConfig_t;

/** Public configuration copied into each controller instance. */
typedef struct
{
    float32_t max_tilt_deg;
    float32_t max_roll_pitch_rate_deg_s;
    float32_t max_yaw_rate_deg_s;
    float32_t angle_kp;
    /** Outer magnetic-heading loop gain in deg/s per degree of yaw error. */
    float32_t yaw_heading_kp;
    float32_t max_yaw_heading_correction_deg_s;
    float32_t stick_deadband;
    float32_t minimum_control_throttle;
    /** I terms remain cleared below this throttle to prevent ground windup. */
    float32_t integrator_enable_throttle;
    /** Low-pass cutoff for acceleration used by the attitude estimator. */
    float32_t accel_filter_cutoff_hz;
    /** First-order low-pass cutoff applied to gyro rates used by control. */
    float32_t rate_filter_cutoff_hz;
    float32_t attitude_correction_time_s;
    FlightControl_PIDConfig_t roll_rate;
    FlightControl_PIDConfig_t pitch_rate;
    FlightControl_PIDConfig_t yaw_rate;
} FlightControl_Config_t;

typedef struct
{
    float32_t roll_deg;
    float32_t pitch_deg;
    bool valid;
} FlightControl_Attitude_t;

/** Absolute magnetic heading and the outer yaw-angle loop state. */
typedef struct
{
    float32_t heading_deg;
    float32_t setpoint_deg;
    float32_t error_deg;
    bool valid;
    bool hold_active;
} FlightControl_Heading_t;

typedef struct
{
    float32_t throttle;
    float32_t roll_angle_deg;
    float32_t pitch_angle_deg;
    float32_t roll_rate_deg_s;
    float32_t pitch_rate_deg_s;
    float32_t yaw_rate_deg_s;
} FlightControl_Setpoint_t;

/** Rate-loop terms are exposed for tuning in CCS Watch Expressions. */
typedef struct
{
    float32_t measured_deg_s;
    float32_t error_deg_s;
    float32_t proportional;
    float32_t integral;
    float32_t derivative;
    float32_t output;
} FlightControl_AxisOutput_t;

/**
 * Normalized outputs for an X frame viewed from above:
 * front-left CCW, front-right CW, rear-right CCW, rear-left CW.
 */
typedef struct
{
    float32_t front_left;
    float32_t front_right;
    float32_t rear_right;
    float32_t rear_left;
} FlightControl_MotorOutput_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    bool active;
    bool valid;
    bool integrator_enabled;
    FlightControl_Attitude_t attitude;
    FlightControl_Heading_t heading;
    FlightControl_Setpoint_t setpoint;
    FlightControl_AxisOutput_t roll;
    FlightControl_AxisOutput_t pitch;
    FlightControl_AxisOutput_t yaw;
    FlightControl_MotorOutput_t motors;
    float32_t mixer_scale;
} FlightControl_Output_t;

typedef struct
{
    float32_t integrator;
    float32_t previous_measurement;
    float32_t filtered_derivative;
    bool initialized;
} FlightControl_PIDState_t;

/** One independent controller instance and all of its observable outputs. */
typedef struct
{
    bool initialized;
    FlightControl_Config_t config;
    FlightControl_Output_t output;
    /** Startup board attitude removed from the public roll and pitch angles. */
    FlightControl_Attitude_t level_reference;
    /** Optional short-horizon IMU drift brake and its observable estimates. */
    HorizontalDrift_Data_t horizontal_drift;
    FlightControl_Status_t last_status;

    /* Private runtime state. */
    float32_t estimated_roll_rad;
    float32_t estimated_pitch_rad;
    float32_t level_roll_rad;
    float32_t level_pitch_rad;
    float32_t upright_accel_z_sign;
    LSM6DS_Vector3f_t filtered_accel_mps2;
    LSM6DS_Vector3f_t filtered_gyro_rad_s;
    bool accel_trusted;
    bool attitude_initialized;
    bool accel_filter_initialized;
    bool rate_filter_initialized;
    MagHeading_Data_t heading_estimator;
    float32_t heading_setpoint_rad;
    bool heading_setpoint_initialized;
    FlightControl_PIDState_t roll_pid;
    FlightControl_PIDState_t pitch_pid;
    FlightControl_PIDState_t yaw_pid;
} FlightControl_Data_t;

/**
 * Pass NULL for the default controller configuration. The level reference
 * must be the stationary acceleration mean captured during IMU calibration.
 */
FlightControl_Status_t FlightControl_Init(
    FlightControl_Data_t *data,
    const FlightControl_Config_t *config,
    const LSM6DS_Calibration_t *imu_calibration);

/**
 * Updates attitude, rate controllers, and normalized motor variables.
 * No PWM or other hardware output is generated.
 */
FlightControl_Status_t FlightControl_Update(
    FlightControl_Data_t *data,
    LSM6DS_Status_t imu_status,
    const LSM6DS_Sample_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const RP4TDM_Controls_t *receiver);

#endif /* INCLUDE_FLIGHT_CONTROL_H_ */
