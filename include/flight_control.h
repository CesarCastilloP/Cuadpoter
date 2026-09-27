/**
 * @file flight_control.h
 * @author Alberto Vazquez
 * @brief Cascaded attitude and angular-rate control for an X quadcopter.
 * @version 1.2.2
 * @date 2026-09-23
 */

#ifndef INCLUDE_FLIGHT_CONTROL_H_
#define INCLUDE_FLIGHT_CONTROL_H_

#include "lsm6ds.h"
#include "rp4tdm.h"

typedef enum
{
    FLIGHT_CONTROL_STATUS_OK = 0,
    FLIGHT_CONTROL_STATUS_IDLE,
    FLIGHT_CONTROL_STATUS_RECEIVER_UNAVAILABLE,
    FLIGHT_CONTROL_STATUS_INVALID_IMU,
    FLIGHT_CONTROL_STATUS_INVALID_ARGUMENT,
    FLIGHT_CONTROL_STATUS_INVALID_CONFIG,
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
    float32_t stick_deadband;
    float32_t minimum_control_throttle;
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
    FlightControl_Attitude_t attitude;
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
    FlightControl_Status_t last_status;

    /* Private runtime state. */
    float32_t estimated_roll_rad;
    float32_t estimated_pitch_rad;
    float32_t upright_accel_z_sign;
    bool attitude_initialized;
    FlightControl_PIDState_t roll_pid;
    FlightControl_PIDState_t pitch_pid;
    FlightControl_PIDState_t yaw_pid;
} FlightControl_Data_t;

/** Pass NULL for conservative initial gains intended for bench validation. */
FlightControl_Status_t FlightControl_Init(
    FlightControl_Data_t *data,
    const FlightControl_Config_t *config);

/**
 * Updates attitude, rate controllers, and normalized motor variables.
 * No PWM or other hardware output is generated.
 */
FlightControl_Status_t FlightControl_Update(
    FlightControl_Data_t *data,
    LSM6DS_Status_t imu_status,
    const LSM6DS_Sample_t *imu,
    const RP4TDM_Controls_t *receiver);

#endif /* INCLUDE_FLIGHT_CONTROL_H_ */
