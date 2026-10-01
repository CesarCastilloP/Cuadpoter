/**
 * @file flight_control.h
 * @author Alberto Vazquez
 * @brief Cascaded attitude and angular-rate control for an X quadcopter.
 *
 * @details FlightControl_Update() is offered work by main() at the 1 kHz IMU
 * polling rate, but it recalculates control only for fresh valid IMU samples,
 * approximately 416 times/second. Input is calibrated acceleration [m/s^2],
 * angular rate [rad/s], receiver controls [normalized], and magnetic field [uT].
 * Output is observable attitude [deg], setpoints [deg and deg/s], individual
 * PID terms [normalized torque], and four mixer commands in the range 0...1.
 * MotorOutput converts those four values to ESC pulse widths in a later layer.
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
    /** Attitude, rate PID, and mixer updated from a valid IMU sample. */
    FLIGHT_CONTROL_STATUS_OK = 0,
    /** Inputs are valid but throttle is at/below the control threshold. */
    FLIGHT_CONTROL_STATUS_IDLE,
    /** CRSF controls are invalid or timed out; motor commands are cleared. */
    FLIGHT_CONTROL_STATUS_RECEIVER_UNAVAILABLE,
    /** IMU status/sample/timing/saturation failed flight requirements. */
    FLIGHT_CONTROL_STATUS_INVALID_IMU,
    /** Required pointer argument was NULL. */
    FLIGHT_CONTROL_STATUS_INVALID_ARGUMENT,
    /** Controller, PID, heading, or drift configuration is invalid. */
    FLIGHT_CONTROL_STATUS_INVALID_CONFIG,
    /** Startup acceleration is missing, non-1g, or tilted more than 15 degrees. */
    FLIGHT_CONTROL_STATUS_INVALID_LEVEL_REFERENCE,
    /** Update was called before FlightControl_Init. */
    FLIGHT_CONTROL_STATUS_NOT_INITIALIZED
} FlightControl_Status_t;

/** PID gains produce a normalized torque command from errors in deg/s. */
typedef struct
{
    /** Proportional gain. Unit: normalized output per (degree/second). */
    float32_t kp;
    /** Integral gain. Unit: normalized output per degree. */
    float32_t ki;
    /** Derivative-on-measurement gain. Unit: normalized output per (deg/s²). */
    float32_t kd;
    /** Absolute anti-windup clamp applied to the I contribution. */
    float32_t integrator_limit;
    /** Absolute normalized torque-command clamp for this axis. */
    float32_t output_limit;
    /** First-order derivative low-pass cutoff. Unit: hertz. */
    float32_t derivative_cutoff_hz;
} FlightControl_PIDConfig_t;

/** Public configuration copied into each controller instance. */
typedef struct
{
    /** Maximum roll or pitch angle commanded by the stick/trim sum. Unit: deg. */
    float32_t max_tilt_deg;
    /** Fixed roll setpoint offset for mechanical/aerodynamic trim. Unit: deg. */
    float32_t flight_roll_trim_deg;
    /** Fixed pitch setpoint offset for mechanical/aerodynamic trim. Unit: deg. */
    float32_t flight_pitch_trim_deg;
    /** Outer attitude-loop roll/pitch rate limit. Unit: degrees/second. */
    float32_t max_roll_pitch_rate_deg_s;
    /** Maximum direct yaw-stick rate. Unit: degrees/second. */
    float32_t max_yaw_rate_deg_s;
    /** Angle-error to angular-rate gain. Unit: (deg/s)/deg. */
    float32_t angle_kp;
    /** Outer magnetic-heading loop gain in deg/s per degree of yaw error. */
    float32_t yaw_heading_kp;
    /** Absolute yaw rate requested by heading hold. Unit: degrees/second. */
    float32_t max_yaw_heading_correction_deg_s;
    /** Symmetric normalized stick region mapped exactly to zero. */
    float32_t stick_deadband;
    /** Throttle at/below which PID/mixer are inactive. Unit: normalized 0...1. */
    float32_t minimum_control_throttle;
    /** Roll/pitch I terms remain cleared below this throttle to prevent windup. */
    float32_t integrator_enable_throttle;
    /** Yaw I term enable threshold; lower than roll/pitch to learn motor torque bias. */
    float32_t yaw_integrator_enable_throttle;
    /** Low-pass cutoff for acceleration used by the attitude estimator. */
    float32_t accel_filter_cutoff_hz;
    /** First-order low-pass cutoff applied to gyro rates used by control. */
    float32_t rate_filter_cutoff_hz;
    /** Accelerometer correction time constant in the complementary filter. Unit: s. */
    float32_t attitude_correction_time_s;
    /** Roll body-rate PID parameters. */
    FlightControl_PIDConfig_t roll_rate;
    /** Pitch body-rate PID parameters. */
    FlightControl_PIDConfig_t pitch_rate;
    /** Yaw body-rate PID parameters. */
    FlightControl_PIDConfig_t yaw_rate;
} FlightControl_Config_t;

/** Public level-referenced roll/pitch estimate. */
typedef struct
{
    /** Right-wing-down rotation around body X. Unit: degrees. */
    float32_t roll_deg;
    /** Nose-up rotation around body Y according to installed sign. Unit: deg. */
    float32_t pitch_deg;
    /** True after the complementary attitude estimator is initialized. */
    bool valid;
} FlightControl_Attitude_t;

/** Absolute magnetic heading and the outer yaw-angle loop state. */
typedef struct
{
    /** Filtered tilt-compensated magnetic heading. Unit: degrees. */
    float32_t heading_deg;
    /** Heading captured when yaw stick returns to center. Unit: degrees. */
    float32_t setpoint_deg;
    /** Shortest wrapped heading-setpoint error. Unit: degrees. */
    float32_t error_deg;
    /** True while the magnetic estimator accepts field and age. */
    bool valid;
    /** True when centered yaw uses heading error instead of direct yaw rate. */
    bool hold_active;
} FlightControl_Heading_t;

/** Commands produced after receiver mapping and all outer loops. */
typedef struct
{
    /** Collective command passed unchanged to the mixer. Range: 0...1. */
    float32_t throttle;
    /** Final roll angle target including inertial trim. Unit: degrees. */
    float32_t roll_angle_deg;
    /** Final pitch angle target including inertial trim. Unit: degrees. */
    float32_t pitch_angle_deg;
    /** Roll rate target generated by the angle loop. Unit: degrees/second. */
    float32_t roll_rate_deg_s;
    /** Pitch rate target generated by the angle loop. Unit: degrees/second. */
    float32_t pitch_rate_deg_s;
    /** Manual or magnetic-hold yaw target. Unit: degrees/second. */
    float32_t yaw_rate_deg_s;
} FlightControl_Setpoint_t;

/** Rate-loop terms are exposed for tuning in CCS Watch Expressions. */
typedef struct
{
    /** Filtered angular-rate feedback. Unit: degrees/second. */
    float32_t measured_deg_s;
    /** setpoint minus measurement. Unit: degrees/second. */
    float32_t error_deg_s;
    /** Kp multiplied by rate error. Unit: normalized torque. */
    float32_t proportional;
    /** Accumulated Ki contribution after anti-windup. Unit: normalized torque. */
    float32_t integral;
    /** Filtered derivative-on-measurement contribution. Unit: normalized torque. */
    float32_t derivative;
    /** Saturated P+I+D result delivered to the mixer. Unit: normalized torque. */
    float32_t output;
} FlightControl_AxisOutput_t;

/**
 * Normalized outputs for an X frame viewed from above:
 * front-left CCW, front-right CW, rear-right CCW, rear-left CW.
 */
typedef struct
{
    /** Normalized front-left command before conversion to microseconds. */
    float32_t front_left;
    /** Normalized front-right command before conversion to microseconds. */
    float32_t front_right;
    /** Normalized rear-right command before conversion to microseconds. */
    float32_t rear_right;
    /** Normalized rear-left command before conversion to microseconds. */
    float32_t rear_left;
} FlightControl_MotorOutput_t;

/** Complete coherent result produced by one FlightControl_Update call. */
typedef struct
{
    /** Timestamp copied from the driving IMU sample. Unit: microseconds. */
    uint64_t timestamp_us;
    /** Number of controller update attempts using valid sample flow. */
    uint32_t sequence;
    /** True when throttle is high enough for PID/mixer actuation. */
    bool active;
    /** True when sensor/receiver prerequisites passed this update. */
    bool valid;
    /** True when throttle permits the roll/pitch I terms to accumulate. */
    bool integrator_enabled;
    /** True when throttle permits the independent yaw I term to accumulate. */
    bool yaw_integrator_enabled;
    /** Current roll/pitch estimate. */
    FlightControl_Attitude_t attitude;
    /** Current magnetic heading and hold state. */
    FlightControl_Heading_t heading;
    /** Outer-loop angle/rate/collective targets. */
    FlightControl_Setpoint_t setpoint;
    /** Roll measured rate, PID terms, and output. */
    FlightControl_AxisOutput_t roll;
    /** Pitch measured rate, PID terms, and output. */
    FlightControl_AxisOutput_t pitch;
    /** Yaw measured rate, PID terms, and output. */
    FlightControl_AxisOutput_t yaw;
    /** Four normalized X-mixer outputs. */
    FlightControl_MotorOutput_t motors;
    /** Common differential scale; below 1 means mixer headroom limiting. */
    float32_t mixer_scale;
} FlightControl_Output_t;

/** Private memory for one derivative-filtered PID axis. */
typedef struct
{
    /** Stored I contribution after anti-windup. Unit: normalized torque. */
    float32_t integrator;
    /** Previous measured angular rate for derivative-on-measurement. Unit: deg/s. */
    float32_t previous_measurement;
    /** Low-pass derivative of measurement. Unit: degrees/second squared. */
    float32_t filtered_derivative;
    /** False until the first measurement seeds derivative history. */
    bool initialized;
} FlightControl_PIDState_t;

/** One independent controller instance and all of its observable outputs. */
typedef struct
{
    /** True after controller, heading, and drift initialization. */
    bool initialized;
    /** Instance-specific controller/PID parameters. */
    FlightControl_Config_t config;
    /** Complete public result consumed by MotorOutput and Telemetry. */
    FlightControl_Output_t output;
    /** Startup board attitude removed from the public roll and pitch angles. */
    FlightControl_Attitude_t level_reference;
    /** Optional short-horizon IMU drift brake and its observable estimates. */
    HorizontalDrift_Data_t horizontal_drift;
    /** Most recent controller result for CCS. */
    FlightControl_Status_t last_status;

    /* Private runtime state. */
    /** Complementary-filter roll state. Unit: radians. */
    float32_t estimated_roll_rad;
    /** Complementary-filter pitch state. Unit: radians. */
    float32_t estimated_pitch_rad;
    /** Mechanical startup roll removed from public attitude. Unit: radians. */
    float32_t level_roll_rad;
    /** Mechanical startup pitch removed from public attitude. Unit: radians. */
    float32_t level_pitch_rad;
    /** +1 or -1 selected from startup accel Z to normalize upright formulas. */
    float32_t upright_accel_z_sign;
    /** 3 Hz acceleration low-pass used for attitude and trust. Unit: m/s². */
    LSM6DS_Vector3f_t filtered_accel_mps2;
    /** 15 Hz gyro low-pass used by attitude/PID. Unit: radians/second. */
    LSM6DS_Vector3f_t filtered_gyro_rad_s;
    /** True while filtered acceleration norm is between 0.90g and 1.10g. */
    bool accel_trusted;
    /** True after gravity angles seed the complementary filter. */
    bool attitude_initialized;
    /** True after the acceleration low-pass has been seeded. */
    bool accel_filter_initialized;
    /** True after the rate low-pass has been seeded. */
    bool rate_filter_initialized;
    /** Tilt-compensated magnetic heading estimator instance. */
    MagHeading_Data_t heading_estimator;
    /** Captured magnetic heading target. Unit: radians. */
    float32_t heading_setpoint_rad;
    /** False until centered yaw captures a valid heading. */
    bool heading_setpoint_initialized;
    /** Private roll rate PID memory. */
    FlightControl_PIDState_t roll_pid;
    /** Private pitch rate PID memory. */
    FlightControl_PIDState_t pitch_pid;
    /** Private yaw rate PID memory. */
    FlightControl_PIDState_t yaw_pid;
} FlightControl_Data_t;

/**
 * Pass NULL for the default controller configuration. The level reference
 * must be the stationary acceleration mean captured during IMU calibration.
 * @param data Writable complete controller instance.
 * @param config Optional configuration; NULL selects flight defaults.
 * @param imu_calibration Valid startup bias/level calibration from LSM6DS.
 * @return FLIGHT_CONTROL_STATUS_OK or an argument/configuration/reference error.
 */
FlightControl_Status_t FlightControl_Init(
    FlightControl_Data_t *data,
    const FlightControl_Config_t *config,
    const LSM6DS_Calibration_t *imu_calibration);

/**
 * Updates attitude, rate controllers, and normalized motor variables.
 * No PWM or other hardware output is generated.
 * @param data Initialized controller instance.
 * @param imu_status Result returned by the matching LSM6DS_Update call.
 * @param imu Latest physical IMU sample.
 * @param magnetometer Latest calibrated LIS2MDL instance.
 * @param receiver Latest normalized CRSF controls.
 * @return OK for active control, IDLE at low throttle, or a precise gating error.
 */
FlightControl_Status_t FlightControl_Update(
    FlightControl_Data_t *data,
    LSM6DS_Status_t imu_status,
    const LSM6DS_Sample_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const RP4TDM_Controls_t *receiver);

#endif /* INCLUDE_FLIGHT_CONTROL_H_ */
