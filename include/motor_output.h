/**
 * @file motor_output.h
 * @author Alberto Vazquez
 * @brief Conversion from normalized motor commands to synchronized ESC pulses.
 *
 * @details main() calls MotorOutput_Update() immediately after FlightControl.
 * This layer validates controller state, requires intentional throttle, selects
 * the safe pulse when control is invalid/idle, maps each normalized command
 * from 0...1 into the configured microsecond range, and calls EscPwm_Write().
 * Current defaults use 1000 us safe/minimum, 2000 us maximum, and a 6000 us
 * PWM frame. The four public pulse_us fields are the exact requested high times.
 * @version 1.3.0
 * @date 2026-09-28
 */

#ifndef INCLUDE_MOTOR_OUTPUT_H_
#define INCLUDE_MOTOR_OUTPUT_H_

#include "esc_pwm.h"
#include "flight_control.h"

typedef enum
{
    /** Flight command was converted and applied to hardware PWM. */
    MOTOR_OUTPUT_STATUS_READY = 0,
    /** Four safe pulses are active because control is disabled/invalid. */
    MOTOR_OUTPUT_STATUS_SAFE,
    /** Required pointer or caller value was invalid. */
    MOTOR_OUTPUT_STATUS_INVALID_ARGUMENT,
    /** Pulse ordering or frame-period configuration is invalid. */
    MOTOR_OUTPUT_STATUS_INVALID_CONFIG,
    /** Update was called before MotorOutput_Init. */
    MOTOR_OUTPUT_STATUS_NOT_INITIALIZED,
    /** EscPwm rejected initialization or a synchronized write. */
    MOTOR_OUTPUT_STATUS_HARDWARE_ERROR
} MotorOutput_Status_t;

/** ESC pulse policy used between normalized control and the PWM driver. */
typedef struct
{
    /** Absolute lower hardware pulse bound. Unit: microseconds. */
    uint16_t minimum_pulse_us;
    /** Absolute upper hardware pulse bound. Unit: microseconds. */
    uint16_t maximum_pulse_us;
    /** Lowest pulse allowed while motors are active. Unit: microseconds. */
    uint16_t active_idle_pulse_us;
    /** Pulse sent whenever control is inactive or invalid. Unit: microseconds. */
    uint16_t safe_pulse_us;
    /** PWM repetition period. Unit: microseconds. */
    uint16_t frame_period_us;
} MotorOutput_Config_t;

/** Four physical ESC pulse widths in frame order FL, FR, RR, RL. */
typedef struct
{
    /** Front-left ESC pulse on PF0. Unit: microseconds. */
    uint16_t front_left;
    /** Front-right ESC pulse on PF2. Unit: microseconds. */
    uint16_t front_right;
    /** Rear-right ESC pulse on PG0. Unit: microseconds. */
    uint16_t rear_right;
    /** Rear-left ESC pulse on PK4. Unit: microseconds. */
    uint16_t rear_left;
} MotorOutput_Pulses_t;

/** One motor-output instance and the pulse widths ready for the PWM driver. */
typedef struct
{
    /** True after pulse policy and hardware PWM initialize successfully. */
    bool initialized;
    /** True while a valid active flight command is applied. */
    bool enabled;
    /** Instance-owned pulse limits and frame period. */
    MotorOutput_Config_t config;
    /** Last four pulses requested from EscPwm. Unit: microseconds. */
    MotorOutput_Pulses_t pulse_us;
    /** Common upward shift used to preserve torque near active idle. */
    float32_t collective_shift_us;
    /** Differential scale; 1.0 means the requested motor spread was kept. */
    float32_t active_range_scale;
    /** Complete low-level PWM driver state and diagnostics. */
    EscPwm_Data_t pwm;
    /** Most recent conversion/write result for CCS. */
    MotorOutput_Status_t last_status;
} MotorOutput_Data_t;

/**
 * Pass NULL for 1000 us safe, 1180 us active idle, 2000 us maximum,
 * and a 6000 us frame.
 * @param data Writable output instance.
 * @param system_clock_hz Current MCU system clock in hertz.
 * @param config Optional pulse policy; NULL selects the documented defaults.
 * @return SAFE after initialized low pulses, or an argument/config/hardware error.
 */
MotorOutput_Status_t MotorOutput_Init(
    MotorOutput_Data_t *data,
    uint32_t system_clock_hz,
    const MotorOutput_Config_t *config);

/**
 * Convert the latest normalized flight-control outputs into pulse widths.
 * @param data Initialized output/PWM instance.
 * @param control Latest FlightControl output, including valid/active flags.
 * @return READY, SAFE, or a precise argument/not-initialized/hardware error.
 */
MotorOutput_Status_t MotorOutput_Update(
    MotorOutput_Data_t *data,
    const FlightControl_Output_t *control);

#endif /* INCLUDE_MOTOR_OUTPUT_H_ */
