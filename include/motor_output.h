/**
 * @file motor_output.h
 * @author Alberto Vazquez
 * @brief Conversion from normalized motor commands to synchronized ESC pulses.
 * @version 1.2.0
 * @date 2026-09-23
 */

#ifndef INCLUDE_MOTOR_OUTPUT_H_
#define INCLUDE_MOTOR_OUTPUT_H_

#include "esc_pwm.h"
#include "flight_control.h"

typedef enum
{
    MOTOR_OUTPUT_STATUS_READY = 0,
    MOTOR_OUTPUT_STATUS_SAFE,
    MOTOR_OUTPUT_STATUS_INVALID_ARGUMENT,
    MOTOR_OUTPUT_STATUS_INVALID_CONFIG,
    MOTOR_OUTPUT_STATUS_NOT_INITIALIZED,
    MOTOR_OUTPUT_STATUS_HARDWARE_ERROR
} MotorOutput_Status_t;

typedef struct
{
    uint16_t minimum_pulse_us;
    uint16_t maximum_pulse_us;
    uint16_t active_idle_pulse_us;
    uint16_t safe_pulse_us;
    uint16_t frame_period_us;
} MotorOutput_Config_t;

typedef struct
{
    uint16_t front_left;
    uint16_t front_right;
    uint16_t rear_right;
    uint16_t rear_left;
} MotorOutput_Pulses_t;

/** One motor-output instance and the pulse widths ready for the PWM driver. */
typedef struct
{
    bool initialized;
    bool enabled;
    MotorOutput_Config_t config;
    MotorOutput_Pulses_t pulse_us;
    EscPwm_Data_t pwm;
    MotorOutput_Status_t last_status;
} MotorOutput_Data_t;

/**
 * Pass NULL for 1000 us safe, 1180 us active idle, 2000 us maximum,
 * and a 6000 us frame.
 */
MotorOutput_Status_t MotorOutput_Init(
    MotorOutput_Data_t *data,
    uint32_t system_clock_hz,
    const MotorOutput_Config_t *config);

/** Converts the latest normalized flight-control outputs into pulse widths. */
MotorOutput_Status_t MotorOutput_Update(
    MotorOutput_Data_t *data,
    const FlightControl_Output_t *control);

#endif /* INCLUDE_MOTOR_OUTPUT_H_ */
