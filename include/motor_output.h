/**
 * @file motor_output.h
 * @author Alberto Vazquez
 * @brief Hardware-independent conversion from normalized motor commands to PWM pulses.
 * @version 1.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_MOTOR_OUTPUT_H_
#define INCLUDE_MOTOR_OUTPUT_H_

#include "flight_control.h"

typedef enum
{
    MOTOR_OUTPUT_STATUS_READY = 0,
    MOTOR_OUTPUT_STATUS_SAFE,
    MOTOR_OUTPUT_STATUS_INVALID_ARGUMENT,
    MOTOR_OUTPUT_STATUS_INVALID_CONFIG,
    MOTOR_OUTPUT_STATUS_NOT_INITIALIZED
} MotorOutput_Status_t;

typedef struct
{
    uint16_t minimum_pulse_us;
    uint16_t maximum_pulse_us;
    uint16_t safe_pulse_us;
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
    MotorOutput_Status_t last_status;
} MotorOutput_Data_t;

/** Pass NULL for 1000 us minimum/safe and 2000 us maximum. */
MotorOutput_Status_t MotorOutput_Init(
    MotorOutput_Data_t *data,
    const MotorOutput_Config_t *config);

/** Converts the latest normalized flight-control outputs into pulse widths. */
MotorOutput_Status_t MotorOutput_Update(
    MotorOutput_Data_t *data,
    const FlightControl_Output_t *control);

/**
 * Hardware integration points. They are intentionally empty in this version.
 * The PWM implementation should configure timers in HardwareInit and update
 * their compare registers in HardwareWrite. No control code must be changed.
 */
void MotorOutput_HardwareInit(const MotorOutput_Config_t *config);
void MotorOutput_HardwareWrite(const MotorOutput_Pulses_t *pulse_us,
                               bool enabled);

#endif /* INCLUDE_MOTOR_OUTPUT_H_ */
