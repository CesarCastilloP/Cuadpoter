/**
 * @file esc_pwm.h
 * @author Alberto Vazquez
 * @brief Four-channel hardware PWM driver for standard ESC pulse commands.
 * @version 1.0.0
 * @date 2026-09-21
 */

#ifndef INCLUDE_ESC_PWM_H_
#define INCLUDE_ESC_PWM_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ESC_PWM_STATUS_OK = 0,
    ESC_PWM_STATUS_INVALID_ARGUMENT,
    ESC_PWM_STATUS_INVALID_CONFIG,
    ESC_PWM_STATUS_HARDWARE_NOT_READY,
    ESC_PWM_STATUS_NOT_INITIALIZED
} EscPwm_Status_t;

/** Runtime state for one four-motor PWM output instance. */
typedef struct
{
    bool initialized;
    uint32_t system_clock_hz;
    uint32_t pwm_clock_hz;
    uint32_t period_ticks;
    uint16_t frame_period_us;
    uint16_t minimum_pulse_us;
    uint16_t maximum_pulse_us;
    uint16_t front_left_us;
    uint16_t front_right_us;
    uint16_t rear_right_us;
    uint16_t rear_left_us;
    uint32_t update_count;
    EscPwm_Status_t last_status;
} EscPwm_Data_t;

/** Configure PWM0 and start all four channels at initial_pulse_us. */
EscPwm_Status_t EscPwm_Init(EscPwm_Data_t *data,
                            uint32_t system_clock_hz,
                            uint16_t frame_period_us,
                            uint16_t minimum_pulse_us,
                            uint16_t maximum_pulse_us,
                            uint16_t initial_pulse_us);

/** Queue four pulse widths and apply them together at the next PWM boundary. */
EscPwm_Status_t EscPwm_Write(EscPwm_Data_t *data,
                             uint16_t front_left_us,
                             uint16_t front_right_us,
                             uint16_t rear_right_us,
                             uint16_t rear_left_us);

#endif /* INCLUDE_ESC_PWM_H_ */
