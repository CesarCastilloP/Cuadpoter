/**
 * @file status_led.h
 * @author Alberto Vazquez
 * @brief LaunchPad initialization-state LED interface.
 * @version 1.0.0
 * @date 2026-09-23
 */

#ifndef INCLUDE_STATUS_LED_H_
#define INCLUDE_STATUS_LED_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    STATUS_LED_STAGE_OFF = 0,
    STATUS_LED_STAGE_ESC_PREPARATION,
    STATUS_LED_STAGE_SENSOR_INITIALIZATION,
    STATUS_LED_STAGE_READY,
    STATUS_LED_STAGE_ERROR
} StatusLed_Stage_t;

typedef enum
{
    STATUS_LED_STATUS_OK = 0,
    STATUS_LED_STATUS_INVALID_ARGUMENT,
    STATUS_LED_STATUS_NOT_INITIALIZED,
    STATUS_LED_STATUS_HARDWARE_NOT_READY
} StatusLed_Status_t;

/** Runtime state for one LaunchPad status indicator instance. */
typedef struct
{
    bool initialized;
    StatusLed_Stage_t stage;
    StatusLed_Status_t last_status;
} StatusLed_Data_t;

/** Configure D1, D2, and D3 without touching motor PWM pin PF0. */
StatusLed_Status_t StatusLed_Init(StatusLed_Data_t *data);

/** Display one initialization stage on the three independent green LEDs. */
StatusLed_Status_t StatusLed_Set(StatusLed_Data_t *data,
                                StatusLed_Stage_t stage);

#endif /* INCLUDE_STATUS_LED_H_ */
