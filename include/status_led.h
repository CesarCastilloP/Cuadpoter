/**
 * @file status_led.h
 * @author Alberto Vazquez
 * @brief LaunchPad initialization-state LED interface.
 *
 * @details The LEDs communicate boot progress without requiring CCS: ESC pulse
 * preparation, sensor initialization/calibration, ready, or error. main()
 * changes stages only during startup; LED state is diagnostic and has no input
 * to sensor sampling, PID calculations, receiver validity, or motor commands.
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
    /** D1, D2, and D3 are off. */
    STATUS_LED_STAGE_OFF = 0,
    /** D1 indicates ESC low-pulse preparation. */
    STATUS_LED_STAGE_ESC_PREPARATION,
    /** D2 indicates I2C sensor setup and IMU calibration. */
    STATUS_LED_STAGE_SENSOR_INITIALIZATION,
    /** D3 remains on after all mandatory initialization succeeds. */
    STATUS_LED_STAGE_READY,
    /** D1+D2+D3 indicate a fatal initialization failure. */
    STATUS_LED_STAGE_ERROR
} StatusLed_Stage_t;

typedef enum
{
    /** GPIO initialization or stage write succeeded. */
    STATUS_LED_STATUS_OK = 0,
    /** API received NULL or an unknown stage value. */
    STATUS_LED_STATUS_INVALID_ARGUMENT,
    /** Stage write was requested before StatusLed_Init. */
    STATUS_LED_STATUS_NOT_INITIALIZED,
    /** One or more GPIO peripherals did not become ready. */
    STATUS_LED_STATUS_HARDWARE_NOT_READY
} StatusLed_Status_t;

/** Runtime state for one LaunchPad status indicator instance. */
typedef struct
{
    /** True after PN1, PN0, and PF4 are configured as outputs. */
    bool initialized;
    /** Stage currently encoded on D1/D2/D3. */
    StatusLed_Stage_t stage;
    /** Most recent API result for CCS. */
    StatusLed_Status_t last_status;
} StatusLed_Data_t;

/**
 * Configure D1, D2, and D3 without touching motor PWM pin PF0.
 * @param data Writable LED runtime state.
 * @return OK or an argument/hardware-readiness error.
 */
StatusLed_Status_t StatusLed_Init(StatusLed_Data_t *data);

/**
 * Display one initialization stage on the three independent green LEDs.
 * @param data Initialized LED runtime state.
 * @param stage Symbolic LED pattern to display.
 * @return OK or an argument/not-initialized error.
 */
StatusLed_Status_t StatusLed_Set(StatusLed_Data_t *data,
                                StatusLed_Stage_t stage);

#endif /* INCLUDE_STATUS_LED_H_ */
