/**
 * @file status_led.c
 * @author Alberto Vazquez
 * @brief LaunchPad initialization-state LED implementation.
 *
 * @details Engineering execution overview:
 * - Init configures three on-board LEDs as digital outputs.
 * - Set translates one named startup state into a visible LED combination.
 * - The LEDs communicate startup progress only; no LED participates in control.
 * - PF0/D4 is deliberately untouched because the front-left ESC PWM owns PF0.
 * @version 1.0.0
 * @date 2026-09-23
 */

#include "status_led.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

/* D4 on PF0 is intentionally excluded because it carries front-left PWM. */
#define STATUS_LED_D1_PIN               GPIO_PIN_1  /* PN1: ESC preparation. */
#define STATUS_LED_D2_PIN               GPIO_PIN_0  /* PN0: sensor initialization. */
#define STATUS_LED_D3_PIN               GPIO_PIN_4  /* PF4: flight system ready. */
/* Mask used when configuring or clearing both Port-N status LEDs together. */
#define STATUS_LED_PORT_N_PINS          (STATUS_LED_D1_PIN | \
                                         STATUS_LED_D2_PIN)
/* Maximum peripheral-ready polls; prevents a hardware fault from hanging boot. */
#define STATUS_LED_READY_WAIT_LIMIT     1000000U

/** Return true if a SysCtl peripheral becomes ready before the poll limit. */
/**
 * @brief Wait a bounded number of polls for a GPIO peripheral clock to stabilize.
 * @param peripheral DriverLib peripheral identifier such as SYSCTL_PERIPH_GPION.
 * @return true when ready; false when the bounded startup wait expires.
 */
static bool peripheral_ready(uint32_t peripheral)
{
    /* remaining is a bounded iteration counter, not a wall-clock duration. */
    uint32_t remaining = STATUS_LED_READY_WAIT_LIMIT;

    while(!SysCtlPeripheralReady(peripheral) && (remaining > 0U))
    {
        remaining--;
    }
    return remaining > 0U;
}

/**
 * @brief Configure D1, D2, and D3 as initially-off digital outputs.
 * @param data Destination state used by CCS Watch and later StatusLed_Set calls.
 * @return OK after both GPIO ports are ready, otherwise argument/hardware error.
 */
StatusLed_Status_t StatusLed_Init(StatusLed_Data_t *data)
{
    /* Reject a missing state object before accessing application memory. */
    if(data == NULL)
    {
        return STATUS_LED_STATUS_INVALID_ARGUMENT;
    }

    /* Publish a deterministic OFF/not-ready state throughout initialization. */
    data->initialized = false;
    data->stage = STATUS_LED_STAGE_OFF;
    data->last_status = STATUS_LED_STATUS_HARDWARE_NOT_READY;

    /* D1/D2 live on Port N and D3 lives on Port F. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    if(!peripheral_ready(SYSCTL_PERIPH_GPION) ||
       !peripheral_ready(SYSCTL_PERIPH_GPIOF))
    {
        return data->last_status;
    }

    /* Configure all selected pins as digital push-pull outputs, initially low. */
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, STATUS_LED_PORT_N_PINS);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, STATUS_LED_D3_PIN);
    GPIOPinWrite(GPIO_PORTN_BASE, STATUS_LED_PORT_N_PINS, 0U);
    GPIOPinWrite(GPIO_PORTF_BASE, STATUS_LED_D3_PIN, 0U);

    data->initialized = true;
    data->last_status = STATUS_LED_STATUS_OK;
    return data->last_status;
}

/**
 * @brief Display one named startup or fault stage on the three available LEDs.
 * @param data Initialized LED state receiving the last selected stage.
 * @param stage Symbolic state: ESC preparation, sensors, ready, or error.
 * @return OK or an argument/not-initialized status.
 */
StatusLed_Status_t StatusLed_Set(StatusLed_Data_t *data,
                                StatusLed_Stage_t stage)
{
    /* Values written to Port N and Port F for the requested display state. */
    uint8_t port_n_value = 0U;
    uint8_t port_f_value = 0U;

    if(data == NULL)
    {
        return STATUS_LED_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = STATUS_LED_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }
    if(stage > STATUS_LED_STAGE_ERROR)
    {
        data->last_status = STATUS_LED_STATUS_INVALID_ARGUMENT;
        return data->last_status;
    }

    /* Exactly one normal stage LED is lit; ERROR deliberately lights all three. */
    switch(stage)
    {
        case STATUS_LED_STAGE_ESC_PREPARATION:
            port_n_value = STATUS_LED_D1_PIN;
            break;

        case STATUS_LED_STAGE_SENSOR_INITIALIZATION:
            port_n_value = STATUS_LED_D2_PIN;
            break;

        case STATUS_LED_STAGE_READY:
            port_f_value = STATUS_LED_D3_PIN;
            break;

        case STATUS_LED_STAGE_ERROR:
            port_n_value = STATUS_LED_PORT_N_PINS;
            port_f_value = STATUS_LED_D3_PIN;
            break;

        case STATUS_LED_STAGE_OFF:
        default:
            break;
    }

    /* Apply both ports before publishing the new stage to the debugger. */
    GPIOPinWrite(GPIO_PORTN_BASE, STATUS_LED_PORT_N_PINS, port_n_value);
    GPIOPinWrite(GPIO_PORTF_BASE, STATUS_LED_D3_PIN, port_f_value);
    data->stage = stage;
    data->last_status = STATUS_LED_STATUS_OK;
    return data->last_status;
}
