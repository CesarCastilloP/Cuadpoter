/**
 * @file status_led.c
 * @author Alberto Vazquez
 * @brief LaunchPad initialization-state LED implementation.
 * @version 1.0.0
 * @date 2026-09-23
 */

#include "status_led.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

/* D4 on PF0 is intentionally excluded because it carries front-left PWM. */
#define STATUS_LED_D1_PIN               GPIO_PIN_1  /* PN1 */
#define STATUS_LED_D2_PIN               GPIO_PIN_0  /* PN0 */
#define STATUS_LED_D3_PIN               GPIO_PIN_4  /* PF4 */
#define STATUS_LED_PORT_N_PINS          (STATUS_LED_D1_PIN | \
                                         STATUS_LED_D2_PIN)
#define STATUS_LED_READY_WAIT_LIMIT     1000000U

static bool peripheral_ready(uint32_t peripheral)
{
    uint32_t remaining = STATUS_LED_READY_WAIT_LIMIT;

    while(!SysCtlPeripheralReady(peripheral) && (remaining > 0U))
    {
        remaining--;
    }
    return remaining > 0U;
}

StatusLed_Status_t StatusLed_Init(StatusLed_Data_t *data)
{
    if(data == NULL)
    {
        return STATUS_LED_STATUS_INVALID_ARGUMENT;
    }

    data->initialized = false;
    data->stage = STATUS_LED_STAGE_OFF;
    data->last_status = STATUS_LED_STATUS_HARDWARE_NOT_READY;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    if(!peripheral_ready(SYSCTL_PERIPH_GPION) ||
       !peripheral_ready(SYSCTL_PERIPH_GPIOF))
    {
        return data->last_status;
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, STATUS_LED_PORT_N_PINS);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, STATUS_LED_D3_PIN);
    GPIOPinWrite(GPIO_PORTN_BASE, STATUS_LED_PORT_N_PINS, 0U);
    GPIOPinWrite(GPIO_PORTF_BASE, STATUS_LED_D3_PIN, 0U);

    data->initialized = true;
    data->last_status = STATUS_LED_STATUS_OK;
    return data->last_status;
}

StatusLed_Status_t StatusLed_Set(StatusLed_Data_t *data,
                                StatusLed_Stage_t stage)
{
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

    GPIOPinWrite(GPIO_PORTN_BASE, STATUS_LED_PORT_N_PINS, port_n_value);
    GPIOPinWrite(GPIO_PORTF_BASE, STATUS_LED_D3_PIN, port_f_value);
    data->stage = stage;
    data->last_status = STATUS_LED_STATUS_OK;
    return data->last_status;
}
