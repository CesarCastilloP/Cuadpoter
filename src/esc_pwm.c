/**
 * @file esc_pwm.c
 * @author Alberto Vazquez
 * @brief Synchronized PWM0 outputs for four standard pulse-input ESCs.
 * @version 1.0.0
 * @date 2026-09-21
 */

#include <string.h>

#include "esc_pwm.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#define ESC_PWM_CLOCK_DIVIDER           16U
#define ESC_PWM_CLOCK_CONFIG            PWM_SYSCLK_DIV_16
#define ESC_PWM_MAX_PERIOD_TICKS        65536ULL
#define ESC_PWM_READY_WAIT_LIMIT        1000000U

#define ESC_PWM_GENERATOR_BITS          (PWM_GEN_0_BIT | PWM_GEN_1_BIT | \
                                         PWM_GEN_2_BIT | PWM_GEN_3_BIT)
#define ESC_PWM_OUTPUT_BITS             (PWM_OUT_0_BIT | PWM_OUT_2_BIT | \
                                         PWM_OUT_4_BIT | PWM_OUT_6_BIT)
#define ESC_PWM_GENERATOR_CONFIG        (PWM_GEN_MODE_DOWN | \
                                         PWM_GEN_MODE_SYNC | \
                                         PWM_GEN_MODE_DBG_RUN)

/* Motor order: front-left, front-right, rear-right, rear-left. */
#define ESC_PWM_FRONT_LEFT_OUTPUT       PWM_OUT_0
#define ESC_PWM_FRONT_RIGHT_OUTPUT      PWM_OUT_2
#define ESC_PWM_REAR_RIGHT_OUTPUT       PWM_OUT_4
#define ESC_PWM_REAR_LEFT_OUTPUT        PWM_OUT_6

static bool peripheral_ready(uint32_t peripheral);
static uint32_t microseconds_to_ticks(const EscPwm_Data_t *data,
                                      uint16_t pulse_us);
static uint16_t clamp_pulse(const EscPwm_Data_t *data, uint16_t pulse_us);
static void configure_generators(uint32_t period_ticks);
static void write_compare_registers(const EscPwm_Data_t *data,
                                    uint16_t front_left_us,
                                    uint16_t front_right_us,
                                    uint16_t rear_right_us,
                                    uint16_t rear_left_us);

EscPwm_Status_t EscPwm_Init(EscPwm_Data_t *data,
                            uint32_t system_clock_hz,
                            uint16_t frame_period_us,
                            uint16_t minimum_pulse_us,
                            uint16_t maximum_pulse_us,
                            uint16_t initial_pulse_us)
{
    uint64_t period_ticks;

    if(data == NULL)
    {
        return ESC_PWM_STATUS_INVALID_ARGUMENT;
    }

    memset(data, 0, sizeof(*data));
    data->last_status = ESC_PWM_STATUS_INVALID_CONFIG;

    if((system_clock_hz == 0U) || (frame_period_us == 0U) ||
       (minimum_pulse_us >= maximum_pulse_us) ||
       (maximum_pulse_us >= frame_period_us) ||
       (initial_pulse_us < minimum_pulse_us) ||
       (initial_pulse_us > maximum_pulse_us))
    {
        return data->last_status;
    }

    data->system_clock_hz = system_clock_hz;
    data->pwm_clock_hz = system_clock_hz / ESC_PWM_CLOCK_DIVIDER;
    data->frame_period_us = frame_period_us;
    data->minimum_pulse_us = minimum_pulse_us;
    data->maximum_pulse_us = maximum_pulse_us;

    period_ticks = (((uint64_t)data->pwm_clock_hz * frame_period_us) +
                    500000ULL) / 1000000ULL;
    if((period_ticks < 2ULL) ||
       (period_ticks > ESC_PWM_MAX_PERIOD_TICKS))
    {
        return data->last_status;
    }
    data->period_ticks = (uint32_t)period_ticks;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    if(!peripheral_ready(SYSCTL_PERIPH_GPIOF) ||
       !peripheral_ready(SYSCTL_PERIPH_GPIOG) ||
       !peripheral_ready(SYSCTL_PERIPH_GPIOK) ||
       !peripheral_ready(SYSCTL_PERIPH_PWM0))
    {
        data->last_status = ESC_PWM_STATUS_HARDWARE_NOT_READY;
        return data->last_status;
    }

    PWMOutputState(PWM0_BASE, ESC_PWM_OUTPUT_BITS, false);
    PWMClockSet(PWM0_BASE, ESC_PWM_CLOCK_CONFIG);
    if(PWMClockGet(PWM0_BASE) != ESC_PWM_CLOCK_CONFIG)
    {
        data->last_status = ESC_PWM_STATUS_HARDWARE_NOT_READY;
        return data->last_status;
    }

    /* PF0 is a protected pin on this family and must be committed first. */
    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_2);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);

    configure_generators(data->period_ticks);
    PWMOutputInvert(PWM0_BASE, ESC_PWM_OUTPUT_BITS, false);
    write_compare_registers(data, initial_pulse_us, initial_pulse_us,
                            initial_pulse_us, initial_pulse_us);
    PWMSyncUpdate(PWM0_BASE, ESC_PWM_GENERATOR_BITS);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMSyncTimeBase(PWM0_BASE, ESC_PWM_GENERATOR_BITS);
    PWMOutputState(PWM0_BASE, ESC_PWM_OUTPUT_BITS, true);

    data->front_left_us = initial_pulse_us;
    data->front_right_us = initial_pulse_us;
    data->rear_right_us = initial_pulse_us;
    data->rear_left_us = initial_pulse_us;
    data->initialized = true;
    data->last_status = ESC_PWM_STATUS_OK;
    return data->last_status;
}

EscPwm_Status_t EscPwm_Write(EscPwm_Data_t *data,
                             uint16_t front_left_us,
                             uint16_t front_right_us,
                             uint16_t rear_right_us,
                             uint16_t rear_left_us)
{
    if(data == NULL)
    {
        return ESC_PWM_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = ESC_PWM_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }

    front_left_us = clamp_pulse(data, front_left_us);
    front_right_us = clamp_pulse(data, front_right_us);
    rear_right_us = clamp_pulse(data, rear_right_us);
    rear_left_us = clamp_pulse(data, rear_left_us);

    if((front_left_us == data->front_left_us) &&
       (front_right_us == data->front_right_us) &&
       (rear_right_us == data->rear_right_us) &&
       (rear_left_us == data->rear_left_us))
    {
        data->last_status = ESC_PWM_STATUS_OK;
        return data->last_status;
    }

    write_compare_registers(data, front_left_us, front_right_us,
                            rear_right_us, rear_left_us);
    PWMSyncUpdate(PWM0_BASE, ESC_PWM_GENERATOR_BITS);

    data->front_left_us = front_left_us;
    data->front_right_us = front_right_us;
    data->rear_right_us = rear_right_us;
    data->rear_left_us = rear_left_us;
    data->update_count++;
    data->last_status = ESC_PWM_STATUS_OK;
    return data->last_status;
}

static bool peripheral_ready(uint32_t peripheral)
{
    uint32_t remaining = ESC_PWM_READY_WAIT_LIMIT;

    while(!SysCtlPeripheralReady(peripheral) && (remaining > 0U))
    {
        remaining--;
    }
    return SysCtlPeripheralReady(peripheral);
}

static uint32_t microseconds_to_ticks(const EscPwm_Data_t *data,
                                      uint16_t pulse_us)
{
    return (uint32_t)((((uint64_t)data->pwm_clock_hz * pulse_us) +
                       500000ULL) / 1000000ULL);
}

static uint16_t clamp_pulse(const EscPwm_Data_t *data, uint16_t pulse_us)
{
    if(pulse_us < data->minimum_pulse_us)
    {
        return data->minimum_pulse_us;
    }
    if(pulse_us > data->maximum_pulse_us)
    {
        return data->maximum_pulse_us;
    }
    return pulse_us;
}

static void configure_generators(uint32_t period_ticks)
{
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, ESC_PWM_GENERATOR_CONFIG);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, ESC_PWM_GENERATOR_CONFIG);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, ESC_PWM_GENERATOR_CONFIG);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, ESC_PWM_GENERATOR_CONFIG);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period_ticks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period_ticks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period_ticks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period_ticks);
}

static void write_compare_registers(const EscPwm_Data_t *data,
                                    uint16_t front_left_us,
                                    uint16_t front_right_us,
                                    uint16_t rear_right_us,
                                    uint16_t rear_left_us)
{
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_FRONT_LEFT_OUTPUT,
                     microseconds_to_ticks(data, front_left_us));
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_FRONT_RIGHT_OUTPUT,
                     microseconds_to_ticks(data, front_right_us));
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_REAR_RIGHT_OUTPUT,
                     microseconds_to_ticks(data, rear_right_us));
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_REAR_LEFT_OUTPUT,
                     microseconds_to_ticks(data, rear_left_us));
}
