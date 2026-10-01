/**
 * @file esc_pwm.c
 * @author Alberto Vazquez
 * @brief Synchronized PWM0 outputs for four standard pulse-input ESCs.
 *
 * @details Engineering execution overview:
 * - Init configures PWM0 outputs PF0, PF2, PG0, and PK4 for the four motors.
 * - Pulse widths are supplied in microseconds. microseconds_to_ticks() converts
 *   them to PWM clock ticks using the configured clock and divider.
 * - Write first updates every compare register and then synchronizes the PWM
 *   generators, so the four requested motor pulses belong to the same update.
 * - This driver clamps electrical pulse limits but does not decide whether the
 *   aircraft should be active; motor_output.c owns that safety decision.
 * @version 1.1.0
 * @date 2026-09-23
 */

#include <string.h>

#include "esc_pwm.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

/* PWM0 clock divider selected to fit the 6 ms ESC frame in a 16-bit generator. */
#define ESC_PWM_CLOCK_DIVIDER           16U
/* DriverLib encoding corresponding to ESC_PWM_CLOCK_DIVIDER. */
#define ESC_PWM_CLOCK_CONFIG            PWM_SYSCLK_DIV_16
/* A TM4C PWM generator period register represents at most 65536 ticks. */
#define ESC_PWM_MAX_PERIOD_TICKS        65536ULL
/* Bounded peripheral-ready poll count used during boot. */
#define ESC_PWM_READY_WAIT_LIMIT        1000000U

/* All four generators are synchronized to prevent motor-to-motor frame skew. */
#define ESC_PWM_GENERATOR_BITS          (PWM_GEN_0_BIT | PWM_GEN_1_BIT | \
                                         PWM_GEN_2_BIT | PWM_GEN_3_BIT)
/* Even-numbered PWM0 outputs connected to the four ESC signal inputs. */
#define ESC_PWM_OUTPUT_BITS             (PWM_OUT_0_BIT | PWM_OUT_2_BIT | \
                                         PWM_OUT_4_BIT | PWM_OUT_6_BIT)
/* Down-counting, synchronized operation that continues while CCS halts the CPU. */
#define ESC_PWM_GENERATOR_CONFIG        (PWM_GEN_MODE_DOWN | \
                                         PWM_GEN_MODE_SYNC | \
                                         PWM_GEN_MODE_DBG_RUN)

/* Physical motor positions confirmed on the assembled airframe. */
#define ESC_PWM_FRONT_LEFT_OUTPUT       PWM_OUT_0  /* PF0 */
#define ESC_PWM_FRONT_RIGHT_OUTPUT      PWM_OUT_2  /* PF2 */
#define ESC_PWM_REAR_RIGHT_OUTPUT       PWM_OUT_4  /* PG0 */
#define ESC_PWM_REAR_LEFT_OUTPUT        PWM_OUT_6  /* PK4 */

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

/**
 * @brief Configure four PWM0 outputs and start them at one known pulse width.
 * @param data Destination driver instance with derived clocks and last outputs.
 * @param system_clock_hz Actual system clock [Hz].
 * @param frame_period_us Complete ESC frame period [microseconds], currently 6000.
 * @param minimum_pulse_us Smallest legal high time [microseconds].
 * @param maximum_pulse_us Largest legal high time [microseconds].
 * @param initial_pulse_us First high time placed on all outputs [microseconds].
 * @return OK after hardware is running or a precise argument/hardware error.
 */
EscPwm_Status_t EscPwm_Init(EscPwm_Data_t *data,
                            uint32_t system_clock_hz,
                            uint16_t frame_period_us,
                            uint16_t minimum_pulse_us,
                            uint16_t maximum_pulse_us,
                            uint16_t initial_pulse_us)
{
    /* Requested frame period converted to PWM clock ticks before range checking. */
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

    /* Store all timing parameters so subsequent writes need no floating point. */
    data->system_clock_hz = system_clock_hz;
    data->pwm_clock_hz = system_clock_hz / ESC_PWM_CLOCK_DIVIDER;
    data->frame_period_us = frame_period_us;
    data->minimum_pulse_us = minimum_pulse_us;
    data->maximum_pulse_us = maximum_pulse_us;

    /* Rounded conversion: ticks = pwm_clock_hz * frame_period_us / 1e6. */
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

    /* Keep ESC pins disabled until periods and safe compare values are loaded. */
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

    /* Program all generators and preload a safe equal pulse on all four motors. */
    configure_generators(data->period_ticks);
    PWMOutputInvert(PWM0_BASE, ESC_PWM_OUTPUT_BITS, false);
    write_compare_registers(data, initial_pulse_us, initial_pulse_us,
                            initial_pulse_us, initial_pulse_us);
    PWMSyncUpdate(PWM0_BASE, ESC_PWM_GENERATOR_BITS);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    /* Start every PWM time base on the same clock edge, then expose the outputs. */
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

/**
 * @brief Apply one pulse width to each motor and synchronize the update.
 * @param data Initialized driver instance.
 * @param front_left_us Front-left ESC high time [microseconds].
 * @param front_right_us Front-right ESC high time [microseconds].
 * @param rear_right_us Rear-right ESC high time [microseconds].
 * @param rear_left_us Rear-left ESC high time [microseconds].
 * @return OK or NOT_INITIALIZED/INVALID_ARGUMENT.
 */
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

    /* Enforce the configured electrical range before touching PWM registers. */
    front_left_us = clamp_pulse(data, front_left_us);
    front_right_us = clamp_pulse(data, front_right_us);
    rear_right_us = clamp_pulse(data, rear_right_us);
    rear_left_us = clamp_pulse(data, rear_left_us);

    /* Skip an unnecessary synchronous register update when no pulse changed. */
    if((front_left_us == data->front_left_us) &&
       (front_right_us == data->front_right_us) &&
       (rear_right_us == data->rear_right_us) &&
       (rear_left_us == data->rear_left_us))
    {
        data->last_status = ESC_PWM_STATUS_OK;
        return data->last_status;
    }

    /* Shadow-register updates become active together at the next PWM boundary. */
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

/** Wait a bounded number of polls for one SysCtl peripheral to become ready. */
static bool peripheral_ready(uint32_t peripheral)
{
    /* remaining bounds startup if a peripheral clock never acknowledges. */
    uint32_t remaining = ESC_PWM_READY_WAIT_LIMIT;

    while(!SysCtlPeripheralReady(peripheral) && (remaining > 0U))
    {
        remaining--;
    }
    return SysCtlPeripheralReady(peripheral);
}

/** Convert ESC high time [us] into PWM-clock ticks using data->pwm_clock_hz. */
static uint32_t microseconds_to_ticks(const EscPwm_Data_t *data,
                                      uint16_t pulse_us)
{
    /* Add 0.5 microsecond-equivalent before division for nearest-tick rounding. */
    return (uint32_t)((((uint64_t)data->pwm_clock_hz * pulse_us) +
                       500000ULL) / 1000000ULL);
}

/** Limit one requested ESC pulse [us] to the configured electrical range. */
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

/** Configure PWM generators 0..3 for count-down operation and one frame period. */
static void configure_generators(uint32_t period_ticks)
{
    /* Each motor uses output A of a separate generator with the same period. */
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, ESC_PWM_GENERATOR_CONFIG);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, ESC_PWM_GENERATOR_CONFIG);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, ESC_PWM_GENERATOR_CONFIG);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, ESC_PWM_GENERATOR_CONFIG);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period_ticks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period_ticks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period_ticks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period_ticks);
}

/**
 * @brief Convert and write all four pulse widths to their PWM compare registers.
 * @param data Driver timing and pulse-limit configuration.
 * @param front_left_us Front-left high time [us].
 * @param front_right_us Front-right high time [us].
 * @param rear_right_us Rear-right high time [us].
 * @param rear_left_us Rear-left high time [us].
 * @return Nothing; register writes take effect on the next generator boundary.
 */
static void write_compare_registers(const EscPwm_Data_t *data,
                                    uint16_t front_left_us,
                                    uint16_t front_right_us,
                                    uint16_t rear_right_us,
                                    uint16_t rear_left_us)
{
    /* DriverLib accepts high-pulse width in PWM clock ticks for each output. */
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_FRONT_LEFT_OUTPUT,
                     microseconds_to_ticks(data, front_left_us));
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_FRONT_RIGHT_OUTPUT,
                     microseconds_to_ticks(data, front_right_us));
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_REAR_RIGHT_OUTPUT,
                     microseconds_to_ticks(data, rear_right_us));
    PWMPulseWidthSet(PWM0_BASE, ESC_PWM_REAR_LEFT_OUTPUT,
                     microseconds_to_ticks(data, rear_left_us));
}
