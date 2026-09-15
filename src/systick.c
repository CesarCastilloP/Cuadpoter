/**
 * @file systick.c
 * @author Alberto Vazquez
 *
 * @brief Millisecond delays and a high-resolution monotonic time base.
 *
 * @version 1.0.0
 * @date 2026-09-15
 */

#include "systick.h"

#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"

static volatile uint32_t g_milliseconds = 0U;
static uint32_t g_timebase_clock_hz = 0U;
static uint32_t g_timebase_cycles_per_us = 0U;
static uint32_t g_last_timer_value = 0U;
static uint64_t g_timer_wrap_cycles = 0ULL;

/**
 * @brief Configure SysTick for a one-millisecond interrupt period.
 */
void SysTick_Init(uint32_t system_clock_hz)
{
    if(system_clock_hz < 1000U)
    {
        return;
    }

    /* SysTickPeriodSet() receives the full period and applies register bias. */
    SysTickPeriodSet(system_clock_hz / 1000U);
    SysTickIntEnable();
    SysTickEnable();
}

/**
 * @brief Return the wrapping 32-bit millisecond counter.
 */
uint32_t SysTick_Millis(void)
{
    return g_milliseconds;
}

/**
 * @brief Perform a wrap-safe blocking delay for startup operations.
 */
void Delay_ms(uint32_t ms)
{
    uint32_t start = g_milliseconds;

    while((uint32_t)(g_milliseconds - start) < ms)
    {
        /* The counter is advanced by the SysTick interrupt. */
    }
}

/**
 * @brief Start TIMER7 as a free-running cycle counter.
 */
bool Timebase_Init(uint32_t system_clock_hz)
{
    if(system_clock_hz == 0U)
    {
        return false;
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER7))
    {
    }

    TimerDisable(TIMER7_BASE, TIMER_A);
    TimerConfigure(TIMER7_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(TIMER7_BASE, TIMER_A, 0xFFFFFFFFU);
    TimerControlStall(TIMER7_BASE, TIMER_A, true);
    TimerEnable(TIMER7_BASE, TIMER_A);

    g_timebase_clock_hz = system_clock_hz;
    g_last_timer_value = TimerValueGet(TIMER7_BASE, TIMER_A);
    g_timer_wrap_cycles = 0ULL;
    if((system_clock_hz % 1000000U) == 0U)
    {
        g_timebase_cycles_per_us = system_clock_hz / 1000000U;
    }
    return true;
}

/**
 * @brief Return elapsed processor-clock cycles since Timebase_Init().
 */
uint64_t Timebase_GetCycles(void)
{
    uint32_t timer_value;

    if(g_timebase_clock_hz == 0U)
    {
        return 0ULL;
    }

    timer_value = TimerValueGet(TIMER7_BASE, TIMER_A);
    if(timer_value < g_last_timer_value)
    {
        g_timer_wrap_cycles += 0x100000000ULL;
    }
    g_last_timer_value = timer_value;

    return g_timer_wrap_cycles + (uint64_t)timer_value;
}

/**
 * @brief Convert the free-running counter to monotonic microseconds.
 *
 * Dividing before multiplying prevents overflow over the timer lifetime.
 */
uint64_t Timebase_GetMicroseconds(void)
{
    uint64_t cycles;
    uint64_t seconds;
    uint64_t remainder;

    if(g_timebase_clock_hz == 0U)
    {
        return 0ULL;
    }

    cycles = Timebase_GetCycles();
    if(g_timebase_cycles_per_us != 0U)
    {
        return cycles / (uint64_t)g_timebase_cycles_per_us;
    }

    seconds = cycles / (uint64_t)g_timebase_clock_hz;
    remainder = cycles % (uint64_t)g_timebase_clock_hz;

    return (seconds * 1000000ULL) +
           ((remainder * 1000000ULL) / (uint64_t)g_timebase_clock_hz);
}

/**
 * @brief Advance the millisecond counter from the interrupt vector.
 */
void systick_isr(void)
{
    g_milliseconds++;
}
