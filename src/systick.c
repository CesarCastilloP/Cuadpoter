/**
 * @file systick.c
 * @author Alberto Vazquez
 *
 * @brief Millisecond delays and a high-resolution monotonic time base.
 *
 * @details Beginner's reading guide:
 * - SysTick produces one interrupt per millisecond for startup delays and coarse
 *   elapsed time. The interrupt adds one to g_milliseconds.
 * - TIMER7A counts processor-clock cycles continuously. Timebase_GetCycles()
 *   extends its 32-bit hardware count to 64 bits; GetMicroseconds() converts
 *   those cycles to microseconds using the configured system frequency.
 * - The flight scheduler and sensor timestamps use TIMER7, while Delay_ms() is a
 *   blocking helper used only during initialization.
 *
 * @version 1.0.0
 * @date 2026-09-15
 */

#include "systick.h"

#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"

/* Milliseconds since SysTick_Init(); incremented only by systick_isr(). */
static volatile uint32_t g_milliseconds = 0U;
/* TIMER7 input frequency in hertz; zero means the time base is unavailable. */
static uint32_t g_timebase_clock_hz = 0U;
/* Exact cycles per microsecond, or zero when the clock is not MHz-aligned. */
static uint32_t g_timebase_cycles_per_us = 0U;
/* Previous 32-bit TIMER7 count used to detect one counter wrap. */
static uint32_t g_last_timer_value = 0U;
/* Accumulated cycles contributed by completed 2^32-count timer epochs. */
static uint64_t g_timer_wrap_cycles = 0ULL;

/**
 * @brief Configure SysTick for a one-millisecond interrupt period.
 * @param system_clock_hz Actual processor clock [Hz].
 * @return Nothing; an input below 1000 Hz leaves SysTick disabled.
 */
void SysTick_Init(uint32_t system_clock_hz)
{
    /* A clock below 1 kHz cannot represent a one-millisecond SysTick period. */
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
 * @return Elapsed time [milliseconds]; wraps after approximately 49.7 days.
 */
uint32_t SysTick_Millis(void)
{
    return g_milliseconds;
}

/**
 * @brief Perform a wrap-safe blocking delay for startup operations.
 * @param ms Requested delay [milliseconds].
 * @return Nothing; execution resumes after at least ms SysTick interrupts.
 */
void Delay_ms(uint32_t ms)
{
    /* Unsigned subtraction keeps the delay correct across 32-bit wraparound. */
    uint32_t start = g_milliseconds;

    while((uint32_t)(g_milliseconds - start) < ms)
    {
        /* The counter is advanced by the SysTick interrupt. */
    }
}

/**
 * @brief Start TIMER7 as a free-running cycle counter.
 * @param system_clock_hz Actual TIMER7 input/system clock [Hz].
 * @return true after TIMER7 and conversion state are ready; false for zero clock.
 */
bool Timebase_Init(uint32_t system_clock_hz)
{
    if(system_clock_hz == 0U)
    {
        return false;
    }

    /* TIMER7A is dedicated to monotonic timestamps and control-loop scheduling. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER7))
    {
    }

    TimerDisable(TIMER7_BASE, TIMER_A);
    /* A 32-bit up-counter gives a direct elapsed-cycle representation. */
    TimerConfigure(TIMER7_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(TIMER7_BASE, TIMER_A, 0xFFFFFFFFU);
    TimerControlStall(TIMER7_BASE, TIMER_A, true);
    TimerEnable(TIMER7_BASE, TIMER_A);

    g_timebase_clock_hz = system_clock_hz;
    g_last_timer_value = TimerValueGet(TIMER7_BASE, TIMER_A);
    g_timer_wrap_cycles = 0ULL;
    /* Cache the fast exact conversion when one microsecond is an integer count. */
    if((system_clock_hz % 1000000U) == 0U)
    {
        g_timebase_cycles_per_us = system_clock_hz / 1000000U;
    }
    return true;
}

/**
 * @brief Return elapsed processor-clock cycles since Timebase_Init().
 * @return Software-extended monotonic count [system-clock cycles], or zero before init.
 */
uint64_t Timebase_GetCycles(void)
{
    /* Raw TIMER7 count in processor-clock cycles. */
    uint32_t timer_value;

    if(g_timebase_clock_hz == 0U)
    {
        return 0ULL;
    }

    timer_value = TimerValueGet(TIMER7_BASE, TIMER_A);
    /* An up-counter value smaller than the previous sample indicates one wrap. */
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
 * @return Elapsed monotonic time [whole microseconds], or zero before init.
 */
uint64_t Timebase_GetMicroseconds(void)
{
    uint64_t cycles;    /* Monotonic elapsed processor cycles. */
    uint64_t seconds;   /* Whole elapsed seconds. */
    uint64_t remainder; /* Cycles left after removing whole seconds. */

    if(g_timebase_clock_hz == 0U)
    {
        return 0ULL;
    }

    cycles = Timebase_GetCycles();
    if(g_timebase_cycles_per_us != 0U)
    {
        return cycles / (uint64_t)g_timebase_cycles_per_us;
    }

    /* Split the conversion so remainder * 1e6 cannot overflow prematurely. */
    seconds = cycles / (uint64_t)g_timebase_clock_hz;
    remainder = cycles % (uint64_t)g_timebase_clock_hz;

    return (seconds * 1000000ULL) +
           ((remainder * 1000000ULL) / (uint64_t)g_timebase_clock_hz);
}

/**
 * @brief Advance the millisecond counter from the interrupt vector.
 * @return Nothing. This function runs in SysTick interrupt context.
 */
void systick_isr(void)
{
    g_milliseconds++;
}
