/**
 * @file systick.h
 * @author Alberto Vazquez
 *
 * @brief Millisecond delays and a high-resolution monotonic time base.
 *
 * SysTick provides millisecond delays. TIMER7 provides the cycle counter used
 * by the runtime scheduler and sensor timestamps.
 * SysTick interrupts every 1 ms and increments a software millisecond counter.
 * TIMER7 is configured as a free-running 64-bit count-up timer at the processor
 * clock, normally 120 MHz; one count therefore represents about 8.333 ns.
 * main() compares absolute TIMER7 deadlines to schedule each sensor without
 * resetting the timer, and Timebase_GetMicroseconds() converts the same clock
 * into timestamps used to calculate IMU dt.
 *
 * @version 1.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_SYSTICK_H_
#define INCLUDE_SYSTICK_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Configure a 1 kHz SysTick interrupt from system_clock_hz.
 * @param system_clock_hz Actual processor clock in hertz.
 * @return Nothing; values below 1000 leave SysTick disabled.
 */
void SysTick_Init(uint32_t system_clock_hz);
/**
 * Return elapsed time since SysTick_Init.
 * @return Milliseconds since initialization, wrapping at unsigned 32-bit maximum.
 */
uint32_t SysTick_Millis(void);
/**
 * Busy-wait for the requested number of SysTick milliseconds.
 * @param ms Delay duration in milliseconds.
 * @return Nothing.
 */
void Delay_ms(uint32_t ms);

/**
 * Configure free-running TIMER7A at system_clock_hz.
 * @param system_clock_hz Actual timer input clock in hertz.
 * @return true when ready; false when the supplied clock is zero.
 */
bool Timebase_Init(uint32_t system_clock_hz);
/**
 * Return software-extended TIMER7 ticks.
 * @return Monotonic system-clock cycles, or zero before Timebase_Init.
 */
uint64_t Timebase_GetCycles(void);
/**
 * Convert the extended cycle count to monotonic integer microseconds.
 * @return Monotonic whole microseconds, or zero before Timebase_Init.
 */
uint64_t Timebase_GetMicroseconds(void);

#endif /* INCLUDE_SYSTICK_H_ */
