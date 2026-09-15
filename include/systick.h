/**
 * @file systick.h
 * @author Alberto Vazquez
 *
 * @brief Millisecond delays and a high-resolution monotonic time base.
 *
 * SysTick provides millisecond delays. TIMER7 provides the cycle counter used
 * by the runtime scheduler and sensor timestamps.
 *
 * @version 1.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_SYSTICK_H_
#define INCLUDE_SYSTICK_H_

#include <stdint.h>
#include <stdbool.h>

void SysTick_Init(uint32_t system_clock_hz);
uint32_t SysTick_Millis(void);
void Delay_ms(uint32_t ms);

bool Timebase_Init(uint32_t system_clock_hz);
uint64_t Timebase_GetCycles(void);
uint64_t Timebase_GetMicroseconds(void);

#endif /* INCLUDE_SYSTICK_H_ */
