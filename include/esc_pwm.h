/**
 * @file esc_pwm.h
 * @author Alberto Vazquez
 * @brief Four-channel hardware PWM driver for standard ESC pulse commands.
 *
 * @details This is the lowest software layer that touches PWM hardware.
 * EscPwm_Init() configures M0PWM0/PF0, M0PWM2/PF2, M0PWM4/PG0, and M0PWM6/PK4.
 * The configured 6000 us frame period is 166.67 Hz. EscPwm_Write() accepts four
 * high times [us], normally 1000...2000 us, converts them to PWM clock ticks,
 * and applies every channel in one synchronized update. It does not calculate
 * PID corrections or decide whether flight is safe; MotorOutput owns that policy.
 * @version 1.1.0
 * @date 2026-09-23
 */

#ifndef INCLUDE_ESC_PWM_H_
#define INCLUDE_ESC_PWM_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    /** PWM peripheral is configured or the four writes were accepted. */
    ESC_PWM_STATUS_OK = 0,
    /** Required pointer or pulse argument was invalid. */
    ESC_PWM_STATUS_INVALID_ARGUMENT,
    /** Clock, frame, or pulse limits cannot form a legal PWM configuration. */
    ESC_PWM_STATUS_INVALID_CONFIG,
    /** A required PWM/GPIO peripheral did not become ready before the limit. */
    ESC_PWM_STATUS_HARDWARE_NOT_READY,
    /** Write was requested before EscPwm_Init completed. */
    ESC_PWM_STATUS_NOT_INITIALIZED
} EscPwm_Status_t;

/** Runtime state for one four-motor PWM output instance. */
typedef struct
{
    /** True after all four PWM generators start successfully. */
    bool initialized;
    /** TM4C system clock supplied by main. Unit: hertz. */
    uint32_t system_clock_hz;
    /** PWM module clock after the configured divider. Unit: hertz. */
    uint32_t pwm_clock_hz;
    /** Full ESC frame length loaded into each PWM generator. Unit: ticks. */
    uint32_t period_ticks;
    /** Requested ESC command frame period. Unit: microseconds. */
    uint16_t frame_period_us;
    /** Smallest accepted output pulse. Unit: microseconds. */
    uint16_t minimum_pulse_us;
    /** Largest accepted output pulse. Unit: microseconds. */
    uint16_t maximum_pulse_us;
    /** Last pulse queued on PF0/M0PWM0. Unit: microseconds. */
    uint16_t front_left_us;
    /** Last pulse queued on PF2/M0PWM2. Unit: microseconds. */
    uint16_t front_right_us;
    /** Last pulse queued on PG0/M0PWM4. Unit: microseconds. */
    uint16_t rear_right_us;
    /** Last pulse queued on PK4/M0PWM6. Unit: microseconds. */
    uint16_t rear_left_us;
    /** Number of successful synchronized four-channel updates. */
    uint32_t update_count;
    /** Most recent driver result for CCS diagnostics. */
    EscPwm_Status_t last_status;
} EscPwm_Data_t;

/**
 * Configure PWM0 and start all four channels at initial_pulse_us.
 * Physical mapping: FL=PF0, FR=PF2, RR=PG0, RL=PK4.
 * @param data Writable PWM runtime instance.
 * @param system_clock_hz TM4C system clock in hertz.
 * @param frame_period_us Repetition period in microseconds.
 * @param minimum_pulse_us Lowest legal pulse in microseconds.
 * @param maximum_pulse_us Highest legal pulse in microseconds.
 * @param initial_pulse_us Pulse applied before the first flight update, in us.
 * @return Driver status describing configuration or hardware readiness.
 */
EscPwm_Status_t EscPwm_Init(EscPwm_Data_t *data,
                            uint32_t system_clock_hz,
                            uint16_t frame_period_us,
                            uint16_t minimum_pulse_us,
                            uint16_t maximum_pulse_us,
                            uint16_t initial_pulse_us);

/**
 * Queue four pulse widths and apply them together at the next PWM boundary.
 * Every pulse argument is expressed in microseconds and is clamped to the
 * initialized minimum/maximum range before conversion to PWM ticks.
 * @param data Initialized PWM runtime instance.
 * @param front_left_us Front-left high time in microseconds.
 * @param front_right_us Front-right high time in microseconds.
 * @param rear_right_us Rear-right high time in microseconds.
 * @param rear_left_us Rear-left high time in microseconds.
 * @return ESC_PWM_STATUS_OK or an argument/not-initialized error.
 */
EscPwm_Status_t EscPwm_Write(EscPwm_Data_t *data,
                             uint16_t front_left_us,
                             uint16_t front_right_us,
                             uint16_t rear_right_us,
                             uint16_t rear_left_us);

#endif /* INCLUDE_ESC_PWM_H_ */
