/**
 * @file motor_output.c
 * @author Alberto Vazquez
 * @brief Safe conversion and delivery of normalized commands to the ESC PWM driver.
 *
 * @details Beginner's reading guide:
 * - FlightControl produces four dimensionless values from 0.0 to 1.0.
 * - MotorOutput_Update() applies the configured stopped/idle rules, preserves
 *   mixer differentials when possible, converts each value to microseconds, and
 *   passes all four pulses to EscPwm_Write() in one synchronized update.
 * - A safe output is exactly safe_pulse_us on all motors. An active output lies
 *   between active_idle_pulse_us and maximum_pulse_us.
 * - This separation keeps policy in this file and timer-register details in
 *   esc_pwm.c, making future PWM-protocol replacement local to the driver.
 * @version 1.3.0
 * @date 2026-09-28
 */

#include <string.h>

#include "motor_output.h"

/* Default electrical limits shared by the mixer-to-ESC conversion. */
static const MotorOutput_Config_t g_default_config = {
    1000U,  /* minimum_pulse_us: normalized 0.0 command, in microseconds */
    2000U,  /* maximum_pulse_us: normalized 1.0 command, in microseconds */
    1180U,  /* active_idle_pulse_us: lowest spinning command, in microseconds */
    1000U,  /* safe_pulse_us: stopped/armed ESC command, in microseconds */
    6000U   /* frame_period_us: 166.67 Hz, validated by the Arduino prototype */
};

static float32_t clamp_normalized(float32_t value);
static float32_t normalized_to_pulse(const MotorOutput_Data_t *data,
                                     float32_t normalized);
static void convert_active_outputs(MotorOutput_Data_t *data,
                                   const FlightControl_MotorOutput_t *motors);
static void set_safe_output(MotorOutput_Data_t *data);

/**
 * @brief Initialize motor-output policy and its four-channel PWM driver.
 * @param data Destination instance retaining limits, status, and last pulse values.
 * @param system_clock_hz Actual MCU system clock [Hz], used to derive PWM timing.
 * @param config Optional pulse policy in microseconds; NULL selects defaults.
 * @return READY after valid 1000 us safe pulses are active, otherwise an error.
 */
MotorOutput_Status_t MotorOutput_Init(
    MotorOutput_Data_t *data,
    uint32_t system_clock_hz,
    const MotorOutput_Config_t *config)
{
    /* Configuration actually applied; NULL selects the documented defaults. */
    const MotorOutput_Config_t *selected_config;
    /* Lower-level PWM initialization result translated to this module's status. */
    EscPwm_Status_t pwm_status;

    if(data == NULL)
    {
        return MOTOR_OUTPUT_STATUS_INVALID_ARGUMENT;
    }

    selected_config = (config == NULL) ? &g_default_config : config;
    if((system_clock_hz == 0U) ||
       (selected_config->safe_pulse_us == 0U) ||
       (selected_config->minimum_pulse_us >=
        selected_config->maximum_pulse_us) ||
       (selected_config->active_idle_pulse_us <
        selected_config->minimum_pulse_us) ||
       (selected_config->active_idle_pulse_us >=
        selected_config->maximum_pulse_us) ||
       (selected_config->safe_pulse_us >
        selected_config->minimum_pulse_us) ||
       (selected_config->maximum_pulse_us >=
        selected_config->frame_period_us))
    {
        return MOTOR_OUTPUT_STATUS_INVALID_CONFIG;
    }

    /* Clear counters and intermediate values before publishing initialization. */
    memset(data, 0, sizeof(*data));
    data->config = *selected_config;
    data->initialized = true;
    set_safe_output(data);
    pwm_status = EscPwm_Init(&data->pwm,
                            system_clock_hz,
                            data->config.frame_period_us,
                            data->config.safe_pulse_us,
                            data->config.maximum_pulse_us,
                            data->config.safe_pulse_us);
    if(pwm_status != ESC_PWM_STATUS_OK)
    {
        data->initialized = false;
        data->last_status = MOTOR_OUTPUT_STATUS_HARDWARE_ERROR;
    }
    return data->last_status;
}

/**
 * @brief Convert one controller output into four synchronized ESC pulse widths.
 * @param data Initialized output instance and destination for pulse diagnostics.
 * @param control Current flight-controller output with motor commands in [0,1].
 * @return READY for active pulses, SAFE for stopped pulses, or a driver/error status.
 * @note Invalid or inactive control always calls set_safe_output().
 */
MotorOutput_Status_t MotorOutput_Update(
    MotorOutput_Data_t *data,
    const FlightControl_Output_t *control)
{
    if((data == NULL) || (control == NULL))
    {
        return MOTOR_OUTPUT_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = MOTOR_OUTPUT_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }

    /* Invalid or inactive flight control always produces four safe stop pulses. */
    if(!control->active || !control->valid)
    {
        set_safe_output(data);
        if(EscPwm_Write(&data->pwm,
                        data->pulse_us.front_left,
                        data->pulse_us.front_right,
                        data->pulse_us.rear_right,
                        data->pulse_us.rear_left) != ESC_PWM_STATUS_OK)
        {
            data->last_status = MOTOR_OUTPUT_STATUS_HARDWARE_ERROR;
        }
        return data->last_status;
    }

    /* Convert and desaturate all motors as one vector to preserve control torque. */
    convert_active_outputs(data, &control->motors);
    data->enabled = true;
    data->last_status = MOTOR_OUTPUT_STATUS_READY;
    if(EscPwm_Write(&data->pwm,
                    data->pulse_us.front_left,
                    data->pulse_us.front_right,
                    data->pulse_us.rear_right,
                    data->pulse_us.rear_left) != ESC_PWM_STATUS_OK)
    {
        data->last_status = MOTOR_OUTPUT_STATUS_HARDWARE_ERROR;
    }
    return data->last_status;
}

/**
 * @brief Restrict a dimensionless motor request to the legal [0,1] interval.
 * @param value Arbitrary normalized request.
 * @return Clamped normalized request.
 */
static float32_t clamp_normalized(float32_t value)
{
    /* Flight-control mixer output is contractually limited to [0.0, 1.0]. */
    if(value < 0.0f)
    {
        return 0.0f;
    }
    if(value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

/**
 * @brief Convert a normalized active command into ESC high time.
 * @param data Instance providing active idle and maximum pulse limits [us].
 * @param normalized Dimensionless request [0,1].
 * @return Pulse width [microseconds] as a float before final integer rounding.
 */
static float32_t normalized_to_pulse(const MotorOutput_Data_t *data,
                                     float32_t normalized)
{
    /* Difference between maximum and minimum electrical pulse widths, in us. */
    float32_t pulse_span;

    normalized = clamp_normalized(normalized);
    pulse_span = (float32_t)(data->config.maximum_pulse_us -
                             data->config.minimum_pulse_us);
    return (float32_t)data->config.minimum_pulse_us +
           (normalized * pulse_span);
}

/**
 * Moves all four active pulses together when one command falls below idle.
 * This retains the requested torque instead of clipping one motor by itself.
 * Differential commands are scaled only if the collective shift would exceed
 * the maximum pulse.
 */
/**
 * @brief Convert and collectively fit four active motor commands into pulse limits.
 * @param data Instance receiving pulse values, common shift, and applied scale.
 * @param motors Four dimensionless mixer outputs [0,1].
 * @return Nothing; data is updated but hardware is written by the caller.
 */
static void convert_active_outputs(MotorOutput_Data_t *data,
                                   const FlightControl_MotorOutput_t *motors)
{
    float32_t pulse[4]; /* FL, FR, RR, RL candidate pulses, in microseconds. */
    float32_t minimum;  /* Smallest candidate pulse, in microseconds. */
    float32_t maximum;  /* Largest candidate pulse, in microseconds. */
    float32_t range;    /* Differential pulse span maximum-minimum, in us. */
    float32_t shift;    /* Collective upward offset applied to every motor, in us. */
    float32_t scale;    /* Differential scale factor in [0,1] when saturation occurs. */
    uint32_t index;     /* Motor-array traversal index. */

    pulse[0] = normalized_to_pulse(data, motors->front_left);
    pulse[1] = normalized_to_pulse(data, motors->front_right);
    pulse[2] = normalized_to_pulse(data, motors->rear_right);
    pulse[3] = normalized_to_pulse(data, motors->rear_left);

    minimum = pulse[0];
    maximum = pulse[0];
    for(index = 1U; index < 4U; index++)
    {
        if(pulse[index] < minimum)
        {
            minimum = pulse[index];
        }
        if(pulse[index] > maximum)
        {
            maximum = pulse[index];
        }
    }

    shift = 0.0f;
    scale = 1.0f;
    /* Raise the complete vector above active idle without changing its torques. */
    if(minimum < (float32_t)data->config.active_idle_pulse_us)
    {
        shift = (float32_t)data->config.active_idle_pulse_us - minimum;
        if((maximum + shift) <=
           (float32_t)data->config.maximum_pulse_us)
        {
            for(index = 0U; index < 4U; index++)
            {
                pulse[index] += shift;
            }
        }
        else
        {
            /* If shifting would clip high, fit the differential range instead. */
            range = maximum - minimum;
            scale = (range > 0.0f) ?
                ((float32_t)(data->config.maximum_pulse_us -
                             data->config.active_idle_pulse_us) / range) :
                1.0f;
            for(index = 0U; index < 4U; index++)
            {
                pulse[index] =
                    (float32_t)data->config.active_idle_pulse_us +
                    ((pulse[index] - minimum) * scale);
            }
        }
    }

    /* Publish desaturation diagnostics, then round each pulse to integer us. */
    data->collective_shift_us = shift;
    data->active_range_scale = scale;
    data->pulse_us.front_left = (uint16_t)(pulse[0] + 0.5f);
    data->pulse_us.front_right = (uint16_t)(pulse[1] + 0.5f);
    data->pulse_us.rear_right = (uint16_t)(pulse[2] + 0.5f);
    data->pulse_us.rear_left = (uint16_t)(pulse[3] + 0.5f);
}

/**
 * @brief Command the configured stopped pulse on every ESC channel.
 * @param data Instance receiving safe pulse values and driver status.
 * @return Nothing; all four PWM compare registers are written synchronously.
 */
static void set_safe_output(MotorOutput_Data_t *data)
{
    /* A single configured stop value is intentionally written to all ESCs. */
    data->pulse_us.front_left = data->config.safe_pulse_us;
    data->pulse_us.front_right = data->config.safe_pulse_us;
    data->pulse_us.rear_right = data->config.safe_pulse_us;
    data->pulse_us.rear_left = data->config.safe_pulse_us;
    data->collective_shift_us = 0.0f;
    data->active_range_scale = 0.0f;
    data->enabled = false;
    data->last_status = MOTOR_OUTPUT_STATUS_SAFE;
}
