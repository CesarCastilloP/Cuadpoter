/**
 * @file motor_output.c
 * @author Alberto Vazquez
 * @brief Safe conversion and delivery of normalized commands to the ESC PWM driver.
 * @version 1.1.0
 * @date 2026-09-21
 */

#include <string.h>

#include "motor_output.h"

static const MotorOutput_Config_t g_default_config = {
    1000U,  /* Minimum active pulse */
    2000U,  /* Maximum active pulse */
    900U,   /* Safe/disarmed pulse */
    6000U   /* 166.67 Hz frame, matching the validated Arduino implementation */
};

static float32_t clamp_normalized(float32_t value);
static uint16_t normalized_to_pulse(const MotorOutput_Data_t *data,
                                    float32_t normalized);
static void set_safe_output(MotorOutput_Data_t *data);

MotorOutput_Status_t MotorOutput_Init(
    MotorOutput_Data_t *data,
    uint32_t system_clock_hz,
    const MotorOutput_Config_t *config)
{
    const MotorOutput_Config_t *selected_config;
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
       (selected_config->safe_pulse_us >
        selected_config->minimum_pulse_us) ||
       (selected_config->maximum_pulse_us >=
        selected_config->frame_period_us))
    {
        return MOTOR_OUTPUT_STATUS_INVALID_CONFIG;
    }

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

    data->pulse_us.front_left = normalized_to_pulse(
        data, control->motors.front_left);
    data->pulse_us.front_right = normalized_to_pulse(
        data, control->motors.front_right);
    data->pulse_us.rear_right = normalized_to_pulse(
        data, control->motors.rear_right);
    data->pulse_us.rear_left = normalized_to_pulse(
        data, control->motors.rear_left);
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

static float32_t clamp_normalized(float32_t value)
{
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

static uint16_t normalized_to_pulse(const MotorOutput_Data_t *data,
                                    float32_t normalized)
{
    float32_t pulse_span;
    float32_t pulse;

    normalized = clamp_normalized(normalized);
    pulse_span = (float32_t)(data->config.maximum_pulse_us -
                             data->config.minimum_pulse_us);
    pulse = (float32_t)data->config.minimum_pulse_us +
            (normalized * pulse_span);
    return (uint16_t)(pulse + 0.5f);
}

static void set_safe_output(MotorOutput_Data_t *data)
{
    data->pulse_us.front_left = data->config.safe_pulse_us;
    data->pulse_us.front_right = data->config.safe_pulse_us;
    data->pulse_us.rear_right = data->config.safe_pulse_us;
    data->pulse_us.rear_left = data->config.safe_pulse_us;
    data->enabled = false;
    data->last_status = MOTOR_OUTPUT_STATUS_SAFE;
}
