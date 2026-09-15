/**
 * @file motor_output.c
 * @author Alberto Vazquez
 * @brief Safe conversion of normalized motor commands to PWM pulse widths.
 * @version 1.0.0
 * @date 2026-09-15
 */

#include <string.h>

#include "motor_output.h"

static const MotorOutput_Config_t g_default_config = {
    1000U,  /* Minimum active pulse */
    2000U,  /* Maximum active pulse */
    900U   /* Safe/disarmed pulse */
};

static float32_t clamp_normalized(float32_t value);
static uint16_t normalized_to_pulse(const MotorOutput_Data_t *data,
                                    float32_t normalized);
static void set_safe_output(MotorOutput_Data_t *data);

MotorOutput_Status_t MotorOutput_Init(
    MotorOutput_Data_t *data,
    const MotorOutput_Config_t *config)
{
    const MotorOutput_Config_t *selected_config;

    if(data == NULL)
    {
        return MOTOR_OUTPUT_STATUS_INVALID_ARGUMENT;
    }

    selected_config = (config == NULL) ? &g_default_config : config;
    if((selected_config->minimum_pulse_us >=
        selected_config->maximum_pulse_us) ||
       (selected_config->safe_pulse_us >
        selected_config->minimum_pulse_us))
    {
        return MOTOR_OUTPUT_STATUS_INVALID_CONFIG;
    }

    memset(data, 0, sizeof(*data));
    data->config = *selected_config;
    data->initialized = true;
    set_safe_output(data);
    MotorOutput_HardwareInit(&data->config);
    MotorOutput_HardwareWrite(&data->pulse_us, false);
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
        MotorOutput_HardwareWrite(&data->pulse_us, false);
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
    MotorOutput_HardwareWrite(&data->pulse_us, true);
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

/**
 * @brief PWM owner: replace this body with timer/GPIO initialization.
 */
void MotorOutput_HardwareInit(const MotorOutput_Config_t *config)
{
    (void)config;
}

/**
 * @brief PWM owner: replace this body with atomic timer compare updates.
 */
void MotorOutput_HardwareWrite(const MotorOutput_Pulses_t *pulse_us,
                               bool enabled)
{
    (void)pulse_us;
    (void)enabled;
}
