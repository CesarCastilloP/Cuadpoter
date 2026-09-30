/**
 * @file mag_heading.c
 * @author Alberto Vazquez
 * @brief Validated and tilt-compensated magnetic heading estimation.
 * @version 1.0.0
 * @date 2026-09-29
 */

#include <math.h>
#include <string.h>

#include "mag_heading.h"

#define DEG_TO_RAD  0.0174532925f
#define RAD_TO_DEG  57.2957795f
#define TWO_PI      (2.0f * M_PI)

static const MagHeading_Config_t g_default_config = {
    3.0f,
    0.20f,
    5.0f,
    100000U
};

static float32_t wrap_angle(float32_t angle_rad)
{
    while(angle_rad > M_PI)
    {
        angle_rad -= TWO_PI;
    }
    while(angle_rad < -M_PI)
    {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

static bool config_is_valid(const MagHeading_Config_t *config)
{
    return (config != NULL) &&
           isfinite(config->filter_cutoff_hz) &&
           (config->filter_cutoff_hz > 0.0f) &&
           isfinite(config->field_tolerance_fraction) &&
           (config->field_tolerance_fraction > 0.0f) &&
           (config->field_tolerance_fraction < 1.0f) &&
           isfinite(config->minimum_horizontal_field_ut) &&
           (config->minimum_horizontal_field_ut > 0.0f) &&
           (config->stale_timeout_us >= 20000U);
}

static MagHeading_Status_t reject_sample(MagHeading_Data_t *data)
{
    data->output.fresh = false;
    data->output.valid = false;
    if(data->rejected_sample_count < 0xFFFFFFFFU)
    {
        data->rejected_sample_count++;
    }
    data->last_status = MAG_HEADING_STATUS_INVALID_FIELD;
    return data->last_status;
}

MagHeading_Status_t MagHeading_Init(MagHeading_Data_t *data,
                                    const MagHeading_Config_t *config)
{
    const MagHeading_Config_t *selected;

    if(data == NULL)
    {
        return MAG_HEADING_STATUS_INVALID_ARGUMENT;
    }
    selected = (config == NULL) ? &g_default_config : config;
    if(!config_is_valid(selected))
    {
        return MAG_HEADING_STATUS_INVALID_CONFIG;
    }

    memset(data, 0, sizeof(*data));
    data->config = *selected;
    data->initialized = true;
    data->last_status = MAG_HEADING_STATUS_NO_NEW_DATA;
    return MAG_HEADING_STATUS_OK;
}

MagHeading_Status_t MagHeading_Update(MagHeading_Data_t *data,
                                      const LIS2MDL_Data_t *magnetometer,
                                      float32_t roll_deg,
                                      float32_t pitch_deg,
                                      uint64_t now_us)
{
    const LIS2MDL_Sample_t *sample;
    float32_t reference_field_ut;
    float32_t minimum_field_ut;
    float32_t maximum_field_ut;
    float32_t roll_rad;
    float32_t pitch_rad;
    float32_t sin_roll;
    float32_t cos_roll;
    float32_t sin_pitch;
    float32_t cos_pitch;
    float32_t horizontal_x;
    float32_t horizontal_y;
    float32_t horizontal_norm;
    float32_t measured_heading_rad;
    float32_t elapsed_s;
    float32_t filter_time_s;
    float32_t alpha;
    uint64_t age_us;

    if((data == NULL) || (magnetometer == NULL))
    {
        return MAG_HEADING_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = MAG_HEADING_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }

    sample = &magnetometer->sample;
    data->output.fresh = false;
    data->output.field_magnitude_ut = sample->body_field_magnitude_ut;

    if(sample->sequence == data->previous_sample_sequence)
    {
        age_us = (now_us >= sample->timestamp_us) ?
            now_us - sample->timestamp_us : 0ULL;
        if(data->heading_initialized && sample->valid &&
           (age_us <= (uint64_t)data->config.stale_timeout_us))
        {
            data->last_status = MAG_HEADING_STATUS_NO_NEW_DATA;
            return data->last_status;
        }
        data->output.valid = false;
        if(data->stale_count < 0xFFFFFFFFU)
        {
            data->stale_count++;
        }
        data->last_status = MAG_HEADING_STATUS_STALE;
        return data->last_status;
    }

    data->previous_sample_sequence = sample->sequence;
    if(!magnetometer->initialized || !sample->valid ||
       !isfinite(sample->body_field_ut.x) ||
       !isfinite(sample->body_field_ut.y) ||
       !isfinite(sample->body_field_ut.z) ||
       !isfinite(sample->body_field_magnitude_ut) ||
       !isfinite(roll_deg) || !isfinite(pitch_deg))
    {
        return reject_sample(data);
    }

    reference_field_ut = magnetometer->calibration.reference_field_ut;
    minimum_field_ut = reference_field_ut *
        (1.0f - data->config.field_tolerance_fraction);
    maximum_field_ut = reference_field_ut *
        (1.0f + data->config.field_tolerance_fraction);
    if(!isfinite(reference_field_ut) || (reference_field_ut <= 0.0f) ||
       (sample->body_field_magnitude_ut < minimum_field_ut) ||
       (sample->body_field_magnitude_ut > maximum_field_ut))
    {
        return reject_sample(data);
    }

    roll_rad = roll_deg * DEG_TO_RAD;
    pitch_rad = pitch_deg * DEG_TO_RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    /* Rotate FRD body field into a level frame while retaining yaw. */
    horizontal_x =
        (sample->body_field_ut.x * cos_pitch) +
        (sample->body_field_ut.y * sin_roll * sin_pitch) +
        (sample->body_field_ut.z * cos_roll * sin_pitch);
    horizontal_y =
        (sample->body_field_ut.y * cos_roll) -
        (sample->body_field_ut.z * sin_roll);
    horizontal_norm = sqrtf((horizontal_x * horizontal_x) +
                            (horizontal_y * horizontal_y));
    if(!isfinite(horizontal_norm) ||
       (horizontal_norm < data->config.minimum_horizontal_field_ut))
    {
        return reject_sample(data);
    }

    measured_heading_rad = atan2f(-horizontal_y, horizontal_x);
    if(!data->heading_initialized)
    {
        data->estimated_heading_rad = measured_heading_rad;
        data->heading_initialized = true;
    }
    else
    {
        elapsed_s = (sample->timestamp_us >
                     data->previous_sample_timestamp_us) ?
            (float32_t)(sample->timestamp_us -
                        data->previous_sample_timestamp_us) / 1000000.0f :
            (1.0f / (float32_t)LIS2MDL_OUTPUT_DATA_RATE_HZ);
        filter_time_s = 1.0f /
            (TWO_PI * data->config.filter_cutoff_hz);
        alpha = elapsed_s / (filter_time_s + elapsed_s);
        data->estimated_heading_rad = wrap_angle(
            data->estimated_heading_rad +
            (alpha * wrap_angle(measured_heading_rad -
                                data->estimated_heading_rad)));
    }

    data->previous_sample_timestamp_us = sample->timestamp_us;
    data->output.timestamp_us = sample->timestamp_us;
    data->output.sequence++;
    data->output.measured_heading_deg =
        measured_heading_rad * RAD_TO_DEG;
    data->output.heading_deg =
        data->estimated_heading_rad * RAD_TO_DEG;
    data->output.field_magnitude_ut = sample->body_field_magnitude_ut;
    data->output.horizontal_field_ut = horizontal_norm;
    data->output.fresh = true;
    data->output.valid = true;
    data->last_status = MAG_HEADING_STATUS_OK;
    return data->last_status;
}
