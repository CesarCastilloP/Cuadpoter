/**
 * @file mag_heading.h
 * @author Alberto Vazquez
 * @brief Tilt-compensated magnetic heading estimator for the aircraft FRD frame.
 * @version 1.0.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_MAG_HEADING_H_
#define INCLUDE_MAG_HEADING_H_

#include "lis2mdl.h"

typedef enum
{
    MAG_HEADING_STATUS_OK = 0,
    MAG_HEADING_STATUS_NO_NEW_DATA,
    MAG_HEADING_STATUS_INVALID_FIELD,
    MAG_HEADING_STATUS_STALE,
    MAG_HEADING_STATUS_INVALID_ARGUMENT,
    MAG_HEADING_STATUS_INVALID_CONFIG,
    MAG_HEADING_STATUS_NOT_INITIALIZED
} MagHeading_Status_t;

typedef struct
{
    float32_t filter_cutoff_hz;
    float32_t field_tolerance_fraction;
    float32_t minimum_horizontal_field_ut;
    uint32_t stale_timeout_us;
} MagHeading_Config_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    float32_t measured_heading_deg;
    float32_t heading_deg;
    float32_t field_magnitude_ut;
    float32_t horizontal_field_ut;
    bool fresh;
    bool valid;
} MagHeading_Output_t;

/** One independent estimator instance and its observable diagnostics. */
typedef struct
{
    bool initialized;
    MagHeading_Config_t config;
    MagHeading_Output_t output;
    MagHeading_Status_t last_status;
    uint32_t rejected_sample_count;
    uint32_t stale_count;

    /* Private runtime state. */
    float32_t estimated_heading_rad;
    uint64_t previous_sample_timestamp_us;
    uint32_t previous_sample_sequence;
    bool heading_initialized;
} MagHeading_Data_t;

/** Pass NULL for the conservative default validation and filter settings. */
MagHeading_Status_t MagHeading_Init(MagHeading_Data_t *data,
                                    const MagHeading_Config_t *config);

/**
 * Update the estimator using calibrated FRD magnetic data and current
 * roll/pitch. Calling faster than the magnetometer ODR is supported.
 */
MagHeading_Status_t MagHeading_Update(MagHeading_Data_t *data,
                                      const LIS2MDL_Data_t *magnetometer,
                                      float32_t roll_deg,
                                      float32_t pitch_deg,
                                      uint64_t now_us);

#endif /* INCLUDE_MAG_HEADING_H_ */
