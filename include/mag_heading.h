/**
 * @file mag_heading.h
 * @author Alberto Vazquez
 * @brief Tilt-compensated magnetic heading estimator for the aircraft FRD frame.
 *
 * @details FlightControl calls MagHeading_Update() during each fresh IMU control
 * iteration using the newest available 50 Hz LIS2MDL sample plus current roll
 * and pitch [rad]. The estimator rejects implausible/stale magnetic fields,
 * compensates aircraft tilt, wraps heading to [-180,+180) degrees, and filters
 * it. The outer yaw-heading loop then converts heading error [deg] into a yaw
 * rate setpoint [deg/s]; the inner yaw PID remains responsible for motor effort.
 * @version 1.0.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_MAG_HEADING_H_
#define INCLUDE_MAG_HEADING_H_

#include "lis2mdl.h"

typedef enum
{
    /** A valid magnetic sample updated the filtered heading. */
    MAG_HEADING_STATUS_OK = 0,
    /** No new magnetic sequence; previous non-stale heading is retained. */
    MAG_HEADING_STATUS_NO_NEW_DATA,
    /** Field magnitude or horizontal projection failed validation. */
    MAG_HEADING_STATUS_INVALID_FIELD,
    /** Last valid magnetic sample is older than stale_timeout_us. */
    MAG_HEADING_STATUS_STALE,
    /** API received NULL or non-finite arguments. */
    MAG_HEADING_STATUS_INVALID_ARGUMENT,
    /** Filter/field/timeout configuration is outside accepted ranges. */
    MAG_HEADING_STATUS_INVALID_CONFIG,
    /** Update was called before MagHeading_Init. */
    MAG_HEADING_STATUS_NOT_INITIALIZED
} MagHeading_Status_t;

/** Validation and filtering parameters for one heading estimator. */
typedef struct
{
    /** Circular first-order heading filter cutoff. Unit: hertz. */
    float32_t filter_cutoff_hz;
    /** Allowed fractional field-magnitude deviation, e.g. 0.20 = ±20%. */
    float32_t field_tolerance_fraction;
    /** Minimum tilt-compensated horizontal field. Unit: microtesla. */
    float32_t minimum_horizontal_field_ut;
    /** Maximum age of retained magnetic data. Unit: microseconds. */
    uint32_t stale_timeout_us;
} MagHeading_Config_t;

/** Public raw/filtered heading result and validity diagnostics. */
typedef struct
{
    /** Timestamp copied from the accepted LIS2MDL sample. Unit: microseconds. */
    uint64_t timestamp_us;
    /** Count of valid samples incorporated into the estimate. */
    uint32_t sequence;
    /** Current unfiltered tilt-compensated heading. Unit: degrees. */
    float32_t measured_heading_deg;
    /** Circular low-pass heading output. Unit: degrees in -180...+180. */
    float32_t heading_deg;
    /** Norm of calibrated XYZ field. Unit: microtesla. */
    float32_t field_magnitude_ut;
    /** Norm of tilt-compensated horizontal field. Unit: microtesla. */
    float32_t horizontal_field_ut;
    /** True only when this call accepted a new magnetometer sequence. */
    bool fresh;
    /** True while a validated sample remains within the stale timeout. */
    bool valid;
} MagHeading_Output_t;

/** One independent estimator instance and its observable diagnostics. */
typedef struct
{
    /** True only after successful MagHeading_Init. */
    bool initialized;
    /** Instance-specific copy of validation/filter parameters. */
    MagHeading_Config_t config;
    /** Public heading and field diagnostics. */
    MagHeading_Output_t output;
    /** Most recent estimator result for CCS. */
    MagHeading_Status_t last_status;
    /** Number of new samples rejected by field validation. */
    uint32_t rejected_sample_count;
    /** Number of transitions/calls reporting stale data. */
    uint32_t stale_count;

    /* Private runtime state. */
    /** Circular filtered heading state. Unit: radians. */
    float32_t estimated_heading_rad;
    /** Previous accepted magnetic timestamp. Unit: microseconds. */
    uint64_t previous_sample_timestamp_us;
    /** Previous LIS2MDL sequence used to detect a new sample. */
    uint32_t previous_sample_sequence;
    /** True after the first accepted sample seeds the circular filter. */
    bool heading_initialized;
} MagHeading_Data_t;

/**
 * Initialize heading validation/filter state.
 * @param data Writable estimator instance.
 * @param config Optional settings; NULL selects conservative defaults.
 * @return MAG_HEADING_STATUS_OK or an argument/configuration error.
 */
MagHeading_Status_t MagHeading_Init(MagHeading_Data_t *data,
                                    const MagHeading_Config_t *config);

/**
 * Update the estimator using calibrated FRD magnetic data and current
 * roll/pitch. Calling faster than the magnetometer ODR is supported.
 * @param data Initialized estimator instance.
 * @param magnetometer Latest LIS2MDL instance and calibrated sample.
 * @param roll_deg Current level-referenced roll angle in degrees.
 * @param pitch_deg Current level-referenced pitch angle in degrees.
 * @param now_us Current monotonic system time in microseconds.
 * @return OK, NO_NEW_DATA, INVALID_FIELD, STALE, or argument error.
 */
MagHeading_Status_t MagHeading_Update(MagHeading_Data_t *data,
                                      const LIS2MDL_Data_t *magnetometer,
                                      float32_t roll_deg,
                                      float32_t pitch_deg,
                                      uint64_t now_us);

#endif /* INCLUDE_MAG_HEADING_H_ */
