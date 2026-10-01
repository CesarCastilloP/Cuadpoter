/**
 * @file lis2mdl.h
 * @author Alberto Vazquez
 * @brief Instance-based LIS2MDL magnetometer acquisition.
 *
 * @details The sensor is configured for 50 samples/second. main() checks it at
 * 100 Hz so new data is noticed with low delay; NO_NEW_DATA on alternate checks
 * is normal. A successful update publishes raw signed counts and calibrated
 * magnetic field X/Y/Z [microtesla]. FlightControl passes the newest valid
 * sample to MagHeading for tilt-compensated yaw heading; this driver itself
 * never calculates yaw and never writes motor outputs.
 * @version 1.1.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_LIS2MDL_H_
#define INCLUDE_LIS2MDL_H_

#include "functions.h"

/** Configured magnetic output-data rate. Unit: samples per second. */
#define LIS2MDL_OUTPUT_DATA_RATE_HZ  50U

typedef enum
{
    /** One calibrated magnetic sample was published. */
    LIS2MDL_STATUS_OK = 0,
    /** Data-ready was clear; retaining the previous sample is normal. */
    LIS2MDL_STATUS_NO_NEW_DATA,
    /** API received a NULL or invalid argument. */
    LIS2MDL_STATUS_INVALID_ARGUMENT,
    /** Update was called before successful initialization. */
    LIS2MDL_STATUS_NOT_INITIALIZED,
    /** I2C master lost bus arbitration. */
    LIS2MDL_STATUS_I2C_ARBITRATION,
    /** Slave address was not acknowledged. */
    LIS2MDL_STATUS_I2C_ADDRESS,
    /** I2C data transfer failed. */
    LIS2MDL_STATUS_I2C_DATA,
    /** Bounded I2C transaction timeout expired. */
    LIS2MDL_STATUS_I2C_TIMEOUT,
    /** Address 0x1E did not return WHO_AM_I 0x40. */
    LIS2MDL_STATUS_DEVICE_NOT_FOUND,
    /** Configuration register readback did not match the requested value. */
    LIS2MDL_STATUS_CONFIGURATION_ERROR
} LIS2MDL_Status_t;

/** Signed 16-bit XYZ sample exactly as stored in LIS2MDL output registers. */
typedef struct
{
    /** Raw sensor X component. Unit: ADC counts. */
    int16_t x;
    /** Raw sensor Y component. Unit: ADC counts. */
    int16_t y;
    /** Raw sensor Z component. Unit: ADC counts. */
    int16_t z;
} LIS2MDL_RawVector3_t;

/** Three-axis floating-point magnetic vector. */
typedef struct
{
    /** X-axis magnetic component. Unit: microtesla. */
    float32_t x;
    /** Y-axis magnetic component. Unit: microtesla. */
    float32_t y;
    /** Z-axis magnetic component. Unit: microtesla. */
    float32_t z;
} LIS2MDL_Vector3f_t;

/**
 * Installation-specific magnetic calibration.
 *
 * The bias is expressed in the physical sensor frame. The matrix combines
 * soft-iron correction and the sensor-to-airframe rotation, producing the
 * right-handed aircraft FRD frame: X forward, Y right, Z down.
 */
typedef struct
{
    /** Additive hard-iron offset in physical sensor axes. Unit: microtesla. */
    LIS2MDL_Vector3f_t hard_iron_bias_sensor_ut;
    /** 3x3 soft-iron correction and sensor-to-FRD rotation. Unit: dimensionless. */
    float32_t sensor_to_body[3][3];
    /** Expected local field magnitude used by heading validation. Unit: microtesla. */
    float32_t reference_field_ut;
} LIS2MDL_Calibration_t;

/** Latest coherent raw, scaled, and calibrated magnetic measurement. */
typedef struct
{
    /** TIMER7 capture time of this sample. Unit: microseconds. */
    uint64_t timestamp_us;
    /** Monotonic count incremented for each new sample. */
    uint32_t sequence;
    /** Signed ADC result before scale or calibration. Unit: counts. */
    LIS2MDL_RawVector3_t raw;
    /** Scaled microtesla values in the physical LIS2MDL coordinate frame. */
    LIS2MDL_Vector3f_t sensor_field_ut;
    /** Hard/soft-iron corrected field in aircraft FRD coordinates. */
    LIS2MDL_Vector3f_t body_field_ut;
    /** Euclidean norm of the calibrated FRD vector. Unit: microtesla. */
    float32_t body_field_magnitude_ut;
    /** STATUS_REG snapshot used for ready/overrun diagnostics. */
    uint8_t status_register;
    /** True only on the call that publishes a new sample. */
    bool fresh;
    /** True after at least one calibrated sample has been published. */
    bool valid;
    /** True when LIS2MDL reported unread XYZ data was overwritten. */
    bool overrun;
} LIS2MDL_Sample_t;

/** One independent LIS2MDL instance and its latest coherent sample. */
typedef struct
{
    /** Fixed 7-bit LIS2MDL address, expected to be 0x1E. */
    uint8_t address;
    /** WHO_AM_I value, expected to be 0x40. */
    uint8_t device_id;
    /** True after identity and configuration readbacks pass. */
    bool initialized;
    /** Installed hard/soft-iron and orientation calibration. */
    LIS2MDL_Calibration_t calibration;
    /** Latest coherent public magnetic sample. */
    LIS2MDL_Sample_t sample;
    /** Most recent API result for CCS diagnostics. */
    LIS2MDL_Status_t last_status;
    /** Accumulated failed I2C transactions. */
    uint32_t communication_error_count;
    /** Accumulated samples for which the sensor reported XYZ overrun. */
    uint32_t data_overrun_count;
} LIS2MDL_Data_t;

/**
 * Detect and configure continuous 50 Hz high-resolution acquisition.
 * @param data Writable instance receiving identity, calibration, and runtime state.
 * @return OK or the exact argument, identity, configuration, or I2C error.
 */
LIS2MDL_Status_t LIS2MDL_Init(LIS2MDL_Data_t *data);

/**
 * Publish at most one coherent magnetic-field sample per call.
 * @param data Initialized instance receiving raw counts and body field in microteslas.
 * @return OK, NO_NEW_DATA, invalid-data, or communication status.
 */
LIS2MDL_Status_t LIS2MDL_Update(LIS2MDL_Data_t *data);

#endif /* INCLUDE_LIS2MDL_H_ */
