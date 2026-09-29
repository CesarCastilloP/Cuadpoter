/**
 * @file lis2mdl.h
 * @author Alberto Vazquez
 * @brief Instance-based LIS2MDL magnetometer acquisition.
 * @version 1.1.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_LIS2MDL_H_
#define INCLUDE_LIS2MDL_H_

#include "functions.h"

#define LIS2MDL_OUTPUT_DATA_RATE_HZ  50U

typedef enum
{
    LIS2MDL_STATUS_OK = 0,
    LIS2MDL_STATUS_NO_NEW_DATA,
    LIS2MDL_STATUS_INVALID_ARGUMENT,
    LIS2MDL_STATUS_NOT_INITIALIZED,
    LIS2MDL_STATUS_I2C_ARBITRATION,
    LIS2MDL_STATUS_I2C_ADDRESS,
    LIS2MDL_STATUS_I2C_DATA,
    LIS2MDL_STATUS_I2C_TIMEOUT,
    LIS2MDL_STATUS_DEVICE_NOT_FOUND,
    LIS2MDL_STATUS_CONFIGURATION_ERROR
} LIS2MDL_Status_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} LIS2MDL_RawVector3_t;

typedef struct
{
    float32_t x;
    float32_t y;
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
    LIS2MDL_Vector3f_t hard_iron_bias_sensor_ut;
    float32_t sensor_to_body[3][3];
    float32_t reference_field_ut;
} LIS2MDL_Calibration_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    LIS2MDL_RawVector3_t raw;
    /** Scaled microtesla values in the physical LIS2MDL coordinate frame. */
    LIS2MDL_Vector3f_t sensor_field_ut;
    /** Hard/soft-iron corrected field in aircraft FRD coordinates. */
    LIS2MDL_Vector3f_t body_field_ut;
    float32_t body_field_magnitude_ut;
    uint8_t status_register;
    bool fresh;
    bool valid;
    bool overrun;
} LIS2MDL_Sample_t;

/** One independent LIS2MDL instance and its latest coherent sample. */
typedef struct
{
    uint8_t address;
    uint8_t device_id;
    bool initialized;
    LIS2MDL_Calibration_t calibration;
    LIS2MDL_Sample_t sample;
    LIS2MDL_Status_t last_status;
    uint32_t communication_error_count;
    uint32_t data_overrun_count;
} LIS2MDL_Data_t;

/** Detects the device and configures continuous high-resolution acquisition. */
LIS2MDL_Status_t LIS2MDL_Init(LIS2MDL_Data_t *data);

/** Publishes at most one coherent magnetic-field sample. */
LIS2MDL_Status_t LIS2MDL_Update(LIS2MDL_Data_t *data);

#endif /* INCLUDE_LIS2MDL_H_ */
