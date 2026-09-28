/**
 * @file lsm6ds.h
 * @author Alberto Vazquez
 * @brief Flight-oriented LSM6DSR interface.
 * @version 2.1.0
 * @date 2026-09-28
 */

#ifndef INCLUDE_LSM6DS_H_
#define INCLUDE_LSM6DS_H_

#include "functions.h"

#define LSM6DS_OUTPUT_DATA_RATE_HZ   416U

typedef enum
{
    LSM6DS_AXIS_X = 0,
    LSM6DS_AXIS_Y,
    LSM6DS_AXIS_Z
} LSM6DS_Axis_t;

typedef enum
{
    LSM6DS_STATUS_OK = 0,
    LSM6DS_STATUS_NO_NEW_DATA,
    LSM6DS_STATUS_INVALID_ARGUMENT,
    LSM6DS_STATUS_NOT_INITIALIZED,
    LSM6DS_STATUS_I2C_ARBITRATION,
    LSM6DS_STATUS_I2C_ADDRESS,
    LSM6DS_STATUS_I2C_DATA,
    LSM6DS_STATUS_I2C_TIMEOUT,
    LSM6DS_STATUS_DEVICE_NOT_FOUND,
    LSM6DS_STATUS_RESET_TIMEOUT,
    LSM6DS_STATUS_CONFIGURATION_ERROR,
    LSM6DS_STATUS_CALIBRATION_MOTION,
    LSM6DS_STATUS_CALIBRATION_TIMEOUT,
    LSM6DS_STATUS_TIMING_FAULT,
    LSM6DS_STATUS_SENSOR_SATURATED
} LSM6DS_Status_t;

typedef struct
{
    float32_t x;
    float32_t y;
    float32_t z;
} LSM6DS_Vector3f_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} LSM6DS_RawVector3_t;

/**
 * Maps sensor axes to the right-handed aircraft FRD frame:
 * X forward, Y right, Z down. Each source axis must be unique, each sign must
 * be +1 or -1, and the complete transformation must remain right-handed.
 */
typedef struct
{
    uint8_t source_axis[3];
    int8_t sign[3];
} LSM6DS_AxisMap_t;

typedef struct
{
    LSM6DS_Vector3f_t gyro_bias_rad_s;
    /** Mean stationary acceleration captured during startup calibration. */
    LSM6DS_Vector3f_t level_accel_mps2;
    bool gyro_valid;
    bool level_valid;
} LSM6DS_Calibration_t;

/**
 * Flight control should consume a sample only when fresh and valid are true.
 * gyro_rad_s contains body rates P/Q/R. accel_mps2 is body specific force;
 * a level stationary vehicle reads approximately [0, 0, -9.81] in FRD.
 */
typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    float32_t dt_s;
    LSM6DS_RawVector3_t raw_gyro;
    LSM6DS_RawVector3_t raw_accel;
    LSM6DS_Vector3f_t gyro_rad_s;
    LSM6DS_Vector3f_t accel_mps2;
    float32_t temperature_c;
    bool fresh;
    bool valid;
    bool timing_valid;
    bool saturated;
} LSM6DS_Sample_t;

/** One independent IMU instance and its latest flight-ready sample. */
typedef struct
{
    uint8_t address;
    uint8_t device_id;
    bool initialized;
    LSM6DS_AxisMap_t axis_map;
    LSM6DS_Calibration_t calibration;
    LSM6DS_Sample_t sample;
    LSM6DS_Status_t last_status;
    uint32_t communication_error_count;
    uint32_t missed_sample_count;
    uint32_t timing_fault_count;
    uint32_t saturation_count;

    /* Private runtime state. */
    uint64_t last_sample_timestamp_us;
} LSM6DS_Data_t;

/**
 * Detects, configures, and calibrates the IMU. Keep the vehicle stationary
 * and level so the acceleration mean can define the aircraft attitude zero.
 * Pass NULL for an identity sensor-to-body axis map.
 */
LSM6DS_Status_t LSM6DS_Init(LSM6DS_Data_t *data,
                            const LSM6DS_AxisMap_t *axis_map);

/**
 * Publishes at most one coherent sample. NO_NEW_DATA is a normal result when
 * polling faster than the configured 416 Hz output rate.
 */
LSM6DS_Status_t LSM6DS_Update(LSM6DS_Data_t *data);

/** Recalculates gyro bias and the level acceleration reference. */
LSM6DS_Status_t LSM6DS_CalibrateGyroscope(LSM6DS_Data_t *data);

#endif /* INCLUDE_LSM6DS_H_ */
