/**
 * @file lsm6ds.h
 * @author Alberto Vazquez
 * @brief Flight-oriented LSM6DSR interface.
 *
 * @details The accelerometer and gyroscope are configured for 416 samples/s.
 * main() calls LSM6DS_Update() from a 1 kHz readiness schedule; only STATUS_OK
 * indicates a new coherent six-axis frame. The driver publishes raw counts,
 * mounted/calibrated acceleration [m/s^2], bias-corrected angular rate [rad/s],
 * temperature [degrees Celsius], measured sample interval [s], timestamp [us],
 * validity flags, and diagnostic counters. Fresh valid samples form the real
 * clock for attitude estimation, PID calculation, mixing, and motor updates.
 * @version 2.1.0
 * @date 2026-09-28
 */

#ifndef INCLUDE_LSM6DS_H_
#define INCLUDE_LSM6DS_H_

#include "functions.h"

/** Configured accelerometer and gyroscope data rate. Unit: samples/second. */
#define LSM6DS_OUTPUT_DATA_RATE_HZ   416U

typedef enum
{
    /** Select source sensor X. */
    LSM6DS_AXIS_X = 0,
    /** Select source sensor Y. */
    LSM6DS_AXIS_Y,
    /** Select source sensor Z. */
    LSM6DS_AXIS_Z
} LSM6DS_Axis_t;

typedef enum
{
    /** A complete acceleration/gyro sample was published. */
    LSM6DS_STATUS_OK = 0,
    /** Combined acceleration/gyro ready bits were not both set. */
    LSM6DS_STATUS_NO_NEW_DATA,
    /** API received an invalid pointer or axis-map argument. */
    LSM6DS_STATUS_INVALID_ARGUMENT,
    /** Update/calibration was called before initialization. */
    LSM6DS_STATUS_NOT_INITIALIZED,
    /** I2C master lost arbitration. */
    LSM6DS_STATUS_I2C_ARBITRATION,
    /** Sensor address was not acknowledged. */
    LSM6DS_STATUS_I2C_ADDRESS,
    /** I2C data phase failed. */
    LSM6DS_STATUS_I2C_DATA,
    /** I2C transaction timeout expired. */
    LSM6DS_STATUS_I2C_TIMEOUT,
    /** Neither 0x6A nor 0x6B returned WHO_AM_I 0x6B. */
    LSM6DS_STATUS_DEVICE_NOT_FOUND,
    /** Software-reset bit did not clear within the allowed time. */
    LSM6DS_STATUS_RESET_TIMEOUT,
    /** Configuration register readback failed. */
    LSM6DS_STATUS_CONFIGURATION_ERROR,
    /** Calibration rejected motion or vibration beyond thresholds. */
    LSM6DS_STATUS_CALIBRATION_MOTION,
    /** Calibration could not collect its required sample count in time. */
    LSM6DS_STATUS_CALIBRATION_TIMEOUT,
    /** Measured sample interval was outside the accepted ODR window. */
    LSM6DS_STATUS_TIMING_FAULT,
    /** One or more signed raw axes approached the int16 rail. */
    LSM6DS_STATUS_SENSOR_SATURATED
} LSM6DS_Status_t;

/** Three-axis floating-point value in the mapped aircraft frame. */
typedef struct
{
    /** X component; unit is defined by the containing field. */
    float32_t x;
    /** Y component; unit is defined by the containing field. */
    float32_t y;
    /** Z component; unit is defined by the containing field. */
    float32_t z;
} LSM6DS_Vector3f_t;

/** Three signed raw register values before scaling or bias correction. */
typedef struct
{
    /** Raw X-axis ADC count. */
    int16_t x;
    /** Raw Y-axis ADC count. */
    int16_t y;
    /** Raw Z-axis ADC count. */
    int16_t z;
} LSM6DS_RawVector3_t;

/**
 * Maps sensor axes to the right-handed aircraft FRD frame:
 * X forward, Y right, Z down. Each source axis must be unique, each sign must
 * be +1 or -1, and the complete transformation must remain right-handed.
 */
typedef struct
{
    /** Sensor-axis index used for each body X/Y/Z destination. */
    uint8_t source_axis[3];
    /** Per-destination multiplier, restricted to +1 or -1. */
    int8_t sign[3];
} LSM6DS_AxisMap_t;

typedef struct
{
    /** Stationary angular-rate mean removed from every gyro sample. Unit: rad/s. */
    LSM6DS_Vector3f_t gyro_bias_rad_s;
    /** Mean stationary acceleration captured during startup calibration. */
    LSM6DS_Vector3f_t level_accel_mps2;
    /** True after gyro bias passes stationary calibration checks. */
    bool gyro_valid;
    /** True after a physical 1g level reference was captured. */
    bool level_valid;
} LSM6DS_Calibration_t;

/**
 * Flight control should consume a sample only when fresh and valid are true.
 * gyro_rad_s contains body rates P/Q/R. accel_mps2 is body specific force;
 * a level stationary vehicle reads approximately [0, 0, -9.81] in FRD.
 */
typedef struct
{
    /** TIMER7 time captured after reading the output burst. Unit: microseconds. */
    uint64_t timestamp_us;
    /** Monotonic published-sample counter. */
    uint32_t sequence;
    /** Interval since previous sample. Unit: seconds. */
    float32_t dt_s;
    /** Mapped gyroscope registers before scale/bias. Unit: counts. */
    LSM6DS_RawVector3_t raw_gyro;
    /** Mapped accelerometer registers before scale. Unit: counts. */
    LSM6DS_RawVector3_t raw_accel;
    /** Bias-corrected body rates P/Q/R. Unit: radians/second. */
    LSM6DS_Vector3f_t gyro_rad_s;
    /** Body specific force in FRD. Unit: metres/second squared. */
    LSM6DS_Vector3f_t accel_mps2;
    /** LSM6DSR internal temperature. Unit: degrees Celsius. */
    float32_t temperature_c;
    /** True only on the Update call that read this sample. */
    bool fresh;
    /** True after fields pass communication and finite-value checks. */
    bool valid;
    /** True when dt_s lies inside the accepted 416 Hz timing window. */
    bool timing_valid;
    /** True if any raw gyro/accel axis exceeds the configured rail threshold. */
    bool saturated;
} LSM6DS_Sample_t;

/** One independent IMU instance and its latest flight-ready sample. */
typedef struct
{
    /** Detected 7-bit address, either 0x6A or 0x6B. */
    uint8_t address;
    /** WHO_AM_I register, expected to equal 0x6B. */
    uint8_t device_id;
    /** True after detection, configuration, and startup calibration. */
    bool initialized;
    /** Installed sensor-to-airframe axis permutation and signs. */
    LSM6DS_AxisMap_t axis_map;
    /** Gyro bias and level acceleration measured at startup. */
    LSM6DS_Calibration_t calibration;
    /** Latest coherent physical sample. */
    LSM6DS_Sample_t sample;
    /** Most recent API result for CCS diagnostics. */
    LSM6DS_Status_t last_status;
    /** Cumulative I2C transaction errors. */
    uint32_t communication_error_count;
    /** Poll opportunities with no new combined gyro/accel sample. */
    uint32_t missed_sample_count;
    /** Samples rejected because their dt was outside limits. */
    uint32_t timing_fault_count;
    /** Samples that reached the raw rail threshold. */
    uint32_t saturation_count;

    /* Private runtime state. */
    /** Previous published timestamp used to compute dt. Unit: microseconds. */
    uint64_t last_sample_timestamp_us;
} LSM6DS_Data_t;

/**
 * Detects, configures, and calibrates the IMU. Keep the vehicle stationary
 * and level so the acceleration mean can define the aircraft attitude zero.
 * Pass NULL for an identity sensor-to-body axis map.
 * @param data Writable instance receiving device, calibration, and sample state.
 * @param axis_map Sensor-to-aircraft axis permutation and signs.
 * @return OK or the exact identity, configuration, calibration, or I2C error.
 */
LSM6DS_Status_t LSM6DS_Init(LSM6DS_Data_t *data,
                            const LSM6DS_AxisMap_t *axis_map);

/**
 * Publishes at most one coherent sample. NO_NEW_DATA is a normal result when
 * polling faster than the configured 416 Hz output rate.
 * @param data Initialized instance receiving counts, rad/s, m/s2, time, and diagnostics.
 * @return OK, NO_NEW_DATA, timing/saturation error, or I2C error.
 */
LSM6DS_Status_t LSM6DS_Update(LSM6DS_Data_t *data);

/**
 * Recalculates gyro bias and the level acceleration reference.
 * @param data Configured instance receiving bias and average gravity reference.
 * @return OK after 512 still samples or a motion/timeout/saturation/I2C error.
 */
LSM6DS_Status_t LSM6DS_CalibrateGyroscope(LSM6DS_Data_t *data);

#endif /* INCLUDE_LSM6DS_H_ */
