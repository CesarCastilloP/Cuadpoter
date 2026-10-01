/**
 * @file bmp390l.h
 * @author Alberto Vazquez
 * @brief Instance-based BMP390 barometer interface.
 *
 * @details BMP390L_Init() detects and configures the device once during boot.
 * main() then calls BMP390L_Update() every 80 ms (12.5 Hz). A successful
 * update publishes raw 24-bit codes, temperature [degrees Celsius], absolute
 * pressure [Pa and hPa], timestamp [us], validity, freshness, and sequence.
 * The current flight controller observes this sensor only for diagnostics;
 * barometer data does not change attitude, PID, mixer, or PWM output.
 * @version 2.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_BMP390L_H_
#define INCLUDE_BMP390L_H_

#include "functions.h"

typedef enum
{
    /** A coherent pressure/temperature sample was published. */
    BMP390L_STATUS_OK = 0,
    /** Sensor data-ready bits were clear; the previous sample remains valid. */
    BMP390L_STATUS_NO_NEW_DATA,
    /** API received a NULL pointer or another invalid caller argument. */
    BMP390L_STATUS_INVALID_ARGUMENT,
    /** Update was requested before initialization completed. */
    BMP390L_STATUS_NOT_INITIALIZED,
    /** I2C controller lost bus arbitration. */
    BMP390L_STATUS_I2C_ARBITRATION,
    /** I2C slave address was not acknowledged. */
    BMP390L_STATUS_I2C_ADDRESS,
    /** I2C data byte was not acknowledged or the controller reported an error. */
    BMP390L_STATUS_I2C_DATA,
    /** Bounded I2C transaction time expired. */
    BMP390L_STATUS_I2C_TIMEOUT,
    /** Neither supported BMP390 address returned the required chip ID. */
    BMP390L_STATUS_DEVICE_NOT_FOUND,
    /** A configuration register readback did not match the written value. */
    BMP390L_STATUS_CONFIGURATION_ERROR,
    /** Compensated pressure or temperature was outside physical limits. */
    BMP390L_STATUS_INVALID_DATA
} BMP390L_Status_t;

/** Factory calibration coefficients decoded from the BMP390 NVM block. */
typedef struct
{
    /** Temperature coefficient T1 after datasheet scaling. */
    float32_t par_t1;
    /** Temperature coefficient T2 after datasheet scaling. */
    float32_t par_t2;
    /** Temperature coefficient T3 after datasheet scaling. */
    float32_t par_t3;
    /** Pressure coefficient P1 after datasheet scaling. */
    float32_t par_p1;
    /** Pressure coefficient P2 after datasheet scaling. */
    float32_t par_p2;
    /** Pressure coefficient P3 after datasheet scaling. */
    float32_t par_p3;
    /** Pressure coefficient P4 after datasheet scaling. */
    float32_t par_p4;
    /** Pressure coefficient P5 after datasheet scaling. */
    float32_t par_p5;
    /** Pressure coefficient P6 after datasheet scaling. */
    float32_t par_p6;
    /** Pressure coefficient P7 after datasheet scaling. */
    float32_t par_p7;
    /** Pressure coefficient P8 after datasheet scaling. */
    float32_t par_p8;
    /** Pressure coefficient P9 after datasheet scaling. */
    float32_t par_p9;
    /** Pressure coefficient P10 after datasheet scaling. */
    float32_t par_p10;
    /** Pressure coefficient P11 after datasheet scaling. */
    float32_t par_p11;
} BMP390L_Calibration_t;

/** Latest raw and compensated barometer measurement. */
typedef struct
{
    /** Capture time from TIMER7. Unit: microseconds. */
    uint64_t timestamp_us;
    /** Monotonic count incremented for every published sample. */
    uint32_t sequence;
    /** Unsigned 24-bit pressure ADC code stored in 32 bits. Unit: counts. */
    uint32_t raw_pressure;
    /** Unsigned 24-bit temperature ADC code stored in 32 bits. Unit: counts. */
    uint32_t raw_temperature;
    /** Factory-compensated absolute pressure. Unit: pascals. */
    float32_t pressure_pa;
    /** Same absolute pressure converted for convenient observation. Unit: hPa. */
    float32_t pressure_hpa;
    /** Factory-compensated die temperature. Unit: degrees Celsius. */
    float32_t temperature_c;
    /** True only during the Update call that publishes this sample. */
    bool fresh;
    /** True after at least one finite sample inside the accepted physical range. */
    bool valid;
} BMP390L_Sample_t;

/** One independent BMP390 instance and its latest coherent sample. */
typedef struct
{
    /** Selected 7-bit I2C address: 0x76 or 0x77. */
    uint8_t address;
    /** CHIP_ID register; a BMP390 must report 0x60. */
    uint8_t device_id;
    /** True after detection, configuration, and NVM calibration decoding. */
    bool initialized;
    /** Instance-owned copy of all scaled factory coefficients. */
    BMP390L_Calibration_t calibration;
    /** Latest coherent public measurement. */
    BMP390L_Sample_t sample;
    /** Most recent API status for CCS diagnostics. */
    BMP390L_Status_t last_status;
    /** Cumulative failed I2C transaction count since reset. */
    uint32_t communication_error_count;
} BMP390L_Data_t;

/**
 * Detect, configure, and read factory calibration data.
 * @param data Writable instance that receives device and calibration state.
 * @return BMP390L_STATUS_OK on success or the exact detection/bus failure.
 */
BMP390L_Status_t BMP390L_Init(BMP390L_Data_t *data);

/**
 * Publish one pressure and temperature sample when both values are ready.
 * @param data Initialized instance to service.
 * @return OK for a new sample, NO_NEW_DATA while waiting, or an error status.
 */
BMP390L_Status_t BMP390L_Update(BMP390L_Data_t *data);

#endif /* INCLUDE_BMP390L_H_ */
