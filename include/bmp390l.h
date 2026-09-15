/**
 * @file bmp390l.h
 * @author Alberto Vazquez
 * @brief Instance-based BMP390 barometer interface.
 * @version 2.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_BMP390L_H_
#define INCLUDE_BMP390L_H_

#include "functions.h"

typedef enum
{
    BMP390L_STATUS_OK = 0,
    BMP390L_STATUS_NO_NEW_DATA,
    BMP390L_STATUS_INVALID_ARGUMENT,
    BMP390L_STATUS_NOT_INITIALIZED,
    BMP390L_STATUS_I2C_ARBITRATION,
    BMP390L_STATUS_I2C_ADDRESS,
    BMP390L_STATUS_I2C_DATA,
    BMP390L_STATUS_I2C_TIMEOUT,
    BMP390L_STATUS_DEVICE_NOT_FOUND,
    BMP390L_STATUS_CONFIGURATION_ERROR,
    BMP390L_STATUS_INVALID_DATA
} BMP390L_Status_t;

typedef struct
{
    float32_t par_t1;
    float32_t par_t2;
    float32_t par_t3;
    float32_t par_p1;
    float32_t par_p2;
    float32_t par_p3;
    float32_t par_p4;
    float32_t par_p5;
    float32_t par_p6;
    float32_t par_p7;
    float32_t par_p8;
    float32_t par_p9;
    float32_t par_p10;
    float32_t par_p11;
} BMP390L_Calibration_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    float32_t pressure_pa;
    float32_t pressure_hpa;
    float32_t temperature_c;
    bool fresh;
    bool valid;
} BMP390L_Sample_t;

/** One independent BMP390 instance and its latest coherent sample. */
typedef struct
{
    uint8_t address;
    uint8_t device_id;
    bool initialized;
    BMP390L_Calibration_t calibration;
    BMP390L_Sample_t sample;
    BMP390L_Status_t last_status;
    uint32_t communication_error_count;
} BMP390L_Data_t;

/** Detects, configures, and reads factory calibration data. */
BMP390L_Status_t BMP390L_Init(BMP390L_Data_t *data);

/** Publishes one pressure and temperature sample when both values are ready. */
BMP390L_Status_t BMP390L_Update(BMP390L_Data_t *data);

#endif /* INCLUDE_BMP390L_H_ */
