/**
 * @file lis2mdl.c
 * @author Alberto Vazquez
 * @brief LIS2MDL identification, configuration, and magnetic-field reading.
 * @version 1.1.0
 * @date 2026-09-29
 */

#include <math.h>
#include <string.h>

#include "lis2mdl.h"
#include "i2c0_drone.h"
#include "systick.h"

#define LIS2MDL_I2C_ADDRESS          0x1EU
#define LIS2MDL_WHO_AM_I_VALUE       0x40U

#define REG_WHO_AM_I                 0x4FU
#define REG_CFG_A                    0x60U
#define REG_CFG_B                    0x61U
#define REG_CFG_C                    0x62U
#define REG_STATUS                   0x67U
#define REG_OUT_X_L                  0x68U

#define CFG_A_50_HZ_CONTINUOUS       0x88U
#define CFG_B_DEFAULT                0x00U
#define CFG_C_BLOCK_DATA_UPDATE      0x10U
#define CFG_A_VERIFY_MASK            0x9FU
#define CFG_B_VERIFY_MASK            0x1FU
#define CFG_C_VERIFY_MASK            0x7FU

#define STATUS_NEW_XYZ_MASK          0x08U
#define STATUS_XYZ_OVERRUN_MASK      0x80U
#define MAGNETIC_FIELD_UT_PER_LSB    0.15f

/*
 * Calibration obtained with the complete aircraft assembled and its motors
 * stopped. The 3x3 matrix maps bias-corrected sensor microtesla directly to
 * the aircraft FRD frame. Its orientation component is diag(-1, -1, +1).
 */
static const LIS2MDL_Calibration_t g_flight_calibration = {
    { -42.635213f, -32.064639f, -17.395767f },
    {
        { -1.06057358f, -0.00184916f, -0.01607728f },
        { -0.00184916f, -0.96908406f, -0.00021050f },
        {  0.01607728f,  0.00021050f,  0.97321318f }
    },
    36.167750f
};

static void convert_and_calibrate(LIS2MDL_Data_t *data,
                                  const LIS2MDL_RawVector3_t *raw)
{
    LIS2MDL_Vector3f_t centered;
    float32_t (*matrix)[3] = data->calibration.sensor_to_body;

    data->sample.sensor_field_ut.x =
        (float32_t)raw->x * MAGNETIC_FIELD_UT_PER_LSB;
    data->sample.sensor_field_ut.y =
        (float32_t)raw->y * MAGNETIC_FIELD_UT_PER_LSB;
    data->sample.sensor_field_ut.z =
        (float32_t)raw->z * MAGNETIC_FIELD_UT_PER_LSB;

    centered.x = data->sample.sensor_field_ut.x -
                 data->calibration.hard_iron_bias_sensor_ut.x;
    centered.y = data->sample.sensor_field_ut.y -
                 data->calibration.hard_iron_bias_sensor_ut.y;
    centered.z = data->sample.sensor_field_ut.z -
                 data->calibration.hard_iron_bias_sensor_ut.z;

    data->sample.body_field_ut.x =
        (matrix[0][0] * centered.x) +
        (matrix[0][1] * centered.y) +
        (matrix[0][2] * centered.z);
    data->sample.body_field_ut.y =
        (matrix[1][0] * centered.x) +
        (matrix[1][1] * centered.y) +
        (matrix[1][2] * centered.z);
    data->sample.body_field_ut.z =
        (matrix[2][0] * centered.x) +
        (matrix[2][1] * centered.y) +
        (matrix[2][2] * centered.z);
    data->sample.body_field_magnitude_ut = sqrtf(
        (data->sample.body_field_ut.x * data->sample.body_field_ut.x) +
        (data->sample.body_field_ut.y * data->sample.body_field_ut.y) +
        (data->sample.body_field_ut.z * data->sample.body_field_ut.z));
}

static LIS2MDL_Status_t status_from_i2c(I2C0_Status_t status)
{
    switch(status)
    {
        case I2C_OK:          return LIS2MDL_STATUS_OK;
        case I2C_ERR_ARB:     return LIS2MDL_STATUS_I2C_ARBITRATION;
        case I2C_ERR_ADDR:    return LIS2MDL_STATUS_I2C_ADDRESS;
        case I2C_ERR_DATA:    return LIS2MDL_STATUS_I2C_DATA;
        case I2C_ERR_TIMEOUT: return LIS2MDL_STATUS_I2C_TIMEOUT;
        default:              return LIS2MDL_STATUS_INVALID_ARGUMENT;
    }
}

static bool is_bus_error(LIS2MDL_Status_t status)
{
    return (status >= LIS2MDL_STATUS_I2C_ARBITRATION) &&
           (status <= LIS2MDL_STATUS_I2C_TIMEOUT);
}

static LIS2MDL_Status_t finish(LIS2MDL_Data_t *data,
                               LIS2MDL_Status_t status)
{
    if(data != NULL)
    {
        data->last_status = status;
        if(is_bus_error(status))
        {
            data->communication_error_count++;
        }
    }
    return status;
}

static LIS2MDL_Status_t read_register(uint8_t reg,
                                      uint8_t *buffer,
                                      uint32_t length)
{
    return status_from_i2c(I2C0_WriteRead(LIS2MDL_I2C_ADDRESS,
                                          reg,
                                          buffer,
                                          length));
}

static LIS2MDL_Status_t write_register(uint8_t reg, uint8_t value)
{
    return status_from_i2c(I2C0_WriteWrite(LIS2MDL_I2C_ADDRESS,
                                           reg,
                                           value));
}

static LIS2MDL_Status_t write_and_verify(uint8_t reg,
                                         uint8_t value,
                                         uint8_t mask)
{
    uint8_t readback = 0U;
    LIS2MDL_Status_t status;

    status = write_register(reg, value);
    if(status == LIS2MDL_STATUS_OK)
    {
        status = read_register(reg, &readback, 1U);
    }
    if((status == LIS2MDL_STATUS_OK) &&
       ((readback & mask) != (value & mask)))
    {
        status = LIS2MDL_STATUS_CONFIGURATION_ERROR;
    }
    return status;
}

static LIS2MDL_Status_t configure_device(void)
{
    static const struct
    {
        uint8_t reg;
        uint8_t value;
        uint8_t mask;
    } configuration[] = {
        { REG_CFG_A, CFG_A_50_HZ_CONTINUOUS,  CFG_A_VERIFY_MASK },
        { REG_CFG_B, CFG_B_DEFAULT,           CFG_B_VERIFY_MASK },
        { REG_CFG_C, CFG_C_BLOCK_DATA_UPDATE, CFG_C_VERIFY_MASK }
    };
    LIS2MDL_Status_t status = LIS2MDL_STATUS_OK;
    uint32_t i;

    for(i = 0U; i < (sizeof(configuration) / sizeof(configuration[0])); i++)
    {
        status = write_and_verify(configuration[i].reg,
                                  configuration[i].value,
                                  configuration[i].mask);
        if(status != LIS2MDL_STATUS_OK)
        {
            return status;
        }
    }

    /* One 50 Hz conversion period ensures the first status poll is coherent. */
    Delay_ms(20U);
    return LIS2MDL_STATUS_OK;
}

LIS2MDL_Status_t LIS2MDL_Init(LIS2MDL_Data_t *data)
{
    uint8_t identity = 0U;
    LIS2MDL_Status_t status;

    if(data == NULL)
    {
        return LIS2MDL_STATUS_INVALID_ARGUMENT;
    }

    memset(data, 0, sizeof(*data));
    data->address = LIS2MDL_I2C_ADDRESS;
    data->calibration = g_flight_calibration;
    data->last_status = LIS2MDL_STATUS_NOT_INITIALIZED;

    status = read_register(REG_WHO_AM_I, &identity, 1U);
    if(status == LIS2MDL_STATUS_I2C_ADDRESS)
    {
        return finish(data, LIS2MDL_STATUS_DEVICE_NOT_FOUND);
    }
    if(status != LIS2MDL_STATUS_OK)
    {
        return finish(data, status);
    }

    data->device_id = identity;
    if(identity != LIS2MDL_WHO_AM_I_VALUE)
    {
        return finish(data, LIS2MDL_STATUS_DEVICE_NOT_FOUND);
    }

    status = configure_device();
    if(status != LIS2MDL_STATUS_OK)
    {
        return finish(data, status);
    }

    data->initialized = true;
    return finish(data, LIS2MDL_STATUS_OK);
}

LIS2MDL_Status_t LIS2MDL_Update(LIS2MDL_Data_t *data)
{
    uint8_t status_register;
    uint8_t raw_bytes[6];
    LIS2MDL_RawVector3_t raw;
    LIS2MDL_Status_t status;

    if(data == NULL)
    {
        return LIS2MDL_STATUS_INVALID_ARGUMENT;
    }
    data->sample.fresh = false;
    if(!data->initialized)
    {
        data->sample.valid = false;
        return finish(data, LIS2MDL_STATUS_NOT_INITIALIZED);
    }

    status = read_register(REG_STATUS, &status_register, 1U);
    if(status != LIS2MDL_STATUS_OK)
    {
        data->sample.valid = false;
        return finish(data, status);
    }

    data->sample.status_register = status_register;
    data->sample.overrun =
        (status_register & STATUS_XYZ_OVERRUN_MASK) != 0U;
    if(data->sample.overrun && (data->data_overrun_count < 0xFFFFFFFFU))
    {
        data->data_overrun_count++;
    }
    if((status_register & STATUS_NEW_XYZ_MASK) == 0U)
    {
        return finish(data, LIS2MDL_STATUS_NO_NEW_DATA);
    }

    status = read_register(REG_OUT_X_L, raw_bytes, sizeof(raw_bytes));
    if(status != LIS2MDL_STATUS_OK)
    {
        data->sample.valid = false;
        return finish(data, status);
    }

    raw.x = (int16_t)((uint16_t)raw_bytes[0] |
                      ((uint16_t)raw_bytes[1] << 8U));
    raw.y = (int16_t)((uint16_t)raw_bytes[2] |
                      ((uint16_t)raw_bytes[3] << 8U));
    raw.z = (int16_t)((uint16_t)raw_bytes[4] |
                      ((uint16_t)raw_bytes[5] << 8U));

    data->sample.raw = raw;
    convert_and_calibrate(data, &raw);
    data->sample.timestamp_us = Timebase_GetMicroseconds();
    data->sample.sequence++;
    data->sample.valid = true;
    data->sample.fresh = true;
    return finish(data, LIS2MDL_STATUS_OK);
}
