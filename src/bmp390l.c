/**
 * @file bmp390l.c
 * @author Alberto Vazquez
 * @brief Instance-based BMP390 acquisition and compensation.
 * @version 2.0.0
 * @date 2026-09-15
 */

#include <math.h>
#include <string.h>

#include "bmp390l.h"
#include "i2c0_drone.h"
#include "systick.h"

#define BMP390_ADDRESS_LOW          0x76U
#define BMP390_ADDRESS_HIGH         0x77U
#define BMP390_CHIP_ID              0x60U

#define REG_CHIP_ID                 0x00U
#define REG_ERROR                   0x02U
#define REG_STATUS                  0x03U
#define REG_PRESSURE_DATA           0x04U
#define REG_POWER_CONTROL           0x1BU
#define REG_OVERSAMPLING            0x1CU
#define REG_OUTPUT_DATA_RATE        0x1DU
#define REG_CONFIG                  0x1FU
#define REG_CALIBRATION             0x31U
#define DATA_READY_MASK             0x60U
#define POWER_SLEEP                 0x00U
#define POWER_PRESS_TEMP_NORMAL     0x33U
#define OVERSAMPLING_P32_T2         0x0DU
#define OUTPUT_DATA_RATE_12_5_HZ    0x04U
#define IIR_FILTER_COEFFICIENT_1    0x02U
#define SENSOR_ERROR_MASK           0x07U

#define PRESSURE_MIN_PA             30000.0f
#define PRESSURE_MAX_PA             125000.0f
#define TEMPERATURE_MIN_C           -40.0f
#define TEMPERATURE_MAX_C           85.0f

static BMP390L_Status_t status_from_i2c(I2C0_Status_t status)
{
    switch(status)
    {
        case I2C_OK:          return BMP390L_STATUS_OK;
        case I2C_ERR_ARB:     return BMP390L_STATUS_I2C_ARBITRATION;
        case I2C_ERR_ADDR:    return BMP390L_STATUS_I2C_ADDRESS;
        case I2C_ERR_DATA:    return BMP390L_STATUS_I2C_DATA;
        case I2C_ERR_TIMEOUT: return BMP390L_STATUS_I2C_TIMEOUT;
        default:              return BMP390L_STATUS_INVALID_ARGUMENT;
    }
}

static bool is_bus_error(BMP390L_Status_t status)
{
    return (status >= BMP390L_STATUS_I2C_ARBITRATION) &&
           (status <= BMP390L_STATUS_I2C_TIMEOUT);
}

static BMP390L_Status_t finish(BMP390L_Data_t *data,
                               BMP390L_Status_t status)
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

static BMP390L_Status_t read_register(uint8_t address, uint8_t reg,
                                      uint8_t *buffer, uint32_t length)
{
    return status_from_i2c(I2C0_WriteRead(address, reg, buffer, length));
}

static BMP390L_Status_t write_register(uint8_t address, uint8_t reg,
                                       uint8_t value)
{
    return status_from_i2c(I2C0_WriteWrite(address, reg, value));
}

static BMP390L_Status_t write_and_verify(const BMP390L_Data_t *data,
                                         uint8_t reg, uint8_t value,
                                         uint8_t mask)
{
    uint8_t readback;
    BMP390L_Status_t status;

    status = write_register(data->address, reg, value);
    if(status == BMP390L_STATUS_OK)
    {
        status = read_register(data->address, reg, &readback, 1U);
    }
    if((status == BMP390L_STATUS_OK) &&
       ((readback & mask) != (value & mask)))
    {
        status = BMP390L_STATUS_CONFIGURATION_ERROR;
    }
    return status;
}

static BMP390L_Status_t detect_device(BMP390L_Data_t *data)
{
    static const uint8_t addresses[] = {
        BMP390_ADDRESS_LOW, BMP390_ADDRESS_HIGH
    };
    BMP390L_Status_t status;
    uint8_t identity;
    uint32_t i;

    for(i = 0U; i < (sizeof(addresses) / sizeof(addresses[0])); i++)
    {
        status = read_register(addresses[i], REG_CHIP_ID, &identity, 1U);
        if(status == BMP390L_STATUS_OK)
        {
            data->address = addresses[i];
            data->device_id = identity;
            if(identity == BMP390_CHIP_ID)
            {
                return BMP390L_STATUS_OK;
            }
        }
        else if(status != BMP390L_STATUS_I2C_ADDRESS)
        {
            return status;
        }
    }
    return BMP390L_STATUS_DEVICE_NOT_FOUND;
}

static BMP390L_Status_t configure_device(BMP390L_Data_t *data)
{
    static const struct
    {
        uint8_t reg;
        uint8_t value;
        uint8_t mask;
    } configuration[] = {
        { REG_CONFIG,           IIR_FILTER_COEFFICIENT_1, 0x0EU },
        { REG_OUTPUT_DATA_RATE, OUTPUT_DATA_RATE_12_5_HZ, 0x1FU },
        { REG_OVERSAMPLING,     OVERSAMPLING_P32_T2,      0x3FU },
        { REG_POWER_CONTROL,    POWER_PRESS_TEMP_NORMAL,  0x33U }
    };
    BMP390L_Status_t status;
    uint8_t value;
    uint32_t i;

    /* A reset command can interrupt its own I2C transaction. Enter sleep and
     * allow any active conversion to finish before changing configuration. */
    status = write_register(data->address, REG_POWER_CONTROL, POWER_SLEEP);
    if(status != BMP390L_STATUS_OK)
    {
        return status;
    }
    Delay_ms(100U);

    status = read_register(data->address, REG_POWER_CONTROL, &value, 1U);
    if((status != BMP390L_STATUS_OK) || ((value & 0x33U) != POWER_SLEEP))
    {
        return (status == BMP390L_STATUS_OK) ?
               BMP390L_STATUS_CONFIGURATION_ERROR : status;
    }

    for(i = 0U; i < (sizeof(configuration) / sizeof(configuration[0])); i++)
    {
        status = write_and_verify(data, configuration[i].reg,
                                  configuration[i].value,
                                  configuration[i].mask);
        if(status != BMP390L_STATUS_OK)
        {
            return status;
        }
    }

    Delay_ms(5U);
    status = read_register(data->address, REG_ERROR, &value, 1U);
    if((status == BMP390L_STATUS_OK) &&
       ((value & SENSOR_ERROR_MASK) != 0U))
    {
        status = BMP390L_STATUS_CONFIGURATION_ERROR;
    }
    return status;
}

static uint16_t unsigned_16(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
}

static int16_t signed_16(const uint8_t *bytes)
{
    return (int16_t)unsigned_16(bytes);
}

static BMP390L_Status_t read_calibration(BMP390L_Data_t *data)
{
    uint8_t raw[21];
    BMP390L_Status_t status;

    status = read_register(data->address, REG_CALIBRATION,
                           raw, sizeof(raw));
    if(status != BMP390L_STATUS_OK)
    {
        return status;
    }

    data->calibration.par_t1 = (float32_t)unsigned_16(&raw[0]) * 256.0f;
    data->calibration.par_t2 = (float32_t)unsigned_16(&raw[2]) /
                               1073741824.0f;
    data->calibration.par_t3 = (float32_t)(int8_t)raw[4] /
                               281474976710656.0f;
    data->calibration.par_p1 = ((float32_t)signed_16(&raw[5]) - 16384.0f) /
                               1048576.0f;
    data->calibration.par_p2 = ((float32_t)signed_16(&raw[7]) - 16384.0f) /
                               536870912.0f;
    data->calibration.par_p3 = (float32_t)(int8_t)raw[9] /
                               4294967296.0f;
    data->calibration.par_p4 = (float32_t)(int8_t)raw[10] /
                               137438953472.0f;
    data->calibration.par_p5 = (float32_t)unsigned_16(&raw[11]) * 8.0f;
    data->calibration.par_p6 = (float32_t)unsigned_16(&raw[13]) / 64.0f;
    data->calibration.par_p7 = (float32_t)(int8_t)raw[15] / 256.0f;
    data->calibration.par_p8 = (float32_t)(int8_t)raw[16] / 32768.0f;
    data->calibration.par_p9 = (float32_t)signed_16(&raw[17]) /
                               281474976710656.0f;
    data->calibration.par_p10 = (float32_t)(int8_t)raw[19] /
                                281474976710656.0f;
    data->calibration.par_p11 = (float32_t)(int8_t)raw[20] /
                                36893488147419103232.0f;
    return BMP390L_STATUS_OK;
}

static float32_t compensate_temperature(const BMP390L_Data_t *data,
                                        uint32_t raw_temperature)
{
    float32_t delta = (float32_t)raw_temperature -
                      data->calibration.par_t1;

    return (delta * data->calibration.par_t2) +
           (delta * delta * data->calibration.par_t3);
}

static float32_t compensate_pressure(const BMP390L_Data_t *data,
                                     uint32_t raw_pressure,
                                     float32_t temperature)
{
    const BMP390L_Calibration_t *cal = &data->calibration;
    float32_t temperature_squared = temperature * temperature;
    float32_t pressure = (float32_t)raw_pressure;
    float32_t pressure_squared = pressure * pressure;
    float32_t offset;
    float32_t sensitivity;

    offset = cal->par_p5 + (cal->par_p6 * temperature) +
             (cal->par_p7 * temperature_squared) +
             (cal->par_p8 * temperature_squared * temperature);
    sensitivity = cal->par_p1 + (cal->par_p2 * temperature) +
                  (cal->par_p3 * temperature_squared) +
                  (cal->par_p4 * temperature_squared * temperature);

    return offset + (pressure * sensitivity) +
           (pressure_squared *
            (cal->par_p9 + (cal->par_p10 * temperature))) +
           (pressure_squared * pressure * cal->par_p11);
}

BMP390L_Status_t BMP390L_Init(BMP390L_Data_t *data)
{
    BMP390L_Status_t status;

    if(data == NULL)
    {
        return BMP390L_STATUS_INVALID_ARGUMENT;
    }

    memset(data, 0, sizeof(*data));
    data->last_status = BMP390L_STATUS_NOT_INITIALIZED;

    status = detect_device(data);
    if(status == BMP390L_STATUS_OK)
    {
        status = configure_device(data);
    }
    if(status == BMP390L_STATUS_OK)
    {
        status = read_calibration(data);
    }
    if(status != BMP390L_STATUS_OK)
    {
        return finish(data, status);
    }

    data->initialized = true;
    return finish(data, BMP390L_STATUS_OK);
}

BMP390L_Status_t BMP390L_Update(BMP390L_Data_t *data)
{
    uint8_t status_register;
    uint8_t raw[6];
    BMP390L_Status_t status;
    float32_t temperature;
    float32_t pressure;

    if(data == NULL)
    {
        return BMP390L_STATUS_INVALID_ARGUMENT;
    }
    data->sample.fresh = false;
    if(!data->initialized)
    {
        data->sample.valid = false;
        return finish(data, BMP390L_STATUS_NOT_INITIALIZED);
    }

    status = read_register(data->address, REG_STATUS,
                           &status_register, 1U);
    if(status != BMP390L_STATUS_OK)
    {
        data->sample.valid = false;
        return finish(data, status);
    }
    if((status_register & DATA_READY_MASK) != DATA_READY_MASK)
    {
        return finish(data, BMP390L_STATUS_NO_NEW_DATA);
    }

    status = read_register(data->address, REG_PRESSURE_DATA,
                           raw, sizeof(raw));
    if(status != BMP390L_STATUS_OK)
    {
        data->sample.valid = false;
        return finish(data, status);
    }

    data->sample.raw_pressure = (uint32_t)raw[0] |
                                ((uint32_t)raw[1] << 8U) |
                                ((uint32_t)raw[2] << 16U);
    data->sample.raw_temperature = (uint32_t)raw[3] |
                                   ((uint32_t)raw[4] << 8U) |
                                   ((uint32_t)raw[5] << 16U);

    temperature = compensate_temperature(data,
                                          data->sample.raw_temperature);
    pressure = compensate_pressure(data, data->sample.raw_pressure,
                                   temperature);
    if(!isfinite(temperature) || !isfinite(pressure) ||
       (temperature < TEMPERATURE_MIN_C) ||
       (temperature > TEMPERATURE_MAX_C) ||
       (pressure < PRESSURE_MIN_PA) || (pressure > PRESSURE_MAX_PA))
    {
        data->sample.valid = false;
        return finish(data, BMP390L_STATUS_INVALID_DATA);
    }

    data->sample.temperature_c = temperature;
    data->sample.pressure_pa = pressure;
    data->sample.pressure_hpa = pressure / 100.0f;
    data->sample.timestamp_us = Timebase_GetMicroseconds();
    data->sample.sequence++;
    data->sample.fresh = true;
    data->sample.valid = true;
    return finish(data, BMP390L_STATUS_OK);
}
