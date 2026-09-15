/**
 * @file lsm6ds.c
 * @author Alberto Vazquez
 * @brief Instance-based LSM6DSR acquisition and calibration.
 * @version 2.0.0
 * @date 2026-09-15
 */

#include <math.h>
#include <string.h>

#include "lsm6ds.h"
#include "i2c0_drone.h"
#include "systick.h"

/* Device identity and registers. */
#define IMU_ADDR_SA0_LOW            0x6AU
#define IMU_ADDR_SA0_HIGH           0x6BU
#define REG_WHO_AM_I                0x0FU
#define WHO_AM_I_VALUE              0x6BU
#define REG_CTRL1_XL                0x10U
#define REG_CTRL2_G                 0x11U
#define REG_CTRL3_C                 0x12U
#define REG_CTRL4_C                 0x13U
#define REG_CTRL6_C                 0x15U
#define REG_CTRL8_XL                0x17U
#define REG_CTRL9_XL                0x18U
#define REG_STATUS                  0x1EU
#define REG_OUT_TEMP_L              0x20U

/* 416 Hz, +/-8 g, +/-2000 dps, BDU, auto-increment, and low-pass filters. */
#define VALUE_CTRL1_XL              0x6EU
#define VALUE_CTRL2_G               0x6CU
#define VALUE_CTRL3_C               0x44U
#define VALUE_CTRL4_C               0x0AU
#define VALUE_CTRL6_C               0x04U
#define VALUE_CTRL8_XL              0x00U
#define CTRL9_I3C_DISABLE           0x02U
#define SOFTWARE_RESET              0x01U
#define DATA_READY_MASK             0x03U

/* Selected full-scale conversion factors. */
#define GYRO_RAD_S_PER_LSB          0.0012217304764f
#define ACCEL_MPS2_PER_LSB          0.0023928226f
#define STANDARD_GRAVITY_MPS2       9.80665f
#define NOMINAL_DT_S                (1.0f / 416.0f)
#define NOMINAL_PERIOD_US           2404ULL

/* Sample health limits. */
#define MIN_VALID_DT_S              0.0005f
#define MAX_VALID_DT_S              0.00421f
#define SATURATION_RAW              32000

/* Startup and stationary calibration limits. */
#define POWER_ON_DELAY_MS           35U
#define FILTER_SETTLE_MS            250U
#define RESET_TIMEOUT_MS            100U
#define CALIBRATION_TIMEOUT_MS      5000U
#define CALIBRATION_SAMPLES         512U
#define CAL_GYRO_MEAN_MAX           0.02617994f
#define CAL_GYRO_SAMPLE_MAX         0.05235988f
#define CAL_GYRO_STD_MAX            0.00872665f
#define CAL_ACCEL_STD_MAX           0.14710f
#define CAL_ACCEL_MEAN_MIN          (0.85f * STANDARD_GRAVITY_MPS2)
#define CAL_ACCEL_MEAN_MAX          (1.15f * STANDARD_GRAVITY_MPS2)

typedef struct
{
    LSM6DS_RawVector3_t gyro;
    LSM6DS_RawVector3_t accel;
    int16_t temperature;
    uint64_t timestamp_us;
} LSM6DS_Frame_t;

typedef struct
{
    uint8_t reg;
    uint8_t value;
} LSM6DS_RegisterValue_t;

static float32_t vector_dot(const LSM6DS_Vector3f_t *a,
                            const LSM6DS_Vector3f_t *b)
{
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

static float32_t vector_norm(const LSM6DS_Vector3f_t *value)
{
    return sqrtf(vector_dot(value, value));
}

static LSM6DS_Status_t status_from_i2c(I2C0_Status_t status)
{
    switch(status)
    {
        case I2C_OK:          return LSM6DS_STATUS_OK;
        case I2C_ERR_ARB:     return LSM6DS_STATUS_I2C_ARBITRATION;
        case I2C_ERR_ADDR:    return LSM6DS_STATUS_I2C_ADDRESS;
        case I2C_ERR_DATA:    return LSM6DS_STATUS_I2C_DATA;
        case I2C_ERR_TIMEOUT: return LSM6DS_STATUS_I2C_TIMEOUT;
        default:              return LSM6DS_STATUS_INVALID_ARGUMENT;
    }
}

static bool is_bus_error(LSM6DS_Status_t status)
{
    return (status >= LSM6DS_STATUS_I2C_ARBITRATION) &&
           (status <= LSM6DS_STATUS_I2C_TIMEOUT);
}

static LSM6DS_Status_t finish(LSM6DS_Data_t *data, LSM6DS_Status_t status)
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

static LSM6DS_Status_t read_reg(uint8_t address, uint8_t reg,
                                uint8_t *buffer, uint32_t length)
{
    return status_from_i2c(I2C0_WriteRead(address, reg, buffer, length));
}

static LSM6DS_Status_t write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    return status_from_i2c(I2C0_WriteWrite(address, reg, value));
}

static int16_t make_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

static bool axis_map_valid(const LSM6DS_AxisMap_t *map)
{
    uint32_t i;
    uint32_t j;
    int32_t inversions = 0;
    int32_t determinant;

    if(map == NULL)
    {
        return false;
    }

    for(i = 0U; i < 3U; i++)
    {
        if((map->source_axis[i] > (uint8_t)LSM6DS_AXIS_Z) ||
           ((map->sign[i] != 1) && (map->sign[i] != -1)))
        {
            return false;
        }
        for(j = i + 1U; j < 3U; j++)
        {
            if(map->source_axis[i] == map->source_axis[j])
            {
                return false;
            }
            if(map->source_axis[i] > map->source_axis[j])
            {
                inversions++;
            }
        }
    }

    determinant = ((inversions & 1) == 0) ? 1 : -1;
    determinant *= (int32_t)map->sign[0] *
                   (int32_t)map->sign[1] *
                   (int32_t)map->sign[2];
    return determinant == 1;
}

static void map_and_scale(const LSM6DS_Data_t *data,
                          const LSM6DS_RawVector3_t *raw,
                          float32_t scale,
                          LSM6DS_Vector3f_t *body)
{
    float32_t sensor[3];
    float32_t mapped[3];
    uint32_t axis;

    sensor[0] = (float32_t)raw->x * scale;
    sensor[1] = (float32_t)raw->y * scale;
    sensor[2] = (float32_t)raw->z * scale;

    for(axis = 0U; axis < 3U; axis++)
    {
        mapped[axis] = (float32_t)data->axis_map.sign[axis] *
                       sensor[data->axis_map.source_axis[axis]];
    }
    body->x = mapped[0];
    body->y = mapped[1];
    body->z = mapped[2];
}

/* STATUS is checked first; the burst is accepted only when gyro and accel are new. */
static LSM6DS_Status_t read_fresh_frame(LSM6DS_Data_t *data,
                                        LSM6DS_Frame_t *frame)
{
    uint8_t status_reg;
    uint8_t bytes[14];
    LSM6DS_Status_t status;

    status = read_reg(data->address, REG_STATUS, &status_reg, 1U);
    if(status != LSM6DS_STATUS_OK)
    {
        return status;
    }
    if((status_reg & DATA_READY_MASK) != DATA_READY_MASK)
    {
        return LSM6DS_STATUS_NO_NEW_DATA;
    }

    status = read_reg(data->address, REG_OUT_TEMP_L, bytes, sizeof(bytes));
    if(status != LSM6DS_STATUS_OK)
    {
        return status;
    }

    frame->temperature = make_int16(bytes[0], bytes[1]);
    frame->gyro.x = make_int16(bytes[2], bytes[3]);
    frame->gyro.y = make_int16(bytes[4], bytes[5]);
    frame->gyro.z = make_int16(bytes[6], bytes[7]);
    frame->accel.x = make_int16(bytes[8], bytes[9]);
    frame->accel.y = make_int16(bytes[10], bytes[11]);
    frame->accel.z = make_int16(bytes[12], bytes[13]);
    frame->timestamp_us = Timebase_GetMicroseconds();
    return LSM6DS_STATUS_OK;
}

static LSM6DS_Status_t detect_device(LSM6DS_Data_t *data)
{
    static const uint8_t addresses[2] = {
        IMU_ADDR_SA0_LOW, IMU_ADDR_SA0_HIGH
    };
    LSM6DS_Status_t transport_error = LSM6DS_STATUS_OK;
    LSM6DS_Status_t status;
    uint8_t identity;
    uint32_t i;

    for(i = 0U; i < 2U; i++)
    {
        status = read_reg(addresses[i], REG_WHO_AM_I, &identity, 1U);
        if(status == LSM6DS_STATUS_OK)
        {
            data->address = addresses[i];
            data->device_id = identity;
            if(identity == WHO_AM_I_VALUE)
            {
                return LSM6DS_STATUS_OK;
            }
        }
        if((status != LSM6DS_STATUS_OK) &&
           (status != LSM6DS_STATUS_I2C_ADDRESS))
        {
            transport_error = status;
        }
    }

    return (transport_error == LSM6DS_STATUS_OK) ?
           LSM6DS_STATUS_DEVICE_NOT_FOUND : transport_error;
}

static LSM6DS_Status_t write_and_verify(LSM6DS_Data_t *data,
                                        uint8_t reg, uint8_t value,
                                        uint8_t mask)
{
    uint8_t readback;
    LSM6DS_Status_t status = write_reg(data->address, reg, value);

    if(status == LSM6DS_STATUS_OK)
    {
        status = read_reg(data->address, reg, &readback, 1U);
    }
    if((status == LSM6DS_STATUS_OK) &&
       ((readback & mask) != (value & mask)))
    {
        status = LSM6DS_STATUS_CONFIGURATION_ERROR;
    }
    return status;
}

static LSM6DS_Status_t configure_device(LSM6DS_Data_t *data)
{
    static const LSM6DS_RegisterValue_t configuration[] = {
        { REG_CTRL3_C,  VALUE_CTRL3_C  },
        { REG_CTRL4_C,  VALUE_CTRL4_C  },
        { REG_CTRL6_C,  VALUE_CTRL6_C  },
        { REG_CTRL8_XL, VALUE_CTRL8_XL },
        { REG_CTRL1_XL, VALUE_CTRL1_XL },
        { REG_CTRL2_G,  VALUE_CTRL2_G  }
    };
    LSM6DS_Status_t status;
    uint8_t value;
    uint32_t start_ms;
    uint32_t i;

    status = write_reg(data->address, REG_CTRL3_C, SOFTWARE_RESET);
    if(status != LSM6DS_STATUS_OK)
    {
        return status;
    }

    start_ms = SysTick_Millis();
    do
    {
        status = read_reg(data->address, REG_CTRL3_C, &value, 1U);
        if(status != LSM6DS_STATUS_OK)
        {
            return status;
        }
        if((value & SOFTWARE_RESET) == 0U)
        {
            break;
        }
        if((uint32_t)(SysTick_Millis() - start_ms) >= RESET_TIMEOUT_MS)
        {
            return LSM6DS_STATUS_RESET_TIMEOUT;
        }
        Delay_ms(1U);
    }
    while(true);

    status = read_reg(data->address, REG_CTRL9_XL, &value, 1U);
    if(status != LSM6DS_STATUS_OK)
    {
        return status;
    }
    status = write_and_verify(data, REG_CTRL9_XL,
                              value | CTRL9_I3C_DISABLE,
                              CTRL9_I3C_DISABLE);
    if(status != LSM6DS_STATUS_OK)
    {
        return status;
    }

    for(i = 0U; i < (sizeof(configuration) / sizeof(configuration[0])); i++)
    {
        status = write_and_verify(data, configuration[i].reg,
                                  configuration[i].value, 0xFFU);
        if(status != LSM6DS_STATUS_OK)
        {
            return status;
        }
    }

    Delay_ms(FILTER_SETTLE_MS);
    return LSM6DS_STATUS_OK;
}

static bool raw_saturated(const LSM6DS_RawVector3_t *raw)
{
    return (raw->x > SATURATION_RAW) || (raw->x < -SATURATION_RAW) ||
           (raw->y > SATURATION_RAW) || (raw->y < -SATURATION_RAW) ||
           (raw->z > SATURATION_RAW) || (raw->z < -SATURATION_RAW);
}

LSM6DS_Status_t LSM6DS_CalibrateGyroscope(LSM6DS_Data_t *data)
{
    LSM6DS_Frame_t frame;
    LSM6DS_Vector3f_t rate;
    LSM6DS_Vector3f_t accel;
    LSM6DS_Vector3f_t rate_sum = { 0.0f, 0.0f, 0.0f };
    LSM6DS_Vector3f_t accel_sum = { 0.0f, 0.0f, 0.0f };
    LSM6DS_Vector3f_t rate_mean;
    LSM6DS_Vector3f_t accel_mean;
    float32_t rate_square_sum = 0.0f;
    float32_t accel_square_sum = 0.0f;
    float32_t max_rate = 0.0f;
    float32_t magnitude;
    float32_t rate_variance;
    float32_t accel_variance;
    float32_t inverse_count;
    uint32_t count = 0U;
    uint32_t start_ms;
    LSM6DS_Status_t status;

    if(data == NULL)
    {
        return LSM6DS_STATUS_INVALID_ARGUMENT;
    }
    if((data->address != IMU_ADDR_SA0_LOW) &&
       (data->address != IMU_ADDR_SA0_HIGH))
    {
        return finish(data, LSM6DS_STATUS_NOT_INITIALIZED);
    }

    start_ms = SysTick_Millis();
    while(count < CALIBRATION_SAMPLES)
    {
        if((uint32_t)(SysTick_Millis() - start_ms) >=
           CALIBRATION_TIMEOUT_MS)
        {
            return finish(data, LSM6DS_STATUS_CALIBRATION_TIMEOUT);
        }

        status = read_fresh_frame(data, &frame);
        if(status == LSM6DS_STATUS_NO_NEW_DATA)
        {
            Delay_ms(1U);
            continue;
        }
        if(status != LSM6DS_STATUS_OK)
        {
            return finish(data, status);
        }

        map_and_scale(data, &frame.gyro, GYRO_RAD_S_PER_LSB, &rate);
        map_and_scale(data, &frame.accel, ACCEL_MPS2_PER_LSB, &accel);
        magnitude = vector_norm(&rate);
        if(magnitude > max_rate)
        {
            max_rate = magnitude;
        }

        rate_sum.x += rate.x;
        rate_sum.y += rate.y;
        rate_sum.z += rate.z;
        accel_sum.x += accel.x;
        accel_sum.y += accel.y;
        accel_sum.z += accel.z;
        rate_square_sum += vector_dot(&rate, &rate);
        accel_square_sum += vector_dot(&accel, &accel);
        count++;
    }

    inverse_count = 1.0f / (float32_t)CALIBRATION_SAMPLES;
    rate_mean.x = rate_sum.x * inverse_count;
    rate_mean.y = rate_sum.y * inverse_count;
    rate_mean.z = rate_sum.z * inverse_count;
    accel_mean.x = accel_sum.x * inverse_count;
    accel_mean.y = accel_sum.y * inverse_count;
    accel_mean.z = accel_sum.z * inverse_count;

    rate_variance = (rate_square_sum * inverse_count) -
                    vector_dot(&rate_mean, &rate_mean);
    accel_variance = (accel_square_sum * inverse_count) -
                     vector_dot(&accel_mean, &accel_mean);
    if(rate_variance < 0.0f)
    {
        rate_variance = 0.0f;
    }
    if(accel_variance < 0.0f)
    {
        accel_variance = 0.0f;
    }

    magnitude = vector_norm(&accel_mean);
    if((vector_norm(&rate_mean) > CAL_GYRO_MEAN_MAX) ||
       (max_rate > CAL_GYRO_SAMPLE_MAX) ||
       (sqrtf(rate_variance) > CAL_GYRO_STD_MAX) ||
       (sqrtf(accel_variance) > CAL_ACCEL_STD_MAX) ||
       (magnitude < CAL_ACCEL_MEAN_MIN) ||
       (magnitude > CAL_ACCEL_MEAN_MAX))
    {
        return finish(data, LSM6DS_STATUS_CALIBRATION_MOTION);
    }

    data->calibration.gyro_bias_rad_s = rate_mean;
    data->calibration.gyro_valid = true;
    data->last_sample_timestamp_us = 0ULL;
    return finish(data, LSM6DS_STATUS_OK);
}

LSM6DS_Status_t LSM6DS_Init(LSM6DS_Data_t *data,
                            const LSM6DS_AxisMap_t *axis_map)
{
    const LSM6DS_AxisMap_t identity = {
        { (uint8_t)LSM6DS_AXIS_X,
          (uint8_t)LSM6DS_AXIS_Y,
          (uint8_t)LSM6DS_AXIS_Z },
        { 1, 1, 1 }
    };
    LSM6DS_AxisMap_t selected;
    LSM6DS_Status_t status;

    if(data == NULL)
    {
        return LSM6DS_STATUS_INVALID_ARGUMENT;
    }
    selected = (axis_map == NULL) ? identity : *axis_map;
    if(!axis_map_valid(&selected))
    {
        return LSM6DS_STATUS_INVALID_ARGUMENT;
    }

    memset(data, 0, sizeof(*data));
    data->axis_map = selected;
    data->last_status = LSM6DS_STATUS_NOT_INITIALIZED;

    Delay_ms(POWER_ON_DELAY_MS);
    status = detect_device(data);
    if(status == LSM6DS_STATUS_OK)
    {
        status = configure_device(data);
    }
    if(status == LSM6DS_STATUS_OK)
    {
        status = LSM6DS_CalibrateGyroscope(data);
    }
    if(status != LSM6DS_STATUS_OK)
    {
        return finish(data, status);
    }

    data->initialized = true;
    return finish(data, LSM6DS_STATUS_OK);
}

static void add_missed_samples(LSM6DS_Data_t *data, uint64_t elapsed_us)
{
    uint64_t intervals;
    uint64_t missing;
    uint32_t available;

    if(elapsed_us <= (NOMINAL_PERIOD_US + (NOMINAL_PERIOD_US / 2ULL)))
    {
        return;
    }

    intervals = (elapsed_us + (NOMINAL_PERIOD_US / 2ULL)) /
                NOMINAL_PERIOD_US;
    missing = intervals - 1ULL;
    available = 0xFFFFFFFFU - data->missed_sample_count;
    data->missed_sample_count =
        (missing > (uint64_t)available) ?
        0xFFFFFFFFU : data->missed_sample_count + (uint32_t)missing;
}

LSM6DS_Status_t LSM6DS_Update(LSM6DS_Data_t *data)
{
    LSM6DS_Frame_t frame;
    LSM6DS_Vector3f_t raw_rate;
    LSM6DS_Vector3f_t accel;
    LSM6DS_Status_t status;
    uint64_t elapsed_us = 0ULL;
    bool had_previous;
    bool gyro_saturated;
    bool accel_saturated;

    if(data == NULL)
    {
        return LSM6DS_STATUS_INVALID_ARGUMENT;
    }
    data->sample.fresh = false;
    if(!data->initialized)
    {
        return finish(data, LSM6DS_STATUS_NOT_INITIALIZED);
    }

    status = read_fresh_frame(data, &frame);
    if(status != LSM6DS_STATUS_OK)
    {
        return finish(data, status);
    }

    map_and_scale(data, &frame.gyro, GYRO_RAD_S_PER_LSB, &raw_rate);
    map_and_scale(data, &frame.accel, ACCEL_MPS2_PER_LSB, &accel);
    data->sample.raw_gyro = frame.gyro;
    data->sample.raw_accel = frame.accel;
    data->sample.gyro_rad_s.x =
        raw_rate.x - data->calibration.gyro_bias_rad_s.x;
    data->sample.gyro_rad_s.y =
        raw_rate.y - data->calibration.gyro_bias_rad_s.y;
    data->sample.gyro_rad_s.z =
        raw_rate.z - data->calibration.gyro_bias_rad_s.z;
    data->sample.accel_mps2 = accel;
    data->sample.temperature_c =
        25.0f + ((float32_t)frame.temperature / 256.0f);
    data->sample.timestamp_us = frame.timestamp_us;
    data->sample.sequence++;
    data->sample.fresh = true;

    gyro_saturated = raw_saturated(&frame.gyro);
    accel_saturated = raw_saturated(&frame.accel);
    data->sample.saturated = gyro_saturated || accel_saturated;
    if(data->sample.saturated)
    {
        data->saturation_count++;
    }

    had_previous = data->last_sample_timestamp_us != 0ULL;
    data->sample.dt_s = NOMINAL_DT_S;
    data->sample.timing_valid = false;
    if(had_previous &&
       (frame.timestamp_us > data->last_sample_timestamp_us))
    {
        elapsed_us = frame.timestamp_us -
                     data->last_sample_timestamp_us;
        data->sample.dt_s = (float32_t)elapsed_us * 0.000001f;
        add_missed_samples(data, elapsed_us);
        data->sample.timing_valid =
            (data->sample.dt_s >= MIN_VALID_DT_S) &&
            (data->sample.dt_s <= MAX_VALID_DT_S);
    }
    data->last_sample_timestamp_us = frame.timestamp_us;

    if(had_previous && !data->sample.timing_valid)
    {
        data->timing_fault_count++;
    }

    data->sample.valid = data->sample.timing_valid &&
                         data->calibration.gyro_valid &&
                         !data->sample.saturated;
    if(data->sample.saturated)
    {
        return finish(data, LSM6DS_STATUS_SENSOR_SATURATED);
    }
    if(had_previous && !data->sample.timing_valid)
    {
        return finish(data, LSM6DS_STATUS_TIMING_FAULT);
    }
    return finish(data, LSM6DS_STATUS_OK);
}
