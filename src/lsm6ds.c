/**
 * @file lsm6ds.c
 * @author Alberto Vazquez
 * @brief Instance-based LSM6DSR acquisition and calibration.
 *
 * @details Engineering execution overview:
 * - Init detects the sensor, verifies WHO_AM_I, configures output data rate and
 *   full scale, and prepares the caller-owned data structure.
 * - CalibrateGyroscope averages stationary samples. Update reads one fresh
 *   six-axis frame and converts raw signed counts into rad/s and m/s^2.
 * - The axis map changes sensor-board axes into the aircraft body convention;
 *   it is configuration, not an artificial change to the measured attitude.
 * - Each public function returns a status code. The caller must accept a sample
 *   only when sample.valid is true and the sequence number has advanced.
 * @version 2.1.0
 * @date 2026-09-28
 */

#include <math.h>
#include <string.h>

#include "lsm6ds.h"
#include "i2c0_drone.h"
#include "systick.h"

/* Legal 7-bit addresses selected by the LSM6DSR SA0 pin. */
#define IMU_ADDR_SA0_LOW            0x6AU
#define IMU_ADDR_SA0_HIGH           0x6BU
/* Register map and expected silicon identity used by this driver. */
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

/* Register encodings for 416 Hz, +/-8 g, +/-2000 dps, BDU and filtering. */
#define VALUE_CTRL1_XL              0x6EU
#define VALUE_CTRL2_G               0x6CU
#define VALUE_CTRL3_C               0x44U
#define VALUE_CTRL4_C               0x0AU
#define VALUE_CTRL6_C               0x04U
#define VALUE_CTRL8_XL              0x00U
#define CTRL9_I3C_DISABLE           0x02U /* Prevent I3C activity on shared I2C pins. */
#define SOFTWARE_RESET              0x01U /* SW_RESET bit in CTRL3_C. */
#define DATA_READY_MASK             0x03U /* Accelerometer and gyro data-ready bits. */

/* Selected full-scale conversion factors. */
#define GYRO_RAD_S_PER_LSB          0.0012217304764f /* 70 mdps/count in rad/s. */
#define ACCEL_MPS2_PER_LSB          0.0023928226f /* 0.244 mg/count in m/s^2. */
#define STANDARD_GRAVITY_MPS2       9.80665f /* Standard gravity in m/s^2. */
#define NOMINAL_DT_S                (1.0f / 416.0f) /* Nominal sample period, s. */
#define NOMINAL_PERIOD_US           2404ULL /* Rounded 416 Hz period, us. */

/* Sample health limits. */
#define MIN_VALID_DT_S              0.0005f /* Reject duplicate/too-fast samples. */
#define MAX_VALID_DT_S              0.00421f /* Reject missing or delayed samples. */
#define SATURATION_RAW              32000 /* Margin below signed 16-bit rail. */

/* Startup and stationary calibration limits. */
#define POWER_ON_DELAY_MS           35U /* Datasheet boot margin after power-up. */
#define FILTER_SETTLE_MS            250U /* Digital-filter settling time. */
#define RESET_TIMEOUT_MS            100U /* Maximum SW_RESET completion time. */
#define CALIBRATION_TIMEOUT_MS      5000U /* Maximum stationary collection time. */
#define CALIBRATION_SAMPLES         512U /* Fresh frames averaged for calibration. */
#define CAL_GYRO_MEAN_MAX           0.02617994f /* 1.5 deg/s mean-rate limit. */
#define CAL_GYRO_SAMPLE_MAX         0.05235988f /* 3 deg/s peak-rate limit. */
#define CAL_GYRO_STD_MAX            0.00872665f /* 0.5 deg/s rate std-dev limit. */
#define CAL_ACCEL_STD_MAX           0.14710f /* 0.015 g accel std-dev limit. */
#define CAL_ACCEL_MEAN_MIN          (0.85f * STANDARD_GRAVITY_MPS2)
#define CAL_ACCEL_MEAN_MAX          (1.15f * STANDARD_GRAVITY_MPS2)

typedef struct
{
    LSM6DS_RawVector3_t gyro;  /**< Raw signed gyroscope counts. */
    LSM6DS_RawVector3_t accel; /**< Raw signed accelerometer counts. */
    int16_t temperature;       /**< Raw signed temperature count. */
    uint64_t timestamp_us;     /**< Burst completion time, microseconds. */
} LSM6DS_Frame_t;

typedef struct
{
    uint8_t reg;   /**< Destination control-register address. */
    uint8_t value; /**< Complete value written and verified. */
} LSM6DS_RegisterValue_t;

/** Return the dot product of two three-axis vectors in their combined units. */
static float32_t vector_dot(const LSM6DS_Vector3f_t *a,
                            const LSM6DS_Vector3f_t *b)
{
    /* Three-dimensional Euclidean dot product in the vectors' native units. */
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

/** Return the Euclidean magnitude of a three-axis vector in the component unit. */
static float32_t vector_norm(const LSM6DS_Vector3f_t *value)
{
    return sqrtf(vector_dot(value, value));
}

/** Translate the shared I2C0 driver's status into an LSM6DS-specific status. */
static LSM6DS_Status_t status_from_i2c(I2C0_Status_t status)
{
    /* Preserve the specific transport failure for debugger diagnosis. */
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

/** Return true when status is a NACK, timeout, or other I2C communication failure. */
static bool is_bus_error(LSM6DS_Status_t status)
{
    return (status >= LSM6DS_STATUS_I2C_ARBITRATION) &&
           (status <= LSM6DS_STATUS_I2C_TIMEOUT);
}

/** Store status/counters in the instance and return status unchanged. */
static LSM6DS_Status_t finish(LSM6DS_Data_t *data, LSM6DS_Status_t status)
{
    /* Publish every exit status and maintain a communication-fault counter. */
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

/** Read sequential bytes from one LSM6DS register at a seven-bit address. */
static LSM6DS_Status_t read_reg(uint8_t address, uint8_t reg,
                                uint8_t *buffer, uint32_t length)
{
    return status_from_i2c(I2C0_WriteRead(address, reg, buffer, length));
}

/** Write one LSM6DS register byte and translate the I2C result. */
static LSM6DS_Status_t write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    return status_from_i2c(I2C0_WriteWrite(address, reg, value));
}

/** Combine low/high little-endian bytes into one signed raw sensor count. */
static int16_t make_int16(uint8_t low, uint8_t high)
{
    /* All LSM6DSR output registers store signed words little-endian. */
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

/** Verify that an axis map uses X/Y/Z once each and only signs -1 or +1. */
static bool axis_map_valid(const LSM6DS_AxisMap_t *map)
{
    uint32_t i;             /* Destination-axis index. */
    uint32_t j;             /* Second index used to detect duplicates. */
    int32_t inversions = 0; /* Permutation inversions used for determinant sign. */
    int32_t determinant;    /* Must be +1 to preserve a right-handed frame. */

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

    /* permutation_sign * axis_signs is the signed-permutation determinant. */
    determinant = ((inversions & 1) == 0) ? 1 : -1;
    determinant *= (int32_t)map->sign[0] *
                   (int32_t)map->sign[1] *
                   (int32_t)map->sign[2];
    return determinant == 1;
}

/**
 * @brief Map sensor axes into body axes and multiply raw counts by a scale.
 * @param data Instance containing the sensor-to-body permutation and signs.
 * @param raw Three signed samples [counts].
 * @param scale Physical units per count: rad/s/count or m/s^2/count.
 * @param body Destination three-axis vector in the physical unit selected by scale.
 */
static void map_and_scale(const LSM6DS_Data_t *data,
                          const LSM6DS_RawVector3_t *raw,
                          float32_t scale,
                          LSM6DS_Vector3f_t *body)
{
    float32_t sensor[3]; /* Scaled X/Y/Z values in physical sensor axes. */
    float32_t mapped[3]; /* Reordered/sign-corrected aircraft body values. */
    uint32_t axis;       /* Destination body-axis index. */

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
/**
 * @brief Check data-ready flags and read one coherent gyro/accelerometer burst.
 * @param data Initialized instance providing the selected I2C address.
 * @param frame Destination raw counts and acquisition timestamp [us].
 * @return OK for fresh gyro+accel, NO_NEW_DATA while waiting, or bus/error status.
 */
static LSM6DS_Status_t read_fresh_frame(LSM6DS_Data_t *data,
                                        LSM6DS_Frame_t *frame)
{
    uint8_t status_reg;      /* Accelerometer and gyro data-ready flags. */
    uint8_t bytes[14];       /* Temperature, gyro XYZ, and accel XYZ burst. */
    LSM6DS_Status_t status;  /* Current bus/driver result. */

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

    /* Decode one coherent auto-incremented register burst. */
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

/** Probe addresses 0x6A/0x6B, require WHO_AM_I 0x6B, and store the active address. */
static LSM6DS_Status_t detect_device(LSM6DS_Data_t *data)
{
    /* Probe only the two addresses defined by the device datasheet. */
    static const uint8_t addresses[2] = {
        IMU_ADDR_SA0_LOW, IMU_ADDR_SA0_HIGH
    };
    LSM6DS_Status_t transport_error = LSM6DS_STATUS_OK; /* Non-NACK bus fault. */
    LSM6DS_Status_t status; /* Result for the current candidate address. */
    uint8_t identity;       /* WHO_AM_I readback. */
    uint32_t i;             /* Address-table index. */

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

/** Write one configuration register and verify its masked readback bits. */
static LSM6DS_Status_t write_and_verify(LSM6DS_Data_t *data,
                                        uint8_t reg, uint8_t value,
                                        uint8_t mask)
{
    uint8_t readback;      /* Value read after the register write. */
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

/** Configure 416 Hz gyro/accelerometer operation, scales, filters, and BDU. */
static LSM6DS_Status_t configure_device(LSM6DS_Data_t *data)
{
    /* Ordered configuration table; ODR registers are enabled last. */
    static const LSM6DS_RegisterValue_t configuration[] = {
        { REG_CTRL3_C,  VALUE_CTRL3_C  },
        { REG_CTRL4_C,  VALUE_CTRL4_C  },
        { REG_CTRL6_C,  VALUE_CTRL6_C  },
        { REG_CTRL8_XL, VALUE_CTRL8_XL },
        { REG_CTRL1_XL, VALUE_CTRL1_XL },
        { REG_CTRL2_G,  VALUE_CTRL2_G  }
    };
    LSM6DS_Status_t status; /* Current reset/configuration result. */
    uint8_t value;          /* Control-register readback. */
    uint32_t start_ms;      /* SW_RESET polling start, milliseconds. */
    uint32_t i;             /* Configuration-table index. */

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

    /* Preserve unrelated CTRL9 bits while disabling I3C on the I2C bus. */
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

/** Return true when any signed raw axis is at/near the converter's numeric limit. */
static bool raw_saturated(const LSM6DS_RawVector3_t *raw)
{
    /* Treat values near either ADC rail as unsafe for flight estimation. */
    return (raw->x > SATURATION_RAW) || (raw->x < -SATURATION_RAW) ||
           (raw->y > SATURATION_RAW) || (raw->y < -SATURATION_RAW) ||
           (raw->z > SATURATION_RAW) || (raw->z < -SATURATION_RAW);
}

/**
 * @brief Average stationary samples into gyro bias and aircraft level reference.
 * @param data Initialized instance receiving bias [rad/s], gravity [m/s^2],
 *        validity flags, counters, and retry information.
 * @return OK only after the configured number of still, valid samples; otherwise
 *         reports motion, saturation, timeout, or communication failure.
 * @warning Keep the aircraft still and physically level throughout this call.
 */
LSM6DS_Status_t LSM6DS_CalibrateGyroscope(LSM6DS_Data_t *data)
{
    LSM6DS_Frame_t frame;       /* Current coherent raw sensor burst. */
    LSM6DS_Vector3f_t rate;     /* Mapped angular rate, rad/s. */
    LSM6DS_Vector3f_t accel;    /* Mapped acceleration, m/s^2. */
    LSM6DS_Vector3f_t rate_sum = { 0.0f, 0.0f, 0.0f };
    LSM6DS_Vector3f_t accel_sum = { 0.0f, 0.0f, 0.0f };
    LSM6DS_Vector3f_t rate_mean;  /* Average stationary gyro bias, rad/s. */
    LSM6DS_Vector3f_t accel_mean; /* Average level gravity vector, m/s^2. */
    float32_t rate_square_sum = 0.0f;  /* Sum of |gyro|^2 for variance. */
    float32_t accel_square_sum = 0.0f; /* Sum of |accel|^2 for variance. */
    float32_t max_rate = 0.0f;    /* Largest sample angular-rate norm, rad/s. */
    float32_t magnitude;          /* Reused vector magnitude in physical units. */
    float32_t rate_variance;      /* Combined three-axis gyro variance. */
    float32_t accel_variance;     /* Combined three-axis accel variance. */
    float32_t inverse_count;      /* 1/CALIBRATION_SAMPLES. */
    uint32_t count = 0U;          /* Fresh calibration frames accepted. */
    uint32_t start_ms;            /* Calibration start time, milliseconds. */
    LSM6DS_Status_t status;       /* Current frame acquisition result. */

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

        /* Accumulate vector means and norm variances without sample storage. */
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

    /* E[|x|^2] - |E[x]|^2 gives total three-axis population variance. */
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

    /* Reject movement, vibration, excessive bias, or a non-gravity accel mean. */
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

    /* Publish calibration only after all stationarity tests pass. */
    data->calibration.gyro_bias_rad_s = rate_mean;
    data->calibration.level_accel_mps2 = accel_mean;
    data->calibration.gyro_valid = true;
    data->calibration.level_valid = true;
    data->last_sample_timestamp_us = 0ULL;
    return finish(data, LSM6DS_STATUS_OK);
}

/**
 * @brief Detect and configure an LSM6DSR while preserving caller axis mapping.
 * @param data Destination device instance.
 * @param axis_map Sensor-to-aircraft axis permutation and sign selection.
 * @return OK or a precise argument, identity, configuration, or bus error.
 */
LSM6DS_Status_t LSM6DS_Init(LSM6DS_Data_t *data,
                            const LSM6DS_AxisMap_t *axis_map)
{
    /* Identity signed permutation used when caller does not specify mounting. */
    const LSM6DS_AxisMap_t identity = {
        { (uint8_t)LSM6DS_AXIS_X,
          (uint8_t)LSM6DS_AXIS_Y,
          (uint8_t)LSM6DS_AXIS_Z },
        { 1, 1, 1 }
    };
    LSM6DS_AxisMap_t selected; /* Axis mapping applied to gyro and accel. */
    LSM6DS_Status_t status;    /* Current initialization stage result. */

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

    /* Detect, configure, and calibrate as one all-or-nothing startup sequence. */
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

/** Estimate and accumulate missed 416 Hz samples from a long elapsed interval [us]. */
static void add_missed_samples(LSM6DS_Data_t *data, uint64_t elapsed_us)
{
    uint64_t intervals; /* Nearest number of nominal sample intervals elapsed. */
    uint64_t missing;   /* Intervals beyond the frame that was received. */
    uint32_t available; /* Remaining uint32 diagnostic counter capacity. */

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

/**
 * @brief Publish one calibrated six-axis sample when fresh data is available.
 * @param data Initialized instance receiving raw counts, rad/s, m/s^2, dt [s],
 *        timestamp [us], validity, sequence, and diagnostic counters.
 * @return OK, NO_NEW_DATA, timing/saturation error, or communication error.
 */
LSM6DS_Status_t LSM6DS_Update(LSM6DS_Data_t *data)
{
    LSM6DS_Frame_t frame;        /* Fresh coherent sensor register burst. */
    LSM6DS_Vector3f_t raw_rate;  /* Mapped gyro before bias subtraction, rad/s. */
    LSM6DS_Vector3f_t accel;     /* Mapped acceleration, m/s^2. */
    LSM6DS_Status_t status;      /* Frame acquisition result. */
    uint64_t elapsed_us = 0ULL;  /* Time since previous accepted frame, us. */
    bool had_previous;           /* False only for the first frame after startup. */
    bool gyro_saturated;         /* Any gyro count near the ADC rail. */
    bool accel_saturated;        /* Any accel count near the ADC rail. */

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

    /* Scale/mount-map measurements, remove gyro bias, and publish temperature. */
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

    /* The first frame has nominal dt but is invalid until a measured interval exists. */
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

    /* Flight validity requires timing, calibration, and nonsaturated measurements. */
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
