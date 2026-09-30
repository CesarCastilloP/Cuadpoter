/**
 * @file telemetry.c
 * @author Alberto Vazquez
 * @brief Fixed-rate binary telemetry with non-blocking UART transmission.
 * @version 1.4.0
 * @date 2026-09-29
 */

#include <string.h>

#include "systick.h"
#include "telemetry.h"
#include "uart.h"

typedef struct
{
    uint8_t *buffer;
    uint16_t capacity;
    uint16_t length;
    bool valid;
} FrameWriter_t;

static void service_transmitter(Telemetry_Data_t *data);
static void account_due_periods(Telemetry_Data_t *data,
                                uint64_t now_us,
                                bool frame_will_be_created);
static bool create_flight_frame(Telemetry_Data_t *data,
                                uint64_t now_us,
                                const LSM6DS_Data_t *imu,
                                const LIS2MDL_Data_t *magnetometer,
                                const FlightControl_Data_t *control,
                                const MotorOutput_Data_t *motor_output);
static void writer_initialize(FrameWriter_t *writer,
                              uint8_t *buffer,
                              uint16_t capacity);
static void writer_u8(FrameWriter_t *writer, uint8_t value);
static void writer_u16(FrameWriter_t *writer, uint16_t value);
static void writer_u32(FrameWriter_t *writer, uint32_t value);
static void writer_u64(FrameWriter_t *writer, uint64_t value);
static void writer_float32(FrameWriter_t *writer, float32_t value);

Telemetry_Status_t Telemetry_Init(Telemetry_Data_t *data,
                                  uint32_t baud_rate,
                                  uint32_t output_rate_hz)
{
    uint64_t required_baud_rate;

    if(data == NULL)
    {
        return TELEMETRY_STATUS_INVALID_ARGUMENT;
    }
    if(baud_rate == 0U)
    {
        baud_rate = TELEMETRY_DEFAULT_BAUD_RATE;
    }
    if(output_rate_hz == 0U)
    {
        output_rate_hz = TELEMETRY_DEFAULT_OUTPUT_RATE_HZ;
    }

    required_baud_rate = (uint64_t)TELEMETRY_FLIGHT_FRAME_SIZE *
                         (uint64_t)output_rate_hz * 10ULL;
    if((output_rate_hz > 100U) ||
       ((uint64_t)baud_rate < required_baud_rate) ||
       (TELEMETRY_FLIGHT_FRAME_SIZE > TELEMETRY_TX_BUFFER_SIZE))
    {
        return TELEMETRY_STATUS_INVALID_CONFIG;
    }

    memset(data, 0, sizeof(*data));
    data->baud_rate = baud_rate;
    data->output_rate_hz = output_rate_hz;
    data->output_period_us = 1000000ULL / (uint64_t)output_rate_hz;
    data->next_output_us = Timebase_GetMicroseconds() +
                           data->output_period_us;
    data->initialized = true;
    data->last_status = TELEMETRY_STATUS_OK;

    uart0_init(baud_rate, NULL);
    return data->last_status;
}

Telemetry_Status_t Telemetry_Update(
    Telemetry_Data_t *data,
    const LSM6DS_Data_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const FlightControl_Data_t *control,
    const MotorOutput_Data_t *motor_output)
{
    uint64_t now_us;

    if((data == NULL) || (imu == NULL) || (magnetometer == NULL) ||
       (control == NULL) || (motor_output == NULL))
    {
        return TELEMETRY_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized)
    {
        data->last_status = TELEMETRY_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }

    service_transmitter(data);
    now_us = Timebase_GetMicroseconds();
    if(now_us < data->next_output_us)
    {
        return data->last_status;
    }

    if(data->tx_index < data->tx_length)
    {
        account_due_periods(data, now_us, false);
        return data->last_status;
    }

    account_due_periods(data, now_us, true);
    if(!create_flight_frame(data, now_us, imu, magnetometer,
                            control, motor_output))
    {
        data->encoding_error_count++;
        data->last_status = TELEMETRY_STATUS_ENCODING_ERROR;
        return data->last_status;
    }

    data->frame_sequence++;
    data->last_status = TELEMETRY_STATUS_OK;
    return data->last_status;
}

static void service_transmitter(Telemetry_Data_t *data)
{
    uint32_t sent;
    uint32_t remaining;

    if(data->tx_index >= data->tx_length)
    {
        data->tx_index = 0U;
        data->tx_length = 0U;
        return;
    }

    remaining = (uint32_t)data->tx_length - (uint32_t)data->tx_index;
    sent = UART0_SendAvailable(&data->tx_buffer[data->tx_index], remaining);
    data->tx_index = (uint16_t)((uint32_t)data->tx_index + sent);
    data->bytes_transmitted += sent;
}

static void account_due_periods(Telemetry_Data_t *data,
                                uint64_t now_us,
                                bool frame_will_be_created)
{
    uint64_t elapsed_periods;
    uint64_t dropped_periods;
    uint32_t available_count;

    elapsed_periods =
        ((now_us - data->next_output_us) / data->output_period_us) + 1ULL;
    data->next_output_us += elapsed_periods * data->output_period_us;
    dropped_periods = frame_will_be_created ?
        (elapsed_periods - 1ULL) : elapsed_periods;

    available_count = 0xFFFFFFFFU - data->dropped_frame_count;
    if(dropped_periods > (uint64_t)available_count)
    {
        data->dropped_frame_count = 0xFFFFFFFFU;
    }
    else
    {
        data->dropped_frame_count += (uint32_t)dropped_periods;
    }
}

static bool create_flight_frame(Telemetry_Data_t *data,
                                uint64_t now_us,
                                const LSM6DS_Data_t *imu,
                                const LIS2MDL_Data_t *magnetometer,
                                const FlightControl_Data_t *control,
                                const MotorOutput_Data_t *motor_output)
{
    FrameWriter_t writer;
    const LSM6DS_Sample_t *sample = &imu->sample;
    const LIS2MDL_Sample_t *magnetic_sample = &magnetometer->sample;
    const FlightControl_Output_t *fc = &control->output;

    writer_initialize(&writer, data->tx_buffer, TELEMETRY_TX_BUFFER_SIZE);
    writer_u32(&writer, TELEMETRY_SYNC_WORD);
    writer_u32(&writer, TELEMETRY_SCHEMA_VERSION);
    writer_u32(&writer, data->frame_sequence);
    writer_u64(&writer, now_us);

    writer_float32(&writer, sample->dt_s);
    writer_float32(&writer, sample->accel_mps2.x);
    writer_float32(&writer, sample->accel_mps2.y);
    writer_float32(&writer, sample->accel_mps2.z);
    writer_float32(&writer, sample->gyro_rad_s.x);
    writer_float32(&writer, sample->gyro_rad_s.y);
    writer_float32(&writer, sample->gyro_rad_s.z);
    writer_float32(&writer, magnetic_sample->body_field_ut.x);
    writer_float32(&writer, magnetic_sample->body_field_ut.y);
    writer_float32(&writer, magnetic_sample->body_field_ut.z);
    writer_float32(&writer, fc->heading.heading_deg);
    writer_float32(&writer, fc->heading.setpoint_deg);
    writer_float32(&writer, fc->heading.error_deg);
    writer_float32(&writer, fc->roll.measured_deg_s);
    writer_float32(&writer, fc->pitch.measured_deg_s);
    writer_float32(&writer, fc->yaw.measured_deg_s);
    writer_float32(&writer, fc->attitude.roll_deg);
    writer_float32(&writer, fc->attitude.pitch_deg);
    writer_float32(&writer, fc->setpoint.throttle);
    writer_float32(&writer, fc->setpoint.roll_angle_deg);
    writer_float32(&writer, fc->setpoint.pitch_angle_deg);
    writer_float32(&writer, fc->setpoint.roll_rate_deg_s);
    writer_float32(&writer, fc->setpoint.pitch_rate_deg_s);
    writer_float32(&writer, fc->setpoint.yaw_rate_deg_s);

    writer_float32(&writer, fc->roll.error_deg_s);
    writer_float32(&writer, fc->roll.proportional);
    writer_float32(&writer, fc->roll.integral);
    writer_float32(&writer, fc->roll.derivative);
    writer_float32(&writer, fc->roll.output);

    writer_float32(&writer, fc->pitch.error_deg_s);
    writer_float32(&writer, fc->pitch.proportional);
    writer_float32(&writer, fc->pitch.integral);
    writer_float32(&writer, fc->pitch.derivative);
    writer_float32(&writer, fc->pitch.output);

    writer_float32(&writer, fc->yaw.error_deg_s);
    writer_float32(&writer, fc->yaw.proportional);
    writer_float32(&writer, fc->yaw.integral);
    writer_float32(&writer, fc->yaw.derivative);
    writer_float32(&writer, fc->yaw.output);

    writer_float32(&writer, (float32_t)motor_output->pulse_us.front_left);
    writer_float32(&writer, (float32_t)motor_output->pulse_us.front_right);
    writer_float32(&writer, (float32_t)motor_output->pulse_us.rear_right);
    writer_float32(&writer, (float32_t)motor_output->pulse_us.rear_left);

    if(!writer.valid ||
       (writer.length != (uint16_t)TELEMETRY_FLIGHT_FRAME_SIZE))
    {
        data->tx_index = 0U;
        data->tx_length = 0U;
        return false;
    }

    data->tx_index = 0U;
    data->tx_length = writer.length;
    return true;
}

static void writer_initialize(FrameWriter_t *writer,
                              uint8_t *buffer,
                              uint16_t capacity)
{
    writer->buffer = buffer;
    writer->capacity = capacity;
    writer->length = 0U;
    writer->valid = true;
}

static void writer_u8(FrameWriter_t *writer, uint8_t value)
{
    if(!writer->valid)
    {
        return;
    }
    if(writer->length >= writer->capacity)
    {
        writer->valid = false;
        return;
    }
    writer->buffer[writer->length++] = value;
}

static void writer_u16(FrameWriter_t *writer, uint16_t value)
{
    writer_u8(writer, (uint8_t)(value & 0xFFU));
    writer_u8(writer, (uint8_t)((value >> 8) & 0xFFU));
}

static void writer_u32(FrameWriter_t *writer, uint32_t value)
{
    writer_u16(writer, (uint16_t)(value & 0xFFFFU));
    writer_u16(writer, (uint16_t)((value >> 16) & 0xFFFFU));
}

static void writer_u64(FrameWriter_t *writer, uint64_t value)
{
    writer_u32(writer, (uint32_t)(value & 0xFFFFFFFFULL));
    writer_u32(writer, (uint32_t)((value >> 32) & 0xFFFFFFFFULL));
}

static void writer_float32(FrameWriter_t *writer, float32_t value)
{
    union
    {
        float32_t floating_point;
        uint32_t unsigned_integer;
    } conversion;

    conversion.floating_point = value;
    writer_u32(writer, conversion.unsigned_integer);
}
