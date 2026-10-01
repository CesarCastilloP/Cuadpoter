/**
 * @file telemetry.c
 * @author Alberto Vazquez
 * @brief Fixed-rate binary telemetry with non-blocking UART transmission.
 *
 * @details Beginner's reading guide:
 * - Update snapshots current module outputs at 100 Hz into one fixed-layout
 *   little-endian frame. Each writer_* helper appends one explicitly sized type.
 * - service_transmitter() offers only bytes that fit in the UART0 hardware FIFO;
 *   it never waits for room, so telemetry cannot delay the flight controller.
 * - If a new period arrives while the previous frame is still being sent, the
 *   new frame is counted as dropped. Existing bytes are never overwritten.
 * - All signal units are defined in telemetry.h and TELEMETRY.md; timestamp_us
 *   is monotonic microseconds and sequence increments once per created frame.
 * @version 1.5.0
 * @date 2026-09-29
 */

#include <string.h>

#include "systick.h"
#include "telemetry.h"
#include "uart.h"

typedef struct
{
    uint8_t *buffer;    /**< Destination byte array owned by Telemetry_Data_t. */
    uint16_t capacity;  /**< Maximum writable bytes in buffer. */
    uint16_t length;    /**< Bytes successfully encoded so far. */
    bool valid;         /**< False after the first attempted buffer overflow. */
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

/**
 * @brief Initialize UART0 telemetry timing, buffers, counters, and schema state.
 * @param data Destination transmitter instance.
 * @param baud_rate Serial line rate [bits/second], normally 460800.
 * @param output_rate_hz Requested frame rate [frames/second], normally 100 Hz.
 * @return OK or an argument/UART initialization status.
 */
Telemetry_Status_t Telemetry_Init(Telemetry_Data_t *data,
                                  uint32_t baud_rate,
                                  uint32_t output_rate_hz)
{
    /* Minimum 8-N-1 line rate: bytes/frame x frames/s x 10 line bits/byte. */
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
    /* Integer microsecond period drives an absolute, drift-free schedule. */
    data->output_period_us = 1000000ULL / (uint64_t)output_rate_hz;
    data->next_output_us = Timebase_GetMicroseconds() +
                           data->output_period_us;
    data->initialized = true;
    data->last_status = TELEMETRY_STATUS_OK;

    uart0_init(baud_rate, NULL);
    return data->last_status;
}

/**
 * @brief Service pending bytes and create one new snapshot when its deadline is due.
 * @param data Initialized telemetry instance.
 * @param imu Latest IMU instance and raw/calibrated sample.
 * @param magnetometer Latest LIS2MDL instance and magnetic sample.
 * @param control Latest attitude, heading, PID, trim, and drift outputs.
 * @param motor_output Latest four commanded ESC pulses [microseconds].
 * @return OK while operating, or an argument/not-initialized/encoding error.
 */
Telemetry_Status_t Telemetry_Update(
    Telemetry_Data_t *data,
    const LSM6DS_Data_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const FlightControl_Data_t *control,
    const MotorOutput_Data_t *motor_output)
{
    /* Monotonic timestamp used for scheduling and embedded in the frame header. */
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

    /* Drain queued bytes first so encoding never blocks the flight loop. */
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

/**
 * @brief Move as many pending frame bytes as possible into the UART0 FIFO.
 * @param data Transmitter instance containing byte buffer, length, and next index.
 * @return Nothing; the function never blocks while the FIFO is full.
 */
static void service_transmitter(Telemetry_Data_t *data)
{
    uint32_t sent;      /* Bytes accepted into UART0 FIFO in this service call. */
    uint32_t remaining; /* Encoded bytes still waiting for the UART FIFO. */

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

/**
 * @brief Advance the absolute telemetry deadline and count periods that were missed.
 * @param data Instance containing period, deadline, and dropped-frame counter.
 * @param now_us Current monotonic time [microseconds].
 * @param frame_will_be_created true when this due period produces a new frame.
 * @return Nothing; next_frame_due_us is advanced beyond now_us.
 */
static void account_due_periods(Telemetry_Data_t *data,
                                uint64_t now_us,
                                bool frame_will_be_created)
{
    uint64_t elapsed_periods; /* Number of scheduled frame slots now due. */
    uint64_t dropped_periods; /* Due slots that cannot receive a frame. */
    uint32_t available_count; /* Counter headroom before uint32 saturation. */

    /* Advance by whole periods from the old deadline to avoid schedule drift. */
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

/**
 * @brief Serialize one immutable flight snapshot into the transmit buffer.
 * @param data Destination buffer and sequence/error counters.
 * @param now_us Frame timestamp [microseconds].
 * @param imu Latest IMU data.
 * @param magnetometer Latest magnetometer data.
 * @param control Latest controller data.
 * @param motor_output Latest motor pulse data.
 * @return true only when exactly TELEMETRY_FRAME_SIZE bytes were written.
 */
static bool create_flight_frame(Telemetry_Data_t *data,
                                uint64_t now_us,
                                const LSM6DS_Data_t *imu,
                                const LIS2MDL_Data_t *magnetometer,
                                const FlightControl_Data_t *control,
                                const MotorOutput_Data_t *motor_output)
{
    FrameWriter_t writer; /* Bounds-checked little-endian frame encoder. */
    const LSM6DS_Sample_t *sample = &imu->sample; /* Latest six-axis IMU data. */
    const LIS2MDL_Sample_t *magnetic_sample = &magnetometer->sample; /* Field data. */
    const FlightControl_Output_t *fc = &control->output; /* Controller snapshot. */

    /* Header: synchronization, schema identity, sequence, and generation time. */
    writer_initialize(&writer, data->tx_buffer, TELEMETRY_TX_BUFFER_SIZE);
    writer_u32(&writer, TELEMETRY_SYNC_WORD);
    writer_u32(&writer, TELEMETRY_SCHEMA_VERSION);
    writer_u32(&writer, data->frame_sequence);
    writer_u64(&writer, now_us);

    /* Payload order is the wire contract documented in TELEMETRY.md. */
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
    writer_float32(&writer,
                   control->horizontal_drift.output.acceleration_mps2.x);
    writer_float32(&writer,
                   control->horizontal_drift.output.acceleration_mps2.y);
    writer_float32(&writer,
                   control->horizontal_drift.output.velocity_mps.x);
    writer_float32(&writer,
                   control->horizontal_drift.output.velocity_mps.y);
    writer_float32(&writer,
                   control->horizontal_drift.output.displacement_m.x);
    writer_float32(&writer,
                   control->horizontal_drift.output.displacement_m.y);
    writer_float32(&writer,
                   control->horizontal_drift.output.roll_correction_deg);
    writer_float32(&writer,
                   control->horizontal_drift.output.pitch_correction_deg);
    writer_float32(&writer, fc->roll.measured_deg_s);
    writer_float32(&writer, fc->pitch.measured_deg_s);
    writer_float32(&writer, fc->yaw.measured_deg_s);
    writer_float32(&writer, fc->attitude.roll_deg);
    writer_float32(&writer, fc->attitude.pitch_deg);
    writer_float32(&writer, fc->setpoint.throttle);
    writer_float32(&writer, control->config.flight_roll_trim_deg);
    writer_float32(&writer, control->config.flight_pitch_trim_deg);
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

    /* A frame is published only when every expected field fits exactly. */
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

/** Initialize a bounded little-endian writer over a caller-owned byte array. */
static void writer_initialize(FrameWriter_t *writer,
                              uint8_t *buffer,
                              uint16_t capacity)
{
    writer->buffer = buffer;
    writer->capacity = capacity;
    writer->length = 0U;
    writer->valid = true;
}

/** Append one unsigned 8-bit value, or mark overflow if capacity is exhausted. */
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

/** Append one unsigned 16-bit value in little-endian byte order. */
static void writer_u16(FrameWriter_t *writer, uint16_t value)
{
    /* The protocol is explicitly little-endian, independent of host tooling. */
    writer_u8(writer, (uint8_t)(value & 0xFFU));
    writer_u8(writer, (uint8_t)((value >> 8) & 0xFFU));
}

/** Append one unsigned 32-bit value in little-endian byte order. */
static void writer_u32(FrameWriter_t *writer, uint32_t value)
{
    writer_u16(writer, (uint16_t)(value & 0xFFFFU));
    writer_u16(writer, (uint16_t)((value >> 16) & 0xFFFFU));
}

/** Append one unsigned 64-bit value in little-endian byte order. */
static void writer_u64(FrameWriter_t *writer, uint64_t value)
{
    writer_u32(writer, (uint32_t)(value & 0xFFFFFFFFULL));
    writer_u32(writer, (uint32_t)((value >> 32) & 0xFFFFFFFFULL));
}

/** Append one IEEE-754 32-bit float using its little-endian binary representation. */
static void writer_float32(FrameWriter_t *writer, float32_t value)
{
    /* Bit-preserving reinterpretation serializes IEEE-754 single precision. */
    union
    {
        float32_t floating_point;   /* Source numerical value. */
        uint32_t unsigned_integer;  /* Same 32 bits used by integer writer. */
    } conversion;

    conversion.floating_point = value;
    writer_u32(writer, conversion.unsigned_integer);
}
