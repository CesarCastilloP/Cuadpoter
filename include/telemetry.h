/**
 * @file telemetry.h
 * @author Alberto Vazquez
 * @brief Non-blocking binary telemetry over the LaunchPad USB virtual COM port.
 *
 * @details main() calls Telemetry_Update() every scheduler pass. The module
 * internally limits complete snapshots to 100 frames/s and transmits only when
 * UART0 has enough FIFO/ring-buffer capacity, so telemetry cannot wait inside
 * the IMU/control path. Schema 7 contains a fixed 232-byte little-endian frame
 * with IMU raw/physical values, attitude/setpoints, PID terms, motor outputs,
 * magnetic field, trims, sequence, and timestamp. It has no CRC by design.
 * @version 1.5.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_TELEMETRY_H_
#define INCLUDE_TELEMETRY_H_

#include "flight_control.h"
#include "lis2mdl.h"
#include "lsm6ds.h"
#include "motor_output.h"

/** UART0 default line rate. Unit: bits/second. */
#define TELEMETRY_DEFAULT_BAUD_RATE       460800U
/** Nominal binary snapshot rate. Unit: frames/second. */
#define TELEMETRY_DEFAULT_OUTPUT_RATE_HZ  100U
/** Static transmit storage capacity. Unit: bytes. */
#define TELEMETRY_TX_BUFFER_SIZE          232U
/** U32 resynchronization marker, transmitted little-endian. */
#define TELEMETRY_SYNC_WORD               0xA55A3CC3U
/** Wire-layout revision required by the Python decoder. */
#define TELEMETRY_SCHEMA_VERSION          7U
/** Number of IEEE-754 float32 payload values per frame. */
#define TELEMETRY_FLIGHT_SIGNAL_COUNT     53U
/** Header plus payload size. Unit: bytes. */
#define TELEMETRY_FLIGHT_FRAME_SIZE       232U

typedef enum
{
    /** UART service and optional frame creation completed normally. */
    TELEMETRY_STATUS_OK = 0,
    /** Required instance/snapshot pointer was NULL. */
    TELEMETRY_STATUS_INVALID_ARGUMENT,
    /** Baud/rate/frame relationship cannot meet the configured schedule. */
    TELEMETRY_STATUS_INVALID_CONFIG,
    /** Update was called before Telemetry_Init. */
    TELEMETRY_STATUS_NOT_INITIALIZED,
    /** Serialized byte count or buffer capacity check failed. */
    TELEMETRY_STATUS_ENCODING_ERROR
} Telemetry_Status_t;

/** One telemetry stream and its observable transmission diagnostics. */
typedef struct
{
    /** True after UART0 and scheduling state initialize. */
    bool initialized;
    /** Configured UART0 line rate. Unit: bits/second. */
    uint32_t baud_rate;
    /** Configured snapshot creation rate. Unit: frames/second. */
    uint32_t output_rate_hz;
    /** Integer interval between frame deadlines. Unit: microseconds. */
    uint64_t output_period_us;
    /** Next absolute TIMER7 deadline. Unit: microseconds. */
    uint64_t next_output_us;
    /** Sequence written into the next frame; increments per created frame. */
    uint32_t frame_sequence;
    /** Frames omitted because TX was pending or deadlines were missed. */
    uint32_t dropped_frame_count;
    /** Serialization attempts rejected by length/capacity checks. */
    uint32_t encoding_error_count;
    /** Cumulative bytes accepted by the UART0 FIFO. */
    uint32_t bytes_transmitted;
    /** Active encoded frame length, zero when no frame is pending. Unit: bytes. */
    uint16_t tx_length;
    /** Index of the next byte to offer to UART0. Unit: bytes. */
    uint16_t tx_index;
    /** Most recent telemetry service result for CCS. */
    Telemetry_Status_t last_status;

    /* Private frame buffer drained incrementally through the UART FIFO. */
    uint8_t tx_buffer[TELEMETRY_TX_BUFFER_SIZE];
} Telemetry_Data_t;

/**
 * Configure UART0 on PA0/PA1. These pins connect to the LaunchPad USB VCOM.
 * Pass zero for either argument to use the default baud rate or output rate.
 * @param data Writable telemetry stream instance.
 * @param baud_rate Requested bits/second, or zero for 460800.
 * @param output_rate_hz Requested frames/second, or zero for 100.
 * @return OK or an argument/configuration error.
 */
Telemetry_Status_t Telemetry_Init(Telemetry_Data_t *data,
                                  uint32_t baud_rate,
                                  uint32_t output_rate_hz);

/**
 * Drain pending bytes and periodically capture a complete binary frame.
 * Call this function on every pass through the main cooperative loop.
 * Input pointers provide one coherent snapshot; this function never owns or
 * mutates sensor, control, or motor state.
 * @param data Initialized telemetry stream.
 * @param imu Latest IMU state and sample.
 * @param magnetometer Latest magnetometer state and sample.
 * @param control Latest estimator/controller state.
 * @param motor_output Latest four ESC pulse values.
 * @return OK or an argument/not-initialized/encoding error.
 */
Telemetry_Status_t Telemetry_Update(
    Telemetry_Data_t *data,
    const LSM6DS_Data_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const FlightControl_Data_t *control,
    const MotorOutput_Data_t *motor_output);

#endif /* INCLUDE_TELEMETRY_H_ */
