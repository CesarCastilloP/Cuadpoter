/**
 * @file telemetry.h
 * @author Alberto Vazquez
 * @brief Non-blocking binary telemetry over the LaunchPad USB virtual COM port.
 * @version 1.3.0
 * @date 2026-09-29
 */

#ifndef INCLUDE_TELEMETRY_H_
#define INCLUDE_TELEMETRY_H_

#include "flight_control.h"
#include "lis2mdl.h"
#include "lsm6ds.h"
#include "motor_output.h"

#define TELEMETRY_DEFAULT_BAUD_RATE       460800U
#define TELEMETRY_DEFAULT_OUTPUT_RATE_HZ  100U
#define TELEMETRY_TX_BUFFER_SIZE          192U
#define TELEMETRY_SYNC_WORD               0xA55A3CC3U
#define TELEMETRY_SCHEMA_VERSION          4U
#define TELEMETRY_FLIGHT_SIGNAL_COUNT     40U
#define TELEMETRY_FLIGHT_FRAME_SIZE       180U

typedef enum
{
    TELEMETRY_STATUS_OK = 0,
    TELEMETRY_STATUS_INVALID_ARGUMENT,
    TELEMETRY_STATUS_INVALID_CONFIG,
    TELEMETRY_STATUS_NOT_INITIALIZED,
    TELEMETRY_STATUS_ENCODING_ERROR
} Telemetry_Status_t;

/** One telemetry stream and its observable transmission diagnostics. */
typedef struct
{
    bool initialized;
    uint32_t baud_rate;
    uint32_t output_rate_hz;
    uint64_t output_period_us;
    uint64_t next_output_us;
    uint32_t frame_sequence;
    uint32_t dropped_frame_count;
    uint32_t encoding_error_count;
    uint32_t bytes_transmitted;
    uint16_t tx_length;
    uint16_t tx_index;
    Telemetry_Status_t last_status;

    /* Private frame buffer drained incrementally through the UART FIFO. */
    uint8_t tx_buffer[TELEMETRY_TX_BUFFER_SIZE];
} Telemetry_Data_t;

/**
 * Configure UART0 on PA0/PA1. These pins connect to the LaunchPad USB VCOM.
 * Pass zero for either argument to use the default baud rate or output rate.
 */
Telemetry_Status_t Telemetry_Init(Telemetry_Data_t *data,
                                  uint32_t baud_rate,
                                  uint32_t output_rate_hz);

/**
 * Drain pending bytes and periodically capture a complete binary frame.
 * Call this function on every pass through the main cooperative loop.
 */
Telemetry_Status_t Telemetry_Update(
    Telemetry_Data_t *data,
    const LSM6DS_Data_t *imu,
    const LIS2MDL_Data_t *magnetometer,
    const FlightControl_Data_t *control,
    const MotorOutput_Data_t *motor_output);

#endif /* INCLUDE_TELEMETRY_H_ */
