/**
 * @file rp4tdm.h
 * @author Alberto Vazquez
 * @brief Minimal CRSF receiver interface for the four primary flight controls.
 *
 * @details UART4 receives CRSF bytes at 420000 bit/s in an interrupt. The ISR
 * appends bytes to a frame buffer; main() calls RP4TDM_Process() every loop to
 * validate address, length, type, and CRC before publishing channel values.
 * RP4TDM_TimeoutTick() runs at 100 Hz, so each age count represents 10 ms.
 * Raw CRSF channels are mapped to throttle [0,1] and roll/pitch/yaw [-1,+1].
 * Invalid or stale frames clear controls.valid, which makes MotorOutput safe.
 * @version 3.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_RP4TDM_H_
#define INCLUDE_RP4TDM_H_

#include "functions.h"

/** Number of 11-bit channels transported by a CRSF RC_CHANNELS_PACKED frame. */
#define RP4TDM_CHANNEL_COUNT    16U

typedef enum
{
    /** A CRC-valid RC frame was decoded and published. */
    RP4TDM_STATUS_OK = 0,
    /** No complete frame is pending; prior controls may remain valid. */
    RP4TDM_STATUS_WAITING_FOR_DATA,
    /** API received invalid pointer, baud, or channel mapping. */
    RP4TDM_STATUS_INVALID_ARGUMENT,
    /** Process/timeout service was called before initialization. */
    RP4TDM_STATUS_NOT_INITIALIZED,
    /** Complete CRSF frame failed CRC8-D5 validation. */
    RP4TDM_STATUS_CRC_ERROR,
    /** No CRC-valid RC frame arrived for the configured 200 ms age. */
    RP4TDM_STATUS_TIMEOUT
} RP4TDM_Status_t;

/** Zero-based CRSF channel indices assigned to the four flight controls. */
typedef struct
{
    /** Zero-based channel used for roll, default 0 (CH1). */
    uint8_t roll_channel;
    /** Zero-based channel used for pitch, default 1 (CH2). */
    uint8_t pitch_channel;
    /** Zero-based channel used for throttle, default 2 (CH3). */
    uint8_t throttle_channel;
    /** Zero-based channel used for yaw, default 3 (CH4). */
    uint8_t yaw_channel;
} RP4TDM_ChannelMap_t;

/**
 * Latest coherent receiver command.
 *
 * The three attitude commands use [-1, 1]. Throttle uses [0, 1]. Raw values
 * are retained so transmitter endpoints and center can be checked directly.
 */
typedef struct
{
    /** TIMER7 time assigned when the valid frame is published. Unit: us. */
    uint64_t timestamp_us;
    /** Monotonic valid-control publication count. */
    uint32_t sequence;
    /** CRSF roll channel before normalization. Unit: 11-bit counts. */
    uint16_t raw_roll;
    /** CRSF pitch channel before normalization. Unit: 11-bit counts. */
    uint16_t raw_pitch;
    /** CRSF throttle channel before normalization. Unit: 11-bit counts. */
    uint16_t raw_throttle;
    /** CRSF yaw channel before normalization. Unit: 11-bit counts. */
    uint16_t raw_yaw;
    /** Normalized roll command. Unit/range: dimensionless -1...+1. */
    float32_t roll;
    /** Normalized pitch command. Unit/range: dimensionless -1...+1. */
    float32_t pitch;
    /** Normalized throttle. Unit/range: dimensionless 0...1. */
    float32_t throttle;
    /** Normalized yaw command. Unit/range: dimensionless -1...+1. */
    float32_t yaw;
    /** True only on the Process call that publishes a new frame. */
    bool fresh;
    /** True while link age remains below timeout after a CRC-valid frame. */
    bool valid;
} RP4TDM_Controls_t;

/** One receiver instance and its latest flight-ready controls. */
typedef struct
{
    /** True after UART4 and parser state initialize. */
    bool initialized;
    /** Instance-specific assignment of CRSF channels to controls. */
    RP4TDM_ChannelMap_t channel_map;
    /** Latest coherent raw and normalized commands. */
    RP4TDM_Controls_t controls;
    /** Most recent parser/link result for CCS. */
    RP4TDM_Status_t last_status;
    /** Complete frames rejected by CRC8-D5. */
    uint32_t crc_error_count;
    /** ISR-observable malformed-length/frame count. */
    volatile uint32_t frame_error_count;
    /** Complete ISR frames lost because the single mailbox was occupied. */
    volatile uint32_t dropped_frame_count;

    /* Private runtime state. */
    /** Age since the last valid RC frame. Unit: 100 Hz timeout ticks. */
    uint16_t link_age_ticks;
} RP4TDM_Data_t;

/**
 * Initializes CRSF reception on UART4. Pass NULL for the default AETR map:
 * roll=CH1, pitch=CH2, throttle=CH3, yaw=CH4.
 * @param data Writable receiver/parser instance.
 * @param baudrate UART line speed in bits/second, normally 420000.
 * @param channel_map Optional zero-based mapping; NULL selects AETR.
 * @return OK or an invalid argument/channel-map status.
 */
RP4TDM_Status_t RP4TDM_Init(RP4TDM_Data_t *data,
                            uint32_t baudrate,
                            const RP4TDM_ChannelMap_t *channel_map);

/**
 * Parse a pending ISR mailbox frame and publish at most one command.
 * @param data Initialized receiver instance.
 * @return OK, WAITING_FOR_DATA, or a CRC/format/timeout error.
 */
RP4TDM_Status_t RP4TDM_Process(RP4TDM_Data_t *data);

/**
 * Advance link age by one 100 Hz tick and neutralize controls at timeout.
 * @param data Initialized receiver instance.
 * @return Nothing; stale controls are marked invalid in place.
 */
void RP4TDM_TimeoutTick(RP4TDM_Data_t *data);

#endif /* INCLUDE_RP4TDM_H_ */
