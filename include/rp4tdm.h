/**
 * @file rp4tdm.h
 * @author Alberto Vazquez
 * @brief Minimal CRSF receiver interface for the four primary flight controls.
 * @version 3.0.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_RP4TDM_H_
#define INCLUDE_RP4TDM_H_

#include "functions.h"

#define RP4TDM_CHANNEL_COUNT    16U

typedef enum
{
    RP4TDM_STATUS_OK = 0,
    RP4TDM_STATUS_WAITING_FOR_DATA,
    RP4TDM_STATUS_INVALID_ARGUMENT,
    RP4TDM_STATUS_NOT_INITIALIZED,
    RP4TDM_STATUS_CRC_ERROR,
    RP4TDM_STATUS_TIMEOUT
} RP4TDM_Status_t;

/** Zero-based CRSF channel indices assigned to the four flight controls. */
typedef struct
{
    uint8_t roll_channel;
    uint8_t pitch_channel;
    uint8_t throttle_channel;
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
    uint64_t timestamp_us;
    uint32_t sequence;
    uint16_t raw_roll;
    uint16_t raw_pitch;
    uint16_t raw_throttle;
    uint16_t raw_yaw;
    float32_t roll;
    float32_t pitch;
    float32_t throttle;
    float32_t yaw;
    bool fresh;
    bool valid;
} RP4TDM_Controls_t;

/** One receiver instance and its latest flight-ready controls. */
typedef struct
{
    bool initialized;
    RP4TDM_ChannelMap_t channel_map;
    RP4TDM_Controls_t controls;
    RP4TDM_Status_t last_status;
    uint32_t crc_error_count;
    volatile uint32_t frame_error_count;
    volatile uint32_t dropped_frame_count;

    /* Private runtime state. */
    uint16_t link_age_ticks;
} RP4TDM_Data_t;

/**
 * Initializes CRSF reception on UART4. Pass NULL for the default AETR map:
 * roll=CH1, pitch=CH2, throttle=CH3, yaw=CH4.
 */
RP4TDM_Status_t RP4TDM_Init(RP4TDM_Data_t *data,
                            uint32_t baudrate,
                            const RP4TDM_ChannelMap_t *channel_map);

/** Parses a pending frame and publishes at most one coherent command. */
RP4TDM_Status_t RP4TDM_Process(RP4TDM_Data_t *data);

/** Advances the link timeout. Call at a fixed 100 Hz rate. */
void RP4TDM_TimeoutTick(RP4TDM_Data_t *data);

#endif /* INCLUDE_RP4TDM_H_ */
