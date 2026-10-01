/**
 * @file rp4tdm.c
 * @author Alberto Vazquez
 * @brief CRSF decoder for RP4TD-M primary flight controls.
 *
 * @details Engineering execution overview:
 * - UART4 interrupt code delivers one byte at a time to receiver_byte_callback().
 * - The callback only assembles a complete CRSF frame. RP4TDM_Process(), called
 *   from the main loop, verifies address/type/CRC and decodes sixteen channels.
 * - Raw 11-bit channel values are mapped to throttle 0.0..1.0 and roll, pitch,
 *   yaw -1.0..+1.0. These numbers are dimensionless stick requests.
 * - TimeoutTick invalidates stale controls and centers them, preventing a lost
 *   receiver frame from leaving the last command active.
 * @version 3.0.0
 * @date 2026-09-15
 */

#include <string.h>

#include "rp4tdm.h"
#include "systick.h"
#include "uart.h"

/* CRSF destination address used for frames sent to a flight controller. */
#define CRSF_FLIGHT_CONTROLLER_ADDRESS    0xC8U
/* CRSF frame type containing sixteen packed 11-bit RC channels. */
#define CRSF_RC_CHANNELS_TYPE             0x16U
/* CRSF length byte for an RC-channel frame: type + payload + CRC. */
#define CRSF_RC_FRAME_LENGTH              24U
/* Sixteen channels x 11 bits = 176 bits = 22 payload bytes. */
#define CRSF_RC_PAYLOAD_SIZE              22U
/* Largest complete CRSF frame accepted by the byte parser. */
#define CRSF_MAX_FRAME_SIZE               64U
/* Legal CRSF length-byte bounds, excluding address and length itself. */
#define CRSF_MIN_LENGTH                   2U
#define CRSF_MAX_LENGTH                   62U
/* DVB-S2 CRC-8 polynomial specified by the CRSF protocol. */
#define CRSF_CRC_POLYNOMIAL               0xD5U

/* Nominal CRSF stick endpoints and center in 11-bit channel counts. */
#define CRSF_RAW_MIN                      172U
#define CRSF_RAW_CENTER                   992U
#define CRSF_RAW_MAX                      1811U
/* Twenty 100 Hz service ticks produce a 200 ms receiver-loss timeout. */
#define RP4TDM_TIMEOUT_TICKS              20U

typedef enum
{
    PARSER_WAIT_ADDRESS = 0, /**< Search incoming bytes for destination 0xC8. */
    PARSER_WAIT_LENGTH,      /**< Validate the following CRSF length byte. */
    PARSER_WAIT_BODY         /**< Collect type, payload, and CRC bytes. */
} ParserState_t;

typedef struct
{
    ParserState_t state;                 /**< Current byte-level parser phase. */
    uint8_t frame[CRSF_MAX_FRAME_SIZE];  /**< Frame being assembled by UART ISR. */
    uint8_t index;                       /**< Next write position in frame[]. */
    uint8_t total_size;                  /**< Complete size including address/length. */
} Parser_t;

/* Default CH1..CH4 assignment: roll, pitch, throttle, and yaw. */
static const RP4TDM_ChannelMap_t g_default_channel_map = {
    0U, 1U, 2U, 3U
};

/* Instance receiving ISR-produced frames; one UART4 receiver is supported. */
static RP4TDM_Data_t *g_active_instance = NULL;
/* Byte parser state owned by the UART4 receive interrupt context. */
static Parser_t g_parser;
/* Completed frame transferred from ISR context to cooperative main context. */
static uint8_t g_pending_frame[CRSF_MAX_FRAME_SIZE];
/* ISR-to-main handoff flag; volatile because both execution contexts access it. */
static volatile bool g_pending_frame_ready = false;

static bool channel_map_is_valid(const RP4TDM_ChannelMap_t *map);
static void receiver_byte_callback(uint8_t byte);
static uint8_t crc8(const uint8_t *data, uint8_t size);
static uint16_t unpack_channel(const uint8_t *payload, uint8_t channel);
static float32_t map_axis(uint16_t raw);
static float32_t map_throttle(uint16_t raw);
static void neutralize_controls(RP4TDM_Controls_t *controls);

/**
 * @brief Initialize CRSF parsing and connect the UART4 receive callback.
 * @param data Destination receiver state, controls, counters, and raw channels.
 * @param baudrate UART line rate [bits/second], normally 420000 for CRSF.
 * @param channel_map Optional roll/pitch/throttle/yaw channel indices; NULL uses defaults.
 * @return OK or an argument/channel-map error.
 */
RP4TDM_Status_t RP4TDM_Init(RP4TDM_Data_t *data,
                            uint32_t baudrate,
                            const RP4TDM_ChannelMap_t *channel_map)
{
    /* Effective channel map, either caller supplied or the conventional default. */
    const RP4TDM_ChannelMap_t *selected_map;

    if((data == NULL) || (baudrate == 0U))
    {
        return RP4TDM_STATUS_INVALID_ARGUMENT;
    }

    selected_map = (channel_map == NULL) ?
        &g_default_channel_map : channel_map;
    if(!channel_map_is_valid(selected_map))
    {
        return RP4TDM_STATUS_INVALID_ARGUMENT;
    }

    memset(data, 0, sizeof(*data));
    data->channel_map = *selected_map;
    data->last_status = RP4TDM_STATUS_WAITING_FOR_DATA;
    data->initialized = true;

    memset(&g_parser, 0, sizeof(g_parser));
    g_parser.state = PARSER_WAIT_ADDRESS;
    g_pending_frame_ready = false;
    g_active_instance = data;

    /* UART4 receives the logic-level-compatible RP4TD-M CRSF byte stream. */
    uart4_init(baudrate, receiver_byte_callback);
    return data->last_status;
}

/**
 * @brief Validate and consume the newest complete frame assembled by the ISR.
 * @param data Initialized receiver instance receiving channels and mapped controls.
 * @return OK for valid RC channels, WAITING when no frame is pending, or the
 *         exact address, length, type, CRC, range, or overflow error.
 */
RP4TDM_Status_t RP4TDM_Process(RP4TDM_Data_t *data)
{
    uint8_t frame[CRSF_MAX_FRAME_SIZE]; /* Main-context copy of the ISR frame. */
    uint8_t received_crc;               /* CRC byte appended by the receiver. */
    uint8_t calculated_crc;             /* Locally calculated CRC over type/payload. */
    const uint8_t *payload;             /* First packed channel byte in the frame. */

    if(data == NULL)
    {
        return RP4TDM_STATUS_INVALID_ARGUMENT;
    }
    if(!data->initialized || (data != g_active_instance))
    {
        data->last_status = RP4TDM_STATUS_NOT_INITIALIZED;
        return data->last_status;
    }

    data->controls.fresh = false;
    if(!g_pending_frame_ready)
    {
        return data->last_status;
    }

    /* Copy before clearing the flag so the ISR may begin publishing a new frame. */
    memcpy(frame, g_pending_frame, sizeof(frame));
    g_pending_frame_ready = false;

    received_crc = frame[CRSF_RC_FRAME_LENGTH + 1U];
    calculated_crc = crc8(&frame[2], CRSF_RC_FRAME_LENGTH - 1U);
    if(calculated_crc != received_crc)
    {
        data->crc_error_count++;
        data->last_status = RP4TDM_STATUS_CRC_ERROR;
        return data->last_status;
    }

    payload = &frame[3];
    /* Extract only the four flight axes; the other twelve channels are ignored. */
    data->controls.raw_roll =
        unpack_channel(payload, data->channel_map.roll_channel);
    data->controls.raw_pitch =
        unpack_channel(payload, data->channel_map.pitch_channel);
    data->controls.raw_throttle =
        unpack_channel(payload, data->channel_map.throttle_channel);
    data->controls.raw_yaw =
        unpack_channel(payload, data->channel_map.yaw_channel);

    /* Map sticks to controller units: axes [-1,+1], throttle [0,1]. */
    data->controls.roll = map_axis(data->controls.raw_roll);
    data->controls.pitch = map_axis(data->controls.raw_pitch);
    data->controls.throttle = map_throttle(data->controls.raw_throttle);
    data->controls.yaw = map_axis(data->controls.raw_yaw);
    data->controls.timestamp_us = Timebase_GetMicroseconds();
    data->controls.sequence++;
    data->controls.fresh = true;
    data->controls.valid = true;
    data->link_age_ticks = 0U;
    data->last_status = RP4TDM_STATUS_OK;
    return data->last_status;
}

/**
 * @brief Invalidate controls when no good CRSF frame arrives before the timeout.
 * @param data Receiver instance whose last-frame time [ms] is checked.
 * @return Nothing; stale controls are neutralized and marked invalid.
 */
void RP4TDM_TimeoutTick(RP4TDM_Data_t *data)
{
    if((data == NULL) || !data->initialized || !data->controls.valid)
    {
        return;
    }

    /* Saturate the counter so long outages cannot wrap back to a valid link. */
    if(data->link_age_ticks < RP4TDM_TIMEOUT_TICKS)
    {
        data->link_age_ticks++;
    }

    if(data->link_age_ticks >= RP4TDM_TIMEOUT_TICKS)
    {
        neutralize_controls(&data->controls);
        data->last_status = RP4TDM_STATUS_TIMEOUT;
    }
}

/** Return true when the four configured CRSF indices are distinct and below 16. */
static bool channel_map_is_valid(const RP4TDM_ChannelMap_t *map)
{
    /* Flight axes must use four distinct channels within the sixteen-channel set. */
    if((map->roll_channel >= RP4TDM_CHANNEL_COUNT) ||
       (map->pitch_channel >= RP4TDM_CHANNEL_COUNT) ||
       (map->throttle_channel >= RP4TDM_CHANNEL_COUNT) ||
       (map->yaw_channel >= RP4TDM_CHANNEL_COUNT))
    {
        return false;
    }

    return (map->roll_channel != map->pitch_channel) &&
           (map->roll_channel != map->throttle_channel) &&
           (map->roll_channel != map->yaw_channel) &&
           (map->pitch_channel != map->throttle_channel) &&
           (map->pitch_channel != map->yaw_channel) &&
           (map->throttle_channel != map->yaw_channel);
}

/** Collects complete CRSF frames in the UART4 receive interrupt. */
/**
 * @brief Consume one UART4 byte inside interrupt context and assemble a CRSF frame.
 * @param byte Newly received wire byte.
 * @return Nothing; a complete frame is copied to the pending buffer atomically.
 * @warning This interrupt callback performs no CRC or floating-point conversion.
 */
static void receiver_byte_callback(uint8_t byte)
{
    /* This state machine executes inside the UART4 receive interrupt. */
    switch(g_parser.state)
    {
    case PARSER_WAIT_ADDRESS:
        if(byte == CRSF_FLIGHT_CONTROLLER_ADDRESS)
        {
            g_parser.frame[0] = byte;
            g_parser.index = 1U;
            g_parser.state = PARSER_WAIT_LENGTH;
        }
        break;

    case PARSER_WAIT_LENGTH:
        if((byte >= CRSF_MIN_LENGTH) && (byte <= CRSF_MAX_LENGTH))
        {
            g_parser.frame[1] = byte;
            g_parser.index = 2U;
            g_parser.total_size = byte + 2U;
            g_parser.state = PARSER_WAIT_BODY;
        }
        else
        {
            if(g_active_instance != NULL)
            {
                g_active_instance->frame_error_count++;
            }
            g_parser.state = PARSER_WAIT_ADDRESS;
        }
        break;

    case PARSER_WAIT_BODY:
        g_parser.frame[g_parser.index++] = byte;
        if(g_parser.index >= g_parser.total_size)
        {
            if((g_parser.frame[1] == CRSF_RC_FRAME_LENGTH) &&
               (g_parser.frame[2] == CRSF_RC_CHANNELS_TYPE))
            {
                /* Preserve the unread frame; count a replacement attempt as dropped. */
                if(!g_pending_frame_ready)
                {
                    memcpy(g_pending_frame, g_parser.frame,
                           g_parser.total_size);
                    g_pending_frame_ready = true;
                }
                else if(g_active_instance != NULL)
                {
                    g_active_instance->dropped_frame_count++;
                }
            }
            g_parser.state = PARSER_WAIT_ADDRESS;
            g_parser.index = 0U;
        }
        break;

    default:
        g_parser.state = PARSER_WAIT_ADDRESS;
        g_parser.index = 0U;
        break;
    }
}

/** Calculate the CRSF DVB-S2 CRC-8 over size bytes and return the checksum byte. */
static uint8_t crc8(const uint8_t *data, uint8_t size)
{
    uint8_t crc;        /* Running CRC-8 remainder. */
    uint8_t byte_index; /* Current type/payload byte. */
    uint8_t bit_index;  /* Bit iteration within the current byte. */

    crc = 0U;
    for(byte_index = 0U; byte_index < size; byte_index++)
    {
        crc ^= data[byte_index];
        for(bit_index = 0U; bit_index < 8U; bit_index++)
        {
            crc = (crc & 0x80U) ?
                (uint8_t)((crc << 1U) ^ CRSF_CRC_POLYNOMIAL) :
                (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

/** Extract one little-endian packed 11-bit CRSF channel value from the payload. */
static uint16_t unpack_channel(const uint8_t *payload, uint8_t channel)
{
    uint8_t byte_index;  /* First payload byte containing this channel. */
    uint8_t bit_shift;   /* Channel's starting bit within that byte. */
    uint16_t bit_offset; /* Starting bit in the 176-bit channel payload. */
    uint32_t packed;     /* Up to three adjacent bytes used for extraction. */

    bit_offset = (uint16_t)channel * 11U;
    byte_index = (uint8_t)(bit_offset >> 3U);
    bit_shift = (uint8_t)(bit_offset & 0x07U);

    packed = payload[byte_index];
    if((uint8_t)(byte_index + 1U) < CRSF_RC_PAYLOAD_SIZE)
    {
        packed |= (uint32_t)payload[byte_index + 1U] << 8U;
    }
    if((uint8_t)(byte_index + 2U) < CRSF_RC_PAYLOAD_SIZE)
    {
        packed |= (uint32_t)payload[byte_index + 2U] << 16U;
    }

    /* Shift the requested field to bit zero and retain exactly eleven bits. */
    return (uint16_t)((packed >> bit_shift) & 0x07FFU);
}

/** Map raw CRSF axis counts to a clamped, dimensionless [-1,+1] command. */
static float32_t map_axis(uint16_t raw)
{
    /* Piecewise mapping preserves an exact zero at the asymmetric CRSF center. */
    if(raw <= CRSF_RAW_MIN)
    {
        return -1.0f;
    }
    if(raw >= CRSF_RAW_MAX)
    {
        return 1.0f;
    }
    if(raw < CRSF_RAW_CENTER)
    {
        return ((float32_t)raw - (float32_t)CRSF_RAW_CENTER) /
               ((float32_t)CRSF_RAW_CENTER - (float32_t)CRSF_RAW_MIN);
    }

    return ((float32_t)raw - (float32_t)CRSF_RAW_CENTER) /
           ((float32_t)CRSF_RAW_MAX - (float32_t)CRSF_RAW_CENTER);
}

/** Map raw CRSF throttle counts to a clamped, dimensionless [0,1] command. */
static float32_t map_throttle(uint16_t raw)
{
    if(raw <= CRSF_RAW_MIN)
    {
        return 0.0f;
    }
    if(raw >= CRSF_RAW_MAX)
    {
        return 1.0f;
    }

    return ((float32_t)raw - (float32_t)CRSF_RAW_MIN) /
           ((float32_t)CRSF_RAW_MAX - (float32_t)CRSF_RAW_MIN);
}

/** Set all axes/throttle to zero and mark the control set invalid. */
static void neutralize_controls(RP4TDM_Controls_t *controls)
{
    /* A timed-out radio link requests zero torque/attitude and stopped motors. */
    controls->roll = 0.0f;
    controls->pitch = 0.0f;
    controls->throttle = 0.0f;
    controls->yaw = 0.0f;
    controls->fresh = false;
    controls->valid = false;
}
