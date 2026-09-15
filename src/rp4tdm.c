/**
 * @file rp4tdm.c
 * @author Alberto Vazquez
 * @brief CRSF decoder for RP4TD-M primary flight controls.
 * @version 3.0.0
 * @date 2026-09-15
 */

#include <string.h>

#include "rp4tdm.h"
#include "systick.h"
#include "uart.h"

#define CRSF_FLIGHT_CONTROLLER_ADDRESS    0xC8U
#define CRSF_RC_CHANNELS_TYPE             0x16U
#define CRSF_RC_FRAME_LENGTH              24U
#define CRSF_RC_PAYLOAD_SIZE              22U
#define CRSF_MAX_FRAME_SIZE               64U
#define CRSF_MIN_LENGTH                   2U
#define CRSF_MAX_LENGTH                   62U
#define CRSF_CRC_POLYNOMIAL               0xD5U

#define CRSF_RAW_MIN                      172U
#define CRSF_RAW_CENTER                   992U
#define CRSF_RAW_MAX                      1811U
#define RP4TDM_TIMEOUT_TICKS              20U

typedef enum
{
    PARSER_WAIT_ADDRESS = 0,
    PARSER_WAIT_LENGTH,
    PARSER_WAIT_BODY
} ParserState_t;

typedef struct
{
    ParserState_t state;
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t index;
    uint8_t total_size;
} Parser_t;

static const RP4TDM_ChannelMap_t g_default_channel_map = {
    0U, 1U, 2U, 3U
};

static RP4TDM_Data_t *g_active_instance = NULL;
static Parser_t g_parser;
static uint8_t g_pending_frame[CRSF_MAX_FRAME_SIZE];
static volatile bool g_pending_frame_ready = false;

static bool channel_map_is_valid(const RP4TDM_ChannelMap_t *map);
static void receiver_byte_callback(uint8_t byte);
static uint8_t crc8(const uint8_t *data, uint8_t size);
static uint16_t unpack_channel(const uint8_t *payload, uint8_t channel);
static float32_t map_axis(uint16_t raw);
static float32_t map_throttle(uint16_t raw);
static void neutralize_controls(RP4TDM_Controls_t *controls);

RP4TDM_Status_t RP4TDM_Init(RP4TDM_Data_t *data,
                            uint32_t baudrate,
                            const RP4TDM_ChannelMap_t *channel_map)
{
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

    uart4_init(baudrate, receiver_byte_callback);
    return data->last_status;
}

RP4TDM_Status_t RP4TDM_Process(RP4TDM_Data_t *data)
{
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t received_crc;
    uint8_t calculated_crc;
    const uint8_t *payload;

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
    data->controls.raw_roll =
        unpack_channel(payload, data->channel_map.roll_channel);
    data->controls.raw_pitch =
        unpack_channel(payload, data->channel_map.pitch_channel);
    data->controls.raw_throttle =
        unpack_channel(payload, data->channel_map.throttle_channel);
    data->controls.raw_yaw =
        unpack_channel(payload, data->channel_map.yaw_channel);

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

void RP4TDM_TimeoutTick(RP4TDM_Data_t *data)
{
    if((data == NULL) || !data->initialized || !data->controls.valid)
    {
        return;
    }

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

static bool channel_map_is_valid(const RP4TDM_ChannelMap_t *map)
{
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
static void receiver_byte_callback(uint8_t byte)
{
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

static uint8_t crc8(const uint8_t *data, uint8_t size)
{
    uint8_t crc;
    uint8_t byte_index;
    uint8_t bit_index;

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

static uint16_t unpack_channel(const uint8_t *payload, uint8_t channel)
{
    uint8_t byte_index;
    uint8_t bit_shift;
    uint16_t bit_offset;
    uint32_t packed;

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

    return (uint16_t)((packed >> bit_shift) & 0x07FFU);
}

static float32_t map_axis(uint16_t raw)
{
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

static void neutralize_controls(RP4TDM_Controls_t *controls)
{
    controls->roll = 0.0f;
    controls->pitch = 0.0f;
    controls->throttle = 0.0f;
    controls->yaw = 0.0f;
    controls->fresh = false;
    controls->valid = false;
}
