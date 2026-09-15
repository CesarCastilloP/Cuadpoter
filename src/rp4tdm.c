/**
 * @file rp4tdm.c
 * @author Alberto Vazquez
 *
 * @brief Source file rp4tdm.c, C code for RP4DT-M ExpressLRS 2.4GHz True
 * Diversity Receiver
 *
 * @version 2.0.0
 * @date 2026-08-31
 */

/**
 * @addtogroup main
 * @{
 */
//------------------------ INCLUDES ------------------------------------------
#include "uart.h"
#include "rp4tdm.h"

//------------------------------ VARIABLES -----------------------------------

uart_rx_context_t rx_ctx = {
        .state = SYNC_0,
        .index = 0,
        .timeout_counter = 0
};

uart_frame_manager_t frame_mgr = {
        .buffer_channels_a.complete = 0,
        .buffer_channels_b.complete = 0,
        .buffer_channels_a.frame = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        .buffer_channels_b.frame = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        .p_write_channels = &frame_mgr.buffer_channels_a,
        .p_read_channels = &frame_mgr.buffer_channels_b,
        .buffer_statistics_a.complete = 0,
        .buffer_statistics_b.complete = 0,
        .buffer_statistics_a.frame = {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        .buffer_statistics_b.frame = {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        .p_write_statistics = &frame_mgr.buffer_statistics_a,
        .p_read_statistics = &frame_mgr.buffer_statistics_b,
        .frame_errors = 0
};

//------------------------- FUNCTION PROTOTYPES ------------------------------
static RP4TDM_Switch_Status_t parse_switch(uint16_t val);


//--------------------------- IMPLEMENTATION ---------------------------------
/**
 * @brief Function to initialize the RP4TDM module with a specified baud rate
 * and a callback function for data reception.
 * @param baudrate The baud rate for UART communication.
 * @param callback A pointer to a callback function that will be called when data is received.
 * @return RP4TDM_Status_t Returns RP4TDM_OK if initialization is successful.
 */
RP4TDM_Status_t RP4TDM_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    uart4_init(baudrate,callback);
    return RP4TDM_OK;
}

/**
 * @brief Function to validate CRC8 calculated
 * @param frame Pointer to the frame to be validated
 * @return true if CRC8 is valid, false otherwise
 * @note This function works for channels frame
 */
bool RP4TDM_crsf_crc8_channel(uint8_t *frame)
{
    uint8_t crc = 0;
    uint8_t i;
    uint8_t j;
        for(i = 2; i < RP4TDM_SIZE-1; i++)
        {
            crc^= frame[i];//frame->p_read->frame[i];
            for(j = 0; j < 8; j++)
            {
                if(crc & 0x80)
                {
                    crc = (crc << 1) ^ 0xD5;
                } else {
                    crc <<= 1;
                }
            }
        }
        // Validate CRC8 calculated
        if(crc == frame[RP4TDM_SIZE-1]) {
            return 0;
        } else {
            return 1;
        }
}

/**
 *  @brief Function to process channels frame
 * @param buff Pointer to the frame manager structure
 * @param rp4tdmdata Pointer to the RP4TDM data structure
 * @return true if processing is error, false if processing is successful
 * @note This function works for channels frame
 */
bool RP4TDM_channels_process(uart_frame_manager_t *buff, RP4TDM_datas_t *rp4tdmdata)
{

    bool stat;
    // Variable to get parsed variables from transmitter
    uint16_t raw[16];

    // Where to save frame
    uint8_t *frames;

    // Check frame
    // Get channels frame
    frames = buff->p_read_channels->frame;
    // Check whether there is communication at startup
    if(frames[0] == 0x00 && frames[1] == 0x00){
        rp4tdmdata->status_channels = RP4TDM_NO_INIT_DATA;
        return 1;
    }

    // Check frame quality
    stat = RP4TDM_crsf_crc8_channel(frames);
    if(stat){
        rp4tdmdata->status_channels = RP4TDM_CRC8_ERROR;
        return 1;
    }

    // Parse channels frame from transmitter values
    raw[0] =  (frames[3]         | frames[4] << 8) & 0x07FF;
    raw[1] =  (frames[4]  >> 3   | frames[5] << 5) & 0x07FF;
    raw[2] =  (frames[5]  >> 6   | frames[6] << 2    | frames[7] << 10) & 0x07FF;
    raw[3] =  (frames[7]  >> 1   | frames[8] << 7) & 0x07FF;
    raw[4] =  (frames[8]  >> 4   | frames[9] << 4) & 0x07FF;
    raw[5] =  (frames[9]  >> 7   | frames[10] << 1   | frames[11] << 9) & 0x07FF;
    raw[6] =  (frames[11] >> 2   | frames[12] << 6) & 0x07FF;
    raw[7] =  (frames[12] >> 5   | frames[13] << 3) & 0x07FF;
    raw[8] =  (frames[14]        | frames[15] << 8) & 0x07FF;
    raw[9] =  (frames[15] >> 3   | frames[16] << 5) & 0x07FF;
    raw[10] = (frames[16] >> 6   | frames[17] << 2   | frames[18] << 10) & 0x07FF;
    raw[11] = (frames[18] >> 1   | frames[19] << 7) & 0x07FF;
    raw[12] = (frames[19] >> 4   | frames[20] << 4) & 0x07FF;
    raw[13] = (frames[20] >> 7   | frames[21] << 1   | frames[22] << 9) & 0x07FF;
    raw[14] = (frames[22] >> 2   | frames[23] << 6) & 0x07FF;
    raw[15] = (frames[23] >> 5   | frames[24] << 3) & 0x07FF;

    //rp4tdmdata.channels.Ailerons    = (float32_t)((((raw[0]) - 172U)*20) / 1638.0f) -10;    // (val - MIN) / (MAX-MIN) to get [-10,10]
    rp4tdmdata->channels.Ailerons    = (float32_t)((((raw[0]) - 172U)*2) / 1638.0f) -1;      // (val - MIN) / (MAX-MIN) to get [-10,10]
    rp4tdmdata->channels.Elevators   = (float32_t)((((raw[1]) - 172U)*2) / 1638.0f) -1;      // (val - MIN) / (MAX-MIN)
    rp4tdmdata->channels.Throttle    = (float32_t)((((raw[2]) - 172U)*2) / 1638.0f) -1;      // (val - MIN) / (MAX-MIN)
    rp4tdmdata->channels.Rudder      = (float32_t)((((raw[3]) - 172U)*2) / 1638.0f) -1;      // (val - MIN) / (MAX-MIN)
    rp4tdmdata->channels.SE          = parse_switch(raw[4]);
    rp4tdmdata->channels.SA          = parse_switch(raw[5]);
    rp4tdmdata->channels.SB          = parse_switch(raw[6]);
    rp4tdmdata->channels.SC          = parse_switch(raw[7]);
    rp4tdmdata->channels.SD          = parse_switch(raw[8]);
    rp4tdmdata->channels.SF          = parse_switch(raw[9]);
    rp4tdmdata->channels.S1          = (float32_t)((((raw[10]) - 172U)*2) / 1638.0f) -1;     // (val - MIN) / (MAX-MIN)
    rp4tdmdata->channels.S2          = (float32_t)((((raw[11]) - 172U)*2) / 1638.0f) -1;     // (val - MIN) / (MAX-MIN)
    rp4tdmdata->channels.SW1         = parse_switch(raw[12]);
    rp4tdmdata->channels.SW2         = parse_switch(raw[13]);
    rp4tdmdata->channels.TitX        = (float32_t)((((raw[14]) - 172U)*2) / 1638.0f) -1;     // (val - MIN) / (MAX-MIN)
    rp4tdmdata->channels.TitY        = (float32_t)((((raw[15]) - 172U)*2) / 1638.0f) -1;     // (val - MIN) / (MAX-MIN)



    // Move parsed values to data structure
    //memcpy(rp4tdmdata.d1,raw, sizeof(raw));

    // return it's all fine
    rp4tdmdata->status_channels = RP4TDM_OK;
    return 0;
}

/**
 * @brief Function to parse switch values from raw data
 * @param val The raw value of the switch
 * @return RP4TDM_Switch_Status_t The parsed switch status (SWITCH_HIGH, SWITCH_MIDDLE, or SWITCH_LOW)
 * @note This function is used to interpret the raw switch values received from the transmitter.
 */
static RP4TDM_Switch_Status_t parse_switch(uint16_t val)
{
    if(val < 500U) return SWITCH_HIGH;
    else if(val >1500U) return SWITCH_LOW;
    else return SWITCH_MIDDLE;
}
/**
 * @brief Function to validate the CRC8 of the statistics frame
 * @note This function is used to validate the CRC8 checksum of the statistics frame
 * received from the transmitter.
 * @param frame Pointer to the statistics frame
 * @return true if CRC8 is valid, false otherwise
 */
bool RP4TDM_crsf_crc8_statistics(uint8_t *frame)
{
    uint8_t crc = 0;
    uint8_t i;
    uint8_t j;
    for(i = 2; i < RP4TDM_STATISTICS_SIZE-1; i++)
    {
        crc^= frame[i];//frame->p_read->frame[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x80)
            {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    // Validate CRC8 calculated
    if(crc == frame[RP4TDM_STATISTICS_SIZE-1]) {
        return 0;
    } else {
        return 1;
    }
}
/**
 * @brief Function to process statistics frame from the transmitter
 * @note This function is used to parse the statistics frame from the
 * transmitter and store the parsed values in a data structure.
 * @param buff Pointer to the UART frame manager
 * @param rp4tdmdata Pointer to the RP4TDM data structure
 * @return true if processing is successful, false otherwise
 */
bool RP4TDM_statistics_process(uart_frame_manager_t *buff, RP4TDM_datas_t *rp4tdmdata)
{

    bool stat;
    // Variable to get parsed variables from transmitter
    //uint16_t raw[16];

    // Where to save frame
    uint8_t *frames;
    // Get statistics frame
       frames = buff->p_read_statistics->frame;
       // Check whether there is communication at startup
       if(frames[0] == 0x00 && frames[1] == 0x00){
           rp4tdmdata->status_statistics = RP4TDM_NO_STATISTICS;
           return 1;
       }

       // Check frame quality
       stat = RP4TDM_crsf_crc8_statistics(frames);
       if(stat){
           rp4tdmdata->status_statistics = RP4TDM_CRC8_ERROR;
           return 1;
       }


       // Parse statistics frame from transmitter values
       /*raw[0] = frames[3];     // Uplink RSSI 1 [dBm]
       raw[1] = frames[4];     // Uplink RSSI 2 [dBm]
       raw[2] = frames[5];     // Uplink Link Quality [%]
       raw[3] = frames[6];     // Uplink SNR [noise-signal ration]
       raw[4] = frames[7];     // Active Antenna: 0, Antenna 1(A); i, Antenna 2(B)
       raw[5] = frames[8];     // RF Mode: MODE 4 (100Hz full)
       raw[6] = frames[9];     // Emitter TX power (index 3 means 50mW)
       raw[7] = frames[10];    // Downlink RSSI [dBm]
       raw[8] = frames[11];    // Download LQ (%)
       raw[9] = frames[12];    // Downlink SNR [dBm]*/

       rp4tdmdata->statistics.rssi_dbm_ant1 = (-1) * frames[3];                         // Uplink RSSI 1 [dBm]
       rp4tdmdata->statistics.rssi_dbm_ant2 = (-1) * frames[4];                         // Uplink RSSI 2 [dBm]
       rp4tdmdata->statistics.link_quality = frames[5];                                 // Uplink Link Quality [%]
       rp4tdmdata->statistics.snr_db = (int8_t)frames[6];                               // Uplink SNR [noise-signal ration]
       rp4tdmdata->statistics.active_antenna = (RP4TDM_Antenna_t)frames[7];             // Active Antenna: 0, Antenna 1(A); i, Antenna 2(B)
       rp4tdmdata->statistics.tx_power_mw = (RP4TDM_Power_Communication_t)frames[9];    // Emitter TX power (index 3 means 50mW)


       // Move parsed values to data structure
       //memcpy(rp4tdmdata.d2,raw, sizeof(raw));

       rp4tdmdata->status_statistics = RP4TDM_OK;
       return 0;
}

/**
 * @brief Function to process data from the transmitter
 *
 * @note Call this function frequently so a complete ISR buffer is consumed
 * before a newer receiver frame can replace it.
 */
void RP4TDM_process(void)
{

    if(frame_mgr.p_read_statistics->complete)
    {
        RP4TDM_statistics_process(&frame_mgr, &rp4tdm_data);
        frame_mgr.p_read_statistics->complete = 0;
        if(rp4tdm_data.status_statistics == RP4TDM_OK)
        {
            rp4tdm_data.index_statistics = 0;
        }
    }
    if(frame_mgr.p_read_channels->complete)
    {
        RP4TDM_channels_process(&frame_mgr, &rp4tdm_data);
        frame_mgr.p_read_channels->complete = 0;
        if(rp4tdm_data.status_channels == RP4TDM_OK)
        {
            rp4tdm_data.index_channel = 0;
        }
    }

}
/**
 * @brief Advance receiver link-age counters at a fixed 100 Hz rate.
 *
 * Separating timeout aging from frame consumption allows RP4TDM_process() to
 * run on every main-loop pass without changing the real-time link timeout.
 */
void RP4TDM_timeout_tick(void)
{
    // Check if new data exist else set timeout
    if(rp4tdm_data.status_statistics != RP4TDM_NO_STATISTICS) {
        if(rp4tdm_data.index_statistics < RP4TDM_TIMEOUT_FRAME) {
            rp4tdm_data.index_statistics++;
        } else {
            rp4tdm_data.status_statistics = RP4TDM_TIMEOUT;
        }
    }
    if(rp4tdm_data.status_channels != RP4TDM_NO_INIT_DATA) {
        if(rp4tdm_data.index_channel < RP4TDM_TIMEOUT_FRAME) {
            rp4tdm_data.index_channel++;
        } else {
            rp4tdm_data.status_channels = RP4TDM_TIMEOUT;
        }
    }
}


/**
 * @brief Callback function for handling incoming data
 *
 * @note This function is called when data is received from the transmitter
 * @param data The received byte of data
 */
void RP4TDM_callback(uint8_t data)
{
    switch(rx_ctx.state)
    {
    case SYNC_0:
        if(data == 0xC8) // 0xC8
        {
            frame_mgr.p_write_statistics->frame[0] = frame_mgr.p_write_channels->frame[0] = data; // First byte storage
            rx_ctx.index = 1; // Move next index
            rx_ctx.state = SYNC_1;
            rx_ctx.timeout_counter = 0;
        }
        break;
    case SYNC_1:
        if(data == 0x18)    // Channels frame
        {
            frame_mgr.p_write_channels->frame[1] = data; // First byte storage
            rx_ctx.index = 2; // Move next index
            rx_ctx.state = CHANNELS_PAYLOAD;
        }
        else if(data == 0x0C)   // Statistics frame
        {
            frame_mgr.p_write_statistics->frame[1] = data; // First byte storage
            rx_ctx.index = 2; // Move next index
            rx_ctx.state = STATISTICS_PAYLOAD;
        }
        else
        {
            rx_ctx.state = SYNC_0;
        }
        break;
    case CHANNELS_PAYLOAD:
        frame_mgr.p_write_channels->frame[rx_ctx.index++] = data;    // Storage channels frame
        if(rx_ctx.index >= RP4TDM_SIZE)
        {
            //frame_mgr.p_write_channels->complete = 0;
            //Interchange buffers
            if(frame_mgr.p_write_channels == &frame_mgr.buffer_channels_a)
            {
                frame_mgr.p_write_channels = &frame_mgr.buffer_channels_b;
                frame_mgr.p_read_channels = &frame_mgr.buffer_channels_a;
            } else {
                frame_mgr.p_write_channels = &frame_mgr.buffer_channels_a;
                frame_mgr.p_read_channels = &frame_mgr.buffer_channels_b;
            }
            frame_mgr.p_read_channels->complete = 1;


            //RP4TDM_channels_process(&frame_mgr, &rp4tdm_data);
            // Reset
            rx_ctx.state = SYNC_0;
            rx_ctx.index = 0;
            rx_ctx.timeout_counter = 0;
        } else {
            rx_ctx.timeout_counter++;
            if(rx_ctx.timeout_counter > 200)
            {
                frame_mgr.frame_errors++;
                // Reset
                rx_ctx.state = SYNC_0;
                rx_ctx.index = 0;
                rx_ctx.timeout_counter = 0;
            }
        }
        break;
    case STATISTICS_PAYLOAD:
        frame_mgr.p_write_statistics->frame[rx_ctx.index++] = data; // Storage statistics frame
        if(rx_ctx.index >= RP4TDM_STATISTICS_SIZE)
        {
            //frame_mgr.p_write_statistics->complete = 0;
            // Interchange buffers
            if(frame_mgr.p_write_statistics == &frame_mgr.buffer_statistics_a)
            {
                frame_mgr.p_write_statistics = &frame_mgr.buffer_statistics_b;
                frame_mgr.p_read_statistics = &frame_mgr.buffer_statistics_a;
            } else {
                frame_mgr.p_write_statistics = &frame_mgr.buffer_statistics_a;
                frame_mgr.p_read_statistics = &frame_mgr.buffer_statistics_b;
            }
            frame_mgr.p_read_statistics->complete = 1;

            //RP4TDM_telemetry_process(&frame_mgr, &rp4tdm_data);
            // Reset
            rx_ctx.state = SYNC_0;
            rx_ctx.index = 0;
            rx_ctx.timeout_counter = 0;
        } else {
            rx_ctx.timeout_counter++;
            if(rx_ctx.timeout_counter > 200)
            {
                frame_mgr.frame_errors++;
                // Reset
                rx_ctx.state = SYNC_0;
                rx_ctx.index = 0;
                rx_ctx.timeout_counter = 0;
            }
        }
        break;
    default:
        rx_ctx.state = SYNC_0;
        break;
    }
}
