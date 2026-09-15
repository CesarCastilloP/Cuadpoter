/**
 * @file rp4tdm.h
 * @author Alberto Vazquez
 *
 * @brief Header file for for RP4DT-M ExpressLRS 2.4GHz True
 * Diversity Receiver
 *
 * @version 2.0.0
 * @date 2026-08-31
 */

/**
 * @addtogroup main
 * @{
 */

#ifndef INCLUDE_RP4TDM_H_
#define INCLUDE_RP4TDM_H_

//------------------------ INCLUDES ------------------------------------------
#include "functions.h"


//------------------------- DEFINES ------------------------------------------
#define RP4TDM_SIZE 26U
#define RP4TDM_STATISTICS_SIZE 14U
#define RP4TDM_TIMEOUT_FRAME 20U



//-------------------------- ENUMS -------------------------------------------
/**
 * @enum rx_state_t
 * @brief enum for RP4TDM frame state
 */
typedef enum
{
    SYNC_0 = 0,
    SYNC_1 = 1,
    CHANNELS_PAYLOAD = 2,
    STATISTICS_PAYLOAD = 3
}rx_state_t;

/**
 * @enum RP4TDM_Frame_type_r
 * @brief enum for know witch frame is.
 *
 */
typedef enum
{
    RP4TDM_STATISTIC = 0,
    RP4TDM_CHANNELS = 1,
}RP4TDM_Frame_type_r;

/**
 * @enum RP4TDM_Antenna_t
 *  @brief enum for check witch antenna is active
 *
 */
typedef enum
{
    ANTENNA_1 = 0,
    ANTENNA_2,
}RP4TDM_Antenna_t;

/**
 * @enum RP4TDM_Power_Communication_t
 * @brief enum for know emitter signal power
 *
 */
typedef enum
{
    EMITTER_10_mW = 0,
    EMITTER_25_mW = 1,
    EMITTER_50_mW = 2,
    EMITTER_100_mW = 3,
    EMITTER_250_mW = 4,
    EMITTER_500_mW = 5,
    EMITTER_1000_mW = 6,    // 1W
    EMITTER_2000_mW = 7,    // 2W
}RP4TDM_Power_Communication_t;

/**
 * @enum RP4TDM_Switch_Status_t
 * @brief enum for know switch status
 *
 */
typedef enum
{
    SWITCH_HIGH = 172U,
    SWITCH_MIDDLE = 992U,
    SWITCH_LOW = 1810U
}RP4TDM_Switch_Status_t;

/**
 * @enum RP4TDM_Status_t
 * @brief enum for know RP4TDM communication status
 *
 */
typedef enum
{
    RP4TDM_OK = 0,
    RP4TDM_ERROR = 1,
    RP4TDM_TIMEOUT = 2,
    RP4TDM_NO_INIT_DATA = 3,
    RP4TDM_NO_STATISTICS = 4,
    RP4TDM_CRC8_ERROR = 5,
}RP4TDM_Status_t;


//------------------------- STRUCTURES ---------------------------------------
/**
 * @struct uart_rx_context_t
 * @brief structure to get frame manage
 */
typedef struct
{
    rx_state_t state;
    uint16_t index;
    uint32_t timeout_counter;
}uart_rx_context_t;
extern uart_rx_context_t rx_ctx;

/**
 * @struct frame_buffer_t
 * @brief structure for RP4TD-M channels frame
 */
typedef struct
{
    uint8_t frame[RP4TDM_SIZE];
    volatile uint8_t complete;
}frame_buffer_channels_t;

/**
 * @struct frame_buffer_t
 * @brief structure for RP4TD-M STATISTICS frame
 */
typedef struct
{
    uint8_t frame[RP4TDM_STATISTICS_SIZE];
    volatile uint8_t complete;
}frame_buffer_statistics_t;

/**
 * @struct uart_frame_manager_t
 * @brief structure to manage frame
 */
typedef struct
{
    frame_buffer_channels_t buffer_channels_a;
    frame_buffer_channels_t buffer_channels_b;
    frame_buffer_channels_t *p_write_channels;    // ISR writes here
    frame_buffer_channels_t *p_read_channels;     // User reads this
    frame_buffer_statistics_t buffer_statistics_a;
    frame_buffer_statistics_t buffer_statistics_b;
    frame_buffer_statistics_t *p_write_statistics;    // ISR writes here
    frame_buffer_statistics_t *p_read_statistics;     // User reads this
    volatile uint32_t frame_errors;
}uart_frame_manager_t;
extern uart_frame_manager_t frame_mgr;

/**
 * @struct RP4TDM_Statistics_t
 * @brief structure for  check telemetry variables from emitter
 */
typedef struct
{
    int16_t rssi_dbm_ant1;  // Power in dBm (negative value)
    int16_t rssi_dbm_ant2;  // Power in dBm (negative value)
    uint8_t link_quality;   // Link Quality in % (0 to 100)
    int8_t snr_db;          // Signal-to-noise ration i n dB
    RP4TDM_Antenna_t active_antenna; // 0 for antenna 1, 1 for antenna 2
    RP4TDM_Power_Communication_t tx_power_mw;   // Power in mW
}RP4TDM_Statistics_t;

/**
 * @struct RP4TDM_Channels_t
 * @brief structure for  check telemetry variables from emitter
 *
 */
typedef struct
{
    float32_t Ailerons;     // Roll Control
    float32_t Elevators;   // Pitch Control
    float32_t Throttle;     // Power/Thrust Control
    float32_t Rudder;       // Yaw Control
    RP4TDM_Switch_Status_t SE;
    RP4TDM_Switch_Status_t SA;
    RP4TDM_Switch_Status_t SB;
    RP4TDM_Switch_Status_t SC;
    RP4TDM_Switch_Status_t SD;
    RP4TDM_Switch_Status_t SF;
    float32_t S1;
    float32_t S2;
    RP4TDM_Switch_Status_t SW1;
    RP4TDM_Switch_Status_t SW2;
    float32_t TitX;
    float32_t TitY;
}RP4TDM_Channels_t;
/**
 * @struct RP4TDM_datas_t
 * @brief structure to know data telemetry and channels
 */
typedef struct
{
    RP4TDM_Status_t status_channels;
    RP4TDM_Status_t status_statistics;
    RP4TDM_Statistics_t statistics;
    RP4TDM_Channels_t channels;
    //uint16_t d1[16];
    //uint16_t d2[10];
    uint32_t index_channel;
    uint32_t index_statistics;
}RP4TDM_datas_t;
extern RP4TDM_datas_t rp4tdm_data;

//------------------------- FUNCTION PROTOTYPES ------------------------------
RP4TDM_Status_t RP4TDM_init(uint32_t baudrate, void (*callback)(uint8_t));
void RP4TDM_callback(uint8_t data);
bool RP4TDM_crsf_crc8_channel(uint8_t *frame);
bool RP4TDM_crsf_crc8_statistics(uint8_t *frame);
bool RP4TDM_channels_process(uart_frame_manager_t *buff, RP4TDM_datas_t *rp4tdmdata);
bool RP4TDM_statistics_process(uart_frame_manager_t *buff, RP4TDM_datas_t *rp4tdmdata);
void RP4TDM_process(void);
void RP4TDM_timeout_tick(void);

#endif /* INCLUDE_RP4TDM_H_ */
