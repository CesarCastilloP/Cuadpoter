/**
 * @file uart.h
 * @author Alberto Vazquez
 *
 * @brief header file uart, h code for UART module
 *
 * @version 1.1.0
 * @date 2026-09-27
 */

/**
 * @addtogroup main
 * @{
 */

#ifndef INCLUDE_UART_H_
#define INCLUDE_UART_H_


#include "stdint.h"
#include "stdbool.h"


//--------------------- CALLBACK FOR INTERRUPTS (ISRs) -------------------------
typedef void (*uart_callback_t)(uint8_t data);


/**
 * @brief Structure for UART3 data
 */
typedef struct
{
    uart_callback_t rx[8]; // Pointer to function
}uart_cb_t;




/**
 * @brief UART0 prototype functions
 *
 * @note PA0 (RX), PA1 (TX)
 *
 **/
void uart0_isr(void);
void uart0_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART0_Sendbyte(uint8_t data);
void UART0_Sendstring(const uint8_t* str, uint16_t length);
uint32_t UART0_SendAvailable(const uint8_t* data, uint32_t length);
int32_t UART0_ReadByte(uint32_t timeout);
uint32_t UART0_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart0_callback(uart_callback_t cb);

/**
 * @brief UART1 prototype functions
 *
 * @note PB0 (RX), PB1 (TX)
 *
 **/
void uart1_isr(void);
void uart1_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART1_Sendbyte(uint8_t data);
int32_t UART1_ReadByte(uint32_t timeout);
uint32_t UART1_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void UART1_Sendstring(const uint8_t* str, uint16_t length);
void uart1_callback(uart_callback_t cb);

/**
 * @brief UART2 prototype functions
 *
 * @note PA6 (RX), PA7 (TX)
 *
 **/
void uart2_isr(void);
void uart2_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART2_Sendbyte(uint8_t data);
void UART2_Sendstring(const uint8_t* str, uint16_t length);
int32_t UART2_ReadByte(uint32_t timeout);
uint32_t UART2_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart2_callback(uart_callback_t cb);

/**
 * @brief UART3 prototype functions
 *
 * @note PA4 (RX), PA5 (TX)
 *
 **/
void uart3_isr(void);
void uart3_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART3_Sendbyte(uint8_t data);
void UART3_Sendstring(const uint8_t* str, uint16_t length);
int32_t UART3_ReadByte(uint32_t timeout);
uint32_t UART3_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart3_callback(uart_callback_t cb);

/**
 * @brief UART4 prototype functions
 *
 * @note PA2 (RX), PA3 (TX)
 *
 **/
void uart4_isr(void);
void uart4_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART4_Sendbyte(uint8_t data);
void UART4_Sendstring(const uint8_t* str, uint16_t length);
int32_t UART4_ReadByte(uint32_t timeout);
uint32_t UART4_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart4_callback(uart_callback_t cb);



/**
 * @brief UART5 prototype functions
 *
 * @note PC5 (RX), PC7 (TX)
 *
 **/
void uart5_isr(void);
void uart5_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART5_Sendbyte(uint8_t data);
void UART5_Sendstring(const uint8_t* str, uint16_t length);
int32_t UART5_ReadByte(uint32_t timeout);
uint32_t UART5_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart5_callback(uart_callback_t cb);


/**
 * @brief UART6 prototype functions
 *
 * @note PP0 (RX), PP1 (TX)
 *
 **/
void uart6_isr(void);
void uart6_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART6_Sendbyte(uint8_t data);
void UART6_Sendstring(const uint8_t* str, uint16_t length);
int32_t UART6_ReadByte(uint32_t timeout);
uint32_t UART6_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart6_callback(uart_callback_t cb);


/**
 * @brief UART7 prototype functions
 *
 * @note PC4 (RX), PC5 (TX)
 *
 **/
void uart7_isr(void);
void uart7_init(uint32_t baudrate, void (*callback)(uint8_t));
void UART7_Sendbyte(uint8_t data);
void UART7_Sendstring(const uint8_t* str, uint16_t length);
int32_t UART7_ReadByte(uint32_t timeout);
uint32_t UART7_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout);
void uart7_callback(uart_callback_t cb);



#endif /* INCLUDE_UART_H_ */
