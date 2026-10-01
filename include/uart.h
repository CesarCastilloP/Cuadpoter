/**
 * @file uart.h
 * @author Alberto Vazquez
 *
 * @brief Interrupt-driven access to the eight TM4C1294NCPDT UART modules.
 *
 * Each UART instance exposes the same interface: initialization, byte/string
 * transmission, blocking reception with a finite timeout, and an optional
 * receive callback.  A timeout is expressed in driver polling iterations; a
 * value of zero performs a non-blocking read.  The flight application uses
 * UART0 for USB telemetry and UART4 for the RP4TDM/CRSF receiver.
 *
 * Shared UART interface contract for UART0 through UART7:
 * - uartN_init(baudrate, callback) configures the pins and peripheral.
 *   baudrate is the serial speed in bit/s.  callback receives one uint8_t byte
 *   whenever an RX interrupt accepts data.  The function returns nothing.
 * - UARTN_Sendbyte(data) queues one raw byte for transmission and returns
 *   nothing.  UARTN_Sendstring(str, length) sends exactly length bytes from
 *   the caller-owned buffer; length is measured in bytes.
 * - UART0_SendAvailable(data, length) is the telemetry-specific non-blocking
 *   transmitter.  It returns the number of bytes actually placed in hardware.
 * - UARTN_ReadByte(timeout) returns one byte as a non-negative int32_t, or -1
 *   if no byte arrives before timeout polling iterations.  timeout has no time
 *   unit because it counts loop iterations; zero means "check only once".
 * - UARTN_ReadBytes(buffer, length, timeout) stores at most length bytes in the
 *   supplied buffer and returns the number stored.  Both lengths are bytes.
 * - uartN_callback(cb) replaces the receive callback.  Passing NULL disables
 *   callback delivery.  It returns nothing.
 * - uartN_isr() is entered by the processor, drains received bytes, reports
 *   hardware errors, calls the callback when present, and returns nothing.
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


/**
 * Function invoked by a UART receive ISR for every accepted byte.
 *
 * @param data Raw received byte, with no protocol interpretation.
 */
typedef void (*uart_callback_t)(uint8_t data);


/**
 * @brief Callback table indexed by UART peripheral number (0 through 7).
 */
typedef struct
{
    uart_callback_t rx[8]; /**< Per-instance RX byte callbacks; NULL disables dispatch. */
}uart_cb_t;




/**
 * @brief UART0 interface used by the ICDI USB virtual serial port.
 *
 * @note PA0 (RX), PA1 (TX)
 * @note The telemetry application configures this interface at 460800 bit/s.
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
 * @brief UART1 generic byte-stream interface.
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
 * @brief UART2 generic byte-stream interface.
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
 * @brief UART3 generic byte-stream interface.
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
 * @brief UART4 receiver interface used by the RP4TDM/CRSF module.
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
 * @brief UART5 generic byte-stream interface.
 *
 * @note PC6 (RX), PC7 (TX)
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
 * @brief UART6 generic byte-stream interface.
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
 * @brief UART7 generic byte-stream interface.
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
