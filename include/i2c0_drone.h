/**
 * @file i2c0_drone.h
 * @author Alberto Vazquez
 *
 * @brief Blocking I2C0 master interface with bounded transaction time.
 *
 * @details One I2C0 peripheral is shared by LSM6DSR, LIS2MDL, and BMP390L.
 * main() initializes it once at 400 kbit/s. Sensor drivers reuse WriteRead()
 * for register-address-plus-read transactions and WriteWrite() for one-byte
 * register writes. Every wait has a finite timeout and every function returns
 * an ACK/arbitration/data/timeout result, so a missing slave cannot freeze the
 * cooperative scheduler or be confused with a valid register value.
 *
 * @version 1.1.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_I2C0_DRONE_H_
#define INCLUDE_I2C0_DRONE_H_

#include "functions.h"

typedef enum
{
    /** Standard-mode I2C bus at 100 kilobits/second. */
    I2C_100 = 0,
    /** Fast-mode I2C bus at 400 kilobits/second. */
    I2C_400 = 1
} I2C_Freq_t;

typedef enum
{
    /** Transaction completed and every addressed byte was acknowledged. */
    I2C_OK = 0,
    /** TM4C master lost arbitration to another bus master. */
    I2C_ERR_ARB,
    /** Slave did not acknowledge its 7-bit address. */
    I2C_ERR_ADDR,
    /** Slave did not acknowledge data or the controller reported a data error. */
    I2C_ERR_DATA,
    /** Hardware remained busy beyond the bounded microsecond timeout. */
    I2C_ERR_TIMEOUT,
    /** Address, buffer, length, clock, or frequency argument was invalid. */
    I2C_ERR_INVALID_ARG
} I2C0_Status_t;

/**
 * Configure I2C0 on PB2/SCL and PB3/SDA.
 * @param system_clock_hz Current TM4C system clock in hertz.
 * @param freq Requested 100 kHz or 400 kHz bus mode.
 * @return I2C_OK after controller and physical bus recovery succeed.
 */
I2C0_Status_t I2C0_Init(uint32_t system_clock_hz, I2C_Freq_t freq);
/**
 * Write an 8-bit register address, issue a repeated START, and read bytes.
 * @param address 7-bit slave address without the R/W bit.
 * @param register_address First 8-bit register to read.
 * @param read_buffer Destination for read_length bytes.
 * @param read_length Number of bytes; must be greater than zero.
 * @return I2C_OK or the exact argument, ACK, arbitration, data, or timeout error.
 */
I2C0_Status_t I2C0_WriteRead(uint8_t address,
                             uint8_t register_address,
                             uint8_t *read_buffer,
                             uint32_t read_length);
/**
 * Write one 8-bit value to one 8-bit slave register.
 * @param address 7-bit slave address without the R/W bit.
 * @param register_address Destination register address.
 * @param data Register value transmitted after the address byte.
 * @return I2C_OK or the exact argument, ACK, arbitration, data, or timeout error.
 */
I2C0_Status_t I2C0_WriteWrite(uint8_t address,
                              uint8_t register_address,
                              uint8_t data);

#endif /* INCLUDE_I2C0_DRONE_H_ */
