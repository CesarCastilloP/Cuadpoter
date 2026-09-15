/**
 * @file i2c0_drone.h
 * @author Alberto Vazquez
 *
 * @brief Blocking I2C0 master interface with bounded transaction time.
 *
 * @version 1.1.0
 * @date 2026-09-15
 */

#ifndef INCLUDE_I2C0_DRONE_H_
#define INCLUDE_I2C0_DRONE_H_

#include "functions.h"

typedef enum
{
    I2C_100 = 0,
    I2C_400 = 1
} I2C_Freq_t;

typedef enum
{
    I2C_OK = 0,
    I2C_ERR_ARB,
    I2C_ERR_ADDR,
    I2C_ERR_DATA,
    I2C_ERR_TIMEOUT,
    I2C_ERR_INVALID_ARG
} I2C0_Status_t;

I2C0_Status_t I2C0_Init(uint32_t system_clock_hz, I2C_Freq_t freq);
I2C0_Status_t I2C0_WriteRead(uint8_t address,
                             uint8_t register_address,
                             uint8_t *read_buffer,
                             uint32_t read_length);
I2C0_Status_t I2C0_WriteWrite(uint8_t address,
                              uint8_t register_address,
                              uint8_t data);

#endif /* INCLUDE_I2C0_DRONE_H_ */
