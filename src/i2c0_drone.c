/**
 * @file i2c0_drone.c
 * @author Alberto Vazquez
 *
 * @brief Blocking I2C0 master interface with bounded transaction time.
 *
 * @version 1.1.0
 * @date 2026-09-15
 */

#include "i2c0_drone.h"
#include "systick.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

/* Real-time bounds for one byte command and an error STOP operation. */
#define I2C_TRANSACTION_TIMEOUT_US       3000ULL
#define I2C_RELEASE_TIMEOUT_US           1000ULL
#define I2C_HARDWARE_CLOCK_TIMEOUT       0xFFU
#define I2C_BUS_PINS                     (GPIO_PIN_2 | GPIO_PIN_3)
#define I2C_CLOCK_PIN                    GPIO_PIN_2
#define I2C_DATA_PIN                     GPIO_PIN_3
#define I2C_RECOVERY_PULSES              9U

static uint32_t g_i2c_system_clock_hz = 0U;
static bool g_i2c_fast_mode = false;
static uint64_t g_i2c_transaction_timeout_cycles = 0ULL;
static uint64_t g_i2c_release_timeout_cycles = 0ULL;

/**
 * @brief Release a slave that was interrupted while holding the bus.
 *
 * Both lines are temporarily open-drain GPIOs. Nine clock pulses complete any
 * partial byte, followed by a STOP condition before I2C0 takes ownership.
 */
static bool i2c0_recover_bus(void)
{
    uint32_t pulse;

    GPIOPinTypeGPIOOutputOD(GPIO_PORTB_BASE, I2C_BUS_PINS);
    GPIOPadConfigSet(GPIO_PORTB_BASE, I2C_BUS_PINS,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
    GPIOPinWrite(GPIO_PORTB_BASE, I2C_BUS_PINS, I2C_BUS_PINS);
    SysCtlDelay(1200U);

    if((GPIOPinRead(GPIO_PORTB_BASE, I2C_CLOCK_PIN) & I2C_CLOCK_PIN) == 0U)
    {
        return false;
    }

    for(pulse = 0U; pulse < I2C_RECOVERY_PULSES; pulse++)
    {
        if((GPIOPinRead(GPIO_PORTB_BASE, I2C_DATA_PIN) & I2C_DATA_PIN) != 0U)
        {
            break;
        }
        GPIOPinWrite(GPIO_PORTB_BASE, I2C_CLOCK_PIN, 0U);
        SysCtlDelay(1200U);
        GPIOPinWrite(GPIO_PORTB_BASE, I2C_CLOCK_PIN, I2C_CLOCK_PIN);
        SysCtlDelay(1200U);
    }

    /* Generate STOP: SDA rises while SCL is released high. */
    GPIOPinWrite(GPIO_PORTB_BASE, I2C_DATA_PIN, 0U);
    SysCtlDelay(1200U);
    GPIOPinWrite(GPIO_PORTB_BASE, I2C_CLOCK_PIN, I2C_CLOCK_PIN);
    SysCtlDelay(1200U);
    GPIOPinWrite(GPIO_PORTB_BASE, I2C_DATA_PIN, I2C_DATA_PIN);
    SysCtlDelay(1200U);

    return (GPIOPinRead(GPIO_PORTB_BASE, I2C_BUS_PINS) & I2C_BUS_PINS) ==
           I2C_BUS_PINS;
}

/**
 * @brief Recover the bus and restore the I2C0 pin/controller configuration.
 */
static bool i2c0_reset_controller(void)
{
    bool bus_recovered = i2c0_recover_bus();

    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    {
    }

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, I2C_CLOCK_PIN);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, I2C_DATA_PIN);
    GPIOPadConfigSet(GPIO_PORTB_BASE, I2C_DATA_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    I2CMasterInitExpClk(I2C0_BASE, g_i2c_system_clock_hz,
                        g_i2c_fast_mode);
    I2CMasterTimeoutSet(I2C0_BASE, I2C_HARDWARE_CLOCK_TIMEOUT);
    I2CMasterEnable(I2C0_BASE);
    return bus_recovered;
}

/**
 * @brief Request an error STOP and allow the controller to release the bus.
 */
static bool i2c0_release_bus(uint32_t error_stop_command)
{
    uint64_t start_cycles = Timebase_GetCycles();

    I2CMasterControl(I2C0_BASE, error_stop_command);
    SysCtlDelay(10U);
    while(I2CMasterBusy(I2C0_BASE) || I2CMasterBusBusy(I2C0_BASE))
    {
        if((Timebase_GetCycles() - start_cycles) >=
           g_i2c_release_timeout_cycles)
        {
            (void)i2c0_reset_controller();
            return false;
        }
    }
    return true;
}

/**
 * @brief Wait for the active command and translate controller errors.
 *
 * A short delay is required because the TM4C master busy flag may not assert
 * immediately after a command is issued. error_stop_command must match the
 * direction of the active burst so the bus is released after any failure.
 */
static I2C0_Status_t i2c0_wait_done(uint32_t error_stop_command)
{
    uint64_t start_cycles;
    uint32_t error;

    SysCtlDelay(500U);
    start_cycles = Timebase_GetCycles();

    while(I2CMasterBusy(I2C0_BASE))
    {
        if((Timebase_GetCycles() - start_cycles) >=
           g_i2c_transaction_timeout_cycles)
        {
            i2c0_release_bus(error_stop_command);
            return I2C_ERR_TIMEOUT;
        }
    }

    error = I2CMasterErr(I2C0_BASE);
    if(error == I2C_MASTER_ERR_NONE)
    {
        return I2C_OK;
    }

    if(!i2c0_release_bus(error_stop_command))
    {
        return I2C_ERR_TIMEOUT;
    }

    if((error & I2C_MASTER_ERR_CLK_TOUT) != 0U)
    {
        return I2C_ERR_TIMEOUT;
    }
    if((error & I2C_MASTER_ERR_ARB_LOST) != 0U)
    {
        return I2C_ERR_ARB;
    }
    if((error & I2C_MASTER_ERR_ADDR_ACK) != 0U)
    {
        return I2C_ERR_ADDR;
    }
    return I2C_ERR_DATA;
}

/**
 * @brief Initialize I2C0 using the clock frequency actually configured.
 */
I2C0_Status_t I2C0_Init(uint32_t system_clock_hz, I2C_Freq_t freq)
{
    if((system_clock_hz == 0U) ||
       ((freq != I2C_100) && (freq != I2C_400)))
    {
        return I2C_ERR_INVALID_ARG;
    }

    g_i2c_system_clock_hz = system_clock_hz;
    g_i2c_fast_mode = freq == I2C_400;
    g_i2c_transaction_timeout_cycles =
        ((uint64_t)system_clock_hz * I2C_TRANSACTION_TIMEOUT_US) /
        1000000ULL;
    g_i2c_release_timeout_cycles =
        ((uint64_t)system_clock_hz * I2C_RELEASE_TIMEOUT_US) /
        1000000ULL;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }

    return i2c0_reset_controller() ? I2C_OK : I2C_ERR_TIMEOUT;
}

/**
 * @brief Write a register address, then read one or more sequential bytes.
 */
I2C0_Status_t I2C0_WriteRead(uint8_t address,
                             uint8_t register_address,
                             uint8_t *read_buffer,
                             uint32_t read_length)
{
    uint32_t index;
    I2C0_Status_t status;

    if((g_i2c_system_clock_hz == 0U) || (address > 0x7FU) ||
       (read_buffer == NULL) || (read_length == 0U))
    {
        return I2C_ERR_INVALID_ARG;
    }

    I2CMasterSlaveAddrSet(I2C0_BASE, address, false);
    I2CMasterDataPut(I2C0_BASE, register_address);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    status = i2c0_wait_done(I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    if(status != I2C_OK)
    {
        return status;
    }

    I2CMasterSlaveAddrSet(I2C0_BASE, address, true);
    if(read_length == 1U)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        status = i2c0_wait_done(I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
        if(status != I2C_OK)
        {
            return status;
        }
        read_buffer[0] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
        return I2C_OK;
    }

    for(index = 0U; index < read_length; index++)
    {
        if(index == 0U)
        {
            I2CMasterControl(I2C0_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
        else if(index == (read_length - 1U))
        {
            I2CMasterControl(I2C0_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        }
        else
        {
            I2CMasterControl(I2C0_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        }

        status = i2c0_wait_done(I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
        if(status != I2C_OK)
        {
            return status;
        }
        read_buffer[index] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
    }

    return I2C_OK;
}

/**
 * @brief Write one byte to a device register.
 */
I2C0_Status_t I2C0_WriteWrite(uint8_t address,
                              uint8_t register_address,
                              uint8_t data)
{
    I2C0_Status_t status;

    if((g_i2c_system_clock_hz == 0U) || (address > 0x7FU))
    {
        return I2C_ERR_INVALID_ARG;
    }

    I2CMasterSlaveAddrSet(I2C0_BASE, address, false);
    I2CMasterDataPut(I2C0_BASE, register_address);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    status = i2c0_wait_done(I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    if(status != I2C_OK)
    {
        return status;
    }

    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    return i2c0_wait_done(I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
}
