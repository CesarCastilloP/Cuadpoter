/**
 * @file i2c0_drone.c
 * @author Alberto Vazquez
 *
 * @brief Blocking I2C0 master interface with bounded transaction time.
 *
 * @details Engineering execution overview:
 * - I2C0_Init() assigns PB2 to clock and PB3 to bidirectional data, then chooses
 *   the requested bus frequency from the real system clock.
 * - WriteRead sends a register address and receives sequential bytes. WriteWrite
 *   sends a register address followed by one value. Both report ACK/bus errors.
 * - Every busy wait has a TIMER7-based timeout. A failed transaction issues STOP,
 *   resets the controller, and if necessary clocks the pins manually to release
 *   a slave that is holding SDA low.
 * - Device addresses are seven-bit I2C addresses; byte counts have no unit;
 *   timeout variables ending in _cycles use 120 MHz system-clock cycles.
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

/* Maximum duration allowed for one master command, in microseconds. */
#define I2C_TRANSACTION_TIMEOUT_US       3000ULL
/* Maximum duration allowed for an error STOP, in microseconds. */
#define I2C_RELEASE_TIMEOUT_US           1000ULL
/* TM4C hardware SCL-low timeout register value. */
#define I2C_HARDWARE_CLOCK_TIMEOUT       0xFFU
/* PB2/SCL and PB3/SDA pin mask used during GPIO-level bus recovery. */
#define I2C_BUS_PINS                     (GPIO_PIN_2 | GPIO_PIN_3)
#define I2C_CLOCK_PIN                    GPIO_PIN_2 /* I2C0 SCL on PB2. */
#define I2C_DATA_PIN                     GPIO_PIN_3 /* I2C0 SDA on PB3. */
/* Nine SCL pulses allow a slave to finish any interrupted data byte. */
#define I2C_RECOVERY_PULSES              9U

/* Processor clock in hertz; zero marks I2C0 as uninitialized. */
static uint32_t g_i2c_system_clock_hz = 0U;
/* false selects 100 kbit/s and true selects 400 kbit/s in DriverLib. */
static bool g_i2c_fast_mode = false;
/* Software command timeout expressed in processor cycles. */
static uint64_t g_i2c_transaction_timeout_cycles = 0ULL;
/* Error-STOP timeout expressed in processor cycles. */
static uint64_t g_i2c_release_timeout_cycles = 0ULL;

/**
 * @brief Release a slave that was interrupted while holding the bus.
 *
 * Both lines are temporarily open-drain GPIOs. Nine clock pulses complete any
 * partial byte, followed by a STOP condition before I2C0 takes ownership.
 * @return true when both SCL and SDA are released high after recovery.
 */
static bool i2c0_recover_bus(void)
{
    uint32_t pulse; /* Recovery clock-pulse index, from zero through eight. */

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
 * @return Result of the physical GPIO bus-release procedure.
 */
static bool i2c0_reset_controller(void)
{
    /* Preserve the physical recovery result while restoring peripheral mode. */
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
 * @param error_stop_command Direction-appropriate DriverLib error-STOP command.
 * @return true when the controller and bus become idle before the timeout.
 */
static bool i2c0_release_bus(uint32_t error_stop_command)
{
    /* Cycle timestamp used for a wrap-safe elapsed-time comparison. */
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
 * @param error_stop_command Direction-appropriate DriverLib error-STOP command.
 * @return I2C_OK or the exact timeout, arbitration, address-ACK, or data error.
 */
static I2C0_Status_t i2c0_wait_done(uint32_t error_stop_command)
{
    uint64_t start_cycles; /* Start of this command wait, in processor cycles. */
    uint32_t error;        /* DriverLib I2C_MASTER_ERR_* bit mask. */

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
 * @param system_clock_hz Actual system clock [Hz].
 * @param freq Requested standard mode (100 kHz) or fast mode (400 kHz).
 * @return I2C_OK after pins/controller and idle bus are ready, otherwise an error.
 */
I2C0_Status_t I2C0_Init(uint32_t system_clock_hz, I2C_Freq_t freq)
{
    if((system_clock_hz == 0U) ||
       ((freq != I2C_100) && (freq != I2C_400)))
    {
        return I2C_ERR_INVALID_ARG;
    }

    /* Cache the clock and convert wall-time limits once during initialization. */
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
 * @param address Seven-bit slave address [0x00,0x7F].
 * @param register_address First slave register address.
 * @param read_buffer Caller-owned destination byte array.
 * @param read_length Number of bytes to receive; must be greater than zero.
 * @return I2C_OK only after every byte is acknowledged and stored.
 */
I2C0_Status_t I2C0_WriteRead(uint8_t address,
                             uint8_t register_address,
                             uint8_t *read_buffer,
                             uint32_t read_length)
{
    uint32_t index;       /* Destination byte index for multi-byte reads. */
    I2C0_Status_t status; /* Result of the most recently issued bus command. */

    if((g_i2c_system_clock_hz == 0U) || (address > 0x7FU) ||
       (read_buffer == NULL) || (read_length == 0U))
    {
        return I2C_ERR_INVALID_ARG;
    }

    /* Write the register pointer without STOP so the following read is atomic. */
    I2CMasterSlaveAddrSet(I2C0_BASE, address, false);
    I2CMasterDataPut(I2C0_BASE, register_address);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    status = i2c0_wait_done(I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    if(status != I2C_OK)
    {
        return status;
    }

    /* Repeated-start the same slave in read direction. */
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

    /* Select START, CONTINUE, and FINISH commands by byte position. */
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
 * @param address Seven-bit slave address [0x00,0x7F].
 * @param register_address Destination slave register.
 * @param data Eight-bit register value.
 * @return I2C_OK only when address, register, and data phases are acknowledged.
 */
I2C0_Status_t I2C0_WriteWrite(uint8_t address,
                              uint8_t register_address,
                              uint8_t data)
{
    /* Result of transmitting the register-address byte. */
    I2C0_Status_t status;

    if((g_i2c_system_clock_hz == 0U) || (address > 0x7FU))
    {
        return I2C_ERR_INVALID_ARG;
    }

    /* First byte selects the destination register; second byte carries its value. */
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
