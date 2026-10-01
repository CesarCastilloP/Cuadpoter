/**
 * @file uart.c
 * @author Alberto Vazquez
 *
 * @brief DriverLib UART initialization, polling I/O, and interrupt callbacks.
 *
 * All eight UART blocks use the 16 MHz precision internal oscillator (PIOSC),
 * 8 data bits, no parity, and one stop bit.  Receive callbacks run directly in
 * interrupt context and therefore must remain short and non-blocking.  Polling
 * read timeouts are iteration counts rather than milliseconds.  Blocking send
 * functions are intended for setup/debug use; flight telemetry uses the UART0
 * non-blocking FIFO writer.
 *
 * Beginner's reading guide:
 * - Each numbered UART family exposes the same operations: Init configures pins
 *   and baud rate, Sendbyte/Sendstring transmit, ReadByte/ReadBytes poll input,
 *   Callback stores a receive handler, and the ISR drains received bytes.
 * - A baud rate is bits per second. All active ports use 8 data bits, no parity,
 *   one stop bit (8N1), and the 16 MHz PIOSC clock.
 * - UART4 carries CRSF receiver bytes. UART0 carries flight telemetry through the
 *   LaunchPad USB virtual COM port. UART0_SendAvailable is the non-blocking path.
 *
 * @version 1.1.0
 * @date 2026-09-27
 */

/**
 * @addtogroup main
 * @{
 */
#include "uart.h"
#include "driverlib/sysctl.h"   // Controls system clock and peripheral power /SysCtlPeripheralEnable()
#include "driverlib/gpio.h"     // Configures GPIO pins for alternate functions /GPIOPinTypeUART()
#include "driverlib/uart.h"     // Manages UART configuration and data transfer /UARTConfigSetExpClk()
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"  // Maps GPIO pins to their UART functions /GPIOPinConfigure()
#include "inc/hw_memmap.h"      // Defines base addresses of peripherals /UART0_BASE, GPIO_PORTA_BASE
#include "inc/hw_ints.h"


#ifndef NULL
#define NULL          0
#endif

/* Callback registry shared by the eight ISR entry points. */
uart_cb_t callbacks = {
      .rx = NULL
};

/* -------------------------------------------------------------------------- */
/* UART0: PA0/RX and PA1/TX, connected to the LaunchPad USB virtual COM port. */
/* -------------------------------------------------------------------------- */
/**
 * @brief Initialize UART0
 * @param baudrate Baud rate for UART0
 * @note This function configures UART0, the GPIO pins are PA0 (RX) and PA1 (TX).
 * It also enables the UART0 interrupt and registers the UART0_Handler function as the interrupt service routine.
 */
void uart0_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    /* Enable UART0 and its GPIO port before accessing either register block. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    //Disable UART0
    UARTDisable(UART0_BASE);
    //Configure pins for UART0 (PA0:RX, PA1:TX)
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,
                    GPIO_PIN_0 | GPIO_PIN_1);

    /* PIOSC is a fixed 16 MHz source independent of the 120 MHz system PLL. */
    UARTClockSourceSet(UART0_BASE,
                       UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART0_BASE,
                        16000000,
                        baudrate,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    /* Low TX threshold maximizes room for non-blocking telemetry bursts. */
    UARTFIFOLevelSet(UART0_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);
    // Set FIFO
    UARTFIFOEnable(UART0_BASE);

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[0] = callback;
        uart0_callback(callback);
        /* Register RX and receive-timeout interrupts only when callback is used. */
        UARTIntRegister(UART0_BASE, uart0_isr); //Register ISR
        UARTIntEnable(UART0_BASE,
                      UART_INT_RX | UART_INT_RT);

        /* Enable UART0 delivery through the NVIC. */
        IntEnable(INT_UART0);
        IntMasterEnable();
    } else {
        callbacks.rx[0] = NULL;
        UARTIntDisable(UART0_BASE,
                      UART_INT_RX | UART_INT_RT);

        /* Keep polling-only operation free of unused UART0 interrupts. */
        IntDisable(INT_UART0);
    }
    //Enable UART0
    UARTEnable(UART0_BASE);
}

/**
 *  @brief Send a byte through UART0
 *  @param data Byte to send
 * */
void UART0_Sendbyte(uint8_t data)
{
    UARTCharPut(UART0_BASE, data);
    UARTBusy(UART0_BASE);
}

/**
 * @brief Send a string through UART0
 * @param str String to send
 */
void UART0_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART0_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Fill the available UART0 FIFO space without waiting.
 * @return Number of bytes accepted by the hardware FIFO.
 */
uint32_t UART0_SendAvailable(const uint8_t* data, uint32_t length)
{
    uint32_t count = 0U; /* Bytes accepted before the hardware FIFO filled. */

    if(data == NULL)
    {
        return 0U;
    }

    while((count < length) &&
          UARTCharPutNonBlocking(UART0_BASE, data[count]))
    {
        count++;
    }
    return count;
}

/**
 * @brief Read a byte from UART0
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART0_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 when none arrives. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART0_BASE))
       {
           data = UARTCharGetNonBlocking(UART0_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART0
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART0_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Most recent byte or the -1 timeout sentinel. */

    while(cont < length && timeout > 0)
    {
        data = UART0_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}

/**
 * @brief UART0 Callback
 *
 */
void uart0_callback(uart_callback_t cb)
{
    callbacks.rx[0] = cb;
}

/**
 * @brief UART0 interrupt handler
 */
void uart0_isr(void)
{
    /* Masked interrupt-cause bits that must be cleared before draining RX FIFO. */
    uint32_t status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, status);

    while(UARTCharsAvail(UART0_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART0_BASE); /* Byte or -1 sentinel. */
        if(c == -1)
        {
            break;
        }
        /* Dispatch one byte at a time to the protocol parser in interrupt context. */
        if(callbacks.rx[0] != NULL)
        {
            callbacks.rx[0]((uint8_t)c);
        }
    }
}
//---------------------------------UART1---------------------------------*/
//---------------------------------UART1---------------------------------*/
//---------------------------------UART1---------------------------------*/

/**
 * @brief Initialize UART1
 * @param baudrate Baud rate for UART1
 * @note This function configures UART1, the GPIO pins are PB0 (RX) and PB1 (TX).
 * It also enables the UART1 interrupt and registers the UART1_Handler function as the interrupt service routine.
 */
void uart1_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART1 and port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    //Disable UART1
    UARTDisable(UART1_BASE);
    //Configure pins for UART1 (PB0:RX, PB1:TX)
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE,
                    GPIO_PIN_0 | GPIO_PIN_1);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART1_BASE,
                       UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART1_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Set FIFO level
    UARTFIFOLevelSet(UART1_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);
    // Set FIFO
    UARTFIFOEnable(UART1_BASE);

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[1] = callback;
        uart1_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART1_BASE, uart1_isr); //Register ISR
        UARTIntEnable(UART1_BASE,
                      UART_INT_RX | UART_INT_RT);

        //Enable interrupt in NVIC
        IntEnable(INT_UART1);
        IntMasterEnable();
    } else {
        callbacks.rx[1] = NULL;
        UARTIntDisable(UART1_BASE,
                      UART_INT_RX | UART_INT_RT);

        //Enable interrupt in NVIC
        IntDisable(INT_UART1);
    }
    //Enable UART1
    UARTEnable(UART1_BASE);
}

/**
 *  @brief Send a byte through UART1
 *  @param data Byte to send
 * */
void UART1_Sendbyte(uint8_t data)
{
    UARTCharPut(UART1_BASE, data);
    UARTBusy(UART1_BASE);
}

/**
 * @brief Send a string through UART1
 * @param str String to send
 */
void UART1_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART1_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART1
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART1_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART1_BASE))
       {
           data = UARTCharGetNonBlocking(UART1_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART1
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART1_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART1_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}

/**
 * @brief UART1 Callback
 *
 */
void uart1_callback(uart_callback_t cb)
{
    callbacks.rx[1] = cb;
}

/**
 * @brief UART1 interrupt handler
 */
void uart1_isr(void)
{
    uint32_t status = UARTIntStatus(UART1_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART1_BASE, status);

    while(UARTCharsAvail(UART1_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART1_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[1] != NULL)
        {
            callbacks.rx[1]((uint8_t)c);
        }
    }
}


//---------------------------------UART2---------------------------------*/
//---------------------------------UART2---------------------------------*/
//---------------------------------UART2---------------------------------*/

/**
 * @brief Initialize UART2
 * @param baudrate Baud rate for UART2
 * @note This function configures UART2, the GPIO pins are PA6 (RX) and PA7 (TX).
 * It also enables the UART2 interrupt and registers the UART2_Handler function as the interrupt service routine.
 */
void uart2_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART2 and port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART2));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    //Disable UART2
    UARTDisable(UART2_BASE);
    //Configure pins for UART2 (PA6:RX, PA7:TX)
    GPIOPinConfigure(GPIO_PA6_U2RX);
    GPIOPinConfigure(GPIO_PA7_U2TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,
                    GPIO_PIN_6 | GPIO_PIN_7);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART2_BASE,
                       UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART2_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Set FIFO level
    UARTFIFOLevelSet(UART2_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);
    // Set FIFO
    UARTFIFOEnable(UART2_BASE);

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[2] = callback;
        uart2_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART2_BASE, uart2_isr); //Register ISR
        UARTIntEnable(UART2_BASE,
                      UART_INT_RX | UART_INT_RT);

        //Enable interrupt in NVIC
        IntEnable(INT_UART2);
        IntMasterEnable();
    } else {
        callbacks.rx[2] = NULL;
        UARTIntDisable(UART2_BASE,
                      UART_INT_RX | UART_INT_RT);

        //Enable interrupt in NVIC
        IntDisable(INT_UART2);

    }
    //Enable UART2
    UARTEnable(UART2_BASE);
}

/**
 *  @brief Send a byte through UART2
 *  @param data Byte to send
 * */
void UART2_Sendbyte(uint8_t data)
{
    UARTCharPut(UART2_BASE, data);
    UARTBusy(UART2_BASE);
}

/**
 * @brief Send a string through UART2
 * @param str String to send
 */
void UART2_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART2_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART2
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART2_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART2_BASE))
       {
           data = UARTCharGetNonBlocking(UART2_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART2
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART2_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART2_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}

/**
 * @brief UART2 Callback
 *
 */
void uart2_callback(uart_callback_t cb)
{
    callbacks.rx[2] = cb;
}


/**
 * @brief UART2 interrupt handler
 */
void uart2_isr(void)
{
    uint32_t status = UARTIntStatus(UART2_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART2_BASE, status);

    while(UARTCharsAvail(UART2_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART2_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[2] != NULL)
        {
            callbacks.rx[2]((uint8_t)c);
        }
    }
}
//---------------------------------UART3---------------------------------*/
//---------------------------------UART3---------------------------------*/
//---------------------------------UART3---------------------------------*/

/**
 * @brief Initialize UART3
 * @param baudrate Baud rate for UART3
 * @note This function configures UART3, the GPIO pins are PA4 (RX) and PA5 (TX).
 * It also enables the UART3 interrupt and registers the UART3_Handler function as the interrupt service routine.
 */
void uart3_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART3 and port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART3));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    //Disable UART3
    UARTDisable(UART3_BASE);
    //Configure pins for UART3 (PA4:RX, PA5:TX)
    GPIOPinConfigure(GPIO_PA4_U3RX);
    GPIOPinConfigure(GPIO_PA5_U3TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,
                    GPIO_PIN_4 | GPIO_PIN_5);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART3_BASE,
                       UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART3_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Set FIFO level
    UARTFIFOLevelSet(UART3_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);
    // Set FIFO
    UARTFIFOEnable(UART3_BASE);

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[3] = callback;
        uart3_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART3_BASE, uart3_isr); //Register ISR
        UARTIntEnable(UART3_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntEnable(INT_UART3);
        IntMasterEnable();
    } else {
        callbacks.rx[3] = NULL;
        UARTIntDisable(UART3_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntDisable(INT_UART3);
    }
    //Enable UART3
    UARTEnable(UART3_BASE);
}

/**
 *  @brief Send a byte through UART3
 *  @param data Byte to send
 * */
void UART3_Sendbyte(uint8_t data)
{
    UARTCharPut(UART3_BASE, data);
    UARTBusy(UART3_BASE);
}

/**
 * @brief Send a string through UART3
 * @param str String to send
 */
void UART3_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART3_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART3
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART3_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART3_BASE))
       {
           data = UARTCharGetNonBlocking(UART3_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART3
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART3_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART3_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}


/**
 * @brief UART3 Callback
 *
 */
void uart3_callback(uart_callback_t cb)
{
    callbacks.rx[3] = cb;
}

/**
 * @brief UART3 interrupt handler
 */
void uart3_isr(void)
{
    uint32_t status = UARTIntStatus(UART3_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART3_BASE, status);

    while(UARTCharsAvail(UART3_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART3_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[3] != NULL)
        {
            callbacks.rx[3]((uint8_t)c);
        }
    }
}




//---------------------------------UART4---------------------------------*/
//---------------------------------UART4---------------------------------*/
//---------------------------------UART4---------------------------------*/

/**
 * @brief Initialize UART4
 * @param baudrate Baud rate for UART4
 * @note This function configures UART4, the GPIO pins are PA2 (RX) and PA3 (TX).
 * It also enables the UART4 interrupt and registers the UART4_Handler function as the interrupt service routine.
 */
void uart4_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART4 and port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART4));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    //Disable UART4
    UARTDisable(UART4_BASE);
    //Configure pins for UART4 (PA2:RX, PA3:TX)
    GPIOPinConfigure(GPIO_PA2_U4RX);
    GPIOPinConfigure(GPIO_PA3_U4TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,
                    GPIO_PIN_2 | GPIO_PIN_3);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART4_BASE,
                       UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART4_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Set FIFO level
    UARTFIFOLevelSet(UART4_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);//UART_FIFO_RX4_8);
    // Set FIFO
    UARTFIFOEnable(UART4_BASE);

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[4] = callback;
        uart4_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART4_BASE, uart4_isr); //Register ISR
        UARTIntEnable(UART4_BASE,
                      UART_INT_RX | UART_INT_RT);

        //Enable interrupt in NVIC
        IntEnable(INT_UART4);
        IntMasterEnable();
    } else {
        callbacks.rx[4] = NULL;
        UARTIntDisable(UART4_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntDisable(INT_UART4);
    }
    //Enable UART4
    UARTEnable(UART4_BASE);
}

/**
 *  @brief Send a byte through UART4
 *  @param data Byte to send
 * */
void UART4_Sendbyte(uint8_t data)
{
    UARTCharPut(UART4_BASE, data);
    UARTBusy(UART4_BASE);
}

/**
 * @brief Send a string through UART4
 * @param str String to send
 */
void UART4_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART4_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART4
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART4_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART4_BASE))
       {
           data = UARTCharGetNonBlocking(UART4_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART4
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART4_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART4_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}


/**
 * @brief UART4 Callback
 *
 */
void uart4_callback(uart_callback_t cb)
{
    callbacks.rx[4] = cb;
}

/**
 * @brief UART4 interrupt handler
 */
void uart4_isr(void)
{
    uint32_t status = UARTIntStatus(UART4_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART4_BASE, status);

    while(UARTCharsAvail(UART4_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART4_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[4] != NULL)
        {
            callbacks.rx[4]((uint8_t)c);
        }
    }
}





//---------------------------------UART5---------------------------------*/
//---------------------------------UART5---------------------------------*/
//---------------------------------UART5---------------------------------*/

/**
 * @brief Initialize UART5
 * @param baudrate Baud rate for UART5
 * @note This function configures UART5, the GPIO pins are PC6 (RX) and PC7 (TX).
 * It also enables the UART5 interrupt and registers the UART5_Handler function as the interrupt service routine.
 */
void uart5_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART5 and port C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART5));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    //Disable UART5
    UARTDisable(UART5_BASE);
    //Configure pins for UART5 (PC6:RX, PC7:TX)
    GPIOPinConfigure(GPIO_PC6_U5RX);
    GPIOPinConfigure(GPIO_PC7_U5TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE,
                    GPIO_PIN_6 | GPIO_PIN_7);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART5_BASE,
                       UART_CLOCK_PIOSC);
    // Set FIFO level
    UARTFIFOLevelSet(UART5_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);

    // Set FIFO
    UARTFIFOEnable(UART5_BASE);
    UARTConfigSetExpClk(UART5_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[5] = callback;
        uart5_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART5_BASE, uart5_isr); //Register ISR
        UARTIntEnable(UART5_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntEnable(INT_UART5);
        IntMasterEnable();
    } else {
        callbacks.rx[5] = NULL;
        UARTIntDisable(UART5_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntDisable(INT_UART5);
    }
    //Enable UART5
    UARTEnable(UART5_BASE);
}

/**
 *  @brief Send a byte through UART5
 *  @param data Byte to send
 * */
void UART5_Sendbyte(uint8_t data)
{
    UARTCharPut(UART5_BASE, data);
    UARTBusy(UART5_BASE);
}

/**
 * @brief Send a string through UART5
 * @param str String to send
 */
void UART5_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART5_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART5
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART5_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART5_BASE))
       {
           data = UARTCharGetNonBlocking(UART5_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART5
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART5_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART5_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}


/**
 * @brief UART5 Callback
 *
 */
void uart5_callback(uart_callback_t cb)
{
    callbacks.rx[5] = cb;
}

/**
 * @brief UART5 interrupt handler
 */
void uart5_isr(void)
{
    uint32_t status = UARTIntStatus(UART5_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART5_BASE, status);

    while(UARTCharsAvail(UART5_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART5_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[5] != NULL)
        {
            callbacks.rx[5]((uint8_t)c);
        }
    }
}




//---------------------------------UART6---------------------------------*/
//---------------------------------UART6---------------------------------*/
//---------------------------------UART6---------------------------------*/

/**
 * @brief Initialize UART6
 * @param baudrate Baud rate for UART6
 * @note This function configures UART6, the GPIO pins are PP0 (RX) and PP1 (TX).
 * It also enables the UART6 interrupt and registers the UART6_Handler function as the interrupt service routine.
 */
void uart6_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART6 and port P
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));

    //Disable UART6
    UARTDisable(UART6_BASE);
    //Configure pins for UART6 (PP0:RX, PP1:TX)
    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    GPIOPinTypeUART(GPIO_PORTP_BASE,
                    GPIO_PIN_0 | GPIO_PIN_1);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART6_BASE,
                       UART_CLOCK_PIOSC);
    // Set FIFO level
    UARTFIFOLevelSet(UART6_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);

    // Set FIFO
    UARTFIFOEnable(UART6_BASE);
    UARTConfigSetExpClk(UART6_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[6] = callback;
        uart6_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART6_BASE, uart6_isr); //Register ISR
        UARTIntEnable(UART6_BASE,
                      UART_INT_RX | UART_INT_RT);

        //Enable interrupt in NVIC
        IntEnable(INT_UART6);
        IntMasterEnable();
    } else {
        callbacks.rx[6] = NULL;
        UARTIntDisable(UART6_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntDisable(INT_UART6);
    }
    //Enable UART6
    UARTEnable(UART6_BASE);
}

/**
 *  @brief Send a byte through UART6
 *  @param data Byte to send
 * */
void UART6_Sendbyte(uint8_t data)
{
    UARTCharPut(UART6_BASE, data);
    UARTBusy(UART6_BASE);
}

/**
 * @brief Send a string through UART6
 * @param str String to send
 */
void UART6_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART6_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART6
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART6_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART6_BASE))
       {
           data = UARTCharGetNonBlocking(UART6_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART6
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART6_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART6_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}


/**
 * @brief UART6 Callback
 *
 */
void uart6_callback(uart_callback_t cb)
{
    callbacks.rx[6] = cb;
}
/**
 * @brief UART6 interrupt handler
 */
void uart6_isr(void)
{
    uint32_t status = UARTIntStatus(UART6_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART6_BASE, status);

    while(UARTCharsAvail(UART6_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART6_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[6] != NULL)
        {
            callbacks.rx[6]((uint8_t)c);
        }
    }
}




//---------------------------------UART7---------------------------------*/
//---------------------------------UART7---------------------------------*/
//---------------------------------UART7---------------------------------*/

/**
 * @brief Initialize UART7
 * @param baudrate Baud rate for UART7
 * @note This function configures UART7, the GPIO pins are PC4 (RX) and PC5 (TX).
 * It also enables the UART7 interrupt and registers the UART7_Handler function as the interrupt service routine.
 */
void uart7_init(uint32_t baudrate, void (*callback)(uint8_t))
{
    //Enable UART7 and port C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    //Disable UART7
    UARTDisable(UART7_BASE);
    //Configure pins for UART7 (PC4:RX, PC5:TX)
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE,
                    GPIO_PIN_4 | GPIO_PIN_5);

    //Configure UART clock and baud rate
    UARTClockSourceSet(UART7_BASE,
                       UART_CLOCK_PIOSC);
    // Set FIFO level
    UARTFIFOLevelSet(UART7_BASE,
                     UART_FIFO_TX1_8,
                     UART_FIFO_RX4_8);

    // Set FIFO
    UARTFIFOEnable(UART7_BASE);
    UARTConfigSetExpClk(UART7_BASE,
                        16000000,
                        //SysCtlClockGet(),   //System freq
                        baudrate,           //Baudrare
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable interrupts
    if(callback != NULL)
    {
        callbacks.rx[7] = callback;
        uart7_callback(callback);
        // Enable interrupts
        UARTIntRegister(UART7_BASE, uart7_isr); //Register ISR
        UARTIntEnable(UART7_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntEnable(INT_UART7);
        IntMasterEnable();
    } else {
        callbacks.rx[7] = NULL;
        UARTIntDisable(UART7_BASE,
                      UART_INT_RX | UART_INT_RT);
        //Enable interrupt in NVIC
        IntDisable(INT_UART7);
    }
    //Enable UART7
    UARTEnable(UART7_BASE);
}

/**
 *  @brief Send a byte through UART7
 *  @param data Byte to send
 * */
void UART7_Sendbyte(uint8_t data)
{
    UARTCharPut(UART7_BASE, data);
    UARTBusy(UART7_BASE);
}

/**
 * @brief Send a string through UART7
 * @param str String to send
 */
void UART7_Sendstring(const uint8_t* str, uint16_t length)
{
    uint16_t i; /* Number of bytes already sent from str. */
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART7_BASE,
                    (uint8_t)str[i]);
    }
}

/**
 * @brief Read a byte from UART7
 * @param timeout Timeout value
 * @return Read byte or -1 if timeout
 */
int32_t UART7_ReadByte(uint32_t timeout)
{
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */
    while(timeout > 0)
    {
       if(UARTCharsAvail(UART7_BASE))
       {
           data = UARTCharGetNonBlocking(UART7_BASE);
           return data;
       }
       timeout--;
    }
    return -1;
}

/**
 * @brief Read multiple bytes from UART7
 * @param buffer Buffer to store the read bytes
 * @param length Number of bytes to read
 * @param timeout Timeout value
 * @return Number of bytes read
 */
uint32_t UART7_ReadBytes(uint8_t* buffer, uint32_t length, uint32_t timeout)
{
    uint32_t cont = 0; /* Bytes copied into the caller's buffer. */
    int32_t data = -1; /* Received byte [0,255], or -1 on timeout. */

    while(cont < length && timeout > 0)
    {
        data = UART7_ReadByte(timeout);
        if(data == -1)
        {
            break;
        }
        buffer[cont++] = (uint8_t)data;
    }
    return cont;
}


/**
 * @brief UART7 Callback
 *
 */
void uart7_callback(uart_callback_t cb)
{
    callbacks.rx[7] = cb;
}

/**
 * @brief UART7 interrupt handler
 */
void uart7_isr(void)
{
    uint32_t status = UARTIntStatus(UART7_BASE, true); /* IRQ cause bits. */
    UARTIntClear(UART7_BASE, status);

    while(UARTCharsAvail(UART7_BASE))
    {
        int32_t c = UARTCharGetNonBlocking(UART7_BASE); /* Byte or -1. */
        if(c == -1)
        {
            break;
        }
        // Executes callback if exist
        if(callbacks.rx[7] != NULL)
        {
            callbacks.rx[7]((uint8_t)c);
        }
    }
}
