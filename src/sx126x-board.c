/*!
 * \file      sx1262mbxcas-board.c
 *
 * \brief     Target board SX1262MBXCAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include <stddef.h>
#include <assert.h>
#include <device/vfs_spi.h>  //  For spi_ioc_transfer_t
#include <hal/soc/spi.h>     //  For hal_spi_transfer
#include <hal_spi.h>         //  For spi_init
#include <bl_gpio.h>         //  For bl_gpio_output_set
#include <bl_irq.h>          //  For bl_irq_register_with_ctx
#include <bl602_glb.h>       //  For GLB_GPIO_Func_Init
#include "nimble_npl.h"      //  For NimBLE Porting Layer (multitasking functions)
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"

static int register_gpio_handler(
    uint8_t gpioPin,         //  GPIO Pin Number
    DioIrqHandler *handler,  //  GPIO Handler Function
    uint8_t intCtrlMod,      //  GPIO Interrupt Control Mode (see below)
    uint8_t intTrgMod,       //  GPIO Interrupt Trigger Mode (see below)
    uint8_t pullup,          //  1 for pullup, 0 for no pullup
    uint8_t pulldown);       //  1 for pulldown, 0 for no pulldown
static void handle_gpio_interrupt(void *arg);

#if defined( USE_RADIO_DEBUG )
/*!
 * \brief Writes new Tx debug pin state
 *
 * \param [IN] state Debug pin state
 */
static void SX126xDbgPinTxWrite( uint8_t state );

/*!
 * \brief Writes new Rx debug pin state
 *
 * \param [IN] state Debug pin state
 */
static void SX126xDbgPinRxWrite( uint8_t state );
#endif

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntPow;
Gpio_t DeviceSel;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

///////////////////////////////////////////////////////////////////////////////
//  GPIO Functions

static void GpioInitOutput(uint8_t pin, uint8_t value) {
    //  Configure pin as a GPIO Pin
    GLB_GPIO_Type pins[1];
    pins[0] = pin;
    BL_Err_Type rc2 = GLB_GPIO_Func_Init(
        GPIO_FUN_SWGPIO,  //  Configure as GPIO 
        pins,             //  Pins to be configured
        sizeof(pins) / sizeof(pins[0])  //  Number of pins (1)
    );
    assert(rc2 == SUCCESS);    

    //  Configure pin as a GPIO Output Pin (instead of GPIO Input)
    int rc = bl_gpio_enable_output(pin, 0, 0);
    assert(rc == 0);

    //  Set pin to Low or High
    rc = bl_gpio_output_set(pin, value);
    assert(rc == 0);
}

static void GpioInitInput(uint8_t pin, uint8_t pullup, uint8_t pulldown) {
    //  Configure pin as a GPIO Pin
    GLB_GPIO_Type pins[1];
    pins[0] = pin;
    BL_Err_Type rc2 = GLB_GPIO_Func_Init(
        GPIO_FUN_SWGPIO,  //  Configure as GPIO 
        pins,             //  Pins to be configured
        sizeof(pins) / sizeof(pins[0])  //  Number of pins (1)
    );
    assert(rc2 == SUCCESS);    

    //  Configure Reset pin as a GPIO Input Pin
    int rc = bl_gpio_enable_input(pin, pullup, pulldown);
    assert(rc == 0);
}

///////////////////////////////////////////////////////////////////////////////
//  SPI Functions

/// SPI Device Instance
spi_dev_t spi_device;

/// SPI Transmit Buffer (1 byte)
static uint8_t spi_tx_buf[1];

/// SPI Receive Buffer (1 byte)
static uint8_t spi_rx_buf[1];

/// Blocking call to send a value on the SPI. Returns the value received from the SPI Peripheral.
/// Assume that we are sending and receiving 8-bit values on SPI.
/// Assume Chip Select Pin has already been set to Low by caller.
/// TODO: We should combine multiple SPI DMA Requests, instead of handling one byte at a time
uint16_t SpiInOut(int spi_num, uint16_t val) {
    //  Populate the transmit buffer
    spi_tx_buf[0] = val;

    //  Clear the receive buffer
    memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));

    //  Prepare SPI Transfer
    static spi_ioc_transfer_t transfer;
    memset(&transfer, 0, sizeof(transfer));    
    transfer.tx_buf = (uint32_t) spi_tx_buf;  //  Transmit Buffer
    transfer.rx_buf = (uint32_t) spi_rx_buf;  //  Receive Buffer
    transfer.len    = 1;                      //  How many bytes

    //  Assume Chip Select Pin has already been set to Low by caller

    //  Execute the SPI Transfer with the DMA Controller
    int rc = hal_spi_transfer(
        &spi_device,  //  SPI Device
        &transfer,    //  SPI Transfers
        1             //  How many transfers (Number of requests, not bytes)
    );
    assert(rc == 0);

    //  Assume Chip Select Pin will be set to High by caller

    //  Return the received byte
    return spi_rx_buf[0];
}

///////////////////////////////////////////////////////////////////////////////

/// Initialise GPIO Pins and SPI Port. Called by SX126xIoIrqInit.
/// Note: This is different from the Reference Implementation,
/// which initialises the GPIO Pins and SPI Port at startup.
void SX126xIoInit( void )
{
    printf("SX126xIoInit\r\n");
    GpioInitOutput( SX126X_SPI_CS_PIN, 1 );
    GpioInitInput( SX126X_BUSY_PIN, 0, 0 );
    GpioInitInput( SX126X_DIO1, 0, 0 );
    ////GpioInitInput( SX126X_DEVICE_SEL_PIN, 0, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { GpioInitOutput( SX126X_DEBUG_CS_PIN, 1 ); }

    //  Note: We must swap MISO and MOSI to comply with the SPI Pin Definitions in BL602 / BL604 Reference Manual
    printf("Swap MISO and MOSI\r\n");
    int rc = GLB_Swap_SPI_0_MOSI_With_MISO(ENABLE);  assert(rc == 0);

    //  Configure the SPI Port
    rc = spi_init(
        &spi_device,     //  SPI Device
        SX126X_SPI_IDX,  //  SPI Port
        0,               //  SPI Mode: 0 for Controller
        //  TODO: Due to a quirk in BL602 SPI, we must set
        //  SPI Polarity-Phase to 1 (CPOL=0, CPHA=1).
        //  But actually Polarity-Phase for SX126X should be 0 (CPOL=0, CPHA=0). 
        1,                    //  SPI Polarity-Phase
        SX126X_SPI_BAUDRATE,  //  SPI Frequency
        2,                    //  Transmit DMA Channel
        3,                    //  Receive DMA Channel
        SX126X_SPI_CLK_PIN,   //  SPI Clock Pin 
        SX126X_SPI_CS_OLD,    //  Unused SPI Chip Select Pin
        SX126X_SPI_SDO_PIN,   //  SPI Serial Data Out Pin (formerly MOSI)
        SX126X_SPI_SDI_PIN    //  SPI Serial Data In Pin  (formerly MISO)
    );
    assert(rc == 0);
}

/// Initialise GPIO Pins and SPI Port. Register GPIO Interrupt Handler for DIO1.
/// Based on hal_button_register_handler_with_dts in https://github.com/lupyuen/bl_iot_sdk/blob/master/components/hal_drv/bl602_hal/hal_button.c
/// Note: This is different from the Reference Implementation,
/// which initialises the GPIO Pins and SPI Port at startup.
void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    //  Initialise GPIO Pins and SPI Port.
    //  Note: This is different from the Reference Implementation,
    //  which initialises the GPIO Pins and SPI Port at startup.
    SX126xIoInit();

    printf("SX126X interrupt init\r\n");
    assert(SX126X_DIO1 >= 0);
    assert(dioIrq != NULL);
    int rc = register_gpio_handler(   //  Register GPIO Handler...
        SX126X_DIO1,                  //  GPIO Pin Number
        dioIrq,                       //  GPIO Handler Function
        GLB_GPIO_INT_CONTROL_ASYNC,   //  Async Control Mode
        GLB_GPIO_INT_TRIG_POS_PULSE,  //  Trigger when GPIO level shifts from Low to High 
        0,                            //  No pullup
        0                             //  No pulldown
    );
    assert(rc == 0);

    //  Register Common Interrupt Handler for GPIO Interrupt
    bl_irq_register_with_ctx(
        GPIO_INT0_IRQn,         //  GPIO Interrupt
        handle_gpio_interrupt,  //  Interrupt Handler
        NULL                    //  Argument for Interrupt Handler
    );

    //  Enable GPIO Interrupt
    bl_irq_enable(GPIO_INT0_IRQn);
}

void SX126xIoDeInit( void )
{
    printf("SX126xIoDeInit\r\n");
    GpioInitOutput( SX126X_SPI_CS_PIN, 1 );
    GpioInitInput( SX126X_BUSY_PIN, 0, 0 );
    GpioInitInput( SX126X_DIO1, 0, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { GpioInitOutput( SX126X_DEBUG_CS_PIN, 1 ); }
}

void SX126xIoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )
    GpioInitOutput( SX126X_DBG_PIN_TX, 0 );
    GpioInitOutput( SX126X_DBG_PIN_RX, 0 );
#endif
}

void SX126xIoTcxoInit( void )
{
    // No TCXO component available on this board design.
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return SX126X_TCXO_WAKEUP_TIME;
}

void SX126xIoRfSwitchInit( void )
{
    SX126xSetDio2AsRfSwitchCtrl( true );
}

RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}

void SX126xSetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
#if defined( USE_RADIO_DEBUG )
    switch( mode )
    {
        case MODE_TX:
            SX126xDbgPinTxWrite( 1 );
            SX126xDbgPinRxWrite( 0 );
            break;
        case MODE_RX:
        case MODE_RX_DC:
            SX126xDbgPinTxWrite( 0 );
            SX126xDbgPinRxWrite( 1 );
            break;
        default:
            SX126xDbgPinTxWrite( 0 );
            SX126xDbgPinRxWrite( 0 );
            break;
    }
#endif
}

#ifdef TODO
void SX126xReset( void )
{
    DelayMs( 10 );
    GpioInitOutput( SX126X_NRESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    DelayMs( 20 );
    GpioInitAnalogic( SX126X_NRESET, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 ); // internal pull-up
    DelayMs( 10 );
}
#endif  //  TODO

void SX126xReset(void)
{
    //// #warning Check SX126xReset

    printf("SX126xReset\r\n");

    //  Configure Reset pin as a GPIO Pin
    GLB_GPIO_Type pins[1];
    pins[0] = SX126X_NRESET;
    BL_Err_Type rc2 = GLB_GPIO_Func_Init(
        GPIO_FUN_SWGPIO,  //  Configure as GPIO 
        pins,             //  Pins to be configured
        sizeof(pins) / sizeof(pins[0])  //  Number of pins (1)
    );
    assert(rc2 == SUCCESS);    

    //  Configure Reset pin as a GPIO Output Pin (instead of GPIO Input)
    int rc = bl_gpio_enable_output(SX126X_NRESET, 0, 0);
    assert(rc == 0);

    //  Set Reset pin to Low
    rc = bl_gpio_output_set(SX126X_NRESET, 1);
    assert(rc == 0);

    // Wait 1 ms
    DelayMs(1);

    //  Configure Reset pin as a GPIO Input Pin, no pullup, no pulldown
    rc = bl_gpio_enable_input(SX126X_NRESET, 0, 0);
    assert(rc == 0);

    // Wait 6 ms
    DelayMs(6);
}

void SX126xWaitOnBusy( void )
{
    while( bl_gpio_input_get_value( SX126X_BUSY_PIN ) == 1 );
}

void SX126xWakeup( void )
{
    printf("SX126xWakeup\r\n");
    CRITICAL_SECTION_BEGIN( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }

    SpiInOut( SX126X_SPI_IDX, RADIO_GET_STATUS );
    SpiInOut( SX126X_SPI_IDX, 0x00 );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    // Update operating mode context variable
    SX126xSetOperatingMode( MODE_STDBY_RC );

    CRITICAL_SECTION_END( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }

    SpiInOut( SX126X_SPI_IDX, ( uint8_t )command );

    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( SX126X_SPI_IDX, buffer[i] );
    }

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    printf("SX126xReadCommand\r\n");
    uint8_t status = 0;

    SX126xCheckDeviceReady( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }

    SpiInOut( SX126X_SPI_IDX, ( uint8_t )command );
    status = SpiInOut( SX126X_SPI_IDX, 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( SX126X_SPI_IDX, 0 );
    }

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    SX126xWaitOnBusy( );

    return status;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }
    
    SpiInOut( SX126X_SPI_IDX, RADIO_WRITE_REGISTER );
    SpiInOut( SX126X_SPI_IDX, ( address & 0xFF00 ) >> 8 );
    SpiInOut( SX126X_SPI_IDX, address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( SX126X_SPI_IDX, buffer[i] );
    }

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }

    SpiInOut( SX126X_SPI_IDX, RADIO_READ_REGISTER );
    SpiInOut( SX126X_SPI_IDX, ( address & 0xFF00 ) >> 8 );
    SpiInOut( SX126X_SPI_IDX, address & 0x00FF );
    SpiInOut( SX126X_SPI_IDX, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( SX126X_SPI_IDX, 0 );
    }
    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }

    SpiInOut( SX126X_SPI_IDX, RADIO_WRITE_BUFFER );
    SpiInOut( SX126X_SPI_IDX, offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( SX126X_SPI_IDX, buffer[i] );
    }
    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    bl_gpio_output_set( SX126X_SPI_CS_PIN, 0 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 0 ); }

    SpiInOut( SX126X_SPI_IDX, RADIO_READ_BUFFER );
    SpiInOut( SX126X_SPI_IDX, offset );
    SpiInOut( SX126X_SPI_IDX, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( SX126X_SPI_IDX, 0 );
    }
    bl_gpio_output_set( SX126X_SPI_CS_PIN, 1 );
    if (SX126X_DEBUG_CS_PIN >= 0) { bl_gpio_output_set( SX126X_DEBUG_CS_PIN, 1 ); }

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    printf("SX126xSetRfTxPower\r\n");
    ////TODO: SX126xSetTxParams( power, RADIO_RAMP_40_US );
    SX126xSetTxParams( power, RADIO_RAMP_3400_US );////TODO
}

uint8_t SX126xGetDeviceId( void )
{
    //  For SX1262
    printf("SX126xGetDeviceId: SX1262\r\n");
    return SX1262;

    //  For SX1261
    //  printf("SX126xGetDeviceId: SX1261\r\n");
    //  return SX1261;

#ifdef NOTUSED
    if( bl_gpio_input_get_value( SX126X_DEVICE_SEL_PIN ) == 1 )
    {
        printf("SX126xGetDeviceId: SX1261\r\n");
        return SX1261;
    }
    else
    {
        printf("SX126xGetDeviceId: SX1262\r\n");
        return SX1262;
    }
#endif  //  NOTUSED
}

void SX126xAntSwOn( void )
{
#if SX126X_HAS_ANT_SW
    GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
#endif  //  SX126X_HAS_ANT_SW
}

void SX126xAntSwOff( void )
{
#if SX126X_HAS_ANT_SW
    GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif  //  SX126X_HAS_ANT_SW
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t SX126xGetDio1PinState( void )
{
    return bl_gpio_input_get_value( SX126X_DIO1 );
}

#if defined( USE_RADIO_DEBUG )
static void SX126xDbgPinTxWrite( uint8_t state )
{
    bl_gpio_output_set( SX126X_DBG_PIN_TX, state );
}

static void SX126xDbgPinRxWrite( uint8_t state )
{
    bl_gpio_output_set( SX126X_DBG_PIN_RX, state );
}
#endif

///////////////////////////////////////////////////////////////////////////////
//  GPIO Interrupt: Handle GPIO Interrupt triggered by received LoRa Packet and other conditions

/// Maximum number of GPIO Pins that can be configured for interrupts
#define MAX_GPIO_INTERRUPTS 6  //  DIO0 to DIO5

/// Array of Events for the GPIO Interrupts.
/// The Events will be triggered to forward the GPIO Interrupt to the Application Task.
/// gpio_events[i] corresponds to gpio_interrupts[i].
static struct ble_npl_event gpio_events[MAX_GPIO_INTERRUPTS];

/// Array of GPIO Pin Numbers that have been configured for interrupts.
/// We shall lookup this array to find the GPIO Pin Number for each GPIO Interrupt Event.
/// gpio_events[i] corresponds to gpio_interrupts[i].
static uint8_t gpio_interrupts[MAX_GPIO_INTERRUPTS];

static int init_interrupt_event(uint8_t gpioPin, DioIrqHandler *handler);
static int enqueue_interrupt_event(uint8_t gpioPin, struct ble_npl_event *event);

/// Register Handler Function for GPIO. Return 0 if successful.
/// GPIO Handler Function will run in the context of the Application Task, not the Interrupt Handler.
/// Based on bl_gpio_register in https://github.com/lupyuen/bl_iot_sdk/blob/master/components/hal_drv/bl602_hal/bl_gpio.c
static int register_gpio_handler(
    uint8_t gpioPin,         //  GPIO Pin Number
    DioIrqHandler *handler,  //  GPIO Handler Function
    uint8_t intCtrlMod,      //  GPIO Interrupt Control Mode (see below)
    uint8_t intTrgMod,       //  GPIO Interrupt Trigger Mode (see below)
    uint8_t pullup,          //  1 for pullup, 0 for no pullup
    uint8_t pulldown)        //  1 for pulldown, 0 for no pulldown
{
    printf("SX126X register handler: GPIO %d\r\n", (int) gpioPin);

    //  Init the Event that will invoke the handler for the GPIO Interrupt
    int rc = init_interrupt_event(
        gpioPin,  //  GPIO Pin Number
        handler   //  GPIO Handler Function that will be triggered by the Event
    );
    assert(rc == 0);

    //  Configure pin as a GPIO Pin
    GLB_GPIO_Type pins[1];
    pins[0] = gpioPin;
    BL_Err_Type rc2 = GLB_GPIO_Func_Init(
        GPIO_FUN_SWGPIO,  //  Configure as GPIO 
        pins,             //  Pins to be configured
        sizeof(pins) / sizeof(pins[0])  //  Number of pins (1)
    );
    assert(rc2 == SUCCESS);    

    //  Configure pin as a GPIO Input Pin
    rc = bl_gpio_enable_input(
        gpioPin,  //  GPIO Pin Number
        pullup,   //  1 for pullup, 0 for no pullup
        pulldown  //  1 for pulldown, 0 for no pulldown
    );
    assert(rc == 0);

    //  Disable GPIO Interrupt for the pin
    bl_gpio_intmask(gpioPin, 1);

    //  Configure GPIO Pin for GPIO Interrupt
    bl_set_gpio_intmod(
        gpioPin,     //  GPIO Pin Number
        intCtrlMod,  //  GPIO Interrupt Control Mode (see below)
        intTrgMod    //  GPIO Interrupt Trigger Mode (see below)
    );

    //  Enable GPIO Interrupt for the pin
    bl_gpio_intmask(gpioPin, 0);
    return 0;
}

//  GPIO Interrupt Control Modes:
//  GLB_GPIO_INT_CONTROL_SYNC:  GPIO interrupt sync mode
//  GLB_GPIO_INT_CONTROL_ASYNC: GPIO interrupt async mode
//  See hal_button_register_handler_with_dts in https://github.com/lupyuen/bl_iot_sdk/blob/master/components/hal_drv/bl602_hal/hal_button.c

//  GPIO Interrupt Trigger Modes:
//  GLB_GPIO_INT_TRIG_NEG_PULSE: GPIO negative edge pulse trigger
//  GLB_GPIO_INT_TRIG_POS_PULSE: GPIO positive edge pulse trigger
//  GLB_GPIO_INT_TRIG_NEG_LEVEL: GPIO negative edge level trigger (32k 3T)
//  GLB_GPIO_INT_TRIG_POS_LEVEL: GPIO positive edge level trigger (32k 3T)
//  See hal_button_register_handler_with_dts in https://github.com/lupyuen/bl_iot_sdk/blob/master/components/hal_drv/bl602_hal/hal_button.c

/// Interrupt Handler for GPIO Pin DIO1. Triggered by SX126X when LoRa Packet is received 
/// and for other conditions.  Based on gpio_interrupt_entry in
/// https://github.com/lupyuen/bl_iot_sdk/blob/master/components/hal_drv/bl602_hal/bl_gpio.c#L151-L164
static void handle_gpio_interrupt(void *arg)
{
    //  Check all GPIO Interrupt Events
    for (int i = 0; i < MAX_GPIO_INTERRUPTS; i++) {
        //  Get the GPIO Interrupt Event
        struct ble_npl_event *ev = &gpio_events[i];

        //  If the Event is unused, skip it
        if (ev->fn == NULL) { continue; }

        //  Get the GPIO Pin Number for the Event
        GLB_GPIO_Type gpioPin = gpio_interrupts[i];

        //  Get the Interrupt Status of the GPIO Pin
        BL_Sts_Type status = GLB_Get_GPIO_IntStatus(gpioPin);

        //  If the GPIO Pin has triggered an interrupt...
        if (status == SET) {
            //  Forward the GPIO Interrupt to the Application Task to process
            enqueue_interrupt_event(
                gpioPin,  //  GPIO Pin Number
                ev        //  Event that will be enqueued for the Application Task
            );
        }
    }
}

/// Interrupt Counters
int g_dio0_counter, g_dio1_counter, g_dio2_counter, g_dio3_counter, g_dio4_counter, g_dio5_counter, g_nodio_counter;

/// Enqueue the GPIO Interrupt to an Event Queue for the Application Task to process
static int enqueue_interrupt_event(
    uint8_t gpioPin,              //  GPIO Pin Number
    struct ble_npl_event *event)  //  Event that will be enqueued for the Application Task
{
    //  Disable GPIO Interrupt for the pin
    bl_gpio_intmask(gpioPin, 1);

    //  Note: DO NOT Clear the GPIO Interrupt Status for the pin!
    //  This will suppress subsequent GPIO Interrupts!
    //  bl_gpio_int_clear(gpioPin, SET);

    //  Increment the Interrupt Counters
    if (SX126X_DIO1 >= 0 && gpioPin == (uint8_t) SX126X_DIO1) { g_dio1_counter++; }
    else { g_nodio_counter++; }

    //  Use Event Queue to invoke Event Handler in the Application Task, 
    //  not in the Interrupt Context
    if (event != NULL && event->fn != NULL) {
        extern struct ble_npl_eventq event_queue;  //  TODO: Move Event Queue to header file
        ble_npl_eventq_put(&event_queue, event);
    }

    //  Enable GPIO Interrupt for the pin
    bl_gpio_intmask(gpioPin, 0);
    return 0;
}

//  Init the Event that will the Interrupt Handler will invoke to process the GPIO Interrupt
static int init_interrupt_event(
    uint8_t gpioPin,         //  GPIO Pin Number
    DioIrqHandler *handler)  //  GPIO Handler Function
{
    //  Find an unused Event with null handler and set it
    for (int i = 0; i < MAX_GPIO_INTERRUPTS; i++) {
        struct ble_npl_event *ev = &gpio_events[i];

        //  If the Event is used, skip it
        if (ev->fn != NULL) { continue; }

        //  Set the Event handler
        ble_npl_event_init(   //  Init the Event for...
            ev,               //  Event
            handler,          //  Event Handler Function
            NULL              //  Argument to be passed to Event Handler
        );

        //  Set the GPIO Pin Number for the Event
        gpio_interrupts[i] = gpioPin;
        return 0;
    }

    //  No unused Events found, should increase MAX_GPIO_INTERRUPTS
    assert(false);
    return -1;
}

///////////////////////////////////////////////////////////////////////////////
//  Timer Functions

/// Initialise a timer
void TimerInit(
    struct ble_npl_callout *timer,  //  The timer to initialize. Cannot be NULL.
    ble_npl_event_fn *f)            //  The timer callback function. Cannot be NULL.
{
    //  Implement with Callout Functions from NimBLE Porting Layer
    assert(timer != NULL);
    assert(f != NULL);

    //  Event Queue containing Events to be processed, defined in demo.c.  TODO: Move to header file.
    extern struct ble_npl_eventq event_queue;

    //  Init the Callout Timer with the Callback Function
    ble_npl_callout_init(
        timer,         //  Callout Timer
        &event_queue,  //  Event Queue that will handle the Callout upon timeout
        f,             //  Callback Function
        NULL           //  Argument to be passed to Callback Function
    );
}

/// Stops a timer from running.  Can be called even if timer is not running.
void TimerStop(
    struct ble_npl_callout *timer)  //  Pointer to timer to stop. Cannot be NULL.
{
    //  Implement with Callout Functions from NimBLE Porting Layer
    assert(timer != NULL);

    //  If Callout Timer is still running...
    if (ble_npl_callout_is_active(timer)) {
        //  Stop the Callout Timer
        ble_npl_callout_stop(timer);
    }
}

/// Sets a timer that will expire ‘millisecs’ milliseconds from the current time.
void TimerStart(
    struct ble_npl_callout *timer,  //  Pointer to timer. Cannot be NULL.
    uint32_t millisecs)             //  The number of milliseconds from now at which the timer will expire.
{
    //  Implement with Callout Functions from NimBLE Porting Layer.
    assert(timer != NULL);

    //  Stop the timer if running
    TimerStop(timer);

    //  Convert milliseconds to ticks
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        millisecs  //  Duration in milliseconds
    );

    //  Wait at least 1 tick
    if (ticks == 0) { ticks = 1; }

    //  Trigger the Callout Timer after the elapsed ticks
    ble_npl_error_t rc = ble_npl_callout_reset(
        timer,  //  Callout Timer
        ticks   //  Number of ticks
    );
    assert(rc == 0);
}

/// Wait until ‘millisecs’ milliseconds has elapsed. This is a blocking delay.
void DelayMs(uint32_t millisecs)  //  The number of milliseconds to wait.
{
    //  Implement with Timer Functions from NimBLE Porting Layer.
    //  Convert milliseconds to ticks.
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        millisecs  //  Duration in milliseconds
    );

    //  Wait at least 1 tick
    if (ticks == 0) { ticks = 1; }

    //  Wait for the ticks
    ble_npl_time_delay(ticks);
}

/// Return current time in microseconds
uint32_t TimerGetCurrentTime(void)
{
    //  Convert ticks to milliseconds then microseconds
    return xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;
}

/// Return elased time in microseconds
uint32_t TimerGetElapsedTime(uint32_t saved_time)
{
    return TimerGetCurrentTime() - saved_time;
}
