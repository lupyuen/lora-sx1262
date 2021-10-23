//  SX1262 Board Functions for Linux
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"

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

static int sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer, const uint8_t size );
static int sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size );
static int sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer, const uint8_t size );
static int sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );
static int sx126x_hal_write( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    const uint8_t* data, const uint16_t data_length );
static int sx126x_hal_read( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    uint8_t* data, const uint16_t data_length );
static int init_spi(void);

///////////////////////////////////////////////////////////////////////////////

/// Initialise GPIO Pins and SPI Port. Called by SX126xIoIrqInit.
/// Note: This is different from the Reference Implementation,
/// which initialises the GPIO Pins and SPI Port at startup.
void SX126xIoInit( void )
{
    printf("SX126xIoInit\r\n");

    //  Init SPI Bus
    int rc = init_spi();
    assert(rc == 0);

    //  TODO: Init GPIO Pins
}

/// Initialise GPIO Pins and SPI Port. Register GPIO Interrupt Handler for DIO1.
/// Note: This is different from the Reference Implementation,
/// which initialises the GPIO Pins and SPI Port at startup.
void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    //  Initialise GPIO Pins and SPI Port.
    //  Note: This is different from the Reference Implementation,
    //  which initialises the GPIO Pins and SPI Port at startup.
    SX126xIoInit();

    //  TODO: Register GPIO Interrupt Handler for DIO1
    printf("TODO: SX126X interrupt init\r\n");
}

void SX126xIoDeInit( void )
{
    printf("SX126xIoDeInit\r\n");
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

void SX126xReset(void)
{
    //// #warning Check SX126xReset

    printf("TODO: SX126xReset\r\n");
#ifdef TODO
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
#endif  //  TODO
}

void SX126xWaitOnBusy( void )
{
    printf("TODO: SX126xWaitOnBusy\r\n");
    usleep(10 * 1000); ////

#ifdef TODO
    while( bl_gpio_input_get_value( SX126X_BUSY_PIN ) == 1 );
#endif  //  TODO
}

void SX126xWakeup( void )
{
    printf("SX126xWakeup\r\n");
    CRITICAL_SECTION_BEGIN( );

    // Write RADIO_GET_STATUS command followed by 0
    uint8_t commands[1] = { RADIO_GET_STATUS };
    uint8_t buffer[1]   = { 0 };
    int rc = sx126x_hal_write(NULL, commands, sizeof(commands), buffer, sizeof(buffer));
    assert(rc == 0);

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    // Update operating mode context variable
    SX126xSetOperatingMode( MODE_STDBY_RC );

    CRITICAL_SECTION_END( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //  Write the command followed by buffer
    uint8_t commands[1] = { (uint8_t) command };
    int rc = sx126x_hal_write(NULL, commands, sizeof(commands), buffer, size);
    assert(rc == 0);

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

    assert(false);
#ifdef TODO
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
#endif  //  TODO

    SX126xWaitOnBusy( );

    return status;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );
    int rc = sx126x_write_register(NULL, address, buffer, size);
    assert(rc == 0);
    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );
    int rc = sx126x_read_register(NULL, address, buffer, size);
    assert(rc == 0);
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
    int rc = sx126x_write_buffer(NULL, offset, buffer, size);
    assert(rc == 0);
    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
    int rc = sx126x_read_buffer(NULL, offset, buffer, size);
    assert(rc == 0);
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
    assert(false); return 0;
#ifdef TODO
    return bl_gpio_input_get_value( SX126X_DIO1 );
#endif  //  TODO
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
//  Timer Functions

/// Initialise a timer
void TimerInit(
    struct ble_npl_callout *timer,  //  The timer to initialize. Cannot be NULL.
    ble_npl_event_fn *f)            //  The timer callback function. Cannot be NULL.
{
    puts("TODO: TimerInit");
    assert(timer != NULL);
    assert(f != NULL);

#ifdef TODO
    //  Event Queue containing Events to be processed, defined in demo.c.  TODO: Move to header file.
    extern struct ble_npl_eventq event_queue;

    //  Init the Callout Timer with the Callback Function
    ble_npl_callout_init(
        timer,         //  Callout Timer
        &event_queue,  //  Event Queue that will handle the Callout upon timeout
        f,             //  Callback Function
        NULL           //  Argument to be passed to Callback Function
    );
#endif  //  TODO
}

/// Stops a timer from running.  Can be called even if timer is not running.
void TimerStop(
    struct ble_npl_callout *timer)  //  Pointer to timer to stop. Cannot be NULL.
{
    puts("TODO: TimerStop");
    assert(timer != NULL);

    assert(false);
#ifdef TODO
    //  If Callout Timer is still running...
    if (ble_npl_callout_is_active(timer)) {
        //  Stop the Callout Timer
        ble_npl_callout_stop(timer);
    }
#endif  //  TODO
}

/// Sets a timer that will expire ‘millisecs’ milliseconds from the current time.
void TimerStart(
    struct ble_npl_callout *timer,  //  Pointer to timer. Cannot be NULL.
    uint32_t millisecs)             //  The number of milliseconds from now at which the timer will expire.
{
    puts("TODO: TimerStart");
    assert(timer != NULL);

    assert(false);
#ifdef TODO
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
#endif  //  TODO
}

/// Wait until ‘millisecs’ milliseconds has elapsed. This is a blocking delay.
void DelayMs(uint32_t millisecs)  //  The number of milliseconds to wait.
{
    assert(false);
#ifdef TODO
    //  Implement with Timer Functions from NimBLE Porting Layer.
    //  Convert milliseconds to ticks.
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        millisecs  //  Duration in milliseconds
    );

    //  Wait at least 1 tick
    if (ticks == 0) { ticks = 1; }

    //  Wait for the ticks
    ble_npl_time_delay(ticks);
#endif  //  TODO
}

/// Return current time in microseconds
uint32_t TimerGetCurrentTime(void)
{
    assert(false); return 0;
#ifdef TODO
    //  Convert ticks to milliseconds then microseconds
    return xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;
#endif  //  TODO
}

/// Return elased time in microseconds
uint32_t TimerGetElapsedTime(uint32_t saved_time)
{
    assert(false); return 0;
#ifdef TODO
    return TimerGetCurrentTime() - saved_time;
#endif  //  TODO
}

///////////////////////////////////////////////////////////////////////////////
//  Register and Buffer Functions

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 3,
} sx126x_commands_size_t;

static int sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
    const uint8_t size ) {
    uint8_t buf[SX126X_SIZE_WRITE_REGISTER] = { 0 };
    buf[0] = RADIO_WRITE_REGISTER;
    buf[1] = ( uint8_t )( address >> 8 );
    buf[2] = ( uint8_t )( address >> 0 );
    return sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_REGISTER, buffer, size );
}

static int sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size ) {
    uint8_t buf[SX126X_SIZE_READ_REGISTER] = { 0 };
    int status = -1;
    buf[0] = RADIO_READ_REGISTER;
    buf[1] = ( uint8_t )( address >> 8 );
    buf[2] = ( uint8_t )( address >> 0 );
    buf[3] = 0;
    status = sx126x_hal_read( context, buf, SX126X_SIZE_READ_REGISTER, buffer, size );
    return status;
}

static int sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
    const uint8_t size ) {
    uint8_t buf[SX126X_SIZE_WRITE_BUFFER] = { 0 };
    buf[0] = RADIO_WRITE_BUFFER;
    buf[1] = offset;
    return sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_BUFFER, buffer, size );
}

static int sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size ) {
    uint8_t buf[SX126X_SIZE_READ_BUFFER] = { 0 };
    int status = -1;
    buf[0] = RADIO_READ_BUFFER;
    buf[1] = offset;
    buf[2] = 0;
    status = sx126x_hal_read( context, buf, SX126X_SIZE_READ_BUFFER, buffer, size );
    return status;
}

///////////////////////////////////////////////////////////////////////////////
//  SPI Functions

/// SPI Bus
static int spi = 0;

/// Max size of SPI transfers
#define SPI_BUFFER_SIZE 1024

/// SPI Transmit Buffer
static uint8_t spi_tx_buf[SPI_BUFFER_SIZE];

/// SPI Receive Buffer
static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];

static int transfer_spi(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
static int sx126x_hal_write( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    const uint8_t* data, const uint16_t data_length ) {
    printf("sx126x_hal_write: command_length=%d, data_length=%d\n", command_length, data_length);

    //  Total length is command + data length
    uint16_t len = command_length + data_length;
    assert(len > 0);
    assert(len <= SPI_BUFFER_SIZE);

    //  Clear the SPI Transmit and Receive buffers
    memset(&spi_tx_buf, 0, len);
    memset(&spi_rx_buf, 0, len);

    //  Copy command bytes to SPI Transmit Buffer
    memcpy(&spi_tx_buf, command, command_length);

    //  Copy data bytes to SPI Transmit Buffer
    memcpy(&spi_tx_buf[command_length], data, data_length);

    //  Transmit and receive the SPI buffers
    int rc = transfer_spi(spi_tx_buf, spi_rx_buf, len);
    assert(rc == 0);
    return 0;
}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
static int sx126x_hal_read( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    uint8_t* data, const uint16_t data_length ) {
    printf("sx126x_hal_read: command_length=%d, data_length=%d\n", command_length, data_length);

    //  Total length is command + data length
    uint16_t len = command_length + data_length;
    assert(len > 0);
    assert(len <= SPI_BUFFER_SIZE);

    //  Clear the SPI Transmit and Receive buffers
    memset(&spi_tx_buf, 0, len);
    memset(&spi_rx_buf, 0, len);

    //  Copy command bytes to SPI Transmit Buffer
    memcpy(&spi_tx_buf, command, command_length);

    //  Transmit and receive the SPI buffers
    int rc = transfer_spi(spi_tx_buf, spi_rx_buf, len);
    assert(rc == 0);

    //  Copy SPI Receive buffer to data buffer
    memcpy(data, &spi_rx_buf[command_length], data_length);
    return 0;
}

#ifdef TODO
/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
static int sx126x_hal_reset( const void* context ) {
    puts("sx126x_hal_reset");
    assert(false);
    return 0;
}
#endif  //  TODO

#ifdef TODO
/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
static int sx126x_hal_wakeup( const void* context ) {
    puts("sx126x_hal_wakeup");
    assert(false);
    return 0;
}
#endif  //  TODO

/// Init the SPI Bus. Return 0 on success.
static int init_spi(void) {
    //  Open the SPI Bus
    spi = open("/dev/spidev1.0", O_RDWR);
    assert(spi > 0);

    //  Set to SPI Mode 0
    uint8_t mmode = SPI_MODE_0;
    int rc = ioctl(spi, SPI_IOC_WR_MODE, &mmode);
    assert(rc == 0);

    //  Set LSB/MSB Mode
    uint8_t lsb = 0;
    rc = ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);
    assert(rc == 0);
    return 0;
}

/// Blocking call to transmit and receive buffers on SPI. Return 0 on success.
static int transfer_spi(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    assert(spi > 0);
    assert(len > 0);

    //  Prepare SPI Transfer
    struct spi_ioc_transfer spi_trans;
    memset(&spi_trans, 0, sizeof(spi_trans));
    spi_trans.tx_buf = (unsigned long) tx_buf;  //  Transmit Buffer
    spi_trans.rx_buf = (unsigned long) rx_buf;  //  Receive Buffer
    spi_trans.cs_change = true;   //  Set SPI Chip Select to Low
    spi_trans.len       = len;    //  How many bytes
    //  printf("spi tx: "); for (int i = 0; i < len; i++) { printf("%02x ", tx_buf[i]); } printf("\n");

    //  Transfer and receive the SPI buffers
    int rc = ioctl(spi, SPI_IOC_MESSAGE(1), &spi_trans);
    assert(rc >= 0);

    //  printf("spi rx: "); for (int i = 0; i < len; i++) { printf("%02x ", rx_buf[i]); } printf("\n");
    return 0;
}
