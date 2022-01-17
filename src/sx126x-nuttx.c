//  LoRa SX1262 Board Functions for NuttX
#ifdef __NuttX__  //  This file is for NuttX only
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <nuttx/ioexpander/gpio.h>
#include "../include/radio.h"
#include "../include/sx126x.h"
#include "../include/sx126x-board.h"

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

/// SX1262 Busy Pin (GPIO Input)
static int busy = 0;

/// SX1262 DIO1 Pin (GPIO Interrupt)
static int dio1 = 0;

static int sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer, const uint8_t size );
static int sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size );
static int sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer, const uint8_t size );
static int sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );
static int sx126x_hal_write( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    const uint8_t* data, const uint16_t data_length );
static int sx126x_hal_read( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    uint8_t* data, const uint16_t data_length, uint8_t *status );
static void init_event_queue(void);
static int init_gpio(void);
static int init_spi(void);

///////////////////////////////////////////////////////////////////////////////

/// Initialise GPIO Pins and SPI Port. Called by SX126xIoIrqInit.
/// Note: This is different from the Reference Implementation,
/// which initialises the GPIO Pins and SPI Port at startup.
void SX126xIoInit( void )
{
    _info("SX126xIoInit\n");

    //  Init the Event Queue if not initialised. 
    //  TimerInit is called before SX126xIoInit, 
    //  so the Event Queue should already be initialised.
    init_event_queue();

    //  Init GPIO Pins. Event Queue must be initialised before this.
    int rc = init_gpio();
    assert(rc == 0);

    //  Init SPI Bus
    rc = init_spi();
    assert(rc == 0);
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
}

void SX126xIoDeInit( void )
{
    puts("TODO: SX126xIoDeInit");
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

    puts("TODO: SX126xReset");
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
    _info("SX126xWaitOnBusy\n");
    assert(busy > 0);

    //  Loop until Busy Pin is Low, but stop after 1,000 tries so we won't get stuck here
    for (int i = 0; i < 1000; i++) {
        //  Read Busy Pin
        bool invalue;
        int ret = ioctl(busy, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
        assert(ret >= 0);

        //  Exit if Busy Pin is Low
        if (invalue == 0) { break; }
    }
}

void SX126xWakeup( void )
{
    _info("SX126xWakeup\n");
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
    _info("SX126xReadCommand: command=0x%02x, size=%d\n", command, size);
    uint8_t status = 0;

    SX126xCheckDeviceReady( );

    //  Write the command, read the status and read the buffer
    uint8_t commandStatus[2] = { 
        (uint8_t) command,  //  Command
        0                   //  Status
    };
    int rc = sx126x_hal_read(NULL, commandStatus, sizeof(commandStatus), buffer, size, &status);
    assert(rc == 0);
    _info("status=0x%02x\n", status);

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
    _info("SX126xSetRfTxPower\n");

    //  TODO: If we're done with troubleshooting
    //  the Transmit Power, remember to 
    //  switch the Power Ramp Up Time from
    //  3,400 microsecs back to the default 
    //  40 microsecs.
    SX126xSetTxParams( power, RADIO_RAMP_3400_US );

    //  Previously:
    //  Power Ramp Up Time is 40 microsecs
    //  SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
    //  For SX1262
    _info("SX126xGetDeviceId: SX1262\n");
    return SX1262;

    //  For SX1261
    //  _info("SX126xGetDeviceId: SX1261\n");
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
    //  Return the value of DIO1 Pin
    assert(dio1 > 0);

    //  Read the GPIO Input
    bool invalue;
    int ret = ioctl(dio1, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
    assert(ret >= 0);

    //  Return the value: 1 or 0
    return invalue ? 1 : 0;
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

/// Timer Table: Maps Timer Address to Timeout Value (millisecs)
#define MAX_TIMERS 32  //  Max number of timers allowed
static struct ble_npl_callout *timer_addr[MAX_TIMERS];  //  Timer Address
static uint32_t timer_timeout[MAX_TIMERS];  //  Timeout Value (millisecs)

/// Initialise a timer
void TimerInit(
    struct ble_npl_callout *timer,  //  The timer to initialize. Cannot be NULL.
    ble_npl_event_fn *f)            //  The timer callback function. Cannot be NULL.
{
    //  Init the Event Queue if not initialised.
    //  TimerInit is called before SX126xIoInit,
    //  so we need to init the Event Queue here.
    init_event_queue();

    printf("TimerInit:     %p\n", timer);
    assert(timer != NULL); 
    assert(f != NULL);

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
    printf("TimerStop:     %p\n", timer);
    assert(timer != NULL);

    //  If Callout Timer is still running...
    if (ble_npl_callout_is_active(timer)) {
        //  Stop the Callout Timer
        ble_npl_callout_stop(timer);
    }
}

/// Set timer new timeout value
void TimerSetValue( 
    struct ble_npl_callout *timer,  //  Pointer to timer. Cannot be NULL.
    uint32_t millisecs)             //  The number of milliseconds from now at which the timer will expire.
{
    printf("TimerSetValue: %p, %ld ms\n", timer, millisecs);
    assert(timer != NULL);
    assert(millisecs > 0);

    //  Find the timer in the Timer Table, or an empty slot
    int i;
    for (i = 0; i < MAX_TIMERS; i++) {
        if (timer_addr[i] == NULL || timer_addr[i] == timer) {
            timer_addr[i] = timer;
            break;
        }
    }
    assert(i < MAX_TIMERS);  //  No space in the Timer Table, increase MAX_TIMERS

    //  Set the new timeout value
    timer_timeout[i] = millisecs;
}

/// Start a timer that will expire ‘millisecs’ milliseconds from the current time, as defined by TimerSetValue
void TimerStart( 
    struct ble_npl_callout *timer)  //  Pointer to timer. Cannot be NULL.
{
    printf("TimerStart:    %p\n", timer);
    assert(timer != NULL);

    //  Find the timer in the Timer Table
    int i;
    for (i = 0; i < MAX_TIMERS; i++) {
        if (timer_addr[i] == timer) { break; }
    }
    assert(i < MAX_TIMERS);  //  Not found in the Timer Table

    //  Get the timeout value
    uint32_t millisecs = timer_timeout[i];
    assert(millisecs > 0);

    //  Start the timer
    TimerStart2(timer, millisecs);
}

/// Start a timer that will expire ‘millisecs’ milliseconds from the current time.
void TimerStart2(
    struct ble_npl_callout *timer,  //  Pointer to timer. Cannot be NULL.
    uint32_t millisecs)             //  The number of milliseconds from now at which the timer will expire.
{
    assert(timer != NULL);

    //  Stop the timer if running
    TimerStop(timer);
    printf("TimerStart2:   %p, %ld ms\n", timer, millisecs);

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
    //  Get ticks (ms) since system start
    ble_npl_time_t ticks = ble_npl_time_get();

    //  Convert ticks to milliseconds
    uint32_t millisecs = ble_npl_time_ticks_to_ms32(ticks);
    _info("TimerGetCurrentTime: %ld ms\n", millisecs);

    //  Return as microseconds
    return millisecs * 1000;
}

/// Return elased time in microseconds
uint32_t TimerGetElapsedTime(uint32_t saved_time)
{
    return TimerGetCurrentTime() - saved_time;
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
    status = sx126x_hal_read( context, buf, SX126X_SIZE_READ_REGISTER, buffer, size, NULL );
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
    status = sx126x_hal_read( context, buf, SX126X_SIZE_READ_BUFFER, buffer, size, NULL );
    return status;
}

///////////////////////////////////////////////////////////////////////////////
//  SPI and GPIO Functions

/// SPI Bus
static int spi = 0;

/// SPI Chip Select Pin (GPIO Output)
static int cs = 0;

/// Max size of SPI transfers
#define SPI_BUFFER_SIZE 1024

/// SPI Transmit Buffer
static uint8_t spi_tx_buf[SPI_BUFFER_SIZE];

/// SPI Receive Buffer
static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];

static int transfer_spi(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);
void *process_dio1(void *arg);

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
    _info("sx126x_hal_write: command_length=%d, data_length=%d\n", command_length, data_length);

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
 * @param [out] status          If not null, return the second SPI byte received as status
 *
 * @returns Operation status
 */
static int sx126x_hal_read( 
    const void* context, const uint8_t* command, const uint16_t command_length,
    uint8_t* data, const uint16_t data_length, uint8_t *status ) {
    _info("sx126x_hal_read: command_length=%d, data_length=%d\n", command_length, data_length);

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

    //  Return the second SPI byte received as status
    if (status != NULL) {
        assert(len >= 2);
        *status = spi_rx_buf[1];
    }
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

/// Init the GPIO Pins. Return 0 on success.
static int init_gpio(void) {
    puts("init_gpio");

    //  Open GPIO Input for SX1262 Busy Pin
    busy = open("/dev/gpio0", O_RDWR);
    assert(busy > 0);

    //  Get SX1262 Busy Pin Type
    enum gpio_pintype_e pintype;
    int ret = ioctl(busy, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
    assert(ret >= 0);

    //  Verify that SX1262 Busy Pin is GPIO Input (not GPIO Output or GPIO Interrupt)
    assert(pintype == GPIO_INPUT_PIN);  //  No pullup / pulldown

    //  Open GPIO Interrupt for SX1262 DIO1 Pin
    dio1 = open("/dev/gpio2", O_RDWR);
    assert(dio1 > 0);

    //  Get SX1262 DIO1 Pin Type
    ret = ioctl(dio1, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
    assert(ret >= 0);
    printf("DIO1 pintype before=%d\n", pintype);

    //  Verify that SX1262 DIO1 Pin is GPIO Interrupt (not GPIO Input or GPIO Output)
    assert(pintype == GPIO_INTERRUPT_PIN);

    //  Change DIO1 Pin to Trigger GPIO Interrupt on Rising Edge
    //  TODO: Crashes at ioexpander/gpio.c (line 544) because change failed apparently
    puts("init_gpio: change DIO1 to Trigger GPIO Interrupt on Rising Edge");
    ret = ioctl(dio1, GPIOC_SETPINTYPE, (unsigned long) GPIO_INTERRUPT_RISING_PIN);
    assert(ret >= 0);

    //  Get SX1262 DIO1 Pin Type again
    ret = ioctl(dio1, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
    assert(ret >= 0);
    printf("DIO1 pintype after=%d\n", pintype);

    //  Verify that SX1262 DIO1 Pin is GPIO Interrupt on Rising Edge
    //  TODO: This fails because the Pin Type remains as GPIO_INTERRUPT_PIN
    //  assert(pintype == GPIO_INTERRUPT_RISING_PIN);  //  Trigger interrupt on rising edge

    //  Init the Background Thread Attributes
    static pthread_attr_t attr;
    ret = pthread_attr_init(&attr);
    assert(ret == 0);

    //  Start the Background Thread to process DIO1 interrupts
    puts("Starting process_dio1");
    static pthread_t thread;
    ret = pthread_create(&thread, &attr, process_dio1, 0);
    assert(ret == 0);

    return 0;
}

/// Init the SPI Bus and Chip Select Pin. Return 0 on success.
static int init_spi(void) {
    puts("init_spi");

    //  Open the SPI Bus (SPI Test Driver)
    spi = open("/dev/spitest0", O_RDWR);
    assert(spi > 0);

    //  Open GPIO Output for SPI Chip Select
    cs = open("/dev/gpio1", O_RDWR);
    assert(cs > 0);

    //  Get SPI Chip Select Pin Type
    enum gpio_pintype_e pintype;
    int ret = ioctl(cs, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
    assert(ret >= 0);

    //  Verify that SPI Chip Select is GPIO Output (not GPIO Input or GPIO Interrupt)
    assert(pintype == GPIO_OUTPUT_PIN);

    //  TODO: Set SPI Chip Select to High for all SPI Devices
    return 0;
}

/// Blocking call to transmit and receive buffers on SPI. Return 0 on success.
static int transfer_spi(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    assert(spi > 0);
    assert(cs  > 0);
    assert(len > 0);
    assert(len <= SPI_BUFFER_SIZE);
    _info("spi tx: "); for (int i = 0; i < len; i++) { _info("%02x ", tx_buf[i]); } _info("\n");

    //  Set SPI Chip Select to Low
    int ret = ioctl(cs, GPIOC_WRITE, 0);
    assert(ret >= 0);

    //  Transmit data over SPI
    int bytes_written = write(spi, tx_buf, len);
    assert(bytes_written == len);

    //  Receive SPI response
    int bytes_read = read(spi, rx_buf, len);
    assert(bytes_read == len);

    //  Set SPI Chip Select to High
    ret = ioctl(cs, GPIOC_WRITE, 1);
    assert(ret >= 0);

    _info("spi rx: "); for (int i = 0; i < len; i++) { _info("%02x ", rx_buf[i]); } _info("\n");
    return 0;
}

/// Handle DIO1 Interrupt by adding to Event Queue
void *process_dio1(void *arg) {
    assert(dio1 > 0);
    puts("process_dio1 started");

    //  Define the DIO1 Interrupt Event
    static struct ble_npl_event ev;
    ble_npl_event_init(   //  Init the Event for...
        &ev,              //  Event
        RadioOnDioIrq,    //  Event Handler Function
        NULL              //  Argument to be passed to Event Handler
    );
    printf("process_dio1: event=%p\n", &ev);

    //  Define the signal
    #define SIG_DIO1 1
    struct sigevent notify;
    notify.sigev_notify = SIGEV_SIGNAL;
    notify.sigev_signo  = SIG_DIO1;

    //  Set up to receive signal from GPIO Interrupt (DIO1 rising edge)
    int ret = ioctl(dio1, GPIOC_REGISTER, (unsigned long)&notify);
    assert(ret >= 0);

    //  Add the signal to the Signal Set
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIG_DIO1);

    //  Loop forever waiting for the signal (DIO1 rising edge)
    for (;;) {
        //  Read the pin value
        bool invalue;
        ret = ioctl(dio1, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
        assert(ret >= 0);
        _info("DIO1 before=%u\n", (unsigned int)invalue);

        //  Wait up to 60 seconds for the Signal Set
        struct timespec ts;
        ts.tv_sec  = 60;
        ts.tv_nsec = 0;
        ret = sigtimedwait(&set, NULL, &ts);

        if (ret >= 0) {
            //  We were signalled. Add the DIO1 Interrupt Event to the Event Queue.
            puts("DIO1 add event");
            ble_npl_eventq_put(&event_queue, &ev);
        } else {
            //  We were not signalled
            int errcode = errno;
            if (errcode == EAGAIN) { puts("DIO1 timeout"); }
            else { fprintf(stderr, "ERROR: Failed to wait signal %d: %d\n", SIG_DIO1, errcode); return NULL; }
        }

        //  Re-read the pin value
        ret = ioctl(dio1, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
        assert(ret >= 0);
        _info("DIO1 after=%u\n", (unsigned int)invalue);
    }

    ////ioctl(dio1, GPIOC_UNREGISTER, 0);
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////
//  Event Queue

/// Event Queue containing Events to be processed. Exposed to NuttX App for Event Loop.
struct ble_npl_eventq event_queue;

/// True if Event Queue has been initialised
static bool is_event_queue_initialised = false;

/// Init the Event Queue
static void init_event_queue(void) {
    //  Init only once
    if (is_event_queue_initialised) { return; }
    is_event_queue_initialised = true;
    puts("init_event_queue");

    //  Init the Event Queue
    ble_npl_eventq_init(&event_queue);
}

#endif  //  __NuttX__