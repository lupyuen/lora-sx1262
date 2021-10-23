#ifndef ARCH_RISCV
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "radio.h"
#include "sx126x-board.h"

/// TODO: We are using LoRa Frequency 923 MHz for Singapore. Change this for your region.
#define USE_BAND_923

#if defined(USE_BAND_433)
    #define RF_FREQUENCY               434000000 /* Hz */
#elif defined(USE_BAND_780)
    #define RF_FREQUENCY               780000000 /* Hz */
#elif defined(USE_BAND_868)
    #define RF_FREQUENCY               868000000 /* Hz */
#elif defined(USE_BAND_915)
    #define RF_FREQUENCY               915000000 /* Hz */
#elif defined(USE_BAND_923)
    #define RF_FREQUENCY               923000000 /* Hz */
#else
    #error "Please define a frequency band in the compiler options."
#endif

/// LoRa Parameters
#define LORAPING_TX_OUTPUT_POWER            14        /* dBm */

#define LORAPING_BANDWIDTH                  0         /* [0: 125 kHz, */
                                                      /*  1: 250 kHz, */
                                                      /*  2: 500 kHz, */
                                                      /*  3: Reserved] */
#define LORAPING_SPREADING_FACTOR           7         /* [SF7..SF12] */
#define LORAPING_CODINGRATE                 1         /* [1: 4/5, */
                                                      /*  2: 4/6, */
                                                      /*  3: 4/7, */
                                                      /*  4: 4/8] */
#define LORAPING_PREAMBLE_LENGTH            8         /* Same for Tx and Rx */
#define LORAPING_SYMBOL_TIMEOUT             5         /* Symbols */
#define LORAPING_FIX_LENGTH_PAYLOAD_ON      false
#define LORAPING_IQ_INVERSION_ON            false

#define LORAPING_TX_TIMEOUT_MS              3000    /* ms */
#define LORAPING_RX_TIMEOUT_MS              5000    /* ms */
#define LORAPING_BUFFER_SIZE                64      /* LoRa message size */

const uint8_t loraping_ping_msg[] = "PING";  //  We send a "PING" message
const uint8_t loraping_pong_msg[] = "PONG";  //  We expect a "PONG" response

static uint8_t loraping_buffer[LORAPING_BUFFER_SIZE];  //  64-byte buffer for our LoRa messages
static int loraping_rx_size;

/// LoRa Statistics
struct {
    int rx_timeout;
    int rx_ping;
    int rx_pong;
    int rx_other;
    int rx_error;
    int tx_timeout;
    int tx_success;
} loraping_stats;

///////////////////////////////////////////////////////////////////////////////
//  Main Function

static void read_registers(void);

int main(void) {
    //  Read SX1262 registers 0x00 to 0x0F
    read_registers();

    //  TODO: Close the SPI Bus
    //  close(spi);
    puts("Done!");
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
//  LoRa Commands

static void send_once(int is_ping);
static void on_tx_done(void);
static void on_rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void on_tx_timeout(void);
static void on_rx_timeout(void);
static void on_rx_error(void);

/// Read SX1262 registers
static void read_registers(void) {
    //  Init the SPI port
    SX126xIoInit();

    //  Read and print the first 16 registers: 0 to 15
    for (uint16_t addr = 0; addr < 0x10; addr++) {
        //  Read the register
        uint8_t val = SX126xReadRegister(addr);      //  For SX1262
        //  uint8_t val = SX1276Read(addr);          //  For SX1276

        //  Print the register value
        printf("Register 0x%02x = 0x%02x\r\n", addr, val);
    }
}

/// Initialise the SX1262 driver.
/// Assume that create_task has been called to init the Event Queue.
static void init_driver(void) {
    //  Set the LoRa Callback Functions
    RadioEvents_t radio_events;
    memset(&radio_events, 0, sizeof(radio_events));  //  Must init radio_events to null, because radio_events lives on stack!
    radio_events.TxDone    = on_tx_done;
    radio_events.RxDone    = on_rx_done;
    radio_events.TxTimeout = on_tx_timeout;
    radio_events.RxTimeout = on_rx_timeout;
    radio_events.RxError   = on_rx_error;

    //  Init the SPI Port and the LoRa Transceiver
    Radio.Init(&radio_events);

    //  Set the LoRa Frequency
    Radio.SetChannel(RF_FREQUENCY);

    //  Configure the LoRa Transceiver for transmitting messages
    Radio.SetTxConfig(
        MODEM_LORA,
        LORAPING_TX_OUTPUT_POWER,
        0,        //  Frequency deviation: Unused with LoRa
        LORAPING_BANDWIDTH,
        LORAPING_SPREADING_FACTOR,
        LORAPING_CODINGRATE,
        LORAPING_PREAMBLE_LENGTH,
        LORAPING_FIX_LENGTH_PAYLOAD_ON,
        true,     //  CRC enabled
        0,        //  Frequency hopping disabled
        0,        //  Hop period: N/A
        LORAPING_IQ_INVERSION_ON,
        LORAPING_TX_TIMEOUT_MS
    );

    //  Configure the LoRa Transceiver for receiving messages
    Radio.SetRxConfig(
        MODEM_LORA,
        LORAPING_BANDWIDTH,
        LORAPING_SPREADING_FACTOR,
        LORAPING_CODINGRATE,
        0,        //  AFC bandwidth: Unused with LoRa
        LORAPING_PREAMBLE_LENGTH,
        LORAPING_SYMBOL_TIMEOUT,
        LORAPING_FIX_LENGTH_PAYLOAD_ON,
        0,        //  Fixed payload length: N/A
        true,     //  CRC enabled
        0,        //  Frequency hopping disabled
        0,        //  Hop period: N/A
        LORAPING_IQ_INVERSION_ON,
        true      //  Continuous receive mode
    );    
}

/// Send a LoRa message. Assume that SX1262 driver has been initialised.
static void send_message(void) {
    //  Send the "PING" message
    send_once(1);
}

/// Send a LoRa message. If is_ping is 0, send "PONG". Otherwise send "PING".
static void send_once(int is_ping) {
    //  Copy the "PING" or "PONG" message to the transmit buffer
    if (is_ping) {
        memcpy(loraping_buffer, loraping_ping_msg, 4);
    } else {
        memcpy(loraping_buffer, loraping_pong_msg, 4);
    }

    //  Fill up the remaining space in the transmit buffer (64 bytes) with values 0, 1, 2, ...
    for (int i = 4; i < sizeof loraping_buffer; i++) {
        loraping_buffer[i] = i - 4;
    }

    //  Send the transmit buffer (64 bytes)
    Radio.Send(loraping_buffer, sizeof loraping_buffer);
}

/// Receive a LoRa message. Assume that SX1262 driver has been initialised.
/// Assume that create_task has been called to init the Event Queue.
static void receive_message(void) {
    //  Receive a LoRa message within the timeout period
    Radio.Rx(LORAPING_RX_TIMEOUT_MS);
}

///////////////////////////////////////////////////////////////////////////////
//  LoRa Callback Functions

/// Callback Function that is called when our LoRa message has been transmitted
static void on_tx_done(void)
{
    printf("Tx done\r\n");

    //  Log the success status
    loraping_stats.tx_success++;

    //  Switch the LoRa Transceiver to low power, sleep mode
    Radio.Sleep();
    
    //  TODO: Receive a "PING" or "PONG" LoRa message
    //  os_eventq_put(os_eventq_dflt_get(), &loraping_ev_rx);
}

/// Callback Function that is called when a LoRa message has been received
static void on_rx_done(
    uint8_t *payload,  //  Buffer containing received LoRa message
    uint16_t size,     //  Size of the LoRa message
    int16_t rssi,      //  Signal strength
    int8_t snr)        //  Signal To Noise ratio
{
    printf("Rx done: \r\n");

    //  Switch the LoRa Transceiver to low power, sleep mode
    Radio.Sleep();

    //  Copy the received packet
    if (size > sizeof loraping_buffer) {
        size = sizeof loraping_buffer;
    }
    loraping_rx_size = size;
    memcpy(loraping_buffer, payload, size);

    //  Log the signal strength, signal to noise ratio
    //  TODO: loraping_rxinfo_rxed(rssi, snr);

    //  Dump the contents of the received packet
    for (int i = 0; i < loraping_rx_size; i++) {
        printf("%02x ", loraping_buffer[i]);
    }
    printf("\r\n");

    //  TODO: Send a "PING" or "PONG" LoRa message
    //  os_eventq_put(os_eventq_dflt_get(), &loraping_ev_tx);
}

/// Callback Function that is called when our LoRa message couldn't be transmitted due to timeout
static void on_tx_timeout(void)
{
    printf("Tx timeout\r\n");

    //  Switch the LoRa Transceiver to low power, sleep mode
    Radio.Sleep();

    //  Log the timeout
    loraping_stats.tx_timeout++;

    //  TODO: Receive a "PING" or "PONG" LoRa message
    //  os_eventq_put(os_eventq_dflt_get(), &loraping_ev_rx);
}

/// Callback Function that is called when no LoRa messages could be received due to timeout
static void on_rx_timeout(void)
{
    printf("Rx timeout\r\n");

    //  Switch the LoRa Transceiver to low power, sleep mode
    Radio.Sleep();

    //  Log the timeout
    loraping_stats.rx_timeout++;
    //  TODO: loraping_rxinfo_timeout();

    //  TODO: Send a "PING" or "PONG" LoRa message
    //  os_eventq_put(os_eventq_dflt_get(), &loraping_ev_tx);
}

/// Callback Function that is called when we couldn't receive a LoRa message due to error
static void on_rx_error(void)
{
    printf("Rx error\r\n");

    //  Log the error
    loraping_stats.rx_error++;

    //  Switch the LoRa Transceiver to low power, sleep mode
    Radio.Sleep();

    //  TODO: Send a "PING" or "PONG" LoRa message
    //  os_eventq_put(os_eventq_dflt_get(), &loraping_ev_tx);
}

#endif  //  !ARCH_RISCV