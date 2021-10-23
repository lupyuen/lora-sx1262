#ifndef ARCH_RISCV
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "sx126x.h"
#include "sx126x-board.h"

static void read_registers(void);

/// SPI Bus
static int spi = 0;

int main(void) {
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

    //  Read SX1262 registers 0x00 to 0x0F
    read_registers();

    //  Close the SPI Bus
    close(spi);
    puts("Done!");
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
//  LoRa Commands

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

#ifdef NOTUSED
///////////////////////////////////////////////////////////////////////////////
//  SPI HAL

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

/// Blocking call to transmit and receive buffers on SPI. Returns 0 on success.
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
#endif  //  NOTUSED

#endif  //  !ARCH_RISCV
