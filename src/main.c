#ifndef ARCH_RISCV
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sx126x.h"
#include "sx126x-board.h"

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

#endif  //  !ARCH_RISCV
