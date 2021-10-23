# LoRa Driver for Semtech SX1262 on Linux (PineDio USB Adapter) and BL602 (PineDio Stack BL604)

Read the article...

- ["PineCone BL602 Talks LoRaWAN"](https://lupyuen.github.io/articles/lorawan)

The design of the SX1262 Driver is similar to the SX1276 Driver, which is explained in these articles...

- ["Connect PineCone BL602 to LoRa Transceiver"](https://lupyuen.github.io/articles/lora)

- ["PineCone BL602 RISC-V Board Receives LoRa Packets"](https://lupyuen.github.io/articles/lora2)

Ported from Semtech's Reference Implementation of SX1262 Driver...

https://github.com/Lora-net/LoRaMac-node/tree/master/src/radio/sx126x

# Build PineDio USB Driver

To build PineDio USB Driver on Pinebook Pro Manjaro Arm64...

```bash
## Install DKMS
sudo pacman -Syu dkms base-devel --needed

## Install Kernel Headers for Manjaro: https://linuxconfig.org/manjaro-linux-kernel-headers-installation
uname -r 
## Should show "5.14.12-1-MANJARO-ARM" or similar
sudo pacman -S linux-headers
pacman -Q | grep headers
## Should show "linux-headers 5.14.12-1" or similar

## Reboot to be safe
sudo reboot now

## Install CH341 SPI Driver
git clone https://github.com/rogerjames99/spi-ch341-usb.git
pushd spi-ch341-usb
## TODO: Edit Makefile and change...
##   KERNEL_DIR  = /usr/src/linux-headers-$(KVERSION)/
## To...
##   KERNEL_DIR  = /lib/modules/$(KVERSION)/build
make
sudo make install
popd

## Unload the module ch341 if it has been automatically loaded
lsmod | grep ch341
sudo rmmod ch341

## Load the new module
sudo modprobe spi-ch341-usb

## Plug in PineDio USB and check that the module has been correctly loaded.
## See dmesg Log below.
dmesg

## Build PineDio USB Driver
git clone --recursive https://github.com/lupyuen/lora-sx1262
cd lora-sx1262
make

## Run PineDio USB Driver Demo.
## See Output Log below.
sudo ./lora-sx1262
```

See https://wiki.pine64.org/wiki/JF%27s_note_on_PineDio_devices#RAW_LoRa_communication_between_USB_LoRa_adapter_and_PineDio_STACK

# PineDio USB Output Log

Read Registers:

```text
SX126xIoInit
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x00 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x01 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x02 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x03 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x04 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x05 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x06 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x07 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x08 = 0x80
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x09 = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x0a = 0x01
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x0b = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x0c = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x0d = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x0e = 0x00
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
Register 0x0f = 0x00
Done!
```

Send Message:

```text
init_driver
TODO: SX126xReset
SX126xIoInit
TODO: SX126X interrupt init
TODO: SX126xWakeup
TODO: SX126xWakeup
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
TODO: SX126xWaitOnBusy
SX126xSetTxParams: power=22, rampTime=7
SX126xGetDeviceId: SX1262
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
TODO: SX126xWaitOnBusy
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=8
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=9
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=9
TODO: SX126xWaitOnBusy
TODO: TimerInit
TODO: TimerInit
RadioSetChannel: freq=923000000
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
TODO: SX126xWaitOnBusy
RadioSetTxConfig: modem=1, power=14, fdev=0, bandwidth=0, datarate=7, coderate=1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=3000
RadioSetTxConfig: SpreadingFactor=7, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=255, CrcMode=1, InvertIQ=0
RadioStandby
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
RadioSetModem
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
TODO: SX126xWaitOnBusy
SX126xSetRfTxPower
SX126xSetTxParams: power=14, rampTime=7
SX126xGetDeviceId: SX1262
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
TODO: SX126xWaitOnBusy
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
TODO: SX126xWaitOnBusy
RadioSetRxConfig
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
RadioStandby
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
RadioSetModem
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
TODO: SX126xWaitOnBusy
RadioSetRxConfig done
send_message
RadioSend: size=64
50 49 4e 47 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=8
TODO: SX126xWaitOnBusy
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=64, CrcMode=1, InvertIQ=0
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=2, data_length=64
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=3
TODO: SX126xWaitOnBusy
TODO: TimerStart
lora-sx1262: src/sx126x-linux.c:424: TimerStart: Assertion `false' failed.
Aborted
```

# PineDio USB dmesg Log

```text
usb 3-1:
new full-speed USB device number 2 using xhci-hcd
New USB device found, idVendor=1a86, idProduct=5512, bcdDevice= 3.04
New USB device strings: Mfr=0, Product=2, SerialNumber=0
Product: USB UART-LPT

spi-ch341-usb 3-1:1.0:
  ch341_usb_probe:
    connect device
    bNumEndpoints=3
      endpoint=0 type=2 dir=1 addr=2
      endpoint=1 type=2 dir=0 addr=2
      endpoint=2 type=3 dir=1 addr=1

  ch341_cfg_probe:
    output cs0 SPI slave with cs=0
    output cs0    gpio=0  irq=0 
    output cs1 SPI slave with cs=1
    output cs1    gpio=1  irq=1 
    output cs2 SPI slave with cs=2
    output cs2    gpio=2  irq=2 
    input  gpio4  gpio=3  irq=3 
    input  gpio6  gpio=4  irq=4 
    input  err    gpio=5  irq=5 
    input  pemp   gpio=6  irq=6 
    input  int    gpio=7  irq=7 (hwirq)
    input  slct   gpio=8  irq=8 
    input  wait   gpio=9  irq=9 
    input  autofd gpio=10 irq=10 
    input  addr   gpio=11 irq=11 
    output ini    gpio=12 irq=12 
    output write  gpio=13 irq=13 
    output scl    gpio=14 irq=14 
    output sda    gpio=15 irq=15 

  ch341_spi_probe:
    start
    SPI master connected to SPI bus 1
    SPI device /dev/spidev1.0 created
    SPI device /dev/spidev1.1 created
    SPI device /dev/spidev1.2 created
    done

  ch341_irq_probe:
    start
    irq_base=94
    done

  ch341_gpio_probe: 
    start

  ch341_gpio_get_direction:
    gpio=cs0    dir=0
    gpio=cs1    dir=0
    gpio=cs2    dir=0
    gpio=gpio4  dir=1
    gpio=gpio6  dir=1
    gpio=err    dir=1
    gpio=pemp   dir=1
    gpio=int    dir=1
    gpio=slct   dir=1
    gpio=wait   dir=1
    gpio=autofd dir=1
    gpio=addr   dir=1
    gpio=ini    dir=0
    gpio=write  dir=0
    gpio=scl    dir=0
    gpio=sda    dir=0

  ch341_gpio_probe:
    registered GPIOs from 496 to 511
    done
    connected

  ch341_gpio_poll_function:
    start

usbcore: registered new interface driver ch341
usbserial: USB Serial support registered for ch341-uart
```

# Connect BL602 to SX1262

The pins are defined here in [`include/sx126x-board.h`](include/sx126x-board.h)

From [`include/sx126x-board.h`](include/sx126x-board.h):

```c
//  Below are the pin numbers for PineDio Stack BL604 with onboard SX1262.
//  TODO: Change the pin numbers for your SX1262 connection to BL602 / BL604
#define SX126X_SPI_IDX           0  //  SPI Port 0
#define SX126X_SPI_SDI_PIN       0  //  SPI Serial Data In Pin  (formerly MISO)
#define SX126X_SPI_SDO_PIN      17  //  SPI Serial Data Out Pin (formerly MOSI)
#define SX126X_SPI_CLK_PIN      11  //  SPI Clock Pin
#define SX126X_SPI_CS_PIN       15  //  SPI Chip Select Pin
#define SX126X_SPI_CS_OLD        8  //  Unused SPI Chip Select Pin
#define SX126X_NRESET           18  //  Reset Pin
#define SX126X_DIO1             19  //  DIO1
#define SX126X_BUSY_PIN         10  //  Busy Pin
#define SX126X_DEBUG_CS_PIN      5  //  Debug Chip Select Pin, mirrors the High / Low State of SX1262 Chip Select Pin. Set to -1 if not needed.
#define SX126X_TCXO_WAKEUP_TIME  5  //  Time required for the TCXO to wakeup (milliseconds)
#define SX126X_SPI_BAUDRATE  (200 * 1000)  //  SPI Frequency (200 kHz)
```

# BL602 Demo Firmware

To transmit and receive LoRa packets with the driver, run the `sdk_app_lora` BL602 Demo Firmware...

- [`sdk_app_lora`: BL602 Demo Firmware for LoRa SX1262 / SX1276 ](https://github.com/lupyuen/bl_iot_sdk/tree/master/customer_app/sdk_app_lora)

Here's a sample log...

```text
# create_task

# init_driver
SX126xReset
SX126xIoInit
SX126X interrupt init
SX126X register handler: GPIO 11
SX126xWakeup
SX126xGetDeviceId: SX1262
SX126xSetRfTxPower
SX126xGetDeviceId: SX1262

# send_message

RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_TX_DONE
Tx done

# send_message
SX126xWakeup

RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_TX_DONE
Tx done

# send_message
SX126xWakeup

RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_TX_DONE
Tx done

# receive_message
SX126xWakeup

# RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_PREAMBLE_DETECTED
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_HEADER_VALID
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_RX_DONE
SX126xReadCommand
SX126xReadCommand
Rx done: 
48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 

# receive_message
SX126xWakeup

# RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_PREAMBLE_DETECTED
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_HEADER_VALID
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_RX_DONE
SX126xReadCommand
SX126xReadCommand
Rx done: 
48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 

# receive_message
SX126xWakeup

# RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_PREAMBLE_DETECTED
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_HEADER_VALID
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand
IRQ_RX_DONE
SX126xReadCommand
SX126xReadCommand
Rx done: 
48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 

# receive_message
SX126xWakeup
RadioOnRxTimeoutIrq
Rx timeout
```
