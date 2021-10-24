# LoRa Driver for Semtech SX1262 on Linux (PineDio USB Adapter) and BL602 (PineDio Stack BL604)

[(Follow the updates on Twitter)](https://twitter.com/MisterTechBlog/status/1451548895461326858)

Read the article...

- ["PineCone BL602 Talks LoRaWAN"](https://lupyuen.github.io/articles/lorawan)

The design of the SX1262 Driver is similar to the SX1276 Driver, which is explained in these articles...

- ["Connect PineCone BL602 to LoRa Transceiver"](https://lupyuen.github.io/articles/lora)

- ["PineCone BL602 RISC-V Board Receives LoRa Packets"](https://lupyuen.github.io/articles/lora2)

__CAUTION: Sending a LoRa Message on PineDio USB (not PineDio BL602) above 29 bytes will cause message corruption!__

__CAUTION: Receiving a LoRa Message on PineDio USB (not PineDio BL602) above 28 bytes will cause message corruption!__

(CH341 SPI seems to have trouble transferring a block of 32 bytes)

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

## If we see "spi_ch341_usb: loading out-of-tree module taints kernel",
## Unplug PineDio USB, run "sudo rmmod ch341", plug in PineDio USB again
## and recheck dmesg.

## Download PineDio USB Driver
git clone --recursive https://github.com/lupyuen/lora-sx1262
cd lora-sx1262

## TODO: Edit src/main.c and uncomment READ_REGISTERS, SEND_MESSAGE or RECEIVE_MESSAGE.
## See "PineDio USB Operations" below

## Build PineDio USB Driver
make

## Run PineDio USB Driver Demo.
## See Output Log below.
sudo ./lora-sx1262
```

More about PineDio USB and CH341 SPI:

https://wiki.pine64.org/wiki/JF%27s_note_on_PineDio_devices#RAW_LoRa_communication_between_USB_LoRa_adapter_and_PineDio_STACK

# PineDio USB Operations

The PineDio USB Demo supports 3 operations...

1.  Read SX1262 Registers:

    Edit [`src/main.c`](src/main.c) and uncomment...

    ```c
    #define READ_REGISTERS
    ```

    (See the Read Register Log below)

1.  Send LoRa Message:

    Edit [`src/main.c`](src/main.c) and uncomment...

    ```c
    #define SEND_MESSAGE
    ```

    (See the Send Message Log below)

1.  Receive LoRa Message:

    Edit [`src/main.c`](src/main.c) and uncomment...

    ```c
    #define RECEIVE_MESSAGE
    ```

    (See the Receive Message Log below)

# PineDio USB Output Log

## Read Registers

Read SX1262 Registers on PineDio USB:

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

## Send Message

Transmit 29-byte LoRa Message on PineDio USB:

__CAUTION: Sending a LoRa Message on PineDio USB (not PineDio BL602) above 29 bytes will cause message corruption!__

```text
gcc -o lora-sx1262 \
npl/linux/src/os_eventq.cc \
 \
src/main.o src/radio.o src/sx126x.o src/sx126x-linux.o npl/linux/src/os_callout.o npl/linux/src/os_sem.o npl/linux/src/os_task.o npl/linux/src/os_atomic.o npl/linux/src/os_time.o npl/linux/src/os_mutex.o \
-g -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-old-style-declaration -I include -I npl/linux/include -I npl/linux/include/nimble  \
-pthread -lrt -lstdc++  \

cc1plus: warning: command-line option ‘-Wno-old-style-declaration’ is valid for C/ObjC but not for C++
create_task
init_driver
TODO: SX126xReset
SX126xIoInit
TODO: SX126X interrupt init
SX126xWakeup
sx126x_hal_write: command_length=1, data_length=1
spi tx: c0 00 
spi rx: ac 2c 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: ac ac 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 9d 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 96 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 8f 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetTxParams: power=22, rampTime=7
SX126xGetDeviceId: SX1262
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 08 d8 00 00 
spi rx: a2 a2 a2 a2 fe 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 d8 fe 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 95 04 07 00 01 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 e7 38 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 8e 16 07 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=8
spi tx: 08 ff ff ff ff 00 00 00 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=9
spi tx: 1d 02 9f 00 00 00 00 00 00 00 00 00 00 
spi rx: a2 a2 a2 a2 02 08 ac 08 89 3a a1 65 9a 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=9
spi tx: 1d 02 9f 00 00 00 00 00 00 00 00 00 00 
spi rx: a2 a2 a2 a2 02 08 ac 08 89 3a a1 65 9a 
TODO: SX126xWaitOnBusy
TimerInit
TimerInit
RadioSetChannel: freq=923000000
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 98 e1 e9 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 86 39 b0 00 00 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSetTxConfig: modem=1, power=14, fdev=0, bandwidth=0, datarate=7, coderate=1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=3000
RadioSetTxConfig: SpreadingFactor=7, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=255, CrcMode=1, InvertIQ=0
RadioStandby
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
RadioSetModem
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 8a 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 8b 07 04 01 00 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
spi tx: 8c 00 08 00 ff 01 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 08 89 00 00 
spi rx: a2 a2 a2 a2 04 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 89 04 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetRfTxPower
SX126xSetTxParams: power=14, rampTime=7
SX126xGetDeviceId: SX1262
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 08 d8 00 00 
spi rx: a2 a2 a2 a2 fe 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 d8 fe 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 95 04 07 00 01 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 e7 38 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 8e 0e 07 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSetRxConfig
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 9f 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
RadioStandby
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
RadioSetModem
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 8a 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 8b 07 04 01 00 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
spi tx: 8c 00 08 00 ff 01 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: a0 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 07 36 00 00 
spi rx: a2 a2 a2 a2 0d 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 07 36 0d 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSetRxConfig done
send_message
RadioSend: size=29
50 49 4e 47 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=8
spi tx: 08 02 01 02 01 00 00 00 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=29, CrcMode=1, InvertIQ=0
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
spi tx: 8c 00 08 00 1d 01 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=2, data_length=29
spi tx: 0e 00 50 49 4e 47 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 
spi rx: a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=3
spi tx: 83 00 00 00 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TimerStart
TimerStop
Done!
```

Note that the 29-byte LoRa Message transmitted by PineDio USB is...

```text
send_message
RadioSend: size=29
50 49 4e 47 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 
```

See below for the dmesg Send Message Log.

## Receive Message

When we run this LoRa Transmitter on RAKwireless WisBlock...

https://github.com/lupyuen/wisblock-lora-transmitter/tree/pinedio

28-byte message transmitted by WisBlock is received OK by PineDio USB.

__CAUTION: Receiving a LoRa Message on PineDio USB (not PineDio BL602) above 28 bytes will cause message corruption!__

Here is the PineDio USB Receive Message Log...

(See below for the WisBlock Transmitter Log)

```text
gcc -c -o src/sx126x-linux.o src/sx126x-linux.c -g -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-old-style-declaration -I include -I npl/linux/include -I npl/linux/include/nimble 
gcc -o lora-sx1262 \
npl/linux/src/os_eventq.cc \
 \
src/main.o src/radio.o src/sx126x.o src/sx126x-linux.o npl/linux/src/os_callout.o npl/linux/src/os_sem.o npl/linux/src/os_task.o npl/linux/src/os_atomic.o npl/linux/src/os_time.o npl/linux/src/os_mutex.o \
-g -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-old-style-declaration -I include -I npl/linux/include -I npl/linux/include/nimble  \
-pthread -lrt -lstdc++  \

cc1plus: warning: command-line option ‘-Wno-old-style-declaration’ is valid for C/ObjC but not for C++
create_task
init_driver
TODO: SX126xReset
SX126xIoInit
TODO: SX126X interrupt init
SX126xWakeup
sx126x_hal_write: command_length=1, data_length=1
spi tx: c0 00 
spi rx: a2 22 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 9d 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 96 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 8f 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetTxParams: power=22, rampTime=7
SX126xGetDeviceId: SX1262
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 08 d8 00 00 
spi rx: a2 a2 a2 a2 fe 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 d8 fe 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 95 04 07 00 01 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 e7 38 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 8e 16 07 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=8
spi tx: 08 ff ff ff ff 00 00 00 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=9
spi tx: 1d 02 9f 00 00 00 00 00 00 00 00 00 00 
spi rx: a2 a2 a2 a2 02 08 ac 08 89 aa a9 61 9b 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=9
spi tx: 1d 02 9f 00 00 00 00 00 00 00 00 00 00 
spi rx: a2 a2 a2 a2 02 08 ac 08 89 aa a9 61 9b 
TODO: SX126xWaitOnBusy
TimerInit
TimerInit
RadioSetChannel: freq=923000000
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 98 e1 e9 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 86 39 b0 00 00 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSetTxConfig: modem=1, power=14, fdev=0, bandwidth=0, datarate=7, coderate=1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=3000
RadioSetTxConfig: SpreadingFactor=7, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=255, CrcMode=1, InvertIQ=0
RadioStandby
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
RadioSetModem
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 8a 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 8b 07 04 01 00 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
spi tx: 8c 00 08 00 ff 01 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 08 89 00 00 
spi rx: a2 a2 a2 a2 04 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 89 04 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetRfTxPower
SX126xSetTxParams: power=14, rampTime=7
SX126xGetDeviceId: SX1262
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 08 d8 00 00 
spi rx: a2 a2 a2 a2 fe 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 d8 fe 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1 
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 95 04 07 00 01 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 e7 38 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 8e 0e 07 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSetRxConfig
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 9f 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
RadioStandby
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 80 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
RadioSetModem
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 8a 01 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=4
spi tx: 8b 07 04 01 00 
spi rx: a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=6
spi tx: 8c 00 08 00 ff 01 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: a0 00 
spi rx: a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=4, data_length=1
spi tx: 1d 07 36 00 00 
spi rx: a2 a2 a2 a2 0d 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 07 36 0d 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioSetRxConfig done
receive_message
RadioRx
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=8
spi tx: 08 ff ff ff ff 00 00 00 00 
spi rx: a2 a2 a2 a2 a2 a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TimerStart
TimerStop
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=3, data_length=1
spi tx: 0d 08 ac 94 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=3
spi tx: 82 ff ff ff 
spi rx: a2 a2 a2 a2 
TODO: SX126xWaitOnBusy
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: d2 d2 00 00 
status=0xd2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: d2 d2 d2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: d2 d2 00 00 
status=0xd2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: d2 d2 d2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: d2 d2 00 00 
status=0xd2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: d2 d2 d2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: d2 d2 00 00 
status=0xd2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: d2 d2 d2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: d4 d4 00 16 
status=0xd4
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 16 
spi rx: d4 d4 d4 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
IRQ_RX_DONE
TimerStop
SX126xReadCommand: command=0x13, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 13 00 00 00 
spi rx: d4 d4 1c 00 
status=0xd4
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=3, data_length=28
spi tx: 1e 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
spi rx: d4 d4 d4 48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
TODO: SX126xWaitOnBusy
SX126xReadCommand: command=0x14, size=3
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=3
spi tx: 14 00 00 00 00 
spi rx: d4 d4 1e 2f 1e 
status=0xd4
TODO: SX126xWaitOnBusy
Rx done: 
RadioSleep
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=1
spi tx: 84 04 
spi rx: d4 d4 
48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
IRQ_PREAMBLE_DETECTED
IRQ_HEADER_VALID
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
SX126xWakeup
sx126x_hal_write: command_length=1, data_length=1
spi tx: c0 00 
spi rx: 80 80 
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: a2 a2 00 00 
status=0xa2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: a2 a2 00 00 
status=0xa2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: a2 a2 00 00 
status=0xa2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: a2 a2 00 00 
status=0xa2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
RadioOnDioIrq
RadioIrqProcess
SX126xReadCommand: command=0x12, size=2
TODO: SX126xWaitOnBusy
sx126x_hal_read: command_length=2, data_length=2
spi tx: 12 00 00 00 
spi rx: a2 a2 00 00 
status=0xa2
TODO: SX126xWaitOnBusy
TODO: SX126xWaitOnBusy
sx126x_hal_write: command_length=1, data_length=2
spi tx: 02 00 00 
spi rx: a2 a2 a2 
TODO: SX126xWaitOnBusy
TODO: SX126xGetDio1PinState
Done!
```

Note that the 28-byte LoRa Message received by PineDio USB is...

```text
Rx done: 
48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
IRQ_PREAMBLE_DETECTED
IRQ_HEADER_VALID
```

See below for the dmesg Receive Message Log.

# WisBlock Receiver Log

When we run this LoRa Receiver on RAKwireless WisBlock...

https://github.com/lupyuen/wisblock-lora-receiver

29-byte message transmitted by PineDio USB is received OK by WisBlock...

```text
> Executing task: platformio device monitor <

--- Available filters and text transformations: colorize, debug, default, direct, hexlify, log2file, nocontrol, printable, send_on_enter, time
--- More details at http://bit.ly/pio-monitor-filters
--- Miniterm on /dev/cu.usbmodem14201  9600,8,N,1 ---
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
=====================================
LoRaP2P Rx Test
=====================================
Starting Radio.Rx
OnRxDone: Timestamp=18, RssiValue=-28 dBm, SnrValue=13, Data=50 49 4E 47 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 
```

__CAUTION: Sending a LoRa Message on PineDio USB (not PineDio BL602) above 29 bytes will cause message corruption!__

When we run the [`sdk_app_lora`](https://github.com/lupyuen/bl_iot_sdk/tree/tsen/customer_app/sdk_app_lora) firmware on PineDio Stack BL604, WisBlock receives the 64-byte message OK.

But 64-byte messages sent by PineDio USB are consistently garbled when received by WisBlock.

(CH341 SPI seems to have trouble transferring a block of 32 bytes)

# WisBlock Transmitter Log

When we run this LoRa Transmitter on RAKwireless WisBlock...

https://github.com/lupyuen/wisblock-lora-transmitter/tree/pinedio

28-byte message transmitted by WisBlock is received OK by PineDio USB.

Here's the WisBlock Transmitter Log...

```text
> Executing task: platformio device monitor <

--- Available filters and text transformations: colorize, debug, default, direct, hexlify, log2file, nocontrol, printable, send_on_enter, time
--- More details at http://bit.ly/pio-monitor-filters
--- Miniterm on /dev/cu.usbmodem14201  9600,8,N,1 ---
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
=====================================
LoRap2p Tx Test
=====================================
send: 48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
OnTxDone
send: 48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
OnTxDone
send: 48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
OnTxDone
send: 48 65 6c 6c 6f 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 
OnTxDone
```

__CAUTION: Receiving a LoRa Message on PineDio USB (not PineDio BL602) above 28 bytes will cause message corruption!__

(CH341 SPI seems to have trouble transferring a block of 32 bytes)

# PineDio USB dmesg Log

## Connect USB

dmesg Log when plugging PineDio USB to Pinebook Pro...

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

This means that the newer CH341 SPI Driver has been loaded.

If we see this instead...

```text
usb 3-1: new full-speed USB device number 2 using xhci-hcd
usb 3-1: New USB device found, idVendor=1a86, idProduct=5512, bcdDevice= 3.04
usb 3-1: New USB device strings: Mfr=0, Product=2, SerialNumber=0
usb 3-1: Product: USB UART-LPT
usbcore: registered new interface driver ch341
usbserial: USB Serial support registered for ch341-uart
ch341 3-1:1.0: ch341-uart converter detected
usb 3-1: ch341-uart converter now attached to ttyUSB0
spi_ch341_usb: loading out-of-tree module taints kernel.
usbcore: registered new interface driver spi-ch341-usb
```

It means the older CH341 Non-SPI Driver has been loaded.

To fix this...

1.  Unplug PineDio USB

1.  Enter...

    ```bash
    sudo rmmod ch341
    ```

1.  Plug in PineDio USB

1.  Enter...

    ```bash
    dmesg
    ```

    And recheck the messages.

## Send Message

dmesg Log when PineDio USB is transmitting a 29-byte LoRa Packet...

__CAUTION: Sending a LoRa Message on PineDio USB (not PineDio BL602) above 29 bytes will cause message corruption!__

```text
audit: type=1105 audit(1634994194.295:1270): pid=72110 uid=1000 auid=1000 ses=4 subj==unconfined msg='op=PAM:session_open grantors=pam_limits,pam_unix,pam_permit acct="root" exe="/usr/bin/sudo" hostname=? addr=? terminal=/dev/pts/3 res=success'
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=9, csChange=1, result=9
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=13, csChange=1, result=13
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=13, csChange=1, result=13
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=7, csChange=1, result=7
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=7, csChange=1, result=7
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=9, csChange=1, result=9
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=7, csChange=1, result=7
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=31, csChange=1, result=31
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
audit: type=1701 audit(1634994203.075:1271): auid=1000 uid=0 gid=0 ses=4 subj==unconfined pid=72111 comm="lora-sx1262" exe="/home/luppy/lora-sx1262/lora-sx1262" sig=6 res=1
```

Note that if we try to transmit a 64-byte packet, it won't appear in the dmesg Log.

## Receive Message

dmesg Log when PineDio USB is receiving a 28-byte LoRa Packet...

__CAUTION: Receiving a LoRa Message on PineDio USB (not PineDio BL602) above 28 bytes will cause message corruption!__

```text
audit: type=1105 audit(1635041245.570:352): pid=19949 uid=1000 auid=1000 ses=7 subj==unconfined msg='op=PAM:session_open grantors=pam_limits,pam_unix,pam_permit acct="root" exe="/usr/bin/sudo" hostname=? addr=? terminal=/dev/pts/5 res=success'
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=9, csChange=1, result=9
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=13, csChange=1, result=13
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=13, csChange=1, result=13
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=7, csChange=1, result=7
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=7, csChange=1, result=7
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=9, csChange=1, result=9
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=31, csChange=1, result=31
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=5, csChange=1, result=5
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=2, csChange=1, result=2
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=4, csChange=1, result=4
spi-ch341-usb 3-1:1.0: ch341_spi_transfer_low: len=3, csChange=1, result=3
audit: type=1106 audit(1635041257.940:353): pid=19949 uid=1000 auid=1000 ses=7 subj==unconfined msg='op=PAM:session_close grantors=pam_limits,pam_unix,pam_permit acct="root" exe="/usr/bin/sudo" hostname=? addr=? terminal=/dev/pts/5 res=success'
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
