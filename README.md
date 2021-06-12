# DWM1004C-anchor
The purpose of this repository is to create a prototype UWB anchor using the DWM1004C board.

## Requirements / dependencies: 
* Libopencm3 for hardware abstraction / register definitions.
* The arm-none-eabi toolchain for compilation and linking 
* OpenOCD for uploading and debugging.

## TODO:

* create interface for connection to dwm chip.
* create interface for connection to accelerometer chip.


## notes:

When using either LIS3DH_I2C_ADDRESS_1 or LIS3DH_I2C_ADDRESS_2 when reading data over I2C. The response is an immediate NAK with the repeated address.

When explicitly using 0x33. (read with SAD set) it doubles the address in the NAK response. 
When using 0x30 or 0x32. the same doubling of the address in the NAK occurs.

When using completely alternate addresses found in other documentation:
*I2C PinsSCL - I2C clock pin, connect to your microcontrollers I2C clock line. Has a 10K pullup already on it.SDA - I2C data pin, connect to your microcontrollers I2C data line. Has a 10K pullup already on it.To use I2C, keep the CS pin either disconnected or tied to a high (3-5V) logic level.SDO - When in I2C mode, this pin can be used for address selection. When connected to GND or leftopen, the address is 0x18 - it can also be connected to 3.3V to set the address to 0x19STEMMA QT (https://adafru.it/Ft4) - These connectors allow you to connectors to dev boardswith STEMMA QT connectors or to other things with various associatedaccessories (https://adafru.it/Ft6).*
0x18: 0x30NAK
0x19: 0x32NAK



27us per clock pulse.
2.22



#define LIS3DH_I2C_ADDRESS_1           0x30  // SDO pin is low
#define LIS3DH_I2C_ADDRESS_2           0x32  // SDO pin is high

#define LIS3DH_I2C_ALT_ADDR_1          0x18 // SDO pin is low
#define LIS3DH_I2C_ALT_ADDR_2          0x19 // SDO pin is high

0x19h = 25d -> 0x32h = 50d


7.2 us per clock pulse.

