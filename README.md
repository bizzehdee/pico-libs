# Set of libraries for Raspberry Pi Pico pico-sdk

All libraries are native C++ libraries for the Raspberry Pi Pico for use with the pico-sdk.

| Library | Description |
| --- | ----------- |
| MPU6050 | I2C library for mpu6050 gyroscope and accelerometer sensor | 
| ILI934X | SPI library for ILI9341 and ILI9342 display drivers (for TFT screens) |
| BMP280 | I2C library for BMP280 barometric pressure and temperature sensor |
| BH1750 | I2C library for BH1750 ambient light sensor |
| MCP23XXX | I2C library for MCP23X08 and MCP23X17 GPIO expander |
| DS1307 | I2C library for the DS1307 RTC module |

## Usage

You will first need to check out the pico-libs repository, and then copy `external/pico_libs_import.cmake` from the repository to the root of your firmware application.

You can add any library from the pico-lib's adding a libary such as ili934x onto the end of your target_link_libraries in cmake:

```cmake
target_link_libraries([your executable] pico_stdlib ili934x)
```

You can then include 'ili934x.h' and use the following methods from the ili934x class.

