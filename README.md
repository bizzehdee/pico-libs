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
| ULN2003 | GPIO library for the generic/cheap ULN2003 stepper motor driver |
| PCA9685 | I2C library for the PCA9685 PWM 16 Channel Servo Driver |
| SSD1306 | I2C library for the SSD1306 OLED display driver |
| LIS331 | I2C library for the LIS331/LIS331HH triple-axis accelerometer |
| WS2812 | PIO library for WS2812/WS2812B/SK6812 addressable RGB(W) LED strips |
| RTClib | I2C library for the DS3231 and PCF8523 RTC modules |
| PN532 | I2C library for the PN532 NFC/RFID reader |
| DRV8830 | I2C library for the DRV8830 DC motor driver |
| ADS1X15 | I2C library for the ADS1015/ADS1115 4-channel ADCs |
| CCS811 | I2C library for the CCS811 eCO2/TVOC air quality sensor |
| MPL115A2 | I2C library for the MPL115A2 barometric pressure and temperature sensor |

## Usage

You will first need to check out the pico-libs repository, and then copy `external/pico_libs_import.cmake` from the repository to the root of your firmware application.

You can add any library from the pico-lib's adding a libary such as ili934x onto the end of your target_link_libraries in cmake:

```cmake
target_link_libraries([your executable] pico_stdlib ili934x)
```

You can then include 'ili934x.h' and use the following methods from the ili934x class.

