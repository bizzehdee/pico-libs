#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ili934x.h"

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define PIN_DC   8
#define PIN_RST  9

#define SPI_PORT spi0

ILI934X *display = NULL;

uint64_t lastTime1hz;

int main()
{
    stdio_init_all();

    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    display = new ILI934X(SPI_PORT, PIN_CS, PIN_DC, PIN_RST);
    display->reset();
    display->init();

    while (1)
    {
        
        if (lastTime1hz + 1000000000 < time_us_64())
        {
            lastTime1hz = time_us_64();
            
            display->clear();
            display->drawCircle(50, 50, 25, display->colour565(255,0,0));
        }
        
    }

    return 0;
}
