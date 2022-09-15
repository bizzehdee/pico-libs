#include "stdlib.h"
#include "stdio.h"
#include "ili934x.h"
#include "hardware/gpio.h"
#include <cstring>

ILI934X::ILI934X(spi_inst_t *spi, uint8_t cs, uint8_t dc, uint8_t rst, uint16_t width, uint16_t height, ILI934X_ROTATION rotation)
{
    _spi = spi;
    _cs = cs;
    _dc = dc;
    _rst = rst;
    _init_width = _width = width;
    _init_height = _height = height;
    _init_rotation = _rotation = rotation;
}

void ILI934X::reset()
{
    gpio_put(_rst, 0);
    sleep_us(30);
    gpio_put(_rst, 1);
}

void ILI934X::init()
{
    reset();

    _write(_RDDSDR, (uint8_t *)"\x03\x80\x02", 3);
    _write(_PWCRTLB, (uint8_t *)"\x00\xc1\x30", 3);
    _write(_PWRONCTRL, (uint8_t *)"\x64\x03\x12\x81", 4);
    _write(_DTCTRLA, (uint8_t *)"\x85\x00\x78", 3);
    _write(_PWCTRLA, (uint8_t *)"\x39\x2c\x00\x34\x02", 5);
    _write(_PRCTRL, (uint8_t *)"\x20", 1);
    _write(_DTCTRLB, (uint8_t *)"\x00\x00", 2);
    _write(_PWCTRL1, (uint8_t *)"\x23", 1);
    _write(_PWCTRL2, (uint8_t *)"\x10", 1);
    _write(_VMCTRL1, (uint8_t *)"\x3e\x28", 2);
    _write(_VMCTRL2, (uint8_t *)"\x86", 1);

    setRotation(_rotation);

    _write(_PIXSET, (uint8_t *)"\x55", 1);
    _write(_FRMCTR1, (uint8_t *)"\x00\x18", 2);
    _write(_DISCTRL, (uint8_t *)"\x08\x82\x27", 3);
    _write(_ENA3G, (uint8_t *)"\x00", 1);
    _write(_GAMSET, (uint8_t *)"\x01", 1);
    _write(_PGAMCTRL, (uint8_t *)"\x0f\x31\x2b\x0c\x0e\x08\x4e\xf1\x37\x07\x10\x03\x0e\x09\x00", 15);
    _write(_NGAMCTRL, (uint8_t *)"\x00\x0e\x14\x03\x11\x07\x31\xc1\x48\x08\x0f\x0c\x31\x36\x0f", 15);

    _write(_SLPOUT);
    _write(_DISPON);
}

void ILI934X::setRotation(ILI934X_ROTATION rotation)
{
    uint8_t mode = MADCTL_R0DEG | MADCTL_BGR;
    switch (rotation)
    {
    case R0DEG:
        mode = MADCTL_MX | MADCTL_BGR;
        this->_width = this->_init_width;
        this->_height = this->_init_height;
        break;
    case R90DEG:
        mode = MADCTL_MV | MADCTL_BGR;
        this->_width = this->_init_height;
        this->_height = this->_init_width;
        break;
    case R180DEG:
        mode = MADCTL_MY | MADCTL_BGR;
        this->_width = this->_init_width;
        this->_height = this->_init_height;
        break;
    case R270DEG:
        mode = MADCTL_MY | MADCTL_MX | MADCTL_MV | MADCTL_BGR;
        this->_width = this->_init_height;
        this->_height = this->_init_width;
        break;
    case MIRRORED0DEG:
        mode = MADCTL_MY | MADCTL_MX | MADCTL_BGR;
        this->_width = this->_init_width;
        this->_height = this->_init_height;
        break;
    case MIRRORED90DEG:
        mode = MADCTL_MX | MADCTL_MV | MADCTL_BGR;
        this->_width = this->_init_height;
        this->_height = this->_init_width;
        break;
    case MIRRORED180DEG:
        mode = MADCTL_BGR;
        this->_width = this->_init_width;
        this->_height = this->_init_height;
        break;
    case MIRRORED270DEG:
        mode = MADCTL_MY | MADCTL_MV | MADCTL_BGR;
        this->_width = this->_init_height;
        this->_height = this->_init_width;
        break;
    }

    uint8_t buffer[1] = { mode };
    _write(_MADCTL, (uint8_t *)buffer, 1);
}

void ILI934X::setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x < 0 || x >= _width || y < 0 || y >= _height)
        return;

    uint16_t colour[1];
    colour[0] = _colour(r, g, b);

    _writeBlock(x, y, x, y, (uint8_t *)colour, 2);
}

void ILI934X::fillRect(uint16_t x, uint16_t y, uint16_t h, uint16_t w, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t _x = MIN(_width - 1, MAX(0, x));
    uint16_t _y = MIN(_height - 1, MAX(0, y));
    uint16_t _w = MIN(_width - x, MAX(1, w));
    uint16_t _h = MIN(_height - y, MAX(1, h));

    uint16_t buffer[_MAX_CHUNK_SIZE];
    for (int x = 0; x < _MAX_CHUNK_SIZE; x++)
    {
        buffer[x] = _colour(r, g, b);
    }

    uint16_t totalChunks = (uint16_t)((double)(w * h) / _MAX_CHUNK_SIZE);
    uint16_t remaining = (uint16_t)((w * h) % _MAX_CHUNK_SIZE);

    _writeBlock(_x, _y, _x + _w - 1, _y + _h - 1);

    for (uint16_t i = 0; i < totalChunks; i++)
    {
        _data((uint8_t *)buffer, _MAX_CHUNK_SIZE * 2);
    }

    if (remaining > 0)
    {
        _data((uint8_t *)buffer, remaining * 2);
    }
}

void ILI934X::clear(uint8_t r, uint8_t g, uint8_t b)
{
    fillRect(0, 0, _height, _width, r, g, b);
}

void ILI934X::_write(uint8_t cmd, uint8_t *data, size_t dataLen)
{
    gpio_put(_dc, 0);
    gpio_put(_cs, 0);

    // spi write
    uint8_t commandBuffer[1];
    commandBuffer[0] = cmd;

    while (!spi_is_writable(_spi))
    {
        sleep_us(1);
    }

    spi_write_blocking(_spi, commandBuffer, 1);

    gpio_put(_cs, 1);

    // do stuff
    if (data != NULL)
    {
        _data(data, dataLen);
    }
}

void ILI934X::_data(uint8_t *data, size_t dataLen)
{
    gpio_put(_dc, 1);
    gpio_put(_cs, 0);

    spi_write_blocking(_spi, data, dataLen);

    gpio_put(_cs, 1);
}

void ILI934X::_writeBlock(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t *data, size_t dataLen)
{
    uint16_t buffer[2];
    buffer[0] = __builtin_bswap16(x0);
    buffer[1] = __builtin_bswap16(x1);

    _write(_CASET, (uint8_t *)buffer, 4);

    buffer[0] = __builtin_bswap16(y0);
    buffer[1] = __builtin_bswap16(y1);

    _write(_PASET, (uint8_t *)buffer, 4);
    _write(_RAMWR, data, dataLen);
}

uint16_t ILI934X::_colour(uint8_t r, uint8_t g, uint8_t b)
{
    return (r & 0xf8) << 8 | (b & 0xfc) << 3 | g >> 3;
}