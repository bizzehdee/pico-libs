#ifndef __ILI934X_H__
#define __ILI934X_H__

#include "hardware/spi.h"

#define _RDDSDR 0x0f    // Read Display Self-Diagnostic Result
#define _SLPOUT 0x11    // Sleep Out
#define _GAMSET 0x26    // Gamma Set
#define _DISPOFF 0x28   // Display Off
#define _DISPON 0x29    // Display On
#define _CASET 0x2a     // Column Address Set
#define _PASET 0x2b     // Page Address Set
#define _RAMWR 0x2c     // Memory Write
#define _RAMRD 0x2e     // Memory Read
#define _MADCTL 0x36    // Memory Access Control
#define _VSCRSADD 0x37  // Vertical Scrolling Start Address
#define _PIXSET 0x3a    // Pixel Format Set
#define _PWCTRLA 0xcb   // Power Control A
#define _PWCRTLB 0xcf   // Power Control B
#define _DTCTRLA 0xe8   // Driver Timing Control A
#define _DTCTRLB 0xea   // Driver Timing Control B
#define _PWRONCTRL 0xed // Power on Sequence Control
#define _PRCTRL 0xf7    // Pump Ratio Control
#define _PWCTRL1 0xc0   // Power Control 1
#define _PWCTRL2 0xc1   // Power Control 2
#define _VMCTRL1 0xc5   // VCOM Control 1
#define _VMCTRL2 0xc7   // VCOM Control 2
#define _FRMCTR1 0xb1   // Frame Rate Control 1
#define _DISCTRL 0xb6   // Display Function Control
#define _ENA3G 0xf2     // Enable 3G
#define _PGAMCTRL 0xe0  // Positive Gamma Control
#define _NGAMCTRL 0xe1  // Negative Gamma Control

#define MADCTL_R0DEG 0x40
#define MADCTL_R90DEG 0x20
#define MADCTL_R180DEG 0x80
#define MADCTL_R270DEG 0xE0

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order

#define _MAX_CHUNK_SIZE 4096

enum ILI934X_ROTATION
{
    R0DEG,
    R90DEG,
    R180DEG,
    R270DEG,
    MIRRORED0DEG,
    MIRRORED90DEG,
    MIRRORED180DEG,
    MIRRORED270DEG
};

class ILI934X
{
public:
    ILI934X(spi_inst_t *spi, uint8_t cs, uint8_t dc, uint8_t rst, uint16_t width = 240, uint16_t height = 320, ILI934X_ROTATION rotation = R0DEG);

    void reset();
    void init();
    void setRotation(ILI934X_ROTATION rotation = R0DEG);
    void setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);
    void fillRect(uint16_t x, uint16_t y, uint16_t h, uint16_t w, uint8_t r, uint8_t g, uint8_t b);
    void clear(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);

private:
    void _write(uint8_t cmd, uint8_t *data = NULL, size_t dataLen = 0);
    void _data(uint8_t *data, size_t dataLen = 0);
    void _writeBlock(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t *data = NULL, size_t dataLen = 0);
    uint16_t _colour(uint8_t r, uint8_t g, uint8_t b);

private:
    spi_inst_t *_spi = NULL;
    uint8_t _cs;
    uint8_t _dc;
    uint8_t _rst;
    uint16_t _width;
    uint16_t _height;
    ILI934X_ROTATION _rotation;
    uint16_t _init_width;
    uint16_t _init_height;
    uint8_t _init_rotation;
};

#endif //__ILI934X_H__

// https://github.com/jeffmer/micropython-ili9341/blob/master/ili934xnew.py
