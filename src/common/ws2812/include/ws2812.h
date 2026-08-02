/* Original Source Ported From https://github.com/adafruit/Adafruit_NeoPixel */
/*
BSD 3-Clause License

Copyright (c) 2022, Darren Horrocks
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __WS2812_H__
#define __WS2812_H__

#include "hardware/pio.h"

//! Driver for WS2812/WS2812B/SK6812 addressable RGB(W) LED strips, using a
//! PIO state machine to generate the one-wire data stream.
class WS2812
{
public:
   WS2812(uint pin, uint16_t numPixels, bool isRGBW = false, float frequencyHz = 800000.0f);
   ~WS2812();

   // claims a PIO state machine and program slot, and allocates the pixel buffer
   bool begin(PIO pioInst = pio0);

   // pushes the pixel buffer out over the data line
   void show(void);

   void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
   void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
   void setPixelColor(uint16_t n, uint32_t c);
   uint32_t getPixelColor(uint16_t n);

   void fill(uint32_t c = 0, uint16_t first = 0, uint16_t count = 0);
   void clear(void);

   uint16_t numPixels(void) { return numLEDs; }
   uint8_t *getPixels(void) { return pixels; }

   static uint32_t Color(uint8_t r, uint8_t g, uint8_t b);
   static uint32_t Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

private:
   uint pin;
   uint16_t numLEDs;
   uint16_t numBytes;
   bool isRGBW;
   float frequencyHz;
   uint8_t *pixels = NULL;

   PIO pio = NULL;
   uint pio_sm = 0;
   uint pio_program_offset = 0;
   bool begun = false;
};

#endif // __WS2812_H__
