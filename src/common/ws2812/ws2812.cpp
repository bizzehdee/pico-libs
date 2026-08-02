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

#include "ws2812.h"
#include "ws2812_pio_program.h"
#include <pico/time.h>
#include <string.h>

WS2812::WS2812(uint pinNum, uint16_t numPixels, bool rgbw, float freq)
{
   pin = pinNum;
   numLEDs = numPixels;
   isRGBW = rgbw;
   frequencyHz = freq;
   numBytes = numLEDs * (isRGBW ? 4 : 3);
}

WS2812::~WS2812()
{
   if (pixels)
   {
      delete[] pixels;
   }

   if (begun)
   {
      pio_sm_set_enabled(pio, pio_sm, false);
      pio_sm_unclaim(pio, pio_sm);
      pio_remove_program(pio, &ws2812_program, pio_program_offset);
   }
}

bool WS2812::begin(PIO pioInst)
{
   pio = pioInst;

   if (!pio_can_add_program(pio, &ws2812_program))
   {
      return false;
   }

   pio_program_offset = pio_add_program(pio, &ws2812_program);

   int sm = pio_claim_unused_sm(pio, false);
   if (sm < 0)
   {
      pio_remove_program(pio, &ws2812_program, pio_program_offset);
      return false;
   }
   pio_sm = (uint)sm;

   ws2812_program_init(pio, pio_sm, pio_program_offset, pin, frequencyHz);

   pixels = new uint8_t[numBytes];
   begun = true;

   clear();

   return true;
}

void WS2812::show(void)
{
   if (!begun)
   {
      return;
   }

   uint8_t *ptr = pixels;
   uint32_t count = numBytes;
   while (count--)
   {
      // bits for transmission must be shifted to the top 8 bits
      pio_sm_put_blocking(pio, pio_sm, ((uint32_t)*ptr++) << 24);
   }

   sleep_us(300); // latch delay
}

void WS2812::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
   setPixelColor(n, r, g, b, 0);
}

void WS2812::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
   if (!pixels || n >= numLEDs)
   {
      return;
   }

   uint8_t *p = &pixels[n * (isRGBW ? 4 : 3)];
   p[0] = g;
   p[1] = r;
   p[2] = b;
   if (isRGBW)
   {
      p[3] = w;
   }
}

void WS2812::setPixelColor(uint16_t n, uint32_t c)
{
   setPixelColor(n, (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c, (uint8_t)(c >> 24));
}

uint32_t WS2812::getPixelColor(uint16_t n)
{
   if (!pixels || n >= numLEDs)
   {
      return 0;
   }

   uint8_t *p = &pixels[n * (isRGBW ? 4 : 3)];
   uint32_t g = p[0], r = p[1], b = p[2];
   uint32_t w = isRGBW ? p[3] : 0;

   return (w << 24) | (r << 16) | (g << 8) | b;
}

void WS2812::fill(uint32_t c, uint16_t first, uint16_t count)
{
   if (first >= numLEDs)
   {
      return;
   }

   uint16_t last = numLEDs;
   if (count > 0 && (uint32_t)(first + count) < last)
   {
      last = first + count;
   }

   for (uint16_t i = first; i < last; i++)
   {
      setPixelColor(i, c);
   }
}

void WS2812::clear(void)
{
   if (pixels)
   {
      memset(pixels, 0, numBytes);
   }
}

uint32_t WS2812::Color(uint8_t r, uint8_t g, uint8_t b)
{
   return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t WS2812::Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
   return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
