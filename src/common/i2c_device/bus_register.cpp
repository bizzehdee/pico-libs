/* Original Source Ported From https://github.com/adafruit/Adafruit_BusIO */
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

#include "bus_register.h"

BusRegister::BusRegister(I2CDevice *i2cdevice, uint16_t reg_addr,
                         uint8_t width, uint8_t byteorder,
                         uint8_t address_width)
{
    _i2cdevice = i2cdevice;
    _addrwidth = address_width;
    _address = reg_addr;
    _byteorder = byteorder;
    _width = width;
}

bool BusRegister::read(uint8_t *buffer, uint8_t len)
{
    uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                            (uint8_t)(_address >> 8)};

    return _i2cdevice->write_then_read(addrbuffer, _addrwidth, buffer, len);
}

bool BusRegister::read(uint8_t *value)
{
  if (!read(_buffer, 1)) {
    return false;
  }

  *value = _buffer[0];
  return true;
}

bool BusRegister::read(uint16_t *value)
{
  if (!read(_buffer, 2)) {
    return false;
  }

  if (_byteorder == LSBFIRST) {
    *value = _buffer[1];
    *value <<= 8;
    *value |= _buffer[0];
  } else {
    *value = _buffer[0];
    *value <<= 8;
    *value |= _buffer[1];
  }
  return true;
}

uint32_t BusRegister::read(void)
{
  if (!read(_buffer, _width)) {
    return -1;
  }

  uint32_t value = 0;

  for (int i = 0; i < _width; i++) {
    value <<= 8;
    if (_byteorder == LSBFIRST) {
      value |= _buffer[_width - i - 1];
    } else {
      value |= _buffer[i];
    }
  }

  return value;
}

uint32_t BusRegister::readCached(void)
{
    return _cached;
}

bool BusRegister::write(uint8_t *buffer, uint8_t len)
{
    uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                            (uint8_t)(_address >> 8)};
    return _i2cdevice->write(buffer, len, true, addrbuffer, _addrwidth);
}

bool BusRegister::write(uint32_t value, uint8_t numbytes)
{
    if (numbytes == 0)
    {
        numbytes = _width;
    }
    if (numbytes > 4)
    {
        return false;
    }

    // store a copy
    _cached = value;

    for (int i = 0; i < numbytes; i++)
    {
        if (_byteorder == LSBFIRST)
        {
            _buffer[i] = value & 0xFF;
        }
        else
        {
            _buffer[numbytes - i - 1] = value & 0xFF;
        }
        value >>= 8;
    }
    return write(_buffer, numbytes);
}

uint8_t BusRegister::width(void)
{
    return _width;
}

void BusRegister::setWidth(uint8_t width)
{
    _width = width;
}

void BusRegister::setAddress(uint16_t address)
{
    _address = address;
}

void BusRegister::setAddressWidth(uint16_t address_width)
{
    _addrwidth = address_width;
}
