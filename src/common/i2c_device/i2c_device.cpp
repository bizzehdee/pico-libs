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

#include "i2c_device.h"
#include "stdio.h"

I2CDevice::I2CDevice(uint8_t addr, i2c_inst_t *i2cInst)
{
  _addr = addr;
  _i2cInst = i2cInst;
  _begun = false;
  _maxBufferSize = 16;
}

uint8_t I2CDevice::address(void)
{
  return _addr;
}

bool I2CDevice::begin(bool addr_detect)
{
  _begun = true;

  if (addr_detect)
  {
    return detected();
  }
  return true;
}

void I2CDevice::end(void)
{
  _begun = false;
}

bool I2CDevice::detected(void)
{
  if (!_begun && !begin())
  {
    return false;
  }

  if (i2c_write_blocking(_i2cInst, _addr, NULL, 0, false) == 0)
  {
    return true;
  }

  return false;
}

bool I2CDevice::read(uint8_t *buffer, size_t len, bool stop)
{
  size_t pos = 0;
  while (pos < len)
  {
    size_t read_len = ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
    bool read_stop = (pos < (len - read_len)) ? false : stop;
    if (!_read(buffer + pos, read_len, read_stop))
    {
      return false;
    }
    pos += read_len;
  }
  return true;
}

bool I2CDevice::write(const uint8_t *buffer, size_t len, bool stop, const uint8_t *prefix_buffer, size_t prefix_len)
{
  uint8_t fullBuffer[prefix_len+len];

  for(int x=0; x<prefix_len; x++)
  {
    fullBuffer[x] = prefix_buffer[x];
  }

  for(int x=0; x<len; x++)
  {
    fullBuffer[prefix_len+x] = buffer[x];
  }

  if ((len + prefix_len) > maxBufferSize())
  {
    return false;
  }

  if (i2c_write_blocking(_i2cInst, _addr, fullBuffer, prefix_len+len, !stop) != prefix_len+len)
  {
    return false;
  }

  return true;
}

bool I2CDevice::write_then_read(const uint8_t *write_buffer, size_t write_len,
                                uint8_t *read_buffer, size_t read_len,
                                bool stop)
{
  if (!write(write_buffer, write_len, stop))
  {
    return false;
  }

  return read(read_buffer, read_len);
}

bool I2CDevice::setSpeed(uint32_t desiredclk)
{
  (void)desiredclk;
  return false;
}

bool I2CDevice::_read(uint8_t *buffer, size_t len, bool stop)
{
  if (i2c_read_blocking(_i2cInst, _addr, buffer, len, !stop) == len)
  {
    return true;
  }

  return false;
}
