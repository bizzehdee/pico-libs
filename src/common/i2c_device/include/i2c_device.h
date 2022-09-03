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

#ifndef __I2CDEVICE_H__
#define __I2CDEVICE_H__

#include "hardware/i2c.h"

class I2CDevice
{
public:
    I2CDevice(uint8_t addr, i2c_inst_t *i2cInst = i2c0);
    uint8_t address(void);
    bool begin(bool addr_detect = true);
    void end(void);
    bool detected(void);

    bool read(uint8_t *buffer, size_t len, bool stop = true);
    bool write(const uint8_t *buffer, size_t len, bool stop = true,
                const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
    bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                        uint8_t *read_buffer, size_t read_len,
                        bool stop = false);
    bool setSpeed(uint32_t desiredclk);
    size_t maxBufferSize() { return _maxBufferSize; }
private:
    bool _read(uint8_t *buffer, size_t len, bool stop);
private:
    uint8_t _addr;
    i2c_inst_t *_i2cInst = NULL;
    bool _begun;
    size_t _maxBufferSize;
};

#endif // __I2CDEVICE_H__
