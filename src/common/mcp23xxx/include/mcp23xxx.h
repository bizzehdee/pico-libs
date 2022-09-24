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

#ifndef __MCP23XXX_H__
#define __MCP23XXX_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

// registers
#define MCP23XXX_IODIR 0x00   //!< I/O direction register
#define MCP23XXX_IPOL 0x01    //!< Input polarity register
#define MCP23XXX_GPINTEN 0x02 //!< Interrupt-on-change control register
#define MCP23XXX_DEFVAL 0x03  //!< Default compare register for interrupt-on-change
#define MCP23XXX_INTCON 0x04  //!< Interrupt control register
#define MCP23XXX_IOCON 0x05   //!< Configuration register
#define MCP23XXX_GPPU 0x06    //!< Pull-up resistor configuration register
#define MCP23XXX_INTF 0x07    //!< Interrupt flag register
#define MCP23XXX_INTCAP 0x08  //!< Interrupt capture register
#define MCP23XXX_GPIO 0x09    //!< Port register
#define MCP23XXX_OLAT 0x0A    //!< Output latch register

#define MCP23XXX_ADDR 0x20 //!< Default I2C Address

#define MCP_PORT(pin) ((pin < 8) ? 0 : 1) //!< Determine port from pin number

class MCP23XXX
{
public:
   ~MCP23XXX();

   // default to i2c0, pins 4/5
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = MCP23XXX_ADDR);
   void gpio_set_dir(uint8_t pin, bool out);
   void gpio_pull_up(uint8_t pin);
   void gpio_pull_down(uint8_t pin);
   bool gpio_get(uint8_t gpio);
   void gpio_put(uint8_t gpio, bool value);
   uint8_t gpio_get_all(uint8_t port = 0); //0 == A, 1 == B
   void gpio_put_all(uint8_t value, uint8_t port = 0); //0 == A, 1 == B
   void gpio_set_irq_enabled(uint8_t gpio, uint32_t event_mask, bool enabled);

protected:
   uint8_t pinCount = 0; ///< Total number of GPIO pins
private:
   I2CDevice *i2c_dev = NULL;
   uint16_t getRegister(uint8_t baseAddress, uint8_t port = 0);
};

#endif // __MCP23XXX_H__
