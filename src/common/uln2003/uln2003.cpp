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

#include "stdlib.h"
#include "stdio.h"
#include "uln2003.h"
#include "hardware/gpio.h"

ULN2003::ULN2003(uint32_t in1, uint32_t in2, uint32_t in3, uint32_t in4)
{
   _pins[0] = in1;
   _pins[1] = in2;
   _pins[2] = in3;
   _pins[3] = in4;
}

void ULN2003::setRpm(int32_t rpm)
{
   _delay = calcDelay(rpm);
}

int32_t ULN2003::getRpm()
{
   return calcRpm(_delay);
}

void ULN2003::step(bool clockwise)
{
   if (clockwise)
   {
      nextCW();
   }
   else
   {
      nextACW();
   }
}

void ULN2003::move(int32_t numSteps, bool clockwise)
{
   for (int32_t n = 0; n < numSteps; n++)
   {
      step(clockwise);
   }
}

void ULN2003::moveTo(int32_t toStep, bool clockwise)
{
   if (toStep >= _totalSteps)
   {
      toStep %= _totalSteps;
   }
   else if (toStep < 0)
   {
      toStep %= _totalSteps;
      if (toStep < 0)
      {
         toStep += _totalSteps;
      }
   }

   while (_stepNumber != toStep)
   {
      step(clockwise);
   }
}

void ULN2003::moveDegrees(int32_t deg, bool clockwise)
{
   int32_t nSteps = (int32_t)deg * _totalSteps / 360;
   move(nSteps, clockwise);
}

void ULN2003::moveToDegree(int32_t deg, bool clockwise)
{
   if (deg >= 360)
   {
      deg %= 360;
   }
   else if (deg < 0)
   {
      deg %= 360;
      if (deg < 0)
      {
         deg += 360;
      }
   }

   int32_t toStep = deg * _totalSteps / 360;
   moveTo(toStep, clockwise);
}

void ULN2003::nextCW()
{
   _seqNumber++;
   if (_seqNumber > 7)
   {
      _seqNumber = 0; // roll over to A seq
   }
   next(_seqNumber);

   _stepNumber++; // track miniSteps
   if (_stepNumber >= _totalSteps)
   {
      _stepNumber -= _totalSteps; // keep stepN within 0-(totalSteps-1)
   }
}

void ULN2003::nextACW()
{
   _seqNumber--;
   if (_seqNumber > 7)
   {
      _seqNumber = 0; // roll over to A seq
   }
   next(_seqNumber);

   _stepNumber--; // track miniSteps
   if (_stepNumber >= _totalSteps)
   {
      _stepNumber += _totalSteps; // keep stepN within 0-(totalSteps-1)
   }
}

void ULN2003::next(int32_t n)
{
   int pattern[4];

   switch (n)
   {
   case 0:
   {
      pattern[0] = 1;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
   }
   case 1:
   {
      pattern[0] = 1;
      pattern[1] = 1;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
   }
   case 2:
   {
      pattern[0] = 0;
      pattern[1] = 1;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
   }
   case 3:
   {
      pattern[0] = 0;
      pattern[1] = 1;
      pattern[2] = 1;
      pattern[3] = 0;
      break;
   }
   case 4:
   {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 1;
      pattern[3] = 0;
      break;
   }
   case 5:
   {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 1;
      pattern[3] = 1;
      break;
   }
   case 6:
   {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 1;
      break;
   }
   case 7:
   {
      pattern[0] = 1;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 1;
      break;
   }
   default:
   {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
   }
   }

   for (int p = 0; p < 4; p++)
   {
      gpio_put(_pins[p], pattern[p] == 1);
   }

   sleep_us(_delay);
}

int32_t ULN2003::calcDelay(int32_t rpm)
{
   if (rpm < MIN_RPM)
   {
      return _delay; // will overheat, no change
   }
   else if (rpm >= MAX_RPM)
   {
      return 600; // highest speed
   }

   uint32_t d = 60000000 / (_totalSteps * (uint32_t)rpm);
   // in range: 600-1465 microseconds (24-1 rpm)
   return (int)d;
}

int32_t ULN2003::calcRpm(int32_t delay)
{
   uint32_t rpm = 60000000 / (uint32_t)delay / _totalSteps;
   return (int32_t)rpm;
}
