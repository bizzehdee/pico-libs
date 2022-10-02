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

#ifndef __ULN2003_H__
#define __ULN2003_H__

#include "pico/stdlib.h"

#define MIN_RPM 6
#define MAX_RPM 24

class ULN2003
{
public:
    ULN2003(uint32_t in1, uint32_t in2, uint32_t in3, uint32_t in4);

    void setRpm(int32_t rpm); // sets speed (10-24 rpm, hi-low torque)
    int32_t getRpm();

    void set4076StepMode() { _totalSteps = 4076; }
    void setTotalSteps(int32_t numSteps) { _totalSteps = numSteps; }

    void step(bool clockwise = true);

    /* blocking */
    void move(int32_t numSteps, bool clockwise = true);
    void moveTo(int32_t toStep, bool clockwise = true);
    void moveDegrees(int32_t deg, bool clockwise = true);
    void moveToDegree(int32_t deg, bool clockwise = true);

private:
    void nextCW();
    void nextACW();
    void next(int32_t n);
    int32_t calcDelay(int32_t rpm);
    int32_t calcRpm(int32_t delay);

private:
    int32_t _totalSteps = 4096;
    uint32_t _delay = 900;
    uint32_t _pins[4];

    int32_t _stepNumber = 0;
    int32_t _seqNumber = -1;
};

#endif //__ULN2003_H__
