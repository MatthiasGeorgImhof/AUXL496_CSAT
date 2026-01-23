/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *  Copyright 2020 SimpleMethod
 *
 *  adapted for C++ and streamlined
 *  Matthias G Imhof
 *  01.06.2025
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#include "GNSS.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32l4xx_hal.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

SimulatedGNSS::SimulatedGNSS(int32_t error_meters) : error_meters(error_meters) {}

int32_t SimulatedGNSS::noise() const
{
	int32_t error_cm_meters = error_meters * 100;
	return std::rand() % (2*error_cm_meters + 1) - error_cm_meters;
}

std::optional<PositionECEF> SimulatedGNSS::getNavPosECEF()
{
    // Approximate ECEF coordinates for Katy, TX (WGS84)
    constexpr int32_t baseX = -1489000 * 100; // meters
    constexpr int32_t baseY = -4743000 * 100;
    constexpr int32_t baseZ = 3980000 * 100;
    constexpr uint32_t baseAcc = 100 * 100; // simulated accuracy

    return PositionECEF{
        .ecefX = baseX + noise(),
        .ecefY = baseY + noise(),
        .ecefZ = baseZ + noise(),
        .pAcc = baseAcc};
}
