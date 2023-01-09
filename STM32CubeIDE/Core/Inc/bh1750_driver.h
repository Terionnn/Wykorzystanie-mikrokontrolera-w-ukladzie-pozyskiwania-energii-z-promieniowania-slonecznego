/*
 * bh1750_driver.h
 *
 *  Created on: 17 sie 2022
 *      Author: Kamil
 *
 *      Na podstawie: https://github.com/lamik/Light_Sensors_STM32
 *
Copyright (c) 2018 Mateusz Salamon <mateusz@msalamon.pl>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
 */

#ifndef INC_BH1750_DRIVER_H_
#define INC_BH1750_DRIVER_H_
#define BH1750_ADDR	(0x23<<1)

void BH1750_PowerOn();
void BH1750_SetContinuousMode();
uint32_t BH1750_GetIlluminance();

#endif
