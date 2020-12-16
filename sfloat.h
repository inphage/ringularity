/**
 * Copyright (c) 2020 Open Ring Project, All rights reserved
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without 
 * restriction, including without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 // disclaimer

#ifndef __SFLOAT_H__
#define __SFLOAT_H__

#include "stdint.h"

#define SFLOAT_NAN (0x07FF) //NaN Not a Number
#define SFLOAT_NRES (0x0800) //NRes (not at this resolution)
#define SFLOAT_POS_INF (0x07FE) //+Infinity
#define SFLOAT_NEG_INF (0x0802) //-Infinity
#define SFLOAT_RESERVED (0x0801) //reserved value - do not use
typedef int16_t sfloat;

typedef struct sfloat {
  int32_t exponent;
  int32_t mantissa;
} sfloat_s;

// pack
#define SFLOAT_PACK(x) (sfloat)(((x.exponent<<12)&0xF000)|(x.mantissa&0x0FFF))

// unpack
#define SFLOAT_UNPACK(x) {(x<<12)&0xF00, x&0x0FFF}

#endif //__SFLOAT_H__
