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
#ifndef __DEV_PULSE_OXIMETER_H__
#define __DEV_PULSE_OXIMETER_H__

#include <stdbool.h>
#include "app_error.h"

typedef struct device_pulse_oximeter_s {
    bool initialized;
    ret_code_t (*read_pulse_rate)(struct device_pulse_oximeter_s *p_dev, uint16_t *p_measurement);
    ret_code_t (*read_spo2)(struct device_pulse_oximeter_s *p_dev, uint16_t *p_measurement);
} device_pulse_oximeter_t;


#define DECL_DEV_PULSE_OXIMETER(x) extern device_pulse_oximeter_t *dev_pulse_oximeter ## x;
#define DEF_DEV_PULSE_OXIMETER(x) device_pulse_oximeter_t *dev_pulse_oximeter ## x;


#endif //__DEV_PULSE_OXIMETER_H__