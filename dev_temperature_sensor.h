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
#ifndef __DEV_TEMPERATURE_SENSOR__
#define __DEV_TEMPERATURE_SENSOR__

#include "stdbool.h"

typedef struct device_temperature_sensor_s {
//    ret_code_t init();
//    ret_code_t start();
    bool initialized;
    uint32_t (*read_temperature)(struct device_temperature_sensor_s *p_dev, uint32_t *reading);
} device_temperature_sensor_t;

#define DECL_DEV_TEMPERATURE_SENSOR(x) extern device_temperature_sensor_t *dev_temperature_sensor ## x;
#define DEF_DEV_TEMPERATURE_SENSOR(x) device_temperature_sensor_t *dev_temperature_sensor ## x;

#endif
