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
 *
 * NOT FOR USE IN DIAGNOSING, TREATING, CURING OR PREVENTING ANY DISEASE OR CONDITION
 */
#ifndef __DEV_MAX30101_H__
#define __DEV_MAX30101_H__

#include "sdk_errors.h"
#include "dev_pulse_oximeter.h"
#include "max30101.h"

typedef struct device_max30101_params_s {
    nrf_twi_sensor_t * p_sensor_data;
    uint8_t sensor_addr;
} device_max30101_params_t;


ret_code_t device$max30101$open(uint32_t device_id, device_max30101_params_t *p_params, 
    device_pulse_oximeter_t **pp_dev);

ret_code_t max30101_samples_available(max30101_instance_t *p_instance, uint32_t *num_samples_available);

#endif //__DEV_MAX30101_H__