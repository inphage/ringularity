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
#ifndef __DEV_INERTIAL_MEASUREMENT_UNIT__
#define __DEV_INERTIAL_MEASUREMENT_UNIT__

typedef struct device_inertial_measurement_unit_s {
    bool initialized;
    ret_code_t (*read_speed)(struct device_inertial_measurement_unit_s *dev, uint16_t *measurement);
    ret_code_t (*read_cadence)(struct device_inertial_measurement_unit_s *dev, uint8_t *measurement);
    ret_code_t (*read_stride_length)(struct device_inertial_measurement_unit_s *dev, uint16_t *measurement);
} device_inertial_measurement_unit_t;

#define DECL_DEV_INERTIAL_MEASUREMENT_UNIT(x) extern device_inertial_measurement_unit_t *dev_inertial_measurement_unit ## x;
#define DEF_DEV_INERTIAL_MEASUREMENT_UNIT(x) device_inertial_measurement_unit_t *dev_inertial_measurement_unit ## x;

#endif //__DEV_INERTIAL_MEASUREMENT_UNIT__