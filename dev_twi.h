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

#ifndef __DEV_TWI_H__
#define __DEV_TWI_H__

#include "nrf_drv_twi.h"
#include "nrf_twim.h"
#include "nrf_twi_sensor.h"
#include "nrf_twi_mngr.h"
#include "boards.h"

#define TWI_IDX_0 0
#define TWI_IDX_1 
#define DEV_TWI_MNGR_QUEUE_SIZE 2

#define DECL_DEV_TWI(x) extern nrf_twi_sensor_t *dev_twi_bus ## x;
    
#define DEF_DEV_TWI(x) \
    nrf_drv_twi_t _twi##x = NRF_DRV_TWI_INSTANCE(0);\
    NRF_TWI_MNGR_DEF(_twi ##x## _mngr, DEV_TWI_MNGR_QUEUE_SIZE, x);\
    NRF_TWI_SENSOR_DEF(_twi_bus##x, &_twi##x##_mngr, TWI_MSG_BUFF_SIZE);\
    nrf_twi_sensor_t *dev_twi_bus ## x = &_twi_bus##x;


ret_code_t device$twi_bus$open(uint32_t device_id, nrf_drv_twi_config_t *params,
    nrf_twi_sensor_t **pp_device);

#endif