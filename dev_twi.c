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

#include "dev_twi.h"
#include "nrf_log.h"
#include "hw_config.h"

ret_code_t device$twi_bus$open(uint32_t device_id, nrf_drv_twi_config_t *p_params,
    nrf_twi_sensor_t **pp_device) {
    ret_code_t err_code;

#ifdef TWI_BUS0
    if(0 == device_id) {
        *pp_device = &_twi_bus0;
    } 
#endif
   
#ifdef TWI_BUS1
    if(1 == device_id) {
        *pp_device = &_twi_bus1;
    }
#endif

    err_code = nrf_twi_mngr_init((*pp_device)->p_twi_mngr, p_params);
    NRF_LOG_INFO("nrf_twi_mngr_init: %u", err_code);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_disable(&((*pp_device)->p_twi_mngr->twi)); //workaround for debug anomaly in enable
    nrf_drv_twi_enable(&((*pp_device)->p_twi_mngr->twi));

    err_code = nrf_twi_sensor_init(*pp_device);
    NRF_LOG_INFO("nrf_twi_sensor_init: %u", err_code);
    APP_ERROR_CHECK(err_code);
    
    return err_code;
}