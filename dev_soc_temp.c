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
#include "app_util.h"
#include "nordic_common.h"
#include "sdk_errors.h"
#include "dev_temperature_sensor.h"
#include "dev_soc_temp.h"
#include "stdlib.h"
#include "hw_config.h"
#include "dev_include.h"
#include "nrf_soc.h"
#include "nrf_log.h"

//// Temperature of the NRF BLE SOC //
static uint32_t read_temperature(device_temperature_sensor_t *p_dev, uint32_t *reading) {
    DEV_READ_CHECK_PARAMS(p_dev, reading);

    uint32_t celciusX100;
    if(sd_temp_get(&celciusX100) == NRF_SUCCESS) {
          celciusX100 *= 25;
          NRF_LOG_INFO("Core temperature: %d.$02d", (int)(celciusX100/100), (int)(celciusX100 % 100));
    }

    *reading = celciusX100;
        
    return NRF_SUCCESS;
}

ret_code_t device$dev_soc_temp$open(uint32_t device_id, device_soc_temperature_params_t *params,
    device_temperature_sensor_t **pp_device) {
    static device_temperature_sensor_t dev = {true, read_temperature};
    uint32_t status = NRF_SUCCESS;
    
    if(device_id >= TEMPERATURE_SIMULATOR_COUNT) {
      return NRF_ERROR_INVALID_PARAM;
    }

    if((pp_device) == NULL) {
      return NRF_ERROR_NULL;
    }

    if(!((*pp_device)->initialized)) {
      return NRF_SUCCESS;
    }

    *pp_device = &dev;

    return status;
}
