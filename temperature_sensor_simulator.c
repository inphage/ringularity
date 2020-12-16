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
#include "nordic_common.h"
#include "sdk_errors.h"
#include "dev_temperature_sensor.h"
#include "temperature_sensor_simulator.h"
#include "sensorsim.h"
#include "stdlib.h"
#include "hw_config.h"

//// TEMPERATURE SENSOR SIMULATOR //
typedef struct device_temperature_simulator_s {
    device_temperature_sensor_t ext_iface;
    sensorsim_cfg_t   sim_cfg;            
    sensorsim_state_t sim_state;          
} device_temperature_simulator_t;

static uint32_t temperature_sim_read_measurement(device_temperature_sensor_t *p_dev, uint32_t *reading) {
    if(p_dev == NULL) {
      return NRF_ERROR_INVALID_PARAM;
    }

    device_temperature_simulator_t *p_sim = (device_temperature_simulator_t *)p_dev;
    if(!p_sim->ext_iface.initialized) {
        return NRF_ERROR_INVALID_STATE;
    }

    *reading = sensorsim_measure(&p_sim->sim_state, &p_sim->sim_cfg);
    
    return NRF_SUCCESS;
}

ret_code_t device$temperature_simulator$open(uint32_t device_id, device_temperature_simulator_params_t *params,
    device_temperature_sensor_t **pp_device) {
    static device_temperature_simulator_t devs[TEMPERATURE_SIMULATOR_COUNT] = {0};
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

    device_temperature_simulator_t *p_sim = &devs[device_id];
   
    p_sim->sim_cfg.min          = MIN_CELCIUS_DEGREES;
    p_sim->sim_cfg.max          = MAX_CELCIUS_DEGRESS;
    p_sim->sim_cfg.incr         = CELCIUS_DEGREES_INCREMENT;
    p_sim->sim_cfg.start_at_max = false;
    sensorsim_init(&p_sim->sim_state, &p_sim->sim_cfg);
    p_sim->ext_iface.read_temperature = temperature_sim_read_measurement;
    p_sim->ext_iface.initialized = true;

    *pp_device = (device_temperature_sensor_t *)p_sim;

    return status;
}
