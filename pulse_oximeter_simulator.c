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
#include "dev_include.h"
#include "hw_config.h"
#include "sensorsim.h"
#include "pulse_oximeter_simulator.h"

typedef struct device_pulse_oximeter_sim_s {
    device_pulse_oximeter_t ext_iface;
    sensorsim_cfg_t   pulse_rate_sim_cfg;           
    sensorsim_state_t pulse_rate_sim_state;        
    sensorsim_cfg_t   spo2_sim_cfg;                  
    sensorsim_state_t spo2_sim_state;              
} device_pulse_oximeter_sim_t;

static ret_code_t plx_read_spo2(device_pulse_oximeter_t *p_dev, uint16_t *p_measurement) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_pulse_oximeter_sim_t *p_sim = (device_pulse_oximeter_sim_t *)p_dev;
    *p_measurement = sensorsim_measure(&p_sim->spo2_sim_state, &p_sim->spo2_sim_cfg);

    return NRF_SUCCESS;
}

static ret_code_t plx_read_pulse_rate(device_pulse_oximeter_t *p_dev, uint16_t *p_measurement) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_pulse_oximeter_sim_t *p_sim = (device_pulse_oximeter_sim_t *)p_dev;
    *p_measurement = sensorsim_measure(&p_sim->pulse_rate_sim_state, &p_sim->pulse_rate_sim_cfg);

    return NRF_SUCCESS;
}

ret_code_t device$pulse_oximeter_simulator$open(uint32_t device_id, device_pulse_oximeter_sim_params_t *p_params, 
    device_pulse_oximeter_t **pp_dev) {
    static device_pulse_oximeter_sim_t sims[PULSE_OXIMETER_SIMULATOR_COUNT];
    if(device_id >= PULSE_OXIMETER_SIMULATOR_COUNT) {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if((pp_dev) == NULL) {
        return NRF_ERROR_NULL;
    }

    device_pulse_oximeter_sim_t *p_sim = &sims[device_id];

    p_sim->pulse_rate_sim_cfg.min          = MIN_HEART_RATE;
    p_sim->pulse_rate_sim_cfg.max          = MAX_HEART_RATE;
    p_sim->pulse_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    p_sim->pulse_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->pulse_rate_sim_state, &p_sim->pulse_rate_sim_cfg);
    p_sim->spo2_sim_cfg.min          = MIN_SPO2_LEVEL;
    p_sim->spo2_sim_cfg.max          = MAX_SPO2_LEVEL;
    p_sim->spo2_sim_cfg.incr         = SPO2_LEVEL_INCREMENT;
    p_sim->spo2_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->spo2_sim_state, &p_sim->spo2_sim_cfg);
    p_sim->ext_iface.read_pulse_rate = plx_read_pulse_rate;
    p_sim->ext_iface.read_spo2 = plx_read_spo2;
    p_sim->ext_iface.initialized = true;
    *pp_dev = (device_pulse_oximeter_t *)p_sim;

    return NRF_SUCCESS;    
}
