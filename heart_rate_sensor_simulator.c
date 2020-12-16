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
#include "hw_config.h"
#include "sensorsim.h"
#include "dev_include.h"

typedef struct device_heart_rate_sensor_sim_s {
    device_heart_rate_sensor_t pub_iface;
    sensorsim_cfg_t   heart_rate_sim_cfg;            /**< Heart Rate sensor simulator configuration. */
    sensorsim_state_t heart_rate_sim_state;          /**< Heart Rate sensor simulator state. */
    sensorsim_cfg_t   rr_interval_sim_cfg;           /**< RR Interval sensor simulator configuration. */
    sensorsim_state_t rr_interval_sim_state;         /**< RR Interval sensor simulator state. */
} device_heart_rate_sensor_sim_t;


ret_code_t read_heart_rate(device_heart_rate_sensor_t *p_dev, uint16_t *p_measurement) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    
    device_heart_rate_sensor_sim_t *p_sim = (device_heart_rate_sensor_sim_t *)p_dev;
    *p_measurement = (uint16_t)sensorsim_measure(&p_sim->heart_rate_sim_state, &p_sim->heart_rate_sim_cfg);
    p_dev->contact_detected = *p_measurement & 1 ? p_dev->contact_detected : ~p_dev->contact_detected;

    return NRF_SUCCESS;
}

ret_code_t read_rr_interval(device_heart_rate_sensor_t *p_dev, uint16_t *p_meas) {
    DEV_READ_CHECK_PARAMS(p_dev, p_meas);
   
    device_heart_rate_sensor_sim_t *p_sim = (device_heart_rate_sensor_sim_t *)p_dev;
    *p_meas = (uint16_t)sensorsim_measure(&p_sim->rr_interval_sim_state,
                                                  &p_sim->rr_interval_sim_cfg);
    
    static uint32_t cnt;
    p_dev->rr_interval_enabled = ((++cnt % 3) != 0);

    return NRF_SUCCESS;
}


ret_code_t device$heart_rate_sensor_simulator$open(uint32_t device_id, 
      device_heart_rate_sensor_sim_params_t *params, device_heart_rate_sensor_t **pp_dev) {
    static device_heart_rate_sensor_sim_t sims[HEART_RATE_SIMULATOR_COUNT];
    if(device_id >= HEART_RATE_SIMULATOR_COUNT) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if((pp_dev) == NULL) {
        return NRF_ERROR_NULL;
    }
    device_heart_rate_sensor_sim_t *p_sim = &sims[device_id];
    
    p_sim->pub_iface.read_heart_rate = read_heart_rate;
    p_sim->pub_iface.read_rr_interval = read_rr_interval;
    
    p_sim->heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    p_sim->heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    p_sim->heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    p_sim->heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->heart_rate_sim_state, &p_sim->heart_rate_sim_cfg);

    p_sim->rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    p_sim->rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    p_sim->rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    p_sim->rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->rr_interval_sim_state, &p_sim->rr_interval_sim_cfg);
    p_sim->pub_iface.initialized = true;
    *pp_dev = (device_heart_rate_sensor_t *)p_sim;

    return NRF_SUCCESS;
}
