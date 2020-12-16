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
#include "battery_simulator.h"
#include "sensorsim.h"
#include "dev_include.h"
#include "hw_config.h"

typedef struct device_battery_sim_s {
    device_battery_t  ext_iface;
    sensorsim_cfg_t   battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
    sensorsim_state_t battery_sim_state;                       /**< Battery Level sensor simulator state. */
} device_battery_sim_t;

static ret_code_t battery_simulator_read_level(device_battery_t *p_dev, uint8_t *p_level) {
    DEV_READ_CHECK_PARAMS(p_dev, p_level);
 
    device_battery_sim_t *p_sim = (device_battery_sim_t*)p_dev;
    *p_level = (uint8_t)sensorsim_measure(&p_sim->battery_sim_state, &p_sim->battery_sim_cfg);
    return NRF_SUCCESS;
}

ret_code_t device$battery_simulator$open(uint32_t device_id, device_battery_simulator_init_t *init, 
      device_battery_t **pp_dev) {
    static device_battery_sim_t devs[DEVICE_BATTERY_COUNT];
    if(device_id >= DEVICE_BATTERY_COUNT) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if((pp_dev) == NULL) {
        return NRF_ERROR_INVALID_PARAM;
    }
    device_battery_sim_t *p_sim = &devs[device_id];

    p_sim->battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    p_sim->battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    p_sim->battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    p_sim->battery_sim_cfg.start_at_max = true;
    sensorsim_init(&p_sim->battery_sim_state, &p_sim->battery_sim_cfg);
    p_sim->ext_iface.read_level = battery_simulator_read_level;
    p_sim->ext_iface.initialized = true;
    *pp_dev = (device_battery_t *)p_sim;
    
    return NRF_SUCCESS;
}
