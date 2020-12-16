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
#include "app_timer.h"
#include "ble_rscs.h"
#include "hw_config.h"

APP_TIMER_DEF(m_rsc_meas_timer_id);                                 /**< RSC measurement timer. */
BLE_RSCS_DEF(m_rscs);                                               /**< Structure used to identify the running speed and cadence service. */


/**@brief Function for populating simulated running speed and cadence measurement.
 */
static void rsc_read_measurement(ble_rscs_meas_t * p_measurement)
{
    p_measurement->is_inst_stride_len_present = true;
    p_measurement->is_total_distance_present  = false;
    p_measurement->is_running                 = false;

    dev_inertial_measurement_unit0->read_speed(dev_inertial_measurement_unit0, &p_measurement->inst_speed);
    dev_inertial_measurement_unit0->read_cadence(dev_inertial_measurement_unit0, &p_measurement->inst_cadence);
    dev_inertial_measurement_unit0->read_stride_length(dev_inertial_measurement_unit0, &p_measurement->inst_stride_length);

    if (p_measurement->inst_speed > (uint32_t)(MIN_RUNNING_SPEED * 256))
    {
        p_measurement->is_running = true;
    }
}

/**@brief Function for handling the Running Speed and Cadence measurement timer timeout.
 *
 * @details This function will be called each time the running speed and cadence
 *          measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void rsc_meas_timeout_handler(void * p_context)
{
    ret_code_t      err_code;
    ble_rscs_meas_t rscs_measurement;

    UNUSED_PARAMETER(p_context);

    rsc_read_measurement(&rscs_measurement);
    err_code = ble_rscs_measurement_send(&m_rscs, &rscs_measurement);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

void srvcon$running_speed_cadence_service$init_timer() {
    ret_code_t err_code;
    uint32_t rsc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL);
    err_code = app_timer_create(&m_rsc_meas_timer_id, APP_TIMER_MODE_REPEATED, rsc_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void srvcon$running_speed_cadence_service$init() {
    ret_code_t         err_code;
    ble_rscs_init_t    rscs_init = {0};

    rscs_init.evt_handler = NULL;
    rscs_init.feature     = BLE_RSCS_FEATURE_INSTANT_STRIDE_LEN_BIT |
                            BLE_RSCS_FEATURE_WALKING_OR_RUNNING_STATUS_BIT;

    rscs_init.initial_rcm.is_inst_stride_len_present = true;
    rscs_init.initial_rcm.is_total_distance_present  = false;
    rscs_init.initial_rcm.is_running                 = false;
    rscs_init.initial_rcm.inst_stride_length         = 0;

    rscs_init.rsc_feature_rd_sec   = SEC_OPEN;
    rscs_init.rsc_meas_cccd_wr_sec = SEC_OPEN;

    err_code = ble_rscs_init(&m_rscs, &rscs_init);
    APP_ERROR_CHECK(err_code);

    srvcon$running_speed_cadence_service$init_timer();
}


void srvcon$running_speed_cadence_service$start() {
    ret_code_t err_code;
    err_code = app_timer_start(m_rsc_meas_timer_id, SPEED_AND_CADENCE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
