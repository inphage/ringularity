/**
 * Copyright (c) 2021 Open Ring Project, All rights reserved
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
#include "stdbool.h"
#include "app_timer.h"
#include "app_error.h"
#include "nordic_common.h"
#include "sfloat.h"
#include "hw_config.h"
#include "ble.h"
#include "ble_types.h" 
#include "nrf_sdh_ble.h"
#include "ble_plxs.h"
#include "peer_manager.h"
#include "bsp.h"
#include "ble_main.h"
#include "nrf_log.h"
#include "app_scheduler.h"

//// PULSE OXIMETER SERVICE CONTROLLER //
// Implements business logic tying pulse oximeter devices to the pulse oximeter service
BLE_PLXS_DEF(m_plxs);                                               /**< Pulse oximeter service instance. */
APP_TIMER_DEF(m_pulse_oximeter_timer_id);                           /**< Pulse oximeter mreasurement timer. */
static bool m_plxs_meas_ind_conf_pending = false;                   /**< Flag to keep track of when an indication confirmation is pending. */


// @brief Function for handling the pulse oximeter measurement timer timeout.
//
// @details This function will be called each time the heart rate measurement timer expires.
//          It will exclude RR Interval data from every third measurement.
//
// @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
//                       app_start_timer() call to the timeout handler.
static ret_code_t plxs_read_measurement(ble_plxm_t *p_measurement)
{
    //NRF_LOG_INFO("plxs_read_measurement");
    ret_code_t      err_code = NRF_ERROR_INTERNAL;
    uint16_t        pulse_rate;
    uint16_t        spo2;
    ble_plxm_t      plxm = {0}; //pulse oximeter measurement
    bool            pulse_valid = false;
    bool            spo2_valid = false;

    ASSERT(dev_pulse_oximeter0);
    ASSERT(dev_pulse_oximeter0->read_pulse_rate);
    err_code = dev_pulse_oximeter0->read_pulse_rate(dev_pulse_oximeter0, &pulse_rate, &pulse_valid);
    if((err_code) == NRF_SUCCESS) {
        p_measurement->pulse_rate.exponent = 0;
        p_measurement->pulse_rate.mantissa = pulse_rate;
    } else {
        return err_code;
    }

    ASSERT(dev_pulse_oximeter0->read_spo2);
    err_code = dev_pulse_oximeter0->read_spo2(dev_pulse_oximeter0, &spo2, &spo2_valid);
    if((err_code) == NRF_SUCCESS) {
        p_measurement->spo2.exponent = -2;
        p_measurement->spo2.mantissa = spo2;
    } else {
        return err_code;
    }

    return err_code;
}

static void spot_check_measurement_send(void);

static void app_sched_scm_send(void * p_event_data, uint16_t event_size) {
    spot_check_measurement_send();
}

static void schedule_spot_check_measurement_send(void) {
    uint32_t err_code = app_sched_event_put(NULL, 0, app_sched_scm_send);
    APP_ERROR_CHECK(err_code);
}

// @brief Function for simulating and sending one Spot Check Measurement.
static void spot_check_measurement_send(void) {
    ble_plxm_t measurement;
    ret_code_t     err_code;

    if (!m_plxs_meas_ind_conf_pending)
    {
        err_code = plxs_read_measurement(&measurement);
        if((err_code) == NRF_SUCCESS) {
            err_code = ble_plxs_spot_check_measurement_send(&m_plxs, &measurement);
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_plxs_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_BUSY: //not enough data to process, try again later (TODO: event-based method)
                schedule_spot_check_measurement_send();
                break;

            case NRF_ERROR_INVALID_STATE:
            case NRF_ERROR_INTERNAL:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
        }
    }
}

/**@brief Function for handling the Pulse Oximeter Service events.
 *
 * @details This function will be called for all Pulse Oximeter Service events which are passed
 *          to the application.
 *
 * @param[in] p_plxs Pulse Oximeter Service structure.
 * @param[in] p_evt  Event received from the Pulse Oximeter Service.
 */
static void on_plxs_evt(ble_plxs_t * p_plxs, ble_plxs_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_PLXS_EVT_NOTIFICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            spot_check_measurement_send();
            break;

        case BLE_PLXS_EVT_NOTIFICATION_CONFIRMED:
            m_plxs_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void pulse_oximeter_meas_timeout_handler(void * p_context)
{
    //NRF_LOG_INFO("pulse_oximeter_meas_timeout_handler");
    ret_code_t      err_code;
    ble_plxm_t      plxm; //pulse oximeter measurement

    UNUSED_PARAMETER(p_context);
    
    //return if no bluetooth connection
    if(m_plxs.conn_handle == BLE_CONN_HANDLE_INVALID) {
       return;
    }

    plxs_read_measurement(&plxm);

   err_code = ble_plxs_continuous_measurement_send(&m_plxs, &plxm);
   /* if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }*/
}


void srvcon$pulse_oximeter_service$init_timer() {
    ret_code_t      err_code;
    err_code = app_timer_create(&m_pulse_oximeter_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                pulse_oximeter_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void srvcon$pulse_oximeter_service$init() {
    NRF_LOG_INFO("pulse_oximeter_service init");
    ret_code_t         err_code;
    ble_plxs_init_t    plxs_init = {0};

    plxs_init.evt_handler = on_plxs_evt;

    // Pulse Oximeter Service security
    plxs_init.plxm_cccd_wr_sec = SEC_OPEN;
    //plxs_init.racp_rd_sec      = SEC_OPEN;

    err_code = ble_plxs_init(&m_plxs, &plxs_init);
    APP_ERROR_CHECK(err_code);

    srvcon$pulse_oximeter_service$init_timer();
}




void srvcon$pulse_oximeter_service$start() {
    ret_code_t err_code;

    err_code = app_timer_start(m_pulse_oximeter_timer_id, PULSE_OXIMETER_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void send_spot_check_measurement_if_indicated() {
    //err_code = ble_plxs_is_indication_enabled(&m_hts, &is_indication_enabled);
    //if (is_indication_enabled) {
        spot_check_measurement_send();
    //}
}

void srvcon$pulse_oximeter_service$on_pm_evt(pm_evt_t const * p_evt) { 
    if((p_evt->evt_id) == PM_EVT_CONN_SEC_SUCCEEDED) {
        send_spot_check_measurement_if_indicated();
    }
}

void srvcon$pulse_oximeter_service$on_bsp_event(bsp_event_t event) { 
    if((event) == BSP_EVENT_KEY_0 //probably need more-specific means to map this
        && get_conn_handle() != BLE_CONN_HANDLE_INVALID) {
        send_spot_check_measurement_if_indicated();
    }
}

void srvcon$pulse_oximeter_service$on_ble_evt(ble_evt_t const * p_ble_evt) {
    if((p_ble_evt->header.evt_id) == BLE_GAP_EVT_DISCONNECTED) {
      m_plxs_meas_ind_conf_pending = false;
    }
}
