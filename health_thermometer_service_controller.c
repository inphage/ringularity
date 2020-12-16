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
#include "ble_config.h"
#include "hw_config.h"
#include "nrf_log.h"
#include "peer_manager.h"
#include "bsp.h"
#include "ble_hts.h"
#include "health_thermometer_service_controller.h"

static bool m_hts_meas_ind_conf_pending = false;                    /**< Flag to keep track of when an indication confirmation is pending. */

BLE_HTS_DEF(m_hts);                                                 /**< Structure used to identify the health thermometer service. */


/**@brief Function for populating simulated health thermometer measurement.
 */
static void hts_get_measurement(ble_hts_meas_t * p_meas)
{
    //TODO use current time service
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };
    ret_code_t      err_code;
    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

    //err_code dev_temperature_sensor0->read_temperature(dev_thermometer0, &celsiusx100)
    //if((err_code) != NRF_SUCCESS) {
    //    APP_ERROR_HANDLER(err_code);
    //}
    if(sd_temp_get(&celciusX100) == NRF_SUCCESS) {
          celciusX100 *= 25;
          NRF_LOG_INFO("Core temperature: %d.$02d", (int)(celciusX100/100), (int)(celciusX100 % 100));
    }

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    // update simulated time stamp
    // TODO: move to time service simulator
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}

/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(void)
{
    //TODO use current time service
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };
    ble_hts_meas_t hts_measurement = {0};
    ret_code_t     err_code;

    if (!m_hts_meas_ind_conf_pending)
    {
        uint32_t celciusX100;
        dev_temperature_sensor0->read_temperature(dev_temperature_sensor0, &celciusX100);
        hts_measurement.temp_in_celcius.mantissa = celciusX100;
        hts_measurement.temp_in_celcius.exponent = -2;
        hts_measurement.temp_in_fahr.exponent    = -2;
        hts_measurement.temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
        hts_measurement.time_stamp               = time_stamp;
        hts_measurement.temp_type                = BLE_HTS_TEMP_TYPE_FINGER;
        int ntemp = (int)(hts_measurement.temp_in_celcius.mantissa);
        NRF_LOG_INFO("temperature: %d", ntemp);

        err_code = ble_hts_measurement_send(&m_hts, &hts_measurement);

        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}


/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in] p_hts  Health Thermometer Service structure.
 * @param[in] p_evt  Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
}

void srvcon$health_thermometer_service$init() {
    ret_code_t         err_code;
    ble_hts_init_t     hts_init = {0};

    hts_init.evt_handler                 = on_hts_evt;
    hts_init.p_gatt_queue                = get_ble_gatt_queue();
    hts_init.error_handler               = service_error_handler;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    // Here the sec level for the Health Thermometer Service can be changed/increased.
    hts_init.ht_meas_cccd_wr_sec = SEC_JUST_WORKS;
    hts_init.ht_type_rd_sec      = SEC_OPEN;

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);
}

static void temperature_measurement_send_if_indicated() {
    bool is_indication_enabled = false;
    ret_code_t err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
    APP_ERROR_CHECK(err_code);
    if (is_indication_enabled) {
        temperature_measurement_send();
    }
}

void srvcon$health_thermometer_service$on_pm_evt(pm_evt_t const * p_evt) {
    if((p_evt->evt_id) == PM_EVT_CONN_SEC_SUCCEEDED) {
        temperature_measurement_send_if_indicated();
    }
}

void srvcon$health_thermometer_service$on_bsp_event(bsp_event_t event) {
    if((event) == BSP_EVENT_KEY_0) {
      temperature_measurement_send_if_indicated();
    }
}

void srvcon$health_thermometer_service$on_ble_evt(ble_evt_t const * p_ble_evt) {
    if((p_ble_evt->header.evt_id) == BLE_GAP_EVT_DISCONNECTED) {
        m_hts_meas_ind_conf_pending = false;
    }
}
