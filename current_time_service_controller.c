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
#include "ble_cts_c.h"
#include "nrf_log.h"
#include "app_error.h"
#include "ble_config.h"
#include "bsp.h"
#include "app_util.h"

static CTS_C_EVT_HANDLERS(cts_c_evt_handlers);

BLE_CTS_C_DEF(m_cts_c);                                             /**< Current Time service instance. */

/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t * p_evt)
{
  static char const * day_of_week[] =
  {
      "Unknown",
      "Monday",
      "Tuesday",
      "Wednesday",
      "Thursday",
      "Friday",
      "Saturday",
      "Sunday"
  };

static char const * month_of_year[] =
{
    "Unknown",
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"
};


    NRF_LOG_INFO("\r\nCurrent Time:");
    NRF_LOG_INFO("\r\nDate:");

    NRF_LOG_INFO("\tDay of week   %s", (uint32_t)day_of_week[p_evt->
                                                         params.
                                                         current_time.
                                                         exact_time_256.
                                                         day_date_time.
                                                         day_of_week]);

    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.day == 0)
    {
        NRF_LOG_INFO("\tDay of month  Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tDay of month  %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.day);
    }

    NRF_LOG_INFO("\tMonth of year %s",
    (uint32_t)month_of_year[p_evt->params.current_time.exact_time_256.day_date_time.date_time.month]);
    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.year == 0)
    {
        NRF_LOG_INFO("\tYear          Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tYear          %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
    }
    NRF_LOG_INFO("\r\nTime:");
    NRF_LOG_INFO("\tHours     %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours);
    NRF_LOG_INFO("\tMinutes   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes);
    NRF_LOG_INFO("\tSeconds   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);
    NRF_LOG_INFO("\tFractions %i/256 of a second",
                   p_evt->params.current_time.exact_time_256.fractions256);

    NRF_LOG_INFO("\r\nAdjust reason:\r");
    NRF_LOG_INFO("\tDaylight savings %x",
                   p_evt->params.current_time.adjust_reason.change_of_daylight_savings_time);
    NRF_LOG_INFO("\tTime zone        %x",
                   p_evt->params.current_time.adjust_reason.change_of_time_zone);
    NRF_LOG_INFO("\tExternal update  %x",
                   p_evt->params.current_time.adjust_reason.external_reference_time_update);
    NRF_LOG_INFO("\tManual update    %x",
                   p_evt->params.current_time.adjust_reason.manual_time_update);
}

static void current_time_service$on_cts_c_evt_discovery_complete(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt) {
    NRF_LOG_INFO("Current Time Service discovered on server.");
    ret_code_t err_code = ble_cts_c_handles_assign(&m_cts_c,
                                        p_evt->conn_handle,
                                        &p_evt->params.char_handles);
    APP_ERROR_CHECK(err_code);
}


// BLE_CTS_C_EVT_DISCOVERY_FAILED
static void current_time_service$on_ble_cts_c_evt_discovery_failed(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt) {
    NRF_LOG_INFO("Current Time Service not found on server. ");
    // CTS not found in this case we just disconnect. There is no reason to stay
    // in the connection for this simple app since it all wants is to interact with CT
    if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID) {
        DISPATCH_EVT2(cts_c_evt_handlers, p_cts, p_evt);
    }
}



// BLE_CTS_C_EVT_CURRENT_TIME
static void current_time_service$on_ble_cts_c_evt_current_time(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt) {
    NRF_LOG_INFO("Current Time received.");
    current_time_print(p_evt);
}

/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void srvcon$current_time_service$on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            current_time_service$on_cts_c_evt_discovery_complete(p_cts, p_evt);
            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            current_time_service$on_ble_cts_c_evt_discovery_failed(p_cts, p_evt);
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            current_time_service$on_ble_cts_c_evt_current_time(p_cts, p_evt);
            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.");
            break;

        default:
            break;
    }
}


void srvcon$current_time_service$init() {
    ret_code_t         err_code;
    ble_cts_c_init_t   cts_init = {0};
    cts_init.evt_handler   = srvcon$current_time_service$on_cts_c_evt;
    cts_init.error_handler = service_error_handler;
    cts_init.p_gatt_queue  = get_ble_gatt_queue();
    err_code               = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);
}


void srvcon$current_time_service$on_db_disc_evt(ble_db_discovery_evt_t * p_evt) {
    ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
}


void srvcon$current_time_service_client$on_bsp_event(bsp_event_t event) {
    if((BSP_EVENT_KEY_0) == event) {
        ret_code_t err_code = ble_cts_c_current_time_read(&m_cts_c);
        if (err_code == NRF_ERROR_NOT_FOUND) {
            NRF_LOG_INFO("Current Time Service is not discovered.");
        }
    }
}

void srvcon$current_time_service_client$on_ble_evt(ble_evt_t const * p_ble_evt) {
    NRF_LOG_INFO("Disconnected"); //, reason %d.",
                  //p_ble_evt->evt.gap_evt.params.disconnected.reason);
    if((p_ble_evt->header.evt_id) == BLE_GAP_EVT_DISCONNECTED
        && p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)  {
        m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
    }
}