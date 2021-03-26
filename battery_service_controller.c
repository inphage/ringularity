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
#include "sdk_errors.h"
#include "battery_service_controller.h"
#include "hw_config.h"
#include "sim_config.h"
#include "app_timer.h"
#include "ble_bas.h"

//// BATTERY CONTROLLER //
APP_TIMER_DEF(m_battery_timer_id);                                  //< Battery timer. 
BLE_BAS_DEF(m_bas);                                                 //< Structure used to identify the battery service.


//@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    err_code = dev_battery0->read_level(dev_battery0, &battery_level);
    if(err_code != NRF_SUCCESS) {
      APP_ERROR_HANDLER(err_code);
    }
    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
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


//@brief Function for handling the Battery measurement timer timeout.
//
// @details This function will be called each time the battery level measurement timer expires.
//
// @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
//                        app_start_timer() call to the timeout handler.
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

static void init_timer() {
    ret_code_t      err_code;
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void srvcon$battery_service$start() {
   ret_code_t err_code;
   err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
   APP_ERROR_CHECK(err_code);
}

void srvcon$battery_service$init() {
    ret_code_t         err_code;
    ble_bas_init_t     bas_init = {0};

    // Security level for battery service is set here
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    init_timer();
}
