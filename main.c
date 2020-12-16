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
// @file
//
// @defgroup main.c
// @{
// @ingroup ringularity
// @brief Ringularity operating environment main file
//
// Contains top-level functionality & glue

//#include "nordic_common.h"
//#include "nrf.h"
//#include "app_error.h"
#include "app_scheduler.h"
//#include "ble.h"
//#include "ble_bas.h"
//#include "ble_cts_c.h"
//#include "ble_hrs.h"
//#include "ble_hts.h"
//#include "ble_dis.h"
//#include "ble_plxs.h"
//#include "ble_rscs.h"
//#include "ble_srv_common.h"
//#include "ble_advdata.h"
//#include "ble_advertising.h"
//#include "ble_conn_params.h"
//#include "ble_conn_state.h"
//#include "nrf_fstorage.h"
//#include "nrf_sdh.h"
//#include "nrf_soc.h"
//#include "nrf_sdh_soc.h"
//#include "nrf_sdh_ble.h"
//#include "nrf_delay.h"
//#include "nrf_temp.h"
//#include "app_timer.h"
//#include "peer_manager.h"
//#include "peer_manager_handler.h"
//#include "bsp.h"
//#include "bsp_btn_ble.h"
//#include "sensorsim.h"
//#include "nrf_ble_gatt.h"
//#include "nrf_ble_qwr.h"
//#include "nrf_ble_bms.h"

//#include "nrf_ble_lesc.h"
//#include "nrf_ble_scan.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "hw_config.h"
#include "ble_config.h"
#include "app_util.h"
#include "ble_main.h"
#include "ui.h"
#include "ble_main.h"

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Features
// over-the-air firmware updates
// sensors:
//  Pulse Oximeter: SpO2, heart rate
//  Temperature: Body temperature
//  Inertial Measurement: steps, movement, interrupt to turn on SOC
//  Battery level
//  Touch

// Configurations:
//  * Physical hardware
//  * Simulation mode
//  * Unit test mode

// Device Stack
// * factory method
// * init method
// * init timers
// * update method
// * error method?
// Also: need #define to bind hardware driver to device service layer

DEVICE_DEFS;
DEVICE_OPENER_DEFS;

// @brief Function for initializing the nrf log module.
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//NOTE: probably do not want shut-down if no button available to wake up
//
// @brief Function for shutdown events.
//
// @param[in]   event       Shutdown type.
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    UI_INDICATION_SET(BSP_INDICATE_IDLE);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            UI_SLEEP_MODE_PREPARE();
            break;
        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


// @brief Initializes power management
static void power_management_init(void) {
    NRF_LOG_INFO("Power management init...");
    ret_code_t err_code;
    
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

// @brief Function for the Event Scheduler initialization.
static void scheduler_init(void) {
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/// @brief Function for the Timer initialization.
//
// @details Initializes the timer module. This creates and starts application timers.
static void timer_module_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

// @brief Function for putting the chip into sleep mode.
//
// @note This function will not return.
void sleep_mode_enter(void)
{
    ret_code_t err_code;

    //TODO: sleep_mode_prepare for all devices
    //TODO: sleep_mode_prepare for all service controllers
    UI_SLEEP_MODE_PREPARE();

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

// @brief Function for handling the idle state (main loop).
//
// @details If there is no pending log operation, then sleep until next the next event occurs.
static void idle_state_handle(void) {
    ret_code_t err_code;

//TODO: determine why this results in failure
//    err_code = nrf_ble_lesc_request_handler();
//    APP_ERROR_CHECK(err_code);

    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

ret_code_t devices_init() {
  NRF_LOG_INFO("Devices init...");
  NRF_LOG_INFO(STRINGIFY(DEV_OPENER(4,0,inertial_measurement_unit_sim,inertial_measurement_unit,{})));
  NRF_LOG_INFO(STRINGIFY(DEVICE_INIT_SEQUENCE2));
  //NRF_LOG_INFO(STRINGIFY({DEV_SEQUENCE(4)}));
  typedef ret_code_t (*f0)();
  const static f0 dev_init_sequence[] = DEVICE_INIT_SEQUENCE;
  for(int i = 0; i < countof(dev_init_sequence); i++) {
    dev_init_sequence[i]();
  }

  return NRF_SUCCESS;
}

static void devices_start() {
    //TODO: implement start functionality in drivers
}

// initialize drivers, services, and controllers
static void initialize_system(bool *erase_bonds) {
    log_init();
    NRF_LOG_INFO("Initializing Ringularity...");
    timer_module_init();
  
    //board support package stuff
    UI_INIT(erase_bonds);
  
    scheduler_init();
    power_management_init();
    devices_init();
    ble$init();
}


void start_operations(bool erase_bonds) {
    NRF_LOG_INFO("Starting Ringulrity...");
    //devices_start();
    ble$start(erase_bonds);
}

void run_app() {
    NRF_LOG_INFO("Ringularity is running");
    for (;;) {
        idle_state_handle();   
    }
}

// @brief Ringularity main entry point
int main(void)
{
    bool erase_bonds;

    initialize_system(&erase_bonds);
    start_operations(erase_bonds);
    run_app();
}

/**
 * @}
 */

