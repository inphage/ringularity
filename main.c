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
// @file
//
// @defgroup main.c
// @{
// @ingroup ringularity
// @brief Ringularity operating environment main file
//
// Contains top-level functionality & glue
#include "sdk_config.h"
#define NRFX_CHECK(module_enabled)  (module_enabled)
#include "nrfx_common.h"

#include "app_scheduler.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "hw_config.h"

#include "ble_config.h"

#include "app_util.h"

#include "ble_main.h"
#include "ui.h"
#include "ble_main.h"

//TWI/I2C headers
#define NRFX_CHECK(module_enabled)  (module_enabled)


#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Features
//  over-the-air firmware updates
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
// * binding hardware driver to device service layer

DEVICE_DEFS;
DEVICE_OPENER_DEFS;

// @brief Function for initializing the nrf log module.
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    NRF_LOG_ERROR("Assert failed at %s line %d", file_name, (uint32_t)line_num);
/*    assert_info_t assert_info =
    {
        .line_num    = line_num,
        .p_file_name = file_name,
    };

    app_error_fault_handler(NRF_FAULT_ID_SDK_ASSERT, 0, (uint32_t)(&assert_info));

    UNUSED_VARIABLE(assert_info);*/
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
    // should this happen to return the error code is NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN 
    err_code = sd_power_system_off();
    //APP_ERROR_CHECK(err_code);
#ifdef DEBUG
        while (true)
        {
            /* Since the CPU is kept on in an emulated System OFF mode, it is recommended
             * to add an infinite loop directly after entering System OFF, to prevent
             * the CPU from executing code that normally should not be executed. */
            __WFE();

        }
#endif

}

// @brief Function for handling the idle state (main loop).
//
// @details If there is no pending log operation, then sleep until next the next event occurs.
// WARNING: nrf_pwr_mgmt_run can put things in to a suspended state
static void idle_state_handle(void) {
    ret_code_t err_code;

//TODO: determine why this results in failure
//    err_code = nrf_ble_lesc_request_handler();
//    APP_ERROR_CHECK(err_code);

    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
#ifndef DEBUG
        nrf_pwr_mgmt_run();
#endif
    }
}

ret_code_t devices_init() {
  NRF_LOG_INFO("Devices init...");
  //NRF_LOG_INFO(STRINGIFY(DEV_OPENER(4,0,inertial_measurement_unit_sim,inertial_measurement_unit,{})));
  //NRF_LOG_INFO(STRINGIFY(DEVICE_INIT_SEQUENCE2));
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
    NRF_LOG_INFO("Starting Ringularity...");
    devices_start();
    ble$start(erase_bonds);
}

void twi_wait_while_busy(const nrf_drv_twi_t *p_twi) {
    //idle_state_handle();
    app_sched_execute();
    while(nrf_drv_twi_is_busy(p_twi)) {
        idle_state_handle();
    }
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

