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
#ifndef __BLE_CONFIG_H__
#define __BLE_CONFIG_H__

#include "app_timer.h"

// General Access Profile (GAP)
#define GAP_DEVICE_NAME             "OpenRing"                       /**< Name of device. Will be included in the advertising data. */
#define GAP_DEVICE_APPEARANCE       BLE_APPEARANCE_GENERIC_PULSE_OXIMETER

// Device Information Service (DIS) values
#define DIS_MANUFACTURER_NAME       "Open Ring Project"               /**< Manufacturer. Will be passed to Device Information Service. */
#define DIS_MODEL_NUMBER            "2020.1.1.1"                      /**< Model number. Will be passed to Device Information Service. */
#define DIS_SERIAL_NUMBER           ""                                /**< Serial Number String. */
#define DIS_HW_REVISION             "0.0.1"                           /**< Hardware Revision String. */
#define DIS_FW_REVISION             "0.0.1"                           /**< Firmware Revision String. */
#define DIS_SW_REVISION             "0.0.1"                           /**< Software Revision String. */

// Bluetooth SIG-defined DIS values
//#define DIS_MANUFACTURER_ID         0x0000000059                            /**< Manufacturer ID for System ID. */
//#define DIS_ORG_UID                 0x123456                                /**< Organizationally unique ID for System ID. */
//#define DIS_VENDOR_ID               0x0059                                  /**< Vendor ID for PnP ID. */
//#define DIS_PRODUCT_ID              0x0001                                  /**< Product ID for PnP ID. */
//#define DIS_PRODUCT_VERSION         0x0002                                  /**< Product Version for PnP ID. */

// IEEE 11073-20601 Regulatory Certification Data List.
//#define DIS_CERT_LIST               {0x00, 0x01, 0x02, 0x03}                /**< IEEE 11073-20601 Regulatory Certification Data List. */

// Manufacturer ID comes from Bluetooth SIG
//#define MANUFACTURER_ID                 0x1122334455                                /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
//#define ORG_UNIQUE_ID                   0x667788                                    /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

// Advertising
//#define APP_ADV_INTERVAL                300                            /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
//#define APP_ADV_DURATION                18000                          /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */

#define APP_ADV_FAST_DURATION           3000                                        /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           18000                                       /**< The advertising duration of slow advertising in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO           3                              /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                              /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

// Bond Management Service
#define USE_AUTHORIZATION_CODE          1
#define AUTHORIZATION_CODE              'R', 'I', 'N', 'G'                      //0x52, 0x49, 0x4E, 0x47

#define SEC_PARAM_TIMEOUT               30                                      /**< Time-out for pairing request or security request (in seconds). */
#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections (not) enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif

#define QWR_MEM_BUFF_SIZE                   512

#define TEMP_TYPE_AS_CHARACTERISTIC     0                                           /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */  

#include "nordic_common.h"
#include "nrf_ble_gq.h"

extern nrf_ble_gq_t *get_ble_gatt_queue();

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
extern void service_error_handler(uint32_t nrf_error);

#include "battery_service_controller.h"
#include "health_thermometer_service_controller.h"
#include "heart_rate_service_controller.h"
#include "pulse_oximeter_service_controller.h"
#include "rscs_controller.h"
#include "current_time_service_controller.h"

void gap$on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt);

typedef void (*cts_c_evt_handler_t)(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt);
#define CTS_C_EVT_HANDLERS(handlers) cts_c_evt_handler_t handlers[]={gap$on_cts_c_evt};

void srvcon$bond_management_service$on_ble_evt(ble_evt_t const * p_ble_evt);

typedef void (*ble_gap_evt_handler_t)(ble_evt_t const * p_ble_evt);
#define BLE_EVT_HANDLERS(handlers) ble_gap_evt_handler_t handlers[]={\
    srvcon$bond_management_service$on_ble_evt,\
    srvcon$health_thermometer_service$on_ble_evt,\
    srvcon$pulse_oximeter_service$on_ble_evt,\
    srvcon$current_time_service_client$on_ble_evt\
    }


typedef void (*pm_evt_handler_t)(pm_evt_t const * p_evt);
#define PM_EVT_HANDLERS(handlers) pm_evt_handler_t handlers[]={\
    srvcon$health_thermometer_service$on_pm_evt,\
    srvcon$pulse_oximeter_service$on_pm_evt\
    }


void (srvcon$current_time_service$on_db_disc_evt)(ble_db_discovery_evt_t * p_evt);

typedef void (*db_discovery_evt_handler_t)(ble_db_discovery_evt_t * p_evt);
#define DB_DISCOVERY_EVT_HANDLERS(handlers) ble_db_discovery_evt_handler_t handlers[]={srvcon$current_time_service$on_db_disc_evt}

static BLE_EVT_HANDLERS(ble_evt_handlers);

#endif //__BLE_CONFIG_H__