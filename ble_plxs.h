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
// 
// @File
//
//

#ifndef __BLE_PLXS_H__
#define __BLE_PLXS_H__

#include <stdint.h>
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "sfloat.h"


// <o> BLE_PLXS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Pulxe Oximeter Service.

#ifndef BLE_PLXS_BLE_OBSERVER_PRIO
#define BLE_PLXS_BLE_OBSERVER_PRIO 2
#endif

/**@brief   Macro for defining a ble_pulseoxsvc instance.
 *
 * @param   _name   Name of the instance.
 * 
 */
#define BLE_PLXS_DEF(_name)                                                                          \
static ble_plxs_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_PLXS_BLE_OBSERVER_PRIO,                                                     \
                     ble_plxs_on_ble_evt, &_name)

typedef enum
{
    BLE_PLXS_EVT_NOTIFICATION_ENABLED,   /**< Pulse Oximeter value notification enabled event. */
    BLE_PLXS_EVT_NOTIFICATION_DISABLED,   /**< Pulse Oximeter value notification disabled event. */
    BLE_PLXS_EVT_NOTIFICATION_CONFIRMED  /**< Confirmation of a Spot Check Measurement indication has been received. */
} ble_plxs_evt_type_t;


/**@brief Pulse Oximeter Service event. */
typedef struct
{
    ble_plxs_evt_type_t evt_type;    /**< Type of event. */
} ble_plxs_evt_t;

// Forward declaration of the ble_pulseoxsvc_t type.
typedef struct ble_plxs_s ble_plxs_t;

/**@brief Pulse Oximeter Service event handler type. */
typedef void (*ble_plxs_evt_handler_t) (ble_plxs_t * p_plxs, ble_plxs_evt_t * p_evt);

typedef struct
{
    ble_plxs_evt_handler_t       evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    //bool                         is_<setting>_supported;                             /**< flag setting */
    security_req_t               plxm_cccd_wr_sec;                                     /**< Security requirement for writing the plx measurement characteristic CCCD. */
    //security_req_t               <characteristic>_rd_sec;                            /**< Security requirement for reading the <characteristic> characteristic value. */
} ble_plxs_init_t;

/**@brief Pulse Oximeter Service structure. This contains various status information for the service. */
struct ble_plxs_s
{
    ble_plxs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    //bool                         is_<boolean type>_supported;                         /**< TRUE if <boolean type> is supported. */
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     scm_handles;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     cm_handles;                                          /**< Handles related to the Body Sensor Location characteristic. */
    ble_gatts_char_handles_t     feat_handles;                                         /**< Handles related to the Heart Rate Control Point characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    //bool                         is_sensor_contact_detected;                           /**< TRUE if sensor contact has been detected. */
    uint8_t                      max_plx_len;                                          /**< Current maximum PLX measurement length, adjusted according to the current ATT MTU. */
};

//spot check measurement packet
// 2: connection handle 1: flags, 1: reserved
// 2: spo2  2: pulse rate
// 2: year 1: month, 1: day
// 1: hour, 1: min, 1: sec, 1: reserved
// 2: measurement status, 2: device and sensor status (lower word)
// 2: device and sensor status (upper word), 2: pulse amplitude index

//measurement status bitfields
// 0..4 reserved for future use
// 5 measurement ongoing
// 6 early estimated data
// 7 validated data
// 8 fully-qualified data
// 9 data from measurement storage
// 10 data for demonstration
// 11 data for testing
// 12 calibration ongoing
// 13 measurement unavailable
// 14 questionable measurement detected
// 15 invalid measurement detected

// 3.1.1.5 device and sensor status bitfields
// 0 extended display update ongoing
// 1 equipment malfunction detected
// 2 signal processing irregularity detected
// 3 inadequate signal detected
// 4 poor signal detected
// 5 low perfusion detected
// 6 erratic signal detected
// 7 non-pulsatile signal detected
// 8 questionable pulse detected
// 9 signal analysis ongoing
// 10 sensor interference detected
// 11 sensor unconnected to user
// 12 unknown sensor connected
// 13 sensor displaced
// 14 sensor malfunctioning
// 15 sensor disconnected
// 16..23 reserved for future use

// 3.1.1.6 pulse amplitude index
// * if present in PLX Spot Measurement "Pulse amplitude index field present" is 1 else 0
// * if Pulse Amplitude index is unavailable set to SFLOAT NaN

// 3.2 sensor and continuous measurement characteristic
// PLX continuous measurement characteristic UUID
// 2: connection handle, 1: flags, 1: reserved
// 2: normal spo2, 2: normal pulse rate
// 2: fast spo2, 2: fast pulse rate
// 2: slow spo2, 2: slow pulse rate
// 2: measurement status (3.1.1.4), 2: device and sensor status (low word) (3.1.1.5)
// 2: device and sensor statuts (upper word), 2: pulse amplitude index
// 3.2.1.7 pulse amplitude is SFLOAT NaN if no value 

// 3.2.1.1 flags
// 0 SpO2PR-fast field is present
// 1 SpO2PR-slow field is present
// 2 measurement status field is present
// 3 device and sensor status field is present
// 4 pulse amplitude index field is present
// 5..7 reserved (set to 0)
#define BLE_PLXS_MEASUREMENT_FLAG_SPO2FAST (0X01)
#define BLE_PLXS_MEASUREMENT_FLAG_SPO2SLOW (0X02)
#define BLE_PLXS_MEASUREMENT_FLAG_STATUS  (0X04)
#define BLE_PLXS_MEASUREMENT_FLAG_DEVICE_SENSOR  (0X08)
#define BLE_PLXS_MEASUREMENT_FLAG_PULSE_AMPLITUDE_INDEX (0X10)
#define BLE_PLXS_MEASUREMENT_FLAG_ALL   (0X1F)

//3.2.1.2 SpO2PR-normal field
// SFLOAT NaN used for non-available values

// how is SpO2PR-slow calculated?
// how is SpO2PR-fast calculated?

// 3.3 plx features
// * feature characteristic UUID "PLX Features"
// fields
// 2: Supported Features
// 2: measurement status support (if present)
// 2: device and sensor status support (if present)

// 3.3.1.1 supported features field
// 0 measurement status support is present
// 1 device and sensor status support is present
// 2 measurement storage for spot-checks is supported
// 3 timestamp for spot-check measurements is supported
// 4 SpO2PR-fast metric is supported
// 5 SpO2PR-slow metric is supported
// 6 pulse amplitude index field is supported
// 7 multiple bonds supported
// 8..15 reserved for future use (zeros)

// 3.3.1.2 measurement status support field
// 0..4 reserved for future use
// 5 measurement ongoing bit supported
// 6 early estimated data bit supported
// 7 validated data bit supported
// 8 fully qualified data bit suported
// 9 data from measurement storage bit supported
// 10 data for demonstration bit supported
// 11 data for testing bit supported
// 12 calibration ongoing bit supported
// 13 measurement unavailable bit supported
// 14 questionable measurement detected bit supported
// 15 invalid measurement bit detected supported

//3.3.1.3 device and sensor status support field
// 0 extended display ongoing bit supported
// 1 equipment malfunction detected bit supported
// 2 signal processing irregularity bit supported
// 3 inadequate signal detected bit supported
// 4 poor signal detected bit supported
// 5 low perfusion detected bit supported
// 6 erratic signal detected bit supported
// 7 nonpulsatile signal detected bit supported
// 8 questionable pulse detected bit supported
// 9 signal analysis ongoing bit supported
// 10 sensor interference detected bit supported
// 11 sensor unconnected to user bit supported
// 12 unknown sensor connected bit supported
// 13 sensor displaced bit supported
// 14 sensor malfunctioning bit supported
// 15 sensor disconnected bit supported
// 16..23 reserved for future use

//3.4 record access control point (RACP)
// 3.4.1 PLX spot-check measurement characteristic value+timestamp
// 3.4.2 PLX RACP procedure
// opcodes, operators, operands
// * report stored records
// * delete stored records
// * abort operation
// * report number of stored records
// Responses
// * number of stored records response
// ** operand: UINT16 number of records
// ** response code: Request op vode | response code value
// * response code
// 

//sensor send record access control point
// 2: connection handle, 1: op code, 1: racp operation
// 2: num_of_records, 1: request op code, 1: response code value

typedef struct ble_plxm
{
   //uint8_t flags;
   sfloat_s spo2; 
   sfloat_s pulse_rate;
   //uint8_t[8] timestamp;
   //uint16_t measurement_status;
   //uint32_t device_and_sensor_status;
   //SFLOAT pulse_amplitude_index;
} ble_plxm_t;

/**@brief Function for initializing the Pulse Oximeter Service.
 *
 * @param[out]  p_plxs       Pulse Oximeter Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_plxs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_plxs_init(ble_plxs_t * p_plxs, ble_plxs_init_t const * p_plxs_init);

/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the Pulse Oximeter Service.
 *
 * @param[in]   p_plxs      Pulse Oximeter Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_plxs_on_gatt_evt(ble_plxs_t * p_pos, nrf_ble_gatt_evt_t const * p_gatt_evt);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Pulse Oximeter Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Pulse Oximeter Service structure.
 */
void ble_plxs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for sending pulse oximeter spot check measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a pulse oximeter measurement.
 *          If notification has been enabled, the pulse oximeter measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_plxs                    Pulse Oximeter Service structure.
 * @param[in]   p_measurement        Pulse Oximeter reading.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_plxs_spot_check_measurement_send(ble_plxs_t * p_plxs, ble_plxm_t * p_measurement);

/**@brief Function for sending pulse oximeter continuous measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a pulse oximeter measurement.
 *          If notification has been enabled, the pulse oximeter measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_plxs               Pulse Oximeter Service structure.
 * @param[in]   p_measurement        Pulse Oximeter reading.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_plxs_continuous_measurement_send(ble_plxs_t * p_plxs, ble_plxm_t * p_measurement);


#endif //__BLE_PULSEOXSERVICE_H__