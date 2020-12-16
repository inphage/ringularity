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
#include "ble_plxs.h"
#include "app_util_bds.h"
#include "sfloat.h"

#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside Pulse Oximeter Measurement packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside Pulse Oximeter Measurement packet. */
#define MAX_PLX_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted Pulse Oximeter Measurement. */

#define INITIAL_VALUE_SPO2  SFLOAT_NAN
#define INITIAL_VALUE_PR    SFLOAT_NAN

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_plxs      Pulse Oximeter Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_plxs_t * p_plxs, ble_evt_t const * p_ble_evt)
{
    p_plxs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_plxs      Pulse Oximeter Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_plxs_t * p_plxs, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_plxs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling write events to the Spot Check/Continuous Measurement characteristic.
 *
 * @param[in]   p_plxs        Pulse Oximeter Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_plxs_cccd_write(ble_plxs_t * p_plxs, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2) //data structure size
    {
        // CCCD written, update notification state
        if (p_plxs->evt_handler != NULL)
        {
            ble_plxs_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_PLXS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_PLXS_EVT_NOTIFICATION_DISABLED;
            }

            p_plxs->evt_handler(p_plxs, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_plxs      Pulse Oximeter Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_plxs_t * p_plxs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    //spot check measurement
    if (p_evt_write->handle == p_plxs->scm_handles.cccd_handle)
    {
        on_plxs_cccd_write(p_plxs, p_evt_write);
    } //continuous measurement
    else if (p_evt_write->handle == p_plxs->cm_handles.cccd_handle)
    {
        on_plxs_cccd_write(p_plxs, p_evt_write);
    }
    //record access control point
}

/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in]   p_plxs      Pulse Oximetry Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_hvc(ble_plxs_t * p_plxs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_hvc_t const * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_plxs->cm_handles.value_handle)
    {
        ble_plxs_evt_t evt;

        evt.evt_type = BLE_PLXS_EVT_NOTIFICATION_CONFIRMED;
        p_plxs->evt_handler(p_plxs, &evt);
    }
}



void ble_plxs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_plxs_t * p_plxs = (ble_plxs_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_plxs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_plxs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_plxs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_plxs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Spot Check Measurement.
 *
 * @param[in]   p_plxs             Heart Rate Service structure.
 * @param[in]   p_scm              Spot Check Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t plxm_scm_encode(ble_plxs_t * p_plxs, ble_plxm_t * p_plxm, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 0;
    int     i;

    // Set flags
    /*
    if (p_hrs->is_sensor_contact_supported)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_hrs->is_sensor_contact_detected)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }*/

    p_encoded_buffer[len++] = flags;
    p_encoded_buffer[len++] = 0; //reserved, set to 0

    // Encode SPO2 measurement/PR measurment
    len += uint16_encode(SFLOAT_PACK(p_plxm->spo2), &p_encoded_buffer[len]);
    len += uint16_encode(SFLOAT_PACK(p_plxm->pulse_rate),  &p_encoded_buffer[len]);

    //TODO: encode timestamp & reserved
    //TODO: encode measurement status
    //TODO: encode device & sensor status
    //TODO: encode amplitude index

    return len;
}

/**@brief Function for encoding a Spot Check Measurement.
 *
 * @param[in]   p_plxs             Heart Rate Service structure.
 * @param[in]   p_scm              Spot Check Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t plxm_cm_encode(ble_plxs_t * p_plxs, ble_plxm_t * p_plxm, uint8_t * p_encoded_buffer)
{
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
  //  0 SpO2PR-fast field is present
  //  1 SpO2PR-slow field is present
  //  2 measurement status field is present
  //  3 device and sensor status field is present
  //  4 pulse amplitude index field is present
  //  5..7 reserved (set to 0)
    uint8_t flags = 0;
    uint8_t len   = 0;
    int     i;

    // Set flags
    /*
    if (p_hrs->is_sensor_contact_supported)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_hrs->is_sensor_contact_detected)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }*/

    p_encoded_buffer[len++] = flags;
    p_encoded_buffer[len++] = 0; //reserved, set to 0

    // Encode SPO2 measurement/PR measurment
    len += uint16_encode(SFLOAT_PACK(p_plxm->spo2), &p_encoded_buffer[len]);
    len += uint16_encode(SFLOAT_PACK(p_plxm->pulse_rate),  &p_encoded_buffer[len]);

    //TODO: SpO2 & HR fast
    //TODO: SpO2 & HR slow
    //TODO: encode measurement status
    //TODO: encode device & sensor status
    //TODO: encode amplitude index

    return len;
}


uint32_t ble_plxs_init(ble_plxs_t * p_plxs, const ble_plxs_init_t * p_plxs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    
    //need pulse ox params
    uint8_t               encoded_initial_plxm[MAX_PLX_LEN];

    // Initialize service structure
    p_plxs->evt_handler                 = p_plxs_init->evt_handler;
    p_plxs->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    //p_hrs->is_sensor_contact_supported = p_plxs_init->is_sensor_contact_supported;
    //p_hrs->is_sensor_contact_detected  = false;
    
    //default pulse ox params
    p_plxs->max_plx_len = MAX_PLX_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_PLX_SERVICE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_plxs->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_plxm_t initial_value_plxm = {
      .spo2 = INITIAL_VALUE_SPO2,
      .pulse_rate = INITIAL_VALUE_PR
    };

    // Add pulxe oximeter spot check measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_PLX_SPOT_CHECK_MEAS;
    add_char_params.max_len           = MAX_PLX_LEN;
    add_char_params.init_len          = plxm_scm_encode(p_plxs, &initial_value_plxm, encoded_initial_plxm);
    add_char_params.p_init_value      = encoded_initial_plxm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_plxs_init->plxm_cccd_wr_sec;

    err_code = characteristic_add(p_plxs->service_handle, &add_char_params, &(p_plxs->scm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

  // Add BLE continuous measurement characteristic
    add_char_params.uuid              = BLE_UUID_PLX_CONTINUOUS_MEAS;
    add_char_params.max_len           = MAX_PLX_LEN;
    add_char_params.init_len          = plxm_cm_encode(p_plxs, &initial_value_plxm, encoded_initial_plxm);
    add_char_params.p_init_value      = encoded_initial_plxm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_plxs_init->plxm_cccd_wr_sec;

    err_code = characteristic_add(p_plxs->service_handle, &add_char_params, &(p_plxs->cm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /*add_char_params.uuid              = BLE_UUID_PLX_FEATURES;
    add_char_params.max_len           = MAX_PLX_LEN;
    add_char_params.init_len          = plx_encode(p_plxs, INITIAL_VALUE_PLXS, encoded_initial_plx);
    add_char_params.p_init_value      = encoded_initial_plx;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    err_code = characteristic_add(p_hrs->service_handle, &add_char_params, &(p_plxs->feat_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }*/

/*    if (p_hrs_init->p_body_sensor_location != NULL)
    {
        // Add body sensor location characteristic
        memset(&add_char_params, 0, sizeof(add_char_params));

        add_char_params.uuid            = BLE_UUID_BODY_SENSOR_LOCATION_CHAR;
        add_char_params.max_len         = sizeof(uint8_t);
        add_char_params.init_len        = sizeof(uint8_t);
        add_char_params.p_init_value    = p_hrs_init->p_body_sensor_location;
        add_char_params.char_props.read = 1;
        add_char_params.read_access     = p_hrs_init->bsl_rd_sec;

        err_code = characteristic_add(p_hrs->service_handle, &add_char_params, &(p_hrs->bsl_handles));
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }*/

    return NRF_SUCCESS;
}


uint32_t ble_plxs_spot_check_measurement_send(ble_plxs_t * p_plxs, ble_plxm_t * p_scm)
{
  
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_plxs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_plxm[MAX_PLX_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = plxm_scm_encode(p_plxs, p_scm, encoded_plxm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_plxs->scm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_plxm;

        err_code = sd_ble_gatts_hvx(p_plxs->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_plxs_continuous_measurement_send(ble_plxs_t * p_plxs, ble_plxm_t * p_scm)
{
  // 3.2 sensor and continuous measurement characteristic
  // PLX continuous measurement characteristic UUID
  // 2: connection handle, 1: flags, 1: reserved
  // 2: normal spo2, 2: normal pulse rate
  // 2: fast spo2, 2: fast pulse rate
  // 2: slow spo2, 2: slow pulse rate
  // 2: measurement status (3.1.1.4), 2: device and sensor status (low word) (3.1.1.5)
  // 2: device and sensor statuts (upper word), 2: pulse amplitude index
  // 3.2.1.7 pulse amplitude is SFLOAT NaN if no value  
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_plxs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_plxm[MAX_PLX_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = plxm_cm_encode(p_plxs, p_scm, encoded_plxm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_plxs->cm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_plxm;

        err_code = sd_ble_gatts_hvx(p_plxs->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/*
uint32_t ble_hrs_sensor_contact_supported_set(ble_hrs_t * p_hrs, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_hrs->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_hrs->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}



void ble_hrs_sensor_contact_detected_update(ble_hrs_t * p_hrs, bool is_sensor_contact_detected)
{
    p_hrs->is_sensor_contact_detected = is_sensor_contact_detected;
}


uint32_t ble_hrs_body_sensor_location_set(ble_hrs_t * p_hrs, uint8_t body_sensor_location)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &body_sensor_location;

    return sd_ble_gatts_value_set(p_hrs->conn_handle, p_hrs->bsl_handles.value_handle, &gatts_value);
}*/

void ble_plxs_on_gatt_evt(ble_plxs_t * p_plxs, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_plxs->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_plxs->max_plx_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
