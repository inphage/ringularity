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
#include "ble_config.h"
#include "sdk_config.h"
#include "ble_conn_params.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_bms.h"
#include "nrf_ble_qwr.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "nrf_log.h"
#include "app_util.h"
#include "ui.h"
#include "ble_dis.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_sdh.h"

NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_BMS_DEF(m_bms);                                             //!< Structure used to identify the Bond Management service.
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);                           /**< DB discovery module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;            /**< Handle of the current connection. */
static pm_peer_id_t m_peer_id;                                                      /**< Device reference handle to the current bonded central. */
//static uint16_t     m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;                    /**< Handle of the current connection. */

static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];            /**< List of peers currently in the whitelist. */
static uint32_t     m_whitelist_peer_cnt;                                           /**< Number of peers currently in the whitelist. */

static uint8_t m_qwr_mem[QWR_MEM_BUFF_SIZE];                            //!< Write buffer for the Queued Write module.
static ble_conn_state_user_flag_id_t m_bms_bonds_to_delete;         //!< Flags used to identify bonds that should be deleted.

#ifdef USE_AUTHORIZATION_CODE
static uint8_t m_auth_code[] = {AUTHORIZATION_CODE};
static int m_auth_code_len = sizeof(m_auth_code);
#endif

// forward declarations
static void delete_disconnected_bonds();
static void peer_manager$delete_all_bonds(nrf_ble_bms_t const * p_bms);
void sleep_mode_enter();


// Indoor positioning? (beacon)
// Link Loss?
// Reconnection configuration?
// Scan Parameters?
// Transport Discovery?
// Tx Power?
// User Data?
static ble_uuid_t m_adv_uuids[] =  /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE},
#if NRF_MODULE_ENABLED(BLE_BAS)
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
#endif
#if NRF_MODULE_ENABLED(BLE_HTS)
    {BLE_UUID_HEALTH_THERMOMETER_SERVICE,   BLE_UUID_TYPE_BLE},
#endif
#if NRF_MODULE_ENABLED(BLE_HRS)
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
#endif
#if NRF_MODULE_ENABLED(BLE_PLXS)
    {BLE_UUID_PLX_SERVICE,                  BLE_UUID_TYPE_BLE},
#endif 
#if NRF_MODULE_ENABLED(BLE_RSCS)
    {BLE_UUID_RUNNING_SPEED_AND_CADENCE,    BLE_UUID_TYPE_BLE},
#endif
    {BLE_UUID_BMS_SERVICE,                  BLE_UUID_TYPE_BLE},
    {BLE_UUID_CURRENT_TIME_SERVICE,         BLE_UUID_TYPE_BLE}
};

nrf_ble_gq_t *get_ble_gatt_queue() {
  return &m_ble_gatt_queue;
}

#include "ble_config.h"

static void advertising$start(bool erase_bonds);
static void delete_all_except_requesting_bond(nrf_ble_bms_t const * p_bms);
static void queued_write_module$on_ble_connected();


//@brief Callback function for asserts in the SoftDevice.
//
// @details This function will be called in case of an assert in the SoftDevice.
//
// @warning This handler is an example only and does not fit a final product. You need to analyze
//          how your product is supposed to react in case of Assert.
// @warning On assert from the SoftDevice, the system can only recover on reset.
//
// @param[in] line_num   Line number of the failing ASSERT call.
// @param[in] file_name  File name of the failing ASSERT call.
/*void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}*/

 void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


uint16_t get_conn_handle() {
    return m_conn_handle;
}


//// BLE GAP MODULE CONTROL //
  
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params$init(void)
{
    NRF_LOG_INFO("GAP params init...");
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params = {0};
    ble_gap_conn_sec_mode_t sec_mode = {0};

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)GAP_DEVICE_NAME,
                                          strlen(GAP_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

#ifdef GAP_DEVICE_APPEARANCE
    err_code = sd_ble_gap_appearance_set(GAP_DEVICE_APPEARANCE);
    APP_ERROR_CHECK(err_code);
#endif

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


static void on_ble_gap_evt_disconnected(ble_evt_t const * p_ble_evt) {
    NRF_LOG_INFO("Disconnected"); //, reason %d.",
                  //p_ble_evt->evt.gap_evt.params.disconnected.reason);
    m_conn_handle               = BLE_CONN_HANDLE_INVALID;
    delete_disconnected_bonds();
    DISPATCH_EVT(ble_evt_handlers, p_ble_evt);
}

static void on_ble_gap_evt_phy_update_request(ble_evt_t const * p_ble_evt) {
    ret_code_t err_code;
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
    {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
    };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
}

static void on_ble_gap_evt_conn_param_update_request(ble_evt_t const * p_ble_evt) {
    ret_code_t err_code;
    // Accepting parameters requested by peer.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                            &p_gap_evt->params.conn_param_update_request.conn_params);
    APP_ERROR_CHECK(err_code);
}

static void on_ble_gap_evt_timeout(ble_evt_t const * p_ble_evt) {
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
        NRF_LOG_INFO("Connection Request timed out.");
    }
}


static void gap$on_ble_gap_evt_connected(ble_evt_t const * p_ble_evt) {
    ret_code_t err_code;
    NRF_LOG_INFO("Connected");
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    UI_INDICATION_SET(BSP_INDICATE_CONNECTED);
    
    DISPATCH_EVT(ble_evt_handlers, p_ble_evt);

    queued_write_module$on_ble_connected();
}

void gap$on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt) {
    // CTS not found in this case we just disconnect. There is no reason to stay
    // in the connection for this simple app since it all wants is to interact with CT
    if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ret_code_t err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
}


//// BLE GATT MODULE CONTROL //


/**@brief GATT module event handler.
 */
static void gatt$on_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                         p_evt->conn_handle,
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                         p_evt->conn_handle,
                         p_evt->params.data_length);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the GATT module.
 */
static void gatt$init(void)
{
  NRF_LOG_INFO("GATT init...");
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt$on_gatt_evt);
  APP_ERROR_CHECK(err_code);
}

//// BLE BOND MANAGEMENT SERVICE CONTROLLER //

// @brief Bond management service event handler
void bms$on_bms_evt(nrf_ble_bms_t * p_ess, nrf_ble_bms_evt_t * p_evt) {
    ret_code_t err_code;
    bool is_authorized = true;

    switch (p_evt->evt_type) {
        case NRF_BLE_BMS_EVT_AUTH:
            NRF_LOG_DEBUG("Authorization request.");
#if USE_AUTHORIZATION_CODE
            if ((p_evt->auth_code.len != m_auth_code_len) ||
                (memcmp(m_auth_code, p_evt->auth_code.code, m_auth_code_len) != 0))
            {
                is_authorized = false;
            }
#endif
            err_code = nrf_ble_bms_auth_response(&m_bms, is_authorized);
            APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for deleting a single bond if it does not belong to a connected peer.
 *
 * This will mark the bond for deferred deletion if the peer is connected.
 */
static void bond_delete(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t   err_code;
    pm_peer_id_t peer_id;

    if (ble_conn_state_status(conn_handle) == BLE_CONN_STATUS_CONNECTED)
    {
        ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, true);
    }
    else
    {
        NRF_LOG_DEBUG("Attempting to delete bond.");
        err_code = pm_peer_id_get(conn_handle, &peer_id);
        APP_ERROR_CHECK(err_code);
        if (peer_id != PM_PEER_ID_INVALID)
        {
            err_code = pm_peer_delete(peer_id);
            APP_ERROR_CHECK(err_code);
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
        }
    }
}

/**@brief Function for performing deferred deletions.
*/
static void delete_disconnected_bonds(void)
{
    uint32_t n_calls = ble_conn_state_for_each_set_user_flag(m_bms_bonds_to_delete, bond_delete, NULL);
    UNUSED_RETURN_VALUE(n_calls);
}


/**@brief Function for marking the requester's bond for deletion.
*/
static void delete_requesting_bond(nrf_ble_bms_t const * p_bms)
{
    NRF_LOG_INFO("Client requested that bond to current device deleted");
    ble_conn_state_user_flag_set(p_bms->conn_handle, m_bms_bonds_to_delete, true);
}



static void srvcon$bond_management_service$init() {
    ret_code_t         err_code;
    nrf_ble_bms_init_t   bms_init = {0};

    m_bms_bonds_to_delete        = ble_conn_state_user_flag_acquire();
    bms_init.evt_handler         = bms$on_bms_evt;
    bms_init.error_handler       = service_error_handler;
#if USE_AUTHORIZATION_CODE
    bms_init.feature.delete_requesting_auth         = true;
    bms_init.feature.delete_all_auth                = true;
    bms_init.feature.delete_all_but_requesting_auth = true;
#else
    bms_init.feature.delete_requesting              = true;
    bms_init.feature.delete_all                     = true;
    bms_init.feature.delete_all_but_requesting      = true;
#endif
    bms_init.bms_feature_sec_req = SEC_JUST_WORKS;
    bms_init.bms_ctrlpt_sec_req  = SEC_JUST_WORKS;

    bms_init.p_qwr                                       = &m_qwr;
    bms_init.bond_callbacks.delete_requesting            = delete_requesting_bond;
    bms_init.bond_callbacks.delete_all                   = peer_manager$delete_all_bonds;
    bms_init.bond_callbacks.delete_all_except_requesting = delete_all_except_requesting_bond;

    err_code = nrf_ble_bms_init(&m_bms, &bms_init);
    APP_ERROR_CHECK(err_code);
}

void srvcon$bond_management_service$on_ble_evt(ble_evt_t const * p_ble_evt) {
    ret_code_t err_code = nrf_ble_bms_set_conn_handle(&m_bms, m_conn_handle);
    APP_ERROR_CHECK(err_code);
}


//// PEER MANAGER //

//@brief Function for deleting all but requesting device bonds
static void delete_all_except_requesting_bond(nrf_ble_bms_t const * p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds except current bond be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        /* Do nothing if this is our own bond. */
        if (conn_handle != p_bms->conn_handle)
        {
            bond_delete(conn_handle, NULL);
        }

        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Function for deleting all bonds
*/
static void peer_manager$delete_all_bonds(nrf_ble_bms_t const * p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        bond_delete(conn_handle, NULL);

        peer_id = pm_next_peer_id_get(peer_id);
    }
}




static void peer_manager$on_pm_evt_conn_sec_succeeded(pm_evt_t const * p_evt) {
    ret_code_t err_code;
    bool is_indication_enabled;
   
    m_peer_id = p_evt->peer_id;

    // Discover peer's services.
    err_code  = ble_db_discovery_start(&m_ble_db_discovery, p_evt->conn_handle);
    APP_ERROR_CHECK(err_code);

    // Send a single temperature measurement if indication is enabled.
    // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
    // pm_evt_handler() in ble_evt_dispatch().
    static const PM_EVT_HANDLERS(handlers);
    DISPATCH_EVT(handlers, p_evt);
}

static void peer_manager$on_pm_evt_peer_data_update_succeeded(pm_evt_t const * p_evt) {
    ret_code_t err_code;

    // Note: You should check on what kind of white list policy your application should use.
    if (     p_evt->params.peer_data_update_succeeded.flash_changed
         && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING)) {
        NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
        NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                       m_whitelist_peer_cnt + 1,
                       BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

        if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) {
            // Bonded to a new peer, add it to the whitelist.
            m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

            // The whitelist has been modified, update it in the Peer Manager.
            err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
            if (err_code != NRF_ERROR_NOT_SUPPORTED) {
                APP_ERROR_CHECK(err_code);
            }

            err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
            APP_ERROR_CHECK(err_code);
        }
    }
}

static void peer_manager$on_pm_evt_peers_delete_succeeded() {
    advertising$start(false);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void peer_manager$on_pm_evt(pm_evt_t const * p_evt) {
    ret_code_t err_code;
    bool       is_indication_enabled;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id) {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            peer_manager$on_pm_evt_conn_sec_succeeded(p_evt);
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            peer_manager$on_pm_evt_peers_delete_succeeded();
            break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            peer_manager$on_pm_evt_peer_data_update_succeeded(p_evt);
            break;

        default:
            break;
    }
}

//@brief Function for the Peer Manager initialization.
static void peer_manager$init(void) {
    ble_gap_sec_params_t sec_param = {0};
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(peer_manager$on_pm_evt);
    APP_ERROR_CHECK(err_code);
}


//@brief Fetch the list of peer manager peer IDs.
//
// @param[inout] p_peers   The buffer where to store the list of peer IDs.
// @param[inout] p_size    In: The size of the @p p_peers buffer.
//                         Out: The number of peers copied in the buffer.
static void peer_manager$peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size) {
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
    *p_size = 0;

    for (pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID); 
        (peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--);
        peer_id = pm_next_peer_id_get(peer_id)) {
        p_peers[(*p_size)++] = peer_id;
    }
}

// @brief Clear bond information from persistent storage.
static void peer_manager$delete_bonds(void) {
    ret_code_t err_code;

    NRF_LOG_INFO("Deleting bonds");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_discovery$evt_handler(ble_db_discovery_evt_t * p_evt) {
    static const DB_DISCOVERY_EVT_HANDLERS(handlers);
    DISPATCH_EVT(handlers, p_evt)
}

// @brief Database discovery collector initialization.
static void db_discovery$init(void) {
    ble_db_discovery_init_t db_init = {0};

    db_init.evt_handler  = db_discovery$evt_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


//// QUEUED WRITE MODULE //


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void queued_write_module$qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


uint16_t queued_write_module$on_qwr_evt(nrf_ble_qwr_t * p_qwr, nrf_ble_qwr_evt_t * p_evt)
{
    return nrf_ble_bms_on_qwr_evt(&m_bms, p_qwr, p_evt);
}

static void queued_write_module$init() {
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
 
    qwr_init.mem_buffer.len   = QWR_MEM_BUFF_SIZE; //optimal value?
    qwr_init.mem_buffer.p_mem = m_qwr_mem;
    qwr_init.callback         = queued_write_module$on_qwr_evt;
    qwr_init.error_handler    = queued_write_module$qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
}

static void queued_write_module$on_ble_connected() {
    ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
}


//// DEVICE INFORMATION SERVICE CONTROLLER //

static void srvcon$device_information_service$init() {
    ret_code_t         err_code;
    ble_dis_init_t     dis_init = {0};

   // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str,  DIS_MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,      DIS_MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str,     DIS_SERIAL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,         DIS_HW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,         DIS_FW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str,         DIS_SW_REVISION);

#ifdef DIS_MANUFACTURER_ID
    ble_dis_sys_id_t   sys_id;
    //Manufacturer ID comes from Bluetooth SIG
    memset(&sys_id, 0, sizeof(sys_id));
    sys_id.manufacturer_id            = DIS_MANUFACTURER_ID;
    sys_id.organizationally_unique_id = DIS_ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;
#endif

#ifdef BLE_DIS_CERT_LIST
    ble_dis_reg_cert_data_list_t cert_list;
    uint8_t  cert_list_data[] = BLE_DIS_CERT_LIST;
    cert_list.p_list   = cert_list_data;
    cert_list.list_len = sizeof(cert_list_data);
    dis_init.p_reg_cert_data_list = &cert_list;
#endif //BLE_DIS_CERT_LIST

#ifdef DIS_VENDOR_ID_SRC_BLUETOOTH_SIG
    ble_dis_pnp_id_t             pnp_id;
    pnp_id.vendor_id_source = DIS_VENDOR_ID_SRC_BLUETOOTH_SIG;
    pnp_id.vendor_id        = DIS_VENDOR_ID;
    pnp_id.product_id       = DIS_PRODUCT_ID;
    pnp_id.product_version  = DIS_PRODUCT_VERSION;
    dis_init.p_pnp_id = &pnp_id;
#endif

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

//// CONNECTION PARAMETERS MODULE //

static void conn_params$on_evt_failed() {
  ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void conn_params$on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {  //connection negotiation failed
        conn_params$on_evt_failed();      
    }
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params$init(void) {
  NRF_LOG_INFO("Connection params init...");
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init = {0};

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    //TODO: need to generalize this next bit
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    //cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle; //BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = conn_params$on_conn_params_evt;
    cp_init.error_handler                  = service_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


//// ADVERTISING //

static void on_adv_evt_whitelist_request() {
    ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
    uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                whitelist_irks,  &irk_cnt);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                   addr_cnt,
                   irk_cnt);

    // Apply the whitelist.
    err_code = ble_advertising_whitelist_reply(&m_advertising,
                                               whitelist_addrs,
                                               addr_cnt,
                                               whitelist_irks,
                                               irk_cnt);
    APP_ERROR_CHECK(err_code);
}


static void advertising$restart_without_whitelist() {
    ret_code_t err_code = ble_advertising_restart_without_whitelist(&m_advertising);
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt_slow_whitelist() {
    NRF_LOG_INFO("Slow advertising with WhiteList");
    UI_INDICATION_SET(BSP_INDICATE_ADVERTISING_WHITELIST);
    advertising$restart_without_whitelist(&m_advertising);
}

static void on_adv_evt_idle() {
    sleep_mode_enter();
}

static void on_adv_evt_fast() {
    NRF_LOG_INFO("Fast advertising.");
    UI_INDICATION_SET(BSP_INDICATE_ADVERTISING);
}

static void on_adv_evt_slow() {
    NRF_LOG_INFO("Slow advertising");
    UI_INDICATION_SET(BSP_INDICATE_ADVERTISING_SLOW);
}

static void on_adv_evt_fast_whitelist() {
    NRF_LOG_INFO("Fast advertising with WhiteList");
    UI_INDICATION_SET(BSP_INDICATE_ADVERTISING_WHITELIST);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void advertising$on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            on_adv_evt_fast();
            break;

        case BLE_ADV_EVT_SLOW:
            on_adv_evt_slow();
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            on_adv_evt_fast_whitelist();
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            on_adv_evt_slow_whitelist();
            break;

        case BLE_ADV_EVT_IDLE:
            on_adv_evt_idle();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
            on_adv_evt_whitelist_request();        
            break;

        default:
            break;
    }
}

/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void advertising$adv_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

// @brief Function for initializing the Advertising functionality.
static void advertising$init(void) {
    NRF_LOG_INFO("advertising init...");
    ret_code_t             err_code;
    ble_advertising_init_t init={0};

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = countof(m_adv_uuids);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled = true;
    init.config.ble_adv_fast_enabled      = true;
    init.config.ble_adv_fast_interval     = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout      = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled      = true;
    init.config.ble_adv_slow_interval     = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout      = APP_ADV_SLOW_DURATION;


   /* TODO: needed? impact?
    init.config.ble_adv_primary_phy      = BLE_GAP_PHY_1MBPS;
    init.config.ble_adv_secondary_phy    = BLE_GAP_PHY_2MBPS;
    init.config.ble_adv_extended_enabled = true;
    */
 
    init.evt_handler = advertising$on_adv_evt;
    init.error_handler = advertising$adv_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

// @brief Starts advertising.
static void advertising$start(bool erase_bonds) {
    if (erase_bonds == true) {
        peer_manager$delete_bonds();
        // TODO: implement PM_EVT_PEERS_DELETED_SUCEEDED handling
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    } else {
        ret_code_t ret;
        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = countof(m_whitelist_peers);

        peer_manager$peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(ret);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (ret != NRF_ERROR_NOT_SUPPORTED) {
            APP_ERROR_CHECK(ret);
        }
        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}

void advertising$on_bsp_event(bsp_event_t event) {
  if((event) == BSP_EVENT_WHITELIST_OFF 
        && m_conn_handle == BLE_CONN_HANDLE_INVALID) {
      ret_code_t err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE) {
          APP_ERROR_CHECK(err_code);
      }
  }
}

// GATT Client

static void on_ble_gattc_evt_timeout(ble_evt_t const * p_ble_evt) {
    ret_code_t err_code;
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
 
    /*if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
    {
        NRF_LOG_INFO("Connection Request timed out.");
    }*/

}

// GATT Server
static void on_ble_gatts_evt_timeout(ble_evt_t const * p_ble_evt) {
    ret_code_t err_code;
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble$ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            gap$on_ble_gap_evt_connected(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:  
            on_ble_gap_evt_disconnected(p_ble_evt);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: 
            on_ble_gap_evt_phy_update_request(p_ble_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            on_ble_gap_evt_conn_param_update_request(p_ble_evt);        
            break;

        case BLE_GAP_EVT_TIMEOUT:
            on_ble_gap_evt_timeout(p_ble_evt);            
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            on_ble_gattc_evt_timeout(p_ble_evt);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            on_ble_gatts_evt_timeout(p_ble_evt);
            
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             /*NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));*/
            break;

        default:
            // No implementation needed.
            //NRF_LOG_DEBUG("ble evt id %d", p_ble_evt->header.evt_id);
            break;
    }
}

//SDH = SoftDevice Handler

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble$stack_init(void)
{
    NRF_LOG_INFO("BLE stack init");
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    // there are parameters set in flash_placement.xml, 
    // and Memory Segments in the project options
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble$ble_evt_handler, NULL);
}

//@brief Function for initializing services that will be used by the application.
static void service_controllers_init(void) {
    srvcon$bond_management_service$init();
#if NRF_MODULE_ENABLED(BLE_CTS)
    srvcon$current_time_service$init();
#endif
#if NRF_MODULE_ENABLED(BLE_BAS)
    srvcon$battery_service$init();
#endif
#if NRF_MODULE_ENABLED(BLE_HRS)
    srvcon$heart_rate_service$init();
#endif
#if NRF_MODULE_ENABLED(BLE_HTS)
    srvcon$health_thermometer_service$init();
#endif
#if NRF_MODULE_ENABLED(BLE_PLXS)
    srvcon$pulse_oximeter_service$init();
#endif
#if NRF_MODULE_ENABLED(BLE_RSCS)
    srvcon$running_speed_cadence_service$init();
#endif
    srvcon$device_information_service$init();
}


/**@brief Start service controllers in preperation for run state
 */
static void service_controllers_start(void)
{
#if NRF_MODULE_ENABLED(BLE_BAS)    
    srvcon$battery_service$start();
#endif
#if NRF_MODULE_ENABLED(BLE_HRS)
    srvcon$heart_rate_service$start();
#endif
#if NRF_MODULE_ENABLED(BLE_PLXS)
    srvcon$pulse_oximeter_service$start();
#endif
#if NRF_MODULE_ENABLED(BLE_RSCS)
    srvcon$running_speed_cadence_service$start();
#endif
}

void ble$on_bsp_event(bsp_event_t event) {
    if((event) == BSP_EVENT_DISCONNECT) {
        ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
    }
    }
}

// initialize BLE
void ble$init() {
    ble$stack_init();
    gap_params$init();
    gatt$init();
    db_discovery$init();
    advertising$init();
    peer_manager$init();
    queued_write_module$init();
    service_controllers_init();
    conn_params$init();
}

// start BLE - initiates advertising making BLE visible to the world
void ble$start(bool erase_bonds) {
    service_controllers_start();
    advertising$start(erase_bonds);
}
