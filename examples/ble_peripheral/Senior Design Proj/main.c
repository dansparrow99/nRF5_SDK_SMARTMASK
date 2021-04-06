/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
// NEW
#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include <stdio.h>
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
//END NEW
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "SMARTMASK"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static nrf_ble_gatt_t                   m_gatt;                                     /**< GATT module instance. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static uint16_t                         m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


// begin Daniel Edits at top---------------------------

//setup debug error handler
#define DEBUG 1;
//number of recorded runtime errors
static int errnum=0;
//sensor detection
static bool bme280_1_detected=false;
static bool bme280_2_detected=false;
static bool battery_monitor_detected=false;
static bool connection_status = false;
static bool send_trim = false;
/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define SENSOR1 (0x76U)
#define SENSOR2 (0x77U)
/* Initial setup register address and their respective data*/

// 2B -> 001 010 11 -> Temp OSx1, Press OSx2, Normal Mode
// 20 -> 001 000 00 -> 62.5ms wait, no IIR Filter, Not 3-wire SPI
#define BME280_SETUP_REG_ADDR 0xF4U
const uint8_t BME280_SETUP_REG_DATA[] = {0x2BU,0x20U};

/* Data register definitions and buffers init*/
/* Burst read will be used to increase throughput, bytes are just in case*/
#define BME280_READ_REG 0xF7U

/* Buffer for samples read from sensor. */
#define SAMPLE_SIZE 8
#define PACKET_SIZE 18
static uint8_t sensor_data[SAMPLE_SIZE] = {0x42,0x41,0x53,0x45,0x42,0x41,0x53,0x45};
static uint8_t output_data[PACKET_SIZE] = {0xDE,0xAD,0xBE,0xEF};
/* Outdated reg storage/buffers below */

/*
#define BME280_PRESS_REG [0xF7U,0xF8U,0xF9U]
uint32_t pressure_data;

#define BME280_TEMP_REG [0xFAU,0xFBU,0xFCU]
uint32_t temperature_data;

#define BME280_HUM_REG [0xFDU,0xFEU]
uint16_t humidity_data;
*/
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/*Trim Params*/

static uint8_t reg_T1[] = {0x88,0x89};//used to retrieve T1-P1

static uint8_t reg_P2[] = {0x90,0x91};//used to retrieve P2-P5

static uint8_t reg_P6[] = {0x98,0x99};//used to retrieve P6-P9

static uint8_t reg_H1[] = {0xA1};
static uint8_t reg_H2[] = {0xE1,0xE2};
static uint8_t reg_H3[] = {0xE3};
static uint8_t reg_H4[] = {0xE4,0xE5};//[11:4]/[3:0]
static uint8_t reg_H5[] = {0xE5,0xE6};//[3:0]//[11:4]

// end Daniel edits------------------------------------

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint32_t err_code;
		//printf("ble stuff!");
    //NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.\r\n");
    //NRF_LOG_HEXDUMP_DEBUG(p_data, length);

    for (uint32_t i = 0; i < length; i++)
    {
        do
        {
            err_code = app_uart_put(p_data[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                //NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. \r\n", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
    if (p_data[length-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }

}
/**@snippet [Handling the data received over BLE] */



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
		switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						//nrf_delay_ms(50);
            printf("Connected!\r\n");
						connection_status = true;
						send_trim = true;						
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            printf("Disconnected!\r\n");
						connection_status = false;
						sd_nvic_SystemReset();
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum ATT MTU.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum event length.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    //NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
		
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
//	printf("something happened");
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                //NRF_LOG_DEBUG("Ready to send data over BLE NUS\r\n");
                //NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    err_code = ble_nus_string_send(&m_nus, data_array, index);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

///////////////////////////////////ADDED METHODS////////////////////////////////////////////

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp[])
{
	
    //NRF_LOG_INFO("First Data Point: %d \r\n", temp[0]);
}
void TWI_scan(void){
		ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
		//uint8_t t=0xFF;
    bool detected_device = false;
		
    for (address = 1; address <= 127; address++)
    {
			//nrf_delay_ms(10);
			err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, 1);
      if (err_code == NRF_SUCCESS)
      {
          detected_device = true;
          printf("TWI device detected at address 0x%x.\n", address);
      }
			
				
    }

    if (!detected_device)
    {
			printf("no device found");
    }
}
/**
 * @brief Function for setting normal T/P/H mode on BME280
 */
void BME280_init(void)
{
		uint8_t sample_data[] = {1,1,1,1};
		uint8_t reg[3] = {BME280_SETUP_REG_ADDR, BME280_SETUP_REG_DATA[0],BME280_SETUP_REG_DATA[1]};
		//printf("begin init\n");
    ret_code_t err_code;
		err_code = nrf_drv_twi_rx(&m_twi, SENSOR1, sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            bme280_1_detected = true;
            //printf("TWI device detected at address 0x76.\n");
						err_code = nrf_drv_twi_tx(&m_twi, SENSOR1, reg, sizeof(reg), false);
						APP_ERROR_CHECK(err_code);
	      }else{//printf("Device 0x76 not found\n");
					}
	
				err_code = nrf_drv_twi_rx(&m_twi, SENSOR2, sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            bme280_2_detected = true;
           //printf("TWI device detected at address 0x77.\n");
						err_code = nrf_drv_twi_tx(&m_twi, SENSOR2, reg, sizeof(reg), false);
						APP_ERROR_CHECK(err_code);
	      }else{//printf("Device 0x76 not found\n");
				}
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(sensor_data);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_BME280_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_BME280_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static bool read_sensor_data(uint8_t target, uint8_t reg)
{
    /* Read 8 bytes from the specified address - skip 3 bits dedicated for fractional part of temperature. */
		ret_code_t err_code = nrf_drv_twi_tx(&m_twi, target, &reg, 1,false);
		//printf("err tx: %i",err_code);
		//APP_ERROR_CHECK(err_code);
		if(err_code==0){
		err_code = nrf_drv_twi_rx(&m_twi, target, sensor_data, sizeof(sensor_data));
		APP_ERROR_CHECK(err_code);
		return false;
		}else{
			if(target==SENSOR1)
				output_data[17]=0xFE & output_data[17];
			else if(target==SENSOR2)
				output_data[17]=0xFD & output_data[17];
			else
				output_data[17]=0xF7 & output_data[17];
			return true;
		}
		
}

/**
 * @brief Function for reading BME280 status
 */
void BME280_params_read(uint8_t target)
{
	
	/* converts hex to ascii equivalent of that hex
	char data_hex_array[16];
		for(int index =0;index<8;index++)
		sprintf(&data_hex_array[index*2], "%02x",sensor_data[index]);
		err_code = ble_nus_string_send(&m_nus, data_hex_array, sizeof(data_hex_array));
	*/
		ret_code_t err_code=NRF_SUCCESS;
		uint8_t hum_trim[] = {0,0,0,0,0,0,0,0};
		read_sensor_data(target,reg_T1[0]);
		//nrf_delay_ms(50);
		//printf("Target: 0x%x\n",target);
		//printf("T1: 0x%x, 0x%x\n",sensor_data[0],sensor_data[1]);
		//printf("T2: 0x%x, 0x%x\n",sensor_data[2],sensor_data[3]);
		//printf("T3: 0x%x, 0x%x\n",sensor_data[4],sensor_data[5]);
		//printf("P1: 0x%x, 0x%x\n",sensor_data[6],sensor_data[7]);
		//nrf_delay_ms(50);
		err_code = ble_nus_string_send(&m_nus, sensor_data, 8);
		
		//printf("err_code: x%x\n",err_code);
		//APP_ERROR_CHECK(err_code);
		
		read_sensor_data(target,reg_P2[0]);
		//nrf_delay_ms(50);
		//printf("P2: 0x%x, 0x%x\n",sensor_data[0],sensor_data[1]);
		//printf("P3: 0x%x, 0x%x\n",sensor_data[2],sensor_data[3]);
		//printf("P4: 0x%x, 0x%x\n",sensor_data[4],sensor_data[5]);
		//printf("P5: 0x%x, 0x%x\n",sensor_data[6],sensor_data[7]);
		//nrf_delay_ms(50);
		err_code = ble_nus_string_send(&m_nus, sensor_data, 8);		
		//APP_ERROR_CHECK(err_code);
		
		read_sensor_data(target,reg_P6[0]);
		//nrf_delay_ms(50);
		//printf("P6: 0x%x, 0x%x\n",sensor_data[0],sensor_data[1]);
		//printf("P7: 0x%x, 0x%x\n",sensor_data[2],sensor_data[3]);
		//printf("P8: 0x%x, 0x%x\n",sensor_data[4],sensor_data[5]);
		//printf("P9: 0x%x, 0x%x\n",sensor_data[6],sensor_data[7]);
		//nrf_delay_ms(50);
		err_code = ble_nus_string_send(&m_nus, sensor_data, 8);		
		//APP_ERROR_CHECK(err_code);
	
		read_sensor_data(target,reg_H1[0]);
		//nrf_delay_ms(50);
		//printf("H1: 0x%x\n",sensor_data[0]);
		
		hum_trim[0]=sensor_data[0];
		
		read_sensor_data(target,reg_H2[0]);
		//nrf_delay_ms(50);
		//printf("H2: 0x%x, 0x%x\n",sensor_data[0],sensor_data[1]);
		
		hum_trim[1]=sensor_data[0];
		hum_trim[2]=sensor_data[1];
		
		read_sensor_data(target,reg_H3[0]);
		//nrf_delay_ms(50);
		//printf("H3: 0x%x\n",sensor_data[0]);
		
		hum_trim[3]=sensor_data[0];
		
		read_sensor_data(target,reg_H4[0]);
		//nrf_delay_ms(50);
		//printf("H3: 0x%x, 0x%x\n",sensor_data[0],sensor_data[1]);
		
		hum_trim[4]=sensor_data[0];
		hum_trim[5]=sensor_data[1];
		
		read_sensor_data(target,reg_H5[0]);
		//printf("H3: 0x%x, 0x%x\n",sensor_data[0],sensor_data[1]);
		
		hum_trim[6]=sensor_data[0];
		hum_trim[7]=sensor_data[1];
		
		err_code = ble_nus_string_send(&m_nus, hum_trim, 8);		
		//APP_ERROR_CHECK(err_code);
}

void bme280_trim_read(){
		ret_code_t err_code=NRF_SUCCESS;
		uint8_t a1[] = {0x76,0x76,0x76,0x76};
		err_code = ble_nus_string_send(&m_nus, a1, 4);
		if(bme280_1_detected)
			BME280_params_read(SENSOR1);
			
		uint8_t a2[] = {0x77,0x77,0x77,0x77};
		err_code = ble_nus_string_send(&m_nus, a2, 4);
		if(bme280_2_detected)
			BME280_params_read(SENSOR2);
}
/**
 * @brief custom error handler

 */

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	
    // On de-assert, the system can only recover with a reset.
#ifndef DEBUG
    NVIC_SystemReset();
#else
	printf("\nerror id: %i\n",id);
	printf("pc: %i\n",pc);
	printf("info: %i\n",info);
	printf("err#: %i\n",errnum);
	errnum++;
	//	NVIC_SystemReset();
#endif // DEBUG
}
/////////////////////////////////END ADDED METHODS//////////////////////////////////////////

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

    // Initialize.
    //err_code = app_timer_init();
    //APP_ERROR_CHECK(err_code);

    uart_init();
		//log_init();

    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

		twi_init();
	//	nrf_gpio_cfg_output(6);
	//	nrf_gpio_cfg_input(8,NRF_GPIO_PIN_PULLDOWN);
	
    //printf("\r\nUART Startt!\r\n");
    //NRF_LOG_INFO("UART Start!\r\n");
		//printf("\r\nUART Startt!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);
		//printf("\r\nUART Startt!\r\n");
		//TWI_scan();	//just in case: can be used to scan to check for available addresses
		BME280_init();
		//printf("end init\n");
		uint8_t iter_arr[16];
		for(int i =0;i<16;i++)
			sensor_data[i] = 0;
		

    for (;;)
    {
			if(connection_status){
				nrf_delay_ms(200);
				output_data[17]=0;
				if(send_trim){
					nrf_delay_ms(5000);
					bme280_trim_read();
					send_trim=false;
					nrf_delay_ms(2000);
				}
				//nrf_gpio_pin_set(6);
				//err_code = NRF_SUCCESS;
				
				if(bme280_1_detected){
					if(!read_sensor_data(SENSOR1,BME280_READ_REG))
						output_data[17]=0x01 | output_data[17];
					for(int i=0;i<8;i++)
					{
						output_data[i]=sensor_data[i];
					}
				}else{
					//output_data[17]=0xFE & output_data[17];
					//printf("sensor 1 not found!\n");
				}
				
				if(bme280_2_detected){
					if(!read_sensor_data(SENSOR2,BME280_READ_REG))
						output_data[17]=0x02 | output_data[17];
					for(int i=0;i<8;i++)
					{
						output_data[i+8]=sensor_data[i];
					}
				}else{
					//output_data[17]= 0xFD & output_data[17];
					//printf("sensor 2 not found!\n");
				}
				//if(nrf_gpio_pin_read(8))
				//{
				//	bme280_trim_read();
				//}else{
					if(bme280_1_detected || bme280_2_detected){
					//for(int x=0;x<17;x++){
						//printf("data[%i]= %i\r\n",x,output_data[x]);
					//}
					/*if(iter_arr[0]==255){
							iter_arr[1]=iter_arr[1]+1;
							iter_arr[0]=0;
							if(iter_arr[1]==255){
								iter_arr[2]++;
								iter_arr[1]=0;
								if(iter_arr[2]==255){
									iter_arr[3]++;
									iter_arr[2]=0;
									if(iter_arr[3]==255){
									iter_arr[4]++;
									iter_arr[3]=0;
									}else{iter_arr[3]++;}
								}else{iter_arr[2]++;}
							}else{iter_arr[1]++;}
					}else{iter_arr[0]=iter_arr[0]+1;}*/
					err_code = ble_nus_string_send(&m_nus, output_data, 18);
				//}
				}
				
				//APP_ERROR_CHECK(err_code);
				//nrf_delay_ms(10);
				
				
			}else{
				if(nrf_gpio_pin_read(6))
					nrf_gpio_pin_clear(6);
			}
		power_manage();	
	}
} 


/**
 * @}
 */
