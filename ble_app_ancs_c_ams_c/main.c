/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_apple_notification_main main.c
 * @{
 * @ingroup ble_sdk_app_apple_notification
 * @brief Apple Notification Client Sample Application main file. Disclaimer:
 * This client implementation of the Apple Notification Center Service can and
 * will be changed at any time by Nordic Semiconductor ASA.
 *
 * Server implementations such as the ones found in iOS can be changed at any
 * time by Apple and may cause this client implementation to stop working.
 *
 * This file contains the source code for a sample application using the Apple
 * Notification Center Service Client.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_err.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "ble_db_discovery.h"
#include "nrf_ble_gatts_c.h"
#include "nrf_ble_ancs_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "app_uart.h"
#include "ble_nus.h"
#include "ble_bas.h"

#include "nrf_drv_saadc.h"

#include "nrf_ble_ams_c.h"

#define ATTR_DATA_SIZE                 BLE_ANCS_ATTR_DATA_MAX                 /**< Allocated size for attribute data. */

#define DISPLAY_MESSAGE_BUTTON_ID      1                                      /**< Button used to request notification attributes. */

#define DEVICE_NAME                    "NRF52_ANCS_AMS"                                 /**< Name of the device. Will be included in the advertising data. */
#define APP_BLE_OBSERVER_PRIO          3                                      /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG           1                                      /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_FAST_INTERVAL          40                                     /**< The advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL          3200                                   /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */

#define APP_ADV_FAST_DURATION          3000                                   /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION          18000                                  /**< The advertising duration of slow advertising in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(500, UNIT_1_25_MS)       /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(1000, UNIT_1_25_MS)      /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                  0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(6000, UNIT_10_MS)        /**< Connection supervisory time-out (4 seconds). */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(120000)                 /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)                  /**< Time from initiating an event (connect or start of notification) to the first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000)                 /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT   3                                      /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE            18                                     /**< Size of buffer holding optional messages in notifications. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(1500)                  /**< Delay after connection until security request is sent, if necessary (ticks). */

#define SEC_PARAM_BOND                 1                                      /**< Perform bonding. */
#define SEC_PARAM_MITM                 0                                      /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                 0                                      /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS             0                                      /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                   /**< No I/O capabilities. */
#define SEC_PARAM_OOB                  0                                      /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         7                                      /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                     /**< Maximum encryption key size. */

#define DEAD_BEEF                      0xDEADBEEF                             /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */

#define SCHED_MAX_EVENT_DATA_SIZE      APP_TIMER_SCHED_EVENT_DATA_SIZE        /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               20                                     /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE               10                                     /**< Maximum number of events in the scheduler queue. */
#endif

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
        ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

BLE_BAS_DEF(m_bas);                                     /**< Battery service instance. */
NRF_BLE_GATTS_C_DEF(m_gatts_c);                                               /**< GATT Service client instance. Handles Service Changed indications from the peer. */
BLE_ANCS_C_DEF(m_ancs_c);                                                     /**< Apple Notification Service Client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                     /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                       /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                           /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                              /**< DB Discovery module instance. */
BLE_AMS_C_DEF(m_ams_c);                                                       /**< Apple Media Service Client instance. */

APP_TIMER_DEF(m_battery_timer_id);                                            /**< Battery measurement timer. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                             /**< BLE NUS service instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                      /**< Handle of the current connection. */

static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];      /**< List of peers currently in the whitelist. */
static uint32_t m_whitelist_peer_cnt;                                         /**< Number of peers currently in the whitelist. */
static uint16_t m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */

static ble_gatt_db_srv_t m_peer_srv_buf[2] = {0};                             /**< Array of services with room to store both GATT Service and ANCS. */

static bool m_ancs_discovered  = false;                                       /**< Bool to keep track of when both ancs and gatts have been disovered. Only then do we want to save the peer data. */
static bool m_gatts_discovered = false;                                       /**< Bool to keep track of when both ancs and gatts have been disovered. Only then do we want to save the peer data. */

static ble_ancs_c_evt_notif_t m_notification_latest;                          /**< Local copy to keep track of the newest arriving notifications. */
static ble_ancs_c_attr_t m_notif_attr_latest;                                 /**< Local copy of the newest notification attribute. */
static ble_ancs_c_attr_t m_notif_attr_app_id_latest;                          /**< Local copy of the newest app attribute. */

static uint8_t m_attr_appid[ATTR_DATA_SIZE];                                  /**< Buffer to store attribute data. */
static uint8_t m_attr_title[ATTR_DATA_SIZE];                                  /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ATTR_DATA_SIZE];                               /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ATTR_DATA_SIZE];                                /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */
static uint8_t m_attr_date[ATTR_DATA_SIZE];                                   /**< Buffer to store attribute data. */
static uint8_t m_attr_posaction[ATTR_DATA_SIZE];                              /**< Buffer to store attribute data. */
static uint8_t m_attr_negaction[ATTR_DATA_SIZE];                              /**< Buffer to store attribute data. */
static uint8_t m_attr_disp_name[ATTR_DATA_SIZE];                              /**< Buffer to store attribute data. */

static uint8_t m_entity_update_artist[BLE_AMS_EU_MAX_DATA_LENGTH];
static uint8_t m_entity_update_album[BLE_AMS_EU_MAX_DATA_LENGTH];
static uint8_t m_entity_update_track[BLE_AMS_EU_MAX_DATA_LENGTH];
static uint8_t m_entity_update_duration[BLE_AMS_EU_MAX_DATA_LENGTH];
static uint8_t m_entity_attribute[BLE_AMS_EA_MAX_DATA_LENGTH];

/* ENTITY ATTRIBUTE TEST */
static uint8_t entity_attribute_id = 0;
/* ENTITY ATTRIBUTE TEST */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static nrf_saadc_value_t adc_buf[2];

/**@brief String literals for the iOS notification categories. used then printing to UART. */
static char const * lit_catid[BLE_ANCS_NB_OF_CATEGORY_ID] =
{
        "Other",
        "Incoming Call",
        "Missed Call",
        "Voice Mail",
        "Social",
        "Schedule",
        "Email",
        "News",
        "Health And Fitness",
        "Business And Finance",
        "Location",
        "Entertainment"
};

/**@brief String literals for the iOS notification event types. Used then printing to UART. */
static char const * lit_eventid[BLE_ANCS_NB_OF_EVT_ID] =
{
        "Added",
        "Modified",
        "Removed"
};

/**@brief String literals for the iOS notification attribute types. Used when printing to UART. */
static char const * lit_attrid[BLE_ANCS_NB_OF_NOTIF_ATTR] =
{
        "App Identifier",
        "Title",
        "Subtitle",
        "Message",
        "Message Size",
        "Date",
        "Positive Action Label",
        "Negative Action Label"
};

/**@brief String literals for the iOS notification attribute types. Used When printing to UART. */
static char const * lit_appid[BLE_ANCS_NB_OF_APP_ATTR] =
{
        "Display Name"
};

static void delete_bonds(void);
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt);

/**@brief Callback function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 *          You must analyze how your product should react to asserts.
 * @warning On assert from the SoftDevice, the system can recover only on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
        pm_peer_id_t peer_id;
        uint32_t peers_to_copy;

        peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                        *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

        peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
        *p_size = 0;

        while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
        {
                p_peers[(*p_size)++] = peer_id;
                peer_id = pm_next_peer_id_get(peer_id);
        }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
        if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
        {
                nrf_saadc_value_t adc_result;
                uint16_t batt_lvl_in_milli_volts;
                uint8_t percentage_batt_lvl;
                uint32_t err_code;

                adc_result = p_event->data.done.p_buffer[0];

                err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
                APP_ERROR_CHECK(err_code);

                batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                          DIODE_FWD_VOLT_DROP_MILLIVOLTS;
                percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

                err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
                if ((err_code != NRF_SUCCESS) &&
                    (err_code != NRF_ERROR_INVALID_STATE) &&
                    (err_code != NRF_ERROR_RESOURCES) &&
                    (err_code != NRF_ERROR_BUSY) &&
                    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    )
                {
                        APP_ERROR_HANDLER(err_code);
                }
                nrf_drv_saadc_uninit();
        }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
        ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
        APP_ERROR_CHECK(err_code);

        nrf_saadc_channel_config_t config =
                NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
        err_code = nrf_drv_saadc_channel_init(0, &config);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
        UNUSED_PARAMETER(p_context);
        adc_configure();

        ret_code_t err_code;
        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t *p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint8_t index = 0;
        uint32_t err_code;

        switch (p_event->evt_type)
        {
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= m_ble_nus_max_data_len))
                {
                        if (index > 1)
                        {
                                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                                do
                                {
                                        uint16_t length = (uint16_t)index;
                                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                            (err_code != NRF_ERROR_RESOURCES) &&
                                            (err_code != NRF_ERROR_NOT_FOUND))
                                        {
                                                APP_ERROR_CHECK(err_code);
                                        }
                                } while (err_code == NRF_ERROR_RESOURCES);
                        }
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

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no = RX_PIN_NUMBER,
                .tx_pin_no = TX_PIN_NUMBER,
                .rts_pin_no = RTS_PIN_NUMBER,
                .cts_pin_no = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity = false,
#if defined(UART_PRESENT)
                .baud_rate = NRF_UART_BAUDRATE_115200
#else
                .baud_rate = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising. */
static void advertising_start(bool erase_bonds)
{
        if (erase_bonds == true)
        {
                delete_bonds();
                // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
        }
        else
        {
                ret_code_t ret;

                memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
                m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

                peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

                ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                APP_ERROR_CHECK(ret);

                // Setup the device identies list.
                // Some SoftDevices do not support this feature.
                ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                if (ret != NRF_ERROR_NOT_SUPPORTED)
                {
                        APP_ERROR_CHECK(ret);
                }

                ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(ret);
        }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
        ret_code_t ret;

        pm_handler_on_pm_evt(p_evt);
        pm_handler_flash_clean(p_evt);

        switch (p_evt->evt_id)
        {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
                if (p_evt->peer_id != PM_PEER_ID_INVALID)
                {
                        uint16_t data_len = sizeof(m_peer_srv_buf);
                        ret = pm_peer_data_remote_db_load(p_evt->peer_id, m_peer_srv_buf, &data_len);
                        if (ret == NRF_ERROR_NOT_FOUND)
                        {
                                NRF_LOG_DEBUG("Could not find the remote database in flash.");
                                ret = nrf_ble_gatts_c_handles_assign(&m_gatts_c, p_evt->conn_handle, NULL);
                                APP_ERROR_CHECK(ret);

                                // Discover peer's services.
                                m_ancs_discovered  = false;
                                m_gatts_discovered = false;
                                memset(&m_db_disc, 0x00, sizeof(m_db_disc));
                                ret = ble_db_discovery_start(&m_db_disc, p_evt->conn_handle);
                                APP_ERROR_CHECK(ret);
                        }
                        else
                        {
                                // Check if the load was successful.
                                ASSERT(data_len == sizeof(m_peer_srv_buf));
                                APP_ERROR_CHECK(ret);
                                NRF_LOG_INFO("Remote Database loaded from flash.");

                                // Assign the loaded handles to the GATT Service client module.
                                ble_gatt_db_char_t srv_changed_handles = m_peer_srv_buf[0].charateristics[0];
                                ret = nrf_ble_gatts_c_handles_assign(&m_gatts_c,
                                                                     p_evt->conn_handle,
                                                                     &srv_changed_handles);
                                APP_ERROR_CHECK(ret);

                                // Enable indications.
                                ret = nrf_ble_gatts_c_enable_indication(&m_gatts_c, true);
                                APP_ERROR_CHECK(ret);

                                //Load the relevant handles into a ble_ancs_c_service_t struct that can be
                                // assigned to the ANCS module.
                                ble_ancs_c_service_t ancs_handles;
                                ble_gatt_db_char_t * p_char           = m_peer_srv_buf[1].charateristics;
                                ancs_handles.control_point_char       = p_char[0].characteristic;
                                ancs_handles.notif_source_char        = p_char[1].characteristic;
                                ancs_handles.notif_source_cccd.handle = p_char[1].cccd_handle;
                                ancs_handles.data_source_char         = p_char[2].characteristic;
                                ancs_handles.data_source_cccd.handle  = p_char[2].cccd_handle;

                                ret = nrf_ble_ancs_c_handles_assign(&m_ancs_c, p_evt->conn_handle,
                                                                    &ancs_handles);
                                APP_ERROR_CHECK(ret);
                        }
                }
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
                // Check it the Service Changed characteristic handle exists in our client instance.
                // If it is invalid, we know service discovery is needed.
                // (No database was loaded during @ref PM_EVT_BONDED_PEER_CONNECTED)
                if (m_gatts_c.srv_changed_char.characteristic.handle_value == BLE_GATT_HANDLE_INVALID)
                {
                        ret = nrf_ble_gatts_c_handles_assign(&m_gatts_c, p_evt->conn_handle, NULL);
                        APP_ERROR_CHECK(ret);

                        // Discover peer's services.
                        m_ancs_discovered  = false;
                        m_gatts_discovered = false;
                        memset(&m_db_disc, 0x00, sizeof(m_db_disc));
                        ret = ble_db_discovery_start(&m_db_disc, p_evt->conn_handle);
                        APP_ERROR_CHECK(ret);
                }

        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
                advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
                // Note: You should check on what kind of white list policy your application should use.
                if (     p_evt->params.peer_data_update_succeeded.flash_changed
                         && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
                {
                        NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
                        NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                                      m_whitelist_peer_cnt + 1,
                                      BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                        if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                        {
                                // Bonded to a new peer, add it to the whitelist.
                                m_whitelist_peers[m_whitelist_peer_cnt++] = p_evt->peer_id;

                                // The whitelist has been modified, update it in the Peer Manager.
                                ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                                if (ret != NRF_ERROR_NOT_SUPPORTED)
                                {
                                        APP_ERROR_CHECK(ret);
                                }

                                ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                                APP_ERROR_CHECK(ret);
                        }
                }
        } break;

        default:
                break;
        }
}


/**@brief Function for setting up GATTC notifications from the Notification Provider.
 *
 * @details This function is called when a successful connection has been established.
 */
static void apple_notification_setup(void)
{
        ret_code_t ret;

        nrf_delay_ms(100); // Delay because we cannot add a CCCD to close to starting encryption. iOS specific.

        ret = ble_ancs_c_notif_source_notif_enable(&m_ancs_c);
        APP_ERROR_CHECK(ret);

        ret = ble_ancs_c_data_source_notif_enable(&m_ancs_c);
        APP_ERROR_CHECK(ret);

        NRF_LOG_DEBUG("Notifications Enabled.");
}

/**@brief Function for setting up GATTC notifications from the Notification Provider.
 *
 * @details This function is called when a successful connection has been established.
 */
static void apple_media_setup(void)
{
        ret_code_t ret;

        nrf_delay_ms(100); // Delay because we cannot add a CCCD to close to starting encryption. iOS specific.

        ret = ble_ams_c_remote_command_notif_enable(&m_ams_c);
        APP_ERROR_CHECK(ret);

        ret = ble_ams_c_entity_update_notif_enable(&m_ams_c);
        APP_ERROR_CHECK(ret);

        NRF_LOG_INFO("Apple Media-Notifications Enabled.");
}

/**@brief Function for printing out errors that originated from the Media Source (iOS).
 *
 * @param[in] err_code_np Error code received from MS.
 */
static void err_code_print(uint16_t err_code_ms)
{
        switch (err_code_ms)
        {
        case BLE_AMS_MS_INVALID_STATE:
                NRF_LOG_INFO("Error: Media Source is currently in an invalid state");
                break;

        case BLE_AMS_MS_INVALID_COMMAND:
                NRF_LOG_INFO("Error: Command ID was not recognized by the Media Source. ");
                break;

        case BLE_AMS_MS_ABSENT_ATTRIBUTE:
                NRF_LOG_INFO("Error: Attribute is absent on the Media Source. ");
                break;

        default:
                break;
        }
}

/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_print(ble_ancs_c_evt_notif_t * p_notif)
{
        NRF_LOG_INFO("\r\nNotification");
        NRF_LOG_INFO("Event:       %s", (uint32_t)lit_eventid[p_notif->evt_id]);
        NRF_LOG_INFO("Category ID: %s", (uint32_t)lit_catid[p_notif->category_id]);
        NRF_LOG_INFO("Category Cnt:%u", (unsigned int) p_notif->category_count);
        NRF_LOG_INFO("UID:         %u", (unsigned int) p_notif->notif_uid);

        NRF_LOG_INFO("Flags:");
        if (p_notif->evt_flags.silent == 1)
        {
                NRF_LOG_INFO(" Silent");
        }
        if (p_notif->evt_flags.important == 1)
        {
                NRF_LOG_INFO(" Important");
        }
        if (p_notif->evt_flags.pre_existing == 1)
        {
                NRF_LOG_INFO(" Pre-existing");
        }
        if (p_notif->evt_flags.positive_action == 1)
        {
                NRF_LOG_INFO(" Positive Action");
        }
        if (p_notif->evt_flags.negative_action == 1)
        {
                NRF_LOG_INFO(" Negative Action");
        }
}


/**@brief Function for printing iOS notification attribute data.
 *
 * @param[in] p_attr Pointer to an iOS notification attribute.
 */
static void notif_attr_print(ble_ancs_c_attr_t * p_attr)
{
        if (p_attr->attr_len != 0)
        {
                NRF_LOG_INFO("%s: %s", (uint32_t)lit_attrid[p_attr->attr_id],
                             nrf_log_push((char *)p_attr->p_attr_data));
        }
        else if (p_attr->attr_len == 0)
        {
                NRF_LOG_INFO("%s: (N/A)", (uint32_t)lit_attrid[p_attr->attr_id]);
        }
}


/**@brief Function for printing iOS notification attribute data.
 *
 * @param[in] p_attr Pointer to an iOS App attribute.
 */
static void app_attr_print(ble_ancs_c_attr_t * p_attr)
{
        if (p_attr->attr_len != 0)
        {
                NRF_LOG_INFO("%s: %s", (uint32_t)lit_appid[p_attr->attr_id], (uint32_t)p_attr->p_attr_data);
        }
        else if (p_attr->attr_len == 0)
        {
                NRF_LOG_INFO("%s: (N/A)", (uint32_t) lit_appid[p_attr->attr_id]);
        }
}


/**@brief Function for printing out errors that originated from the Notification Provider (iOS).
 *
 * @param[in] err_code_np Error code received from NP.
 */
static void err_code_print_AMS(uint16_t err_code_np)
{
        switch (err_code_np)
        {
        case BLE_ANCS_NP_UNKNOWN_COMMAND:
                NRF_LOG_INFO("Error: Command ID was not recognized by the Notification Provider. ");
                break;

        case BLE_ANCS_NP_INVALID_COMMAND:
                NRF_LOG_INFO("Error: Command failed to be parsed on the Notification Provider. ");
                break;

        case BLE_ANCS_NP_INVALID_PARAMETER:
                NRF_LOG_INFO("Error: Parameter does not refer to an existing object on the Notification Provider. ");
                break;

        case BLE_ANCS_NP_ACTION_FAILED:
                NRF_LOG_INFO("Error: Perform Notification Action Failed on the Notification Provider. ");
                break;

        default:
                break;
        }
}

/**@brief Function for handling the Apple Notification Service client.
 *
 * @details This function is called for all events in the Apple Notification client that
 *          are passed to the application.
 *
 * @param[in] p_evt  Event received from the Apple Notification Service client.
 */
static void ams_c_evt_handler(ble_ams_c_evt_t * p_evt)
{
        ret_code_t ret = NRF_SUCCESS;

        switch (p_evt->evt_type)
        {
        case BLE_AMS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_INFO("Apple Media Service discovered on the server.");
                ret = nrf_ble_ams_c_handles_assign(&m_ams_c, p_evt->conn_handle, &p_evt->service);
                APP_ERROR_CHECK(ret);
                apple_media_setup();
                break;

        case BLE_AMS_C_EVT_REMOTE_COMMAND_NOTIFICATION:
                //NRF_LOG_INFO("BLE_AMS_C_EVT_REMOTE_COMMAND_NOTIFICATION: ListSize: %d.", p_evt->remote_command_data.remote_command_len);
                break;

        case BLE_AMS_C_EVT_ENTITY_UPDATE_NOTIFICATION:
                switch(p_evt->entity_update_data.attribute_id)
                {
                case BLE_AMS_TRACK_ATTRIBUTE_ID_ARTIST:
                        NRF_LOG_INFO("Artist\r\n");
                        if(p_evt->entity_update_data.entity_update_flag == 1)
                        {
                                NRF_LOG_INFO("Truncated. Needs to read out manually\r\n");
                        }
                        memcpy(m_entity_update_artist, p_evt->entity_update_data.p_entity_update_data, p_evt->entity_update_data.entity_update_data_len);
                        for(uint8_t i = 0; i < p_evt->entity_update_data.entity_update_data_len; i++)
                        {
                                NRF_LOG_INFO("%c", m_entity_update_artist[i]);
                        }
                        NRF_LOG_INFO("\r\n");
                        break;

                case BLE_AMS_TRACK_ATTRIBUTE_ID_ALBUM:
                        NRF_LOG_INFO("Album\r\n");
                        if(p_evt->entity_update_data.entity_update_flag == 1)
                        {
                                NRF_LOG_INFO("Truncated. Needs to read out manually\r\n");
                        }
                        memcpy(m_entity_update_album, p_evt->entity_update_data.p_entity_update_data, p_evt->entity_update_data.entity_update_data_len);
                        for(uint8_t i = 0; i < p_evt->entity_update_data.entity_update_data_len; i++)
                        {
                                NRF_LOG_INFO("%c", m_entity_update_album[i]);
                        }
                        NRF_LOG_INFO("\r\n");
                        break;

                case BLE_AMS_TRACK_ATTRIBUTE_ID_TITLE:
                        NRF_LOG_INFO("Title\r\n");
                        if(p_evt->entity_update_data.entity_update_flag == 1)
                        {
                                NRF_LOG_INFO("Truncated. Needs to read out manually\r\n");
                        }
                        memcpy(m_entity_update_track, p_evt->entity_update_data.p_entity_update_data, p_evt->entity_update_data.entity_update_data_len);
                        for(uint8_t i = 0; i < p_evt->entity_update_data.entity_update_data_len; i++)
                        {
                                NRF_LOG_INFO("%c", m_entity_update_track[i]);
                        }
                        NRF_LOG_INFO("\r\n");
                        break;

                case BLE_AMS_TRACK_ATTRIBUTE_ID_DURATION:
                {
                        NRF_LOG_INFO("Duration\r\n");
                        if(p_evt->entity_update_data.entity_update_flag == 1)
                        {
                                NRF_LOG_INFO("Truncated. Needs to read out manually\r\n");
                        }
                        memcpy(m_entity_update_duration, p_evt->entity_update_data.p_entity_update_data, p_evt->entity_update_data.entity_update_data_len);
                        for(uint8_t i = 0; i < p_evt->entity_update_data.entity_update_data_len; i++)
                        {
                                NRF_LOG_INFO("%c", m_entity_update_duration[i]);
                        }
                        uint8_t durUnits, durDecimals;
                        double duration_units = ((m_entity_update_duration[0]-0x30)*100) + ((m_entity_update_duration[1]-0x30)*10) + (m_entity_update_duration[2]-0x30);
                        double duration_decimals = ((m_entity_update_duration[4]-0x30) * 100) + ((m_entity_update_duration[5]-0x30)*10) + ((m_entity_update_duration[6]-0x30));
                        double duration = duration_units + (duration_decimals / 1000);
                        durUnits = duration / 60;
                        durDecimals = (durUnits - (duration / 60)) * 60;
                        NRF_LOG_INFO("duration: %d:%d\r\n", durUnits, durDecimals);
                        break;
                }

                default:
                        break;
                }
                break;

        case BLE_AMS_C_EVT_ENTITY_ATTRIBUTE_READ_RESP:
        {
                memcpy(m_entity_attribute, p_evt->entity_attribute_data.p_entity_attribute_data, p_evt->entity_attribute_data.attribute_len);
                for(uint8_t i = 0; i < p_evt->entity_attribute_data.attribute_len; i++)
                {
                        NRF_LOG_INFO("%c", m_entity_attribute[i]);
                }
                NRF_LOG_INFO("\r\n");
                if(p_evt->entity_attribute_data.attribute_len < (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3))
                {
                        NRF_LOG_INFO("Read Complete!\r\n");
                }
                else
                {
                        NRF_LOG_INFO("Read not complete. Call sd_ble_gattc_read() with offset: %d\r\n", (p_evt->entity_attribute_data.attribute_offset + p_evt->entity_attribute_data.attribute_len));
                        ble_ams_c_entity_attribute_read(&m_ams_c, (p_evt->entity_attribute_data.attribute_offset + p_evt->entity_attribute_data.attribute_len));
                }
                break;
        }

        case BLE_AMS_C_EVT_DISCOVERY_FAILED:
                NRF_LOG_INFO("Apple Media Service not discovered on the server.");
                break;

        case BLE_AMS_C_EVT_ENTITY_UPDATE_ATTRIBUTE_ERROR:
                err_code_print(p_evt->err_code_ms);
                break;
        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
        ret_code_t ret;

        ret = app_timer_init();
        APP_ERROR_CHECK(ret);

        // Create battery timer.
        ret = app_timer_create(&m_battery_timer_id,
                               APP_TIMER_MODE_REPEATED,
                               battery_level_meas_timeout_handler);
        APP_ERROR_CHECK(ret);
}

static void timers_start(void)
{
        ret_code_t ret;
        // Create battery timer.
        ret = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
        APP_ERROR_CHECK(ret);
}


/**@brief Function for handling events from the GATT Servive client module.

   @param[in] p_evt GATT Service event.
 */
static void gatts_c_evt_handler(nrf_ble_gatts_c_evt_t * p_evt)
{
        ret_code_t ret = NRF_SUCCESS;

        switch (p_evt->evt_type)
        {
        case NRF_BLE_GATTS_C_EVT_DISCOVERY_COMPLETE:
        {
                NRF_LOG_DEBUG("GATT Service and Service Changed characteristic found on server.");

                ret = nrf_ble_gatts_c_handles_assign(&m_gatts_c,
                                                     p_evt->conn_handle,
                                                     &p_evt->params.srv_changed_char);
                APP_ERROR_CHECK(ret);

                pm_peer_id_t peer_id;
                ret = pm_peer_id_get(p_evt->conn_handle, &peer_id);
                APP_ERROR_CHECK(ret);

                memset(&m_peer_srv_buf[0], 0, sizeof(m_peer_srv_buf[0]));
                m_peer_srv_buf[0].charateristics[0] = p_evt->params.srv_changed_char;

                m_gatts_discovered = true;
                if(m_gatts_discovered && m_ancs_discovered)
                {
                        ret = pm_peer_data_remote_db_store(peer_id,
                                                           (ble_gatt_db_srv_t *)m_peer_srv_buf,
                                                           sizeof(m_peer_srv_buf),
                                                           NULL);
                        if (ret == NRF_ERROR_STORAGE_FULL)
                        {
                                ret = fds_gc();
                        }
                        APP_ERROR_CHECK(ret);
                }
                ret = nrf_ble_gatts_c_enable_indication(&m_gatts_c, true);
                APP_ERROR_CHECK(ret);
        } break;

        case NRF_BLE_GATTS_C_EVT_DISCOVERY_FAILED:
                NRF_LOG_DEBUG("GATT Service or Service Changed characteristic not found on server.");
                break;

        case NRF_BLE_GATTS_C_EVT_DISCONN_COMPLETE:
                NRF_LOG_DEBUG("GATTS Service client disconnected connection handle %i.",
                              p_evt->conn_handle);
                break;

        case NRF_BLE_GATTS_C_EVT_SRV_CHANGED:
                NRF_LOG_DEBUG("Service Changed indication received.");

                // Discover peer's services.
                m_ancs_discovered  = false;
                m_gatts_discovered = false;
                ret = ble_db_discovery_start(&m_db_disc, p_evt->conn_handle);
                APP_ERROR_CHECK(ret);
                break;

        default:
                break;
        }
}


/**@brief Function for handling the Apple Notification Service client.
 *
 * @details This function is called for all events in the Apple Notification client that
 *          are passed to the application.
 *
 * @param[in] p_evt  Event received from the Apple Notification Service client.
 */
static void ancs_c_evt_handler(ble_ancs_c_evt_t * p_evt)
{

        switch (p_evt->evt_type)
        {
        case BLE_ANCS_C_EVT_DISCOVERY_COMPLETE:
        {
                ret_code_t ret = NRF_SUCCESS;
                NRF_LOG_DEBUG("Apple Notification Center Service discovered on the server.");
                ret = nrf_ble_ancs_c_handles_assign(&m_ancs_c, p_evt->conn_handle, &p_evt->service);
                APP_ERROR_CHECK(ret);

                pm_peer_id_t peer_id;
                ret = pm_peer_id_get(p_evt->conn_handle, &peer_id);
                APP_ERROR_CHECK(ret);

                // Copy the needed ANCS handles into a ble_gatt_db_srv_t struct that will be stored in
                // flash.
                ble_gatt_db_char_t * p_char = m_peer_srv_buf[1].charateristics;
                memset(&m_peer_srv_buf[1], 0, sizeof(m_peer_srv_buf[1]));

                p_char[0].characteristic = p_evt->service.control_point_char;
                p_char[1].characteristic = p_evt->service.notif_source_char;
                p_char[1].cccd_handle    = p_evt->service.notif_source_cccd.handle;
                p_char[2].characteristic = p_evt->service.data_source_char;
                p_char[2].cccd_handle    = p_evt->service.data_source_cccd.handle;

                m_ancs_discovered = true;

                if (m_gatts_discovered && m_ancs_discovered)
                {
                        ret = pm_peer_data_remote_db_store(peer_id,
                                                           (ble_gatt_db_srv_t *)m_peer_srv_buf,
                                                           sizeof(m_peer_srv_buf),
                                                           NULL);
                        if (ret == NRF_ERROR_STORAGE_FULL)
                        {
                                ret = fds_gc();
                        }
                        APP_ERROR_CHECK(ret);
                }
                apple_notification_setup();
        } break;

        case BLE_ANCS_C_EVT_NOTIF:
                m_notification_latest = p_evt->notif;
                notif_print(&m_notification_latest);
                break;

        case BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE:
                m_notif_attr_latest = p_evt->attr;
                notif_attr_print(&m_notif_attr_latest);
                if (p_evt->attr.attr_id == BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER)
                {
                        m_notif_attr_app_id_latest = p_evt->attr;
                }
                break;
        case BLE_ANCS_C_EVT_DISCOVERY_FAILED:
                NRF_LOG_DEBUG("Apple Notification Center Service not discovered on the server.");
                break;

        case BLE_ANCS_C_EVT_APP_ATTRIBUTE:
                app_attr_print(&p_evt->attr);
                break;
        case BLE_ANCS_C_EVT_NP_ERROR:
                err_code_print(p_evt->err_code_np);
                break;
        default:
                // No implementation needed.
                break;
        }
}


/**@brief Function for initializing GAP connection parameters.
 *
 * @details Use this function to set up all necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
        ret_code_t ret;
        ble_gap_conn_params_t gap_conn_params;
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        ret = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
        APP_ERROR_CHECK(ret);

        memset(&gap_conn_params, 0, sizeof(gap_conn_params));

        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
        gap_conn_params.slave_latency     = SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

        ret = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(ret);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                //m_ble_params_info.mtu = m_ble_its_max_data_len;
                m_ble_nus_max_data_len = data_length;
                NRF_LOG_INFO("gatt_event: ATT MTU is set to 0x%X (%d)", data_length, data_length);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
                m_ble_nus_max_data_len = data_length;
        }
        NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
        ret_code_t ret = nrf_ble_gatt_init(&m_gatt, NULL);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(ret);
}


/**@brief Function for handling the Apple Notification Service client errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void apple_notification_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Apple Notification Service client errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void apple_media_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling a Connection Parameters error.
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
        ret_code_t ret;
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = true;
        cp_init.evt_handler                    = NULL;
        cp_init.error_handler                  = conn_params_error_handler;

        ret = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery
 *          module. Depending on the UUIDs that are discovered, this function should forward the
 *          events to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        ble_ancs_c_on_db_disc_evt(&m_ancs_c, p_evt);
        ble_ams_c_on_db_disc_evt(&m_ams_c, p_evt);
        nrf_ble_gatts_c_on_db_disc_evt(&m_gatts_c, p_evt);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
        ble_gap_sec_params_t sec_param;
        ret_code_t ret;

        ret = pm_init();
        APP_ERROR_CHECK(ret);

        memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

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

        ret = pm_sec_params_set(&sec_param);
        APP_ERROR_CHECK(ret);

        ret = pm_register(pm_evt_handler);
        APP_ERROR_CHECK(ret);
}


/**
 * @brief Delete all data stored for all peers.
 */
static void delete_bonds(void)
{
        ret_code_t err_code;

        NRF_LOG_INFO("Erase bonds!");

        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
        uint32_t ret = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(ret);

        // Prepare wakeup buttons.
        ret = bsp_btn_ble_sleep_mode_prepare();
        APP_ERROR_CHECK(ret);

        // Go to system-off mode (this function will not return; wakeup will cause a reset).
        ret = sd_power_system_off();
        APP_ERROR_CHECK(ret);
}


/**@brief Function for handling advertising events.
 *
 * @details This function is called for advertising events that are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
        ret_code_t ret;

        switch (ble_adv_evt)
        {
        case BLE_ADV_EVT_FAST:
                NRF_LOG_INFO("Fast advertising");
                ret = bsp_indication_set(BSP_INDICATE_ADVERTISING);
                APP_ERROR_CHECK(ret);
                break;

        case BLE_ADV_EVT_SLOW:
                NRF_LOG_INFO("Slow advertising");
                ret = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
                APP_ERROR_CHECK(ret);
                break;

        case BLE_ADV_EVT_FAST_WHITELIST:
                NRF_LOG_INFO("Fast advertising with Whitelist");
                ret = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
                APP_ERROR_CHECK(ret);
                break;

        case BLE_ADV_EVT_IDLE:
                sleep_mode_enter();
                break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
                ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
                ble_gap_irk_t whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
                uint32_t addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
                uint32_t irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

                ret = pm_whitelist_get(whitelist_addrs, &addr_cnt, whitelist_irks, &irk_cnt);
                APP_ERROR_CHECK(ret);
                NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                              addr_cnt,
                              irk_cnt);

                // Apply the whitelist.
                ret = ble_advertising_whitelist_reply(&m_advertising,
                                                      whitelist_addrs,
                                                      addr_cnt,
                                                      whitelist_irks,
                                                      irk_cnt);
                APP_ERROR_CHECK(ret);
        }
        break;

        default:
                break;
        }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        ret_code_t ret = NRF_SUCCESS;

        pm_handler_secure_on_connection(p_ble_evt);
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("Connected.");
                ret = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(ret);

                m_cur_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

                ret = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_cur_conn_handle);
                APP_ERROR_CHECK(ret);
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected.");
                m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                NRF_LOG_INFO("Disconnected, handle: %d, reason: 0x%x",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                if (p_ble_evt->evt.gap_evt.conn_handle == m_ancs_c.conn_handle)
                {
                        m_ancs_c.conn_handle = BLE_CONN_HANDLE_INVALID;
                }
                break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                ret = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(ret);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                ret = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(ret);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                ret = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(ret);
                break;

        default:
                // No implementation needed.
                break;
        }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
        ret_code_t ret;

        switch (event)
        {
        case BSP_EVENT_SLEEP:
                sleep_mode_enter();
                break;

        case BSP_EVENT_DISCONNECT:
                ret = sd_ble_gap_disconnect(m_cur_conn_handle,
                                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (ret != NRF_ERROR_INVALID_STATE)
                {
                        APP_ERROR_CHECK(ret);
                }
                break;

        case BSP_EVENT_WHITELIST_OFF:
                if (m_ancs_c.conn_handle == BLE_CONN_HANDLE_INVALID)
                {
                        ret = ble_advertising_restart_without_whitelist(&m_advertising);
                        if (ret != NRF_ERROR_INVALID_STATE)
                        {
                                APP_ERROR_CHECK(ret);
                        }
                }
                break;

/* ANCS_C */
#if 1
        case BSP_EVENT_KEY_0:
                ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
                APP_ERROR_CHECK(ret);
                break;

        case BSP_EVENT_KEY_1:
                if (m_notif_attr_app_id_latest.attr_id == BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER
                    && m_notif_attr_app_id_latest.attr_len != 0)
                {
                        NRF_LOG_INFO("Request for %s: ", (uint32_t)m_notif_attr_app_id_latest.p_attr_data);
                        ret = nrf_ble_ancs_c_app_attr_request(&m_ancs_c,
                                                              m_notif_attr_app_id_latest.p_attr_data,
                                                              m_notif_attr_app_id_latest.attr_len);
                        APP_ERROR_CHECK(ret);
                }
                break;

        case BSP_EVENT_KEY_2:
                if (m_notification_latest.evt_flags.positive_action == true)
                {
                        NRF_LOG_INFO("Performing Positive Action.");
                        ret = nrf_ancs_perform_notif_action(&m_ancs_c,
                                                            m_notification_latest.notif_uid,
                                                            ACTION_ID_POSITIVE);
                        APP_ERROR_CHECK(ret);
                }
                break;

        case BSP_EVENT_KEY_3:
                if (m_notification_latest.evt_flags.negative_action == true)
                {
                        NRF_LOG_INFO("Performing Negative Action.");
                        ret = nrf_ancs_perform_notif_action(&m_ancs_c,
                                                            m_notification_latest.notif_uid,
                                                            ACTION_ID_NEGATIVE);
                        APP_ERROR_CHECK(ret);
                }
                break;
#else    /* Apple media Service */
        case BSP_EVENT_KEY_1:
        {
                NRF_LOG_INFO("KEY 1 is pressed. Register for all EntityTrack Attributes (TrackArtist, TrackAlbum, TrackTitle, TrackDuration");
                uint8_t attribute_number = 4;
                uint8_t attribute_list[] = {BLE_AMS_TRACK_ATTRIBUTE_ID_ARTIST,
                                            BLE_AMS_TRACK_ATTRIBUTE_ID_ALBUM,
                                            BLE_AMS_TRACK_ATTRIBUTE_ID_TITLE,
                                            BLE_AMS_TRACK_ATTRIBUTE_ID_DURATION};
                ble_ams_c_entity_update_write(&m_ams_c, BLE_AMS_ENTITY_ID_TRACK, attribute_number, attribute_list);
                break;
        }

        case BSP_EVENT_KEY_2:
        {
                entity_attribute_id++;
                if(entity_attribute_id > 3)
                {
                        entity_attribute_id = 0;
                }

                switch(entity_attribute_id)
                {
                case 0:
                        NRF_LOG_INFO("KEY 2 is pressed. Send EntityAttribute request for Attribute TrackArtist of EntityTrack");
                        ble_ams_c_entity_attribute_write(&m_ams_c, BLE_AMS_ENTITY_ID_TRACK, BLE_AMS_TRACK_ATTRIBUTE_ID_ARTIST);
                        break;

                case 1:
                        NRF_LOG_INFO("KEY 2 is pressed. Send EntityAttribute request for Attribute TrackAlbum of EntityTrack");
                        ble_ams_c_entity_attribute_write(&m_ams_c, BLE_AMS_ENTITY_ID_TRACK, BLE_AMS_TRACK_ATTRIBUTE_ID_ALBUM);
                        break;

                case 2:
                        NRF_LOG_INFO("KEY 2 is pressed. Send EntityAttribute request for Attribute TrackTitle of EntityTrack");
                        ble_ams_c_entity_attribute_write(&m_ams_c, BLE_AMS_ENTITY_ID_TRACK, BLE_AMS_TRACK_ATTRIBUTE_ID_TITLE);
                        break;

                case 3:
                        NRF_LOG_INFO("KEY 2 is pressed. Send EntityAttribute request for Attribute TrackDuration of EntityTrack");
                        ble_ams_c_entity_attribute_write(&m_ams_c, BLE_AMS_ENTITY_ID_TRACK, BLE_AMS_TRACK_ATTRIBUTE_ID_DURATION);
                        break;

                default:
                        break;
                }
                break;
        }

        case BSP_EVENT_KEY_3:
                NRF_LOG_INFO("KEY 3 is pressed. Read Entity Attribute value");
                ble_ams_c_entity_attribute_read(&m_ams_c, 0);
                break;
#endif
        default:
                break;
        }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t *p_evt)
{
        if (p_evt->type == BLE_NUS_EVT_RX_DATA)
        {
                uint32_t err_code;

                for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
                {
                        do
                        {
                                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                                {
                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                        APP_ERROR_CHECK(err_code);
                                }
                        } while (err_code == NRF_ERROR_BUSY);
                }
                if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
                {
                        while (app_uart_put('\n') == NRF_ERROR_BUSY);
                }
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
        {
                NRF_LOG_INFO("NUS : BLE_NUS_EVT_COMM_STARTED");
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
        {
                NRF_LOG_INFO("NUS : BLE_NUS_EVT_COMM_STOPPED");
        }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
        ret_code_t err_code;
        ble_bas_init_t bas_init_obj;

        memset(&bas_init_obj, 0, sizeof(bas_init_obj));

        bas_init_obj.evt_handler          = on_bas_evt;
        bas_init_obj.support_notification = true;
        bas_init_obj.p_report_ref         = NULL;
        bas_init_obj.initial_batt_level   = 100;

        bas_init_obj.bl_rd_sec        = SEC_OPEN;
        bas_init_obj.bl_cccd_wr_sec   = SEC_OPEN;
        bas_init_obj.bl_report_rd_sec = SEC_OPEN;

        err_code = ble_bas_init(&m_bas, &bas_init_obj);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        ret_code_t ret;
        ble_ancs_c_init_t ancs_c_init;
        ble_ams_c_init_t ams_c_init;
        ble_nus_init_t nus_init;
        nrf_ble_gatts_c_init_t gatts_c_init;
        nrf_ble_qwr_init_t qwr_init = {0};

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        ret = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(ret);

        // Init the GATTS client module.
        memset(&gatts_c_init, 0, sizeof(gatts_c_init));

        gatts_c_init.evt_handler = gatts_c_evt_handler;

        ret = nrf_ble_gatts_c_init(&m_gatts_c, &gatts_c_init);
        APP_ERROR_CHECK(ret);

        // Init the Apple Notification Center Service client module.
        memset(&ancs_c_init, 0, sizeof(ancs_c_init));

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,
                                      m_attr_appid,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_app_attr_add(&m_ancs_c,
                                          BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME,
                                          m_attr_disp_name,
                                          sizeof(m_attr_disp_name));
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_TITLE,
                                      m_attr_title,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_MESSAGE,
                                      m_attr_message,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,
                                      m_attr_subtitle,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,
                                      m_attr_message_size,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_DATE,
                                      m_attr_date,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,
                                      m_attr_posaction,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                      BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,
                                      m_attr_negaction,
                                      ATTR_DATA_SIZE);
        APP_ERROR_CHECK(ret);

        ancs_c_init.evt_handler   = ancs_c_evt_handler;
        ancs_c_init.error_handler = apple_notification_error_handler;

        ret = ble_ancs_c_init(&m_ancs_c, &ancs_c_init);
        APP_ERROR_CHECK(ret);

        // Init the Apple Media Service client module.
        memset(&ams_c_init, 0, sizeof(ams_c_init));

        ams_c_init.evt_handler   = ams_c_evt_handler;
        ams_c_init.error_handler = apple_media_error_handler;

        ret = ble_ams_c_init(&m_ams_c, &ams_c_init);
        APP_ERROR_CHECK(ret);

        // Initialize NUS.
        memset(&nus_init, 0, sizeof(nus_init));

        nus_init.data_handler = nus_data_handler;

        ret = ble_nus_init(&m_nus, &nus_init);

        APP_ERROR_CHECK(ret);

        bas_init();
}


/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
        ret_code_t err_code;

        switch (p_evt->evt_type)
        {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
                // Start battery timer
                // err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
                // APP_ERROR_CHECK(err_code);
                break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
                // err_code = app_timer_stop(m_battery_timer_id);
                // APP_ERROR_CHECK(err_code);
                break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
                // No implementation needed.
                break;
        }
}
/**@brief Function for initializing the advertising functionality.
 */
static void advertising_init(void)
{
        ret_code_t ret;
        ble_advertising_init_t init;

        memset(&init, 0, sizeof(init));

        static ble_uuid_t m_adv_uuids[1]; /**< Universally unique service identifiers. */

        m_adv_uuids[0].uuid = ANCS_UUID_SERVICE;
        m_adv_uuids[0].type = m_ancs_c.service.service.uuid.type;

        // init.advdata.name_type                = BLE_ADVDATA_FULL_NAME;
        // init.advdata.include_appearance       = true;
        init.srdata.name_type                = BLE_ADVDATA_FULL_NAME;
        init.srdata.include_appearance       = true;

        init.advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
        init.advdata.uuids_complete.uuid_cnt  = 0;
        init.advdata.uuids_complete.p_uuids   = NULL;
        init.advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        init.advdata.uuids_solicited.p_uuids  = m_adv_uuids;

        init.config.ble_adv_whitelist_enabled = true;
        init.config.ble_adv_fast_enabled      = true;
        init.config.ble_adv_fast_interval     = APP_ADV_FAST_INTERVAL;
        init.config.ble_adv_fast_timeout      = APP_ADV_FAST_DURATION;
        init.config.ble_adv_slow_enabled      = true;
        init.config.ble_adv_slow_interval     = APP_ADV_SLOW_INTERVAL;
        init.config.ble_adv_slow_timeout      = APP_ADV_SLOW_DURATION;

        init.evt_handler = on_adv_evt;

        ret = ble_advertising_init(&m_advertising, &init);
        APP_ERROR_CHECK(ret);

        ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
        uint32_t ret;
        bsp_event_t startup_event;

        ret = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
        APP_ERROR_CHECK(ret);

        ret = bsp_btn_ble_init(NULL, &startup_event);
        APP_ERROR_CHECK(ret);

        *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the Event Scheduler.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the database discovery module.
 */
static void db_discovery_init(void)
{
        ret_code_t ret = ble_db_discovery_init(db_disc_handler);
        APP_ERROR_CHECK(ret);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
        ret_code_t ret = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(ret);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
                nrf_pwr_mgmt_run();
        }
}

/**@brief Function for application main entry.
 */
int main(void)
{
        bool erase_bonds;

        /* enable instruction cache */
        NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) +
                              (NVMC_ICACHECNF_CACHEPROFEN_Disabled << NVMC_ICACHECNF_CACHEPROFEN_Pos);

        // Initialize.
        log_init();
        uart_init();
        timers_init();
        buttons_leds_init(&erase_bonds);
        scheduler_init();
        power_management_init();
        ble_stack_init();
        gap_params_init();
        gatt_init();
        db_discovery_init();
        services_init();
        advertising_init();
        conn_params_init();
        peer_manager_init();

        // Start execution.
        NRF_LOG_INFO("Apple Notification Center Service client example started.");
        NRF_LOG_INFO("Apple Media Service Client example started.");
        advertising_start(erase_bonds);

        timers_start();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }

}


/**
 * @}
 */
