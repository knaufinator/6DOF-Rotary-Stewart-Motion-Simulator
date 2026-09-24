#ifdef ENABLE_BLE

#include "BleTransport.h"
#include "CobsTransport.h"
#include "helpers.h"
#include "debug_uart.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "ble_transport";

// Binary protocol constants (shared with serial path)
#define BIN_SYNC_0       0xAA
#define BIN_SYNC_1       0x55
#define BIN_PAYLOAD_SIZE 12

// BLE state
typedef enum {
    BLE_STATE_IDLE = 0,
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTED,
} ble_state_t;

static volatile ble_state_t s_ble_state = BLE_STATE_IDLE;
static void (*s_packet_callback)(const uint8_t *payload) = NULL;
static void (*s_accel_callback)(const float *data) = NULL;

// GATT handles
static uint16_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0;
static uint16_t s_service_handle = 0;
static uint16_t s_char_motion_handle = 0;   // writable: receives motion packets
static uint16_t s_char_status_handle = 0;   // notify: sends status/telemetry
static uint16_t s_char_accel_handle = 0;    // writable: receives accel packets
static bool s_notify_enabled = false;
static int s_char_add_phase = 0;            // tracks which char we're adding

// Service UUID: 42100001-0001-1000-8000-00805f9b34fb
static uint8_t s_service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x01, 0x00, 0x01, 0x00, 0x10, 0x42
};

// Advertising data
static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,  // 7.5ms
    .max_interval = 0x0010,  // 20ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(s_service_uuid),
    .p_service_uuid = s_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x20,     // 20ms
    .adv_int_max = 0x40,     // 40ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define DEVICE_NAME "Mini6DOF"
#define GATTS_APP_ID 0
#define GATTS_NUM_HANDLE 12

// ── Process received BLE data ────────────────────────────────────────

static void process_ble_write(const uint8_t *data, uint16_t len)
{
    if (s_packet_callback == NULL) return;

    // Accept 12-byte raw payload
    if (len == BIN_PAYLOAD_SIZE) {
        s_packet_callback(data);
        return;
    }

    // Accept 15-byte framed packet (sync + payload + checksum)
    if (len == BIN_PAYLOAD_SIZE + 3 &&
        data[0] == BIN_SYNC_0 && data[1] == BIN_SYNC_1) {
        uint8_t xor_check = 0;
        for (int i = 0; i < BIN_PAYLOAD_SIZE; i++)
            xor_check ^= data[2 + i];
        if (xor_check == data[14]) {
            s_packet_callback(&data[2]);
        } else {
            DEBUG_PRINTLN("BLE checksum fail");
        }
        return;
    }

    DEBUG_PRINTLN("BLE: unexpected %d bytes", len);
}

// ── Process received BLE accel data ──────────────────────────────────

static void process_ble_accel_write(const uint8_t *data, uint16_t len)
{
    if (s_accel_callback == NULL) return;

    // Accept 24-byte raw payload: 6 x float32 LE
    if (len == 24) {
        float values[6];
        memcpy(values, data, 24);
        s_accel_callback(values);
        return;
    }

    DEBUG_PRINTLN("BLE accel: unexpected %d bytes", len);
}

// ── GAP event handler ────────────────────────────────────────────────

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&s_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                s_ble_state = BLE_STATE_ADVERTISING;
                ESP_LOGI(TAG, "BLE advertising started");
            }
            break;
        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE packet length updated");
            break;
        default:
            break;
    }
}

// ── GATTS event handler ──────────────────────────────────────────────

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            s_gatts_if = gatts_if;
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&s_adv_data);

            // Create service
            esp_gatt_srvc_id_t service_id = {};
            service_id.is_primary = true;
            service_id.id.inst_id = 0;
            service_id.id.uuid.len = ESP_UUID_LEN_128;
            memcpy(service_id.id.uuid.uuid.uuid128, s_service_uuid, 16);
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;
        }

        case ESP_GATTS_CREATE_EVT: {
            s_service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(s_service_handle);
            s_char_add_phase = 0;

            // Motion RX characteristic (writable — receives binary motion packets)
            esp_bt_uuid_t motion_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = { .uuid16 = 0xFF01 },
            };
            esp_gatt_perm_t motion_perm = ESP_GATT_PERM_WRITE;
            esp_gatt_char_prop_t motion_prop = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
            esp_ble_gatts_add_char(s_service_handle, &motion_uuid, motion_perm, motion_prop, NULL, NULL);
            break;
        }

        case ESP_GATTS_ADD_CHAR_EVT: {
            if (s_char_add_phase == 0) {
                // Phase 0: Motion RX char just added
                s_char_motion_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Motion RX char handle: %d", s_char_motion_handle);
                s_char_add_phase = 1;

                // Add Status TX characteristic (notify — sends telemetry/status)
                esp_bt_uuid_t status_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = 0xFF02 },
                };
                esp_gatt_perm_t status_perm = ESP_GATT_PERM_READ;
                esp_gatt_char_prop_t status_prop = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
                esp_ble_gatts_add_char(s_service_handle, &status_uuid, status_perm, status_prop, NULL, NULL);
            } else if (s_char_add_phase == 1) {
                // Phase 1: Status TX char just added
                s_char_status_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Status TX char handle: %d", s_char_status_handle);
                s_char_add_phase = 2;

                // Add Accel RX characteristic (writable — receives 24-byte accel packets)
                esp_bt_uuid_t accel_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = 0xFF03 },
                };
                esp_gatt_perm_t accel_perm = ESP_GATT_PERM_WRITE;
                esp_gatt_char_prop_t accel_prop = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
                esp_ble_gatts_add_char(s_service_handle, &accel_uuid, accel_perm, accel_prop, NULL, NULL);
            } else {
                // Phase 2: Accel RX char just added
                s_char_accel_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Accel RX char handle: %d", s_char_accel_handle);
            }
            break;
        }

        case ESP_GATTS_CONNECT_EVT: {
            s_conn_id = param->connect.conn_id;
            s_ble_state = BLE_STATE_CONNECTED;

            // Request higher connection priority for low latency
            esp_ble_conn_update_params_t conn_params = {};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x0006;  // 7.5ms
            conn_params.min_int = 0x0006;  // 7.5ms
            conn_params.timeout = 400;     // 4s
            esp_ble_gap_update_conn_params(&conn_params);

            // Request preferred connection params for higher throughput
            esp_ble_gap_set_prefer_conn_params(param->connect.remote_bda, 6, 6, 0, 400);

            ESP_LOGI(TAG, "BLE client connected (conn_id=%d)", s_conn_id);
            cobs_send_fmt(COBS_CH_LOG, "BLE:CONNECTED");
            break;
        }

        case ESP_GATTS_DISCONNECT_EVT: {
            s_ble_state = BLE_STATE_ADVERTISING;
            s_notify_enabled = false;
            ESP_LOGI(TAG, "BLE client disconnected, restarting advertising");
            cobs_send_fmt(COBS_CH_LOG, "BLE:DISCONNECTED");
            esp_ble_gap_start_advertising(&s_adv_params);
            break;
        }

        case ESP_GATTS_WRITE_EVT: {
            if (param->write.handle == s_char_motion_handle) {
                process_ble_write(param->write.value, param->write.len);
            }
            if (param->write.handle == s_char_accel_handle) {
                process_ble_accel_write(param->write.value, param->write.len);
            }
            // Handle CCCD write for notifications
            if (param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    s_notify_enabled = true;
                    ESP_LOGI(TAG, "Notifications enabled");
                } else {
                    s_notify_enabled = false;
                }
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                    param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        default:
            break;
    }
}

// ── Public API ──────────────────────────────────────────────────────

bool ble_transport_init(void (*process_packet)(const uint8_t *payload))
{
    s_packet_callback = process_packet;

    // Release classic BT memory (we only need BLE)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(GATTS_APP_ID);

    // Set MTU high enough for 24-byte accel packets + ATT overhead
    esp_ble_gatt_set_local_mtu(128);

    ESP_LOGI(TAG, "BLE transport initialized, device name: %s (MTU=128)", DEVICE_NAME);
    return true;
}

void ble_transport_set_accel_callback(void (*process_accel)(const float *data))
{
    s_accel_callback = process_accel;
}

bool ble_transport_connected(void)
{
    return s_ble_state == BLE_STATE_CONNECTED;
}

const char* ble_transport_state_str(void)
{
    switch (s_ble_state) {
        case BLE_STATE_CONNECTED:    return "connected";
        case BLE_STATE_ADVERTISING:  return "advertising";
        case BLE_STATE_IDLE:
        default:                     return "idle";
    }
}

int ble_transport_notify(const uint8_t *data, uint16_t len)
{
    if (s_ble_state != BLE_STATE_CONNECTED || !s_notify_enabled) return -1;
    if (s_char_status_handle == 0) return -1;

    esp_err_t ret = esp_ble_gatts_send_indicate(
        s_gatts_if, s_conn_id, s_char_status_handle, len, (uint8_t*)data, false);
    return (ret == ESP_OK) ? 0 : -1;
}

#endif // ENABLE_BLE
