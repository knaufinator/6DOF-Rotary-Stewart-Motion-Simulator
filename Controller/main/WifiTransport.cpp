#ifdef ENABLE_WIFI

#include "WifiTransport.h"
#include "helpers.h"
#include "debug_uart.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"

static const char *TAG = "wifi_transport";

// NVS namespace and keys
#define WIFI_NVS_NAMESPACE "wifi_cfg"
#define WIFI_NVS_KEY_SSID  "ssid"
#define WIFI_NVS_KEY_PASS  "pass"

// Event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group = NULL;

// Callback for received packets
static void (*s_packet_callback)(const uint8_t *payload) = NULL;

// Binary protocol constants (shared with serial path)
#define BIN_SYNC_0       0xAA
#define BIN_SYNC_1       0x55
#define BIN_PAYLOAD_SIZE 12

// WiFi state
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED,
} wifi_state_t;

static volatile wifi_state_t s_wifi_state = WIFI_STATE_DISCONNECTED;
static char s_ssid[33] = {0};
static char s_pass[65] = {0};
static char s_ip_str[16] = "0.0.0.0";
static int s_retry_count = 0;
#define WIFI_MAX_RETRY 5

// ── WiFi event handlers ─────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi STA started, connecting...");
                s_wifi_state = WIFI_STATE_CONNECTING;
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED: {
                s_retry_count++;
                if (s_retry_count < WIFI_MAX_RETRY) {
                    ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d", s_retry_count, WIFI_MAX_RETRY);
                    s_wifi_state = WIFI_STATE_CONNECTING;
                    vTaskDelay(pdMS_TO_TICKS(1000));  // backoff before retry
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRY);
                    s_wifi_state = WIFI_STATE_FAILED;
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                snprintf(s_ip_str, sizeof(s_ip_str), "0.0.0.0");
                break;
            }
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WiFi associated with AP: %s", s_ssid);
                s_retry_count = 0;
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", s_ip_str);
        s_wifi_state = WIFI_STATE_CONNECTED;
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
}

// ── UDP listener task ───────────────────────────────────────────────

static void wifi_udp_listener_task(void *pvParameters)
{
    uint8_t rx_buf[64];

    // Wait for IP
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(ETH_UDP_PORT),  // reuse same port as Ethernet
        .sin_addr = { .s_addr = htonl(INADDR_ANY) },
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "WiFi UDP listening on port %d", ETH_UDP_PORT);

    // Report ready over serial
    printf("WIFI:UDP_READY port=%d ip=%s\r\n", ETH_UDP_PORT, s_ip_str);
    fflush(stdout);

    for (;;) {
        // If WiFi drops, wait for reconnect
        if (s_wifi_state != WIFI_STATE_CONNECTED) {
            xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);
            // Rebind might be needed after reconnect — for now just continue
            ESP_LOGI(TAG, "WiFi reconnected, resuming UDP");
        }

        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0, NULL, NULL);
        if (len < 0) {
            if (errno == ENOTCONN || errno == EBADF) {
                // Socket error from WiFi drop — wait and retry
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            ESP_LOGE(TAG, "recvfrom error: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (s_packet_callback == NULL) continue;

        // Accept 12-byte raw payload (UDP provides framing + CRC)
        if (len == BIN_PAYLOAD_SIZE) {
            s_packet_callback(rx_buf);
            continue;
        }

        // Also accept 15-byte framed packet (sync + payload + checksum)
        if (len == BIN_PAYLOAD_SIZE + 3 &&
            rx_buf[0] == BIN_SYNC_0 && rx_buf[1] == BIN_SYNC_1) {
            uint8_t xor_check = 0;
            for (int i = 0; i < BIN_PAYLOAD_SIZE; i++)
                xor_check ^= rx_buf[2 + i];
            if (xor_check == rx_buf[14]) {
                s_packet_callback(&rx_buf[2]);
            } else {
                DEBUG_PRINTF("WiFi UDP checksum fail\n");
            }
            continue;
        }

        DEBUG_PRINTF("WiFi UDP: unexpected %d bytes\n", len);
    }
}

// ── WiFi status report task (periodic serial output) ─────────────────

static void wifi_status_task(void *pvParameters)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // report every 5 seconds
        if (s_wifi_state == WIFI_STATE_CONNECTED) {
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                printf("WIFI:STATUS connected,ip=%s,rssi=%d,ssid=%s\r\n",
                       s_ip_str, ap.rssi, s_ssid);
                fflush(stdout);
            }
        }
    }
}

// ── Public API ──────────────────────────────────────────────────────

bool wifi_transport_init(void (*process_packet)(const uint8_t *payload))
{
    s_packet_callback = process_packet;
    s_wifi_event_group = xEventGroupCreate();

    // Initialize TCP/IP and event loop (may already be done by Ethernet)
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(err));
        return false;
    }

    // Create default WiFi STA netif
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Set WiFi mode to STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Try to load saved credentials and auto-connect
    if (wifi_transport_load_credentials() == 0 && strlen(s_ssid) > 0) {
        ESP_LOGI(TAG, "Auto-connecting to saved SSID: %s", s_ssid);
        wifi_config_t wifi_config = {};
        memcpy(wifi_config.sta.ssid, s_ssid, strnlen(s_ssid, sizeof(wifi_config.sta.ssid) - 1));
        memcpy(wifi_config.sta.password, s_pass, strnlen(s_pass, sizeof(wifi_config.sta.password) - 1));
        wifi_config.sta.threshold.authmode = strlen(s_pass) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    } else {
        ESP_LOGI(TAG, "No saved WiFi credentials — waiting for WIFI:SSID/WIFI:PASS commands");
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    // Start UDP listener task (waits for connection internally)
    xTaskCreatePinnedToCore(
        wifi_udp_listener_task,
        "WiFiUDPListener",
        4096,
        NULL,
        2,      // Same priority as InterfaceMonitorTask
        NULL,
        0);     // Core 0

    // Start periodic status reporter
    xTaskCreatePinnedToCore(
        wifi_status_task,
        "WiFiStatus",
        2048,
        NULL,
        1,
        NULL,
        0);

    ESP_LOGI(TAG, "WiFi transport initialized");
    return true;
}

bool wifi_transport_connected(void)
{
    if (s_wifi_event_group == NULL) return false;
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}

const char* wifi_transport_get_ip(void)
{
    return s_ip_str;
}

int wifi_transport_get_rssi(void)
{
    if (s_wifi_state != WIFI_STATE_CONNECTED) return 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

const char* wifi_transport_get_ssid(void)
{
    return s_ssid;
}

void wifi_transport_set_credentials(const char *ssid, const char *password)
{
    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1);
    s_ssid[sizeof(s_ssid) - 1] = '\0';
    if (password) {
        strncpy(s_pass, password, sizeof(s_pass) - 1);
        s_pass[sizeof(s_pass) - 1] = '\0';
    } else {
        s_pass[0] = '\0';
    }
}

void wifi_transport_connect(void)
{
    if (strlen(s_ssid) == 0) {
        ESP_LOGW(TAG, "Cannot connect — no SSID set");
        return;
    }

    s_retry_count = 0;
    s_wifi_state = WIFI_STATE_CONNECTING;

    // Stop WiFi if already running, reconfigure, restart
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));

    wifi_config_t wifi_config = {};
    memcpy(wifi_config.sta.ssid, s_ssid, strnlen(s_ssid, sizeof(wifi_config.sta.ssid) - 1));
    memcpy(wifi_config.sta.password, s_pass, strnlen(s_pass, sizeof(wifi_config.sta.password) - 1));
    wifi_config.sta.threshold.authmode = strlen(s_pass) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to SSID: %s", s_ssid);
}

void wifi_transport_disconnect(void)
{
    esp_wifi_disconnect();
    s_wifi_state = WIFI_STATE_DISCONNECTED;
    snprintf(s_ip_str, sizeof(s_ip_str), "0.0.0.0");
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "WiFi disconnected");
}

int wifi_transport_save_credentials(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return -1;

    err = nvs_set_str(nvs, WIFI_NVS_KEY_SSID, s_ssid);
    if (err != ESP_OK) { nvs_close(nvs); return -1; }

    err = nvs_set_str(nvs, WIFI_NVS_KEY_PASS, s_pass);
    if (err != ESP_OK) { nvs_close(nvs); return -1; }

    err = nvs_commit(nvs);
    nvs_close(nvs);
    return (err == ESP_OK) ? 0 : -1;
}

int wifi_transport_load_credentials(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) return -1;

    size_t ssid_len = sizeof(s_ssid);
    err = nvs_get_str(nvs, WIFI_NVS_KEY_SSID, s_ssid, &ssid_len);
    if (err != ESP_OK) { nvs_close(nvs); return -1; }

    size_t pass_len = sizeof(s_pass);
    err = nvs_get_str(nvs, WIFI_NVS_KEY_PASS, s_pass, &pass_len);
    if (err != ESP_OK) {
        // Password might not exist for open networks
        s_pass[0] = '\0';
    }

    nvs_close(nvs);
    return 0;
}

const char* wifi_transport_state_str(void)
{
    switch (s_wifi_state) {
        case WIFI_STATE_CONNECTED:    return "connected";
        case WIFI_STATE_CONNECTING:   return "connecting";
        case WIFI_STATE_FAILED:       return "failed";
        case WIFI_STATE_DISCONNECTED:
        default:                      return "disconnected";
    }
}

#endif // ENABLE_WIFI
