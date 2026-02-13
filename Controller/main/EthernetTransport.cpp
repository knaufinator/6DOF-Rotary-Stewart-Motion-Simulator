#ifdef ENABLE_ETHERNET

#include "EthernetTransport.h"
#include "helpers.h"
#include "debug_uart.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "lwip/sockets.h"

static const char *TAG = "eth_transport";

// Event bits
#define ETH_CONNECTED_BIT BIT0
static EventGroupHandle_t s_eth_event_group = NULL;

// Callback for received packets
static void (*s_packet_callback)(const uint8_t *payload) = NULL;

// Binary protocol constants (shared with serial path)
#define BIN_SYNC_0       0xAA
#define BIN_SYNC_1       0x55
#define BIN_PAYLOAD_SIZE 12

// ── Ethernet event handlers ─────────────────────────────────────────

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Ethernet link up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Ethernet link down");
            xEventGroupClearBits(s_eth_event_group, ETH_CONNECTED_BIT);
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet stopped");
            break;
        default:
            break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
    }
}

// ── UDP listener task ───────────────────────────────────────────────

static void udp_listener_task(void *pvParameters)
{
    uint8_t rx_buf[64];

    // Wait for IP
    xEventGroupWaitBits(s_eth_event_group, ETH_CONNECTED_BIT,
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
        .sin_port = htons(ETH_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP listening on port %d", ETH_UDP_PORT);

    for (;;) {
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0, NULL, NULL);
        if (len < 0) {
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
                DEBUG_PRINTF("UDP checksum fail\n");
            }
            continue;
        }

        DEBUG_PRINTF("UDP: unexpected %d bytes\n", len);
    }
}

// ── Public API ──────────────────────────────────────────────────────

bool ethernet_transport_init(void (*process_packet)(const uint8_t *payload))
{
    s_packet_callback = process_packet;
    s_eth_event_group = xEventGroupCreate();

    // ── Initialize TCP/IP and event loop ──
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    // ── SPI bus ──
    spi_bus_config_t buscfg = {
        .mosi_io_num   = ETH_SPI_MOSI,
        .miso_io_num   = ETH_SPI_MISO,
        .sclk_io_num   = ETH_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // ── SPI device for W5500 ──
    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num   = ETH_SPI_CS,
        .queue_size     = 20,
    };
    spi_device_handle_t spi_handle = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(ETH_SPI_HOST, &devcfg, &spi_handle));

    // ── W5500 MAC + PHY ──
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
    w5500_config.int_gpio_num = ETH_SPI_INT;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = -1;  // No HW reset pin
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // Attach to netif
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &ip_event_handler, NULL));

    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "W5500 Ethernet initialized (SPI CS=%d, INT=%d)",
             ETH_SPI_CS, ETH_SPI_INT);

    // Start UDP listener task
    xTaskCreatePinnedToCore(
        udp_listener_task,
        "UDPListener",
        4096,
        NULL,
        2,      // Same priority as InterfaceMonitorTask
        NULL,
        0);     // Core 0

    return true;
}

bool ethernet_transport_connected(void)
{
    if (s_eth_event_group == NULL) return false;
    return (xEventGroupGetBits(s_eth_event_group) & ETH_CONNECTED_BIT) != 0;
}

#endif // ENABLE_ETHERNET
