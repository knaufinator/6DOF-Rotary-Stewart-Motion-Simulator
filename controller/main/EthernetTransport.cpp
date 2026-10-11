// EthernetTransport.cpp — W5500 Ethernet for the 6DOF 2 (r13) controller board.
//
// Two paths share one W5500 and one SPI device:
//
//   socket 0  MACRAW  -> esp_eth + lwIP   : DHCP, ARP for lwIP, discovery, anything TCP.
//   socket 1  UDP     -> this file        : the motion/command stream, no lwIP involved.
//
// Why: the IDF W5500 driver + lwIP costs ~0.9 ms of board time per packet (about ten SPI
// register transactions and two task hops per frame), which caps the stream near 1 kHz with
// no margin. Reading the W5500's own hardware UDP socket costs ~90 us per packet
// (measured on the r13 first article, hardware/pcb/6dof2_r13/bringup/BRINGUP_LOG.md §6).
//
// Wire format on the UDP socket is exactly the USB wire format: one COBS frame per datagram,
// [channel][payload...] COBS-encoded with a trailing 0x00. Frames are decoded and handed to
// the same dispatcher the serial path uses (cobs_dispatch_frame), so CMD / DATA18 /
// DATA18_SEQ all work; RESP / LOG / TEL frames the firmware emits are mirrored back to the
// last peer that sent us a frame.
//
// SPI sharing: esp_eth is given a custom SPI driver (eth_spi_custom_driver_t) that routes its
// register access through our spi_device + mutex, so the driver and the socket-1 code never
// fight over the bus or the CS pin.
//
// Safety: nothing here touches STEP/DIR/INHIBIT/ARM. Motion frames go through the normal
// activeInputSource gate in main.cpp.

#ifdef ENABLE_ETHERNET

#include "EthernetTransport.h"
#include "CobsTransport.h"
#include "helpers.h"
#include "debug_uart.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_netif_glue.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "eth";

// ── W5500 register map (only what we use) ───────────────────────────
#define W5_BSB_COMMON   0
#define W5_BSB_REG(s)   ((s) * 4 + 1)
#define W5_BSB_TX(s)    ((s) * 4 + 2)
#define W5_BSB_RX(s)    ((s) * 4 + 3)
#define W5_MR           0x0000
#define W5_GAR          0x0001
#define W5_SUBR         0x0005
#define W5_SIPR         0x000F
#define SN_MR           0x0000
#define SN_CR           0x0001
#define SN_IR           0x0002
#define SN_SR           0x0003
#define SN_PORT         0x0004
#define SN_DHAR         0x0006
#define SN_DIPR         0x000C
#define SN_DPORT        0x0010
#define SN_RXBUF_SIZE   0x001E
#define SN_TXBUF_SIZE   0x001F
#define SN_TX_FSR       0x0020
#define SN_TX_WR        0x0024
#define SN_RX_RSR       0x0026
#define SN_RX_RD        0x0028
#define SCR_OPEN        0x01
#define SCR_CLOSE       0x10
#define SCR_SEND        0x20
#define SCR_SEND_MAC    0x21              // UDP send using Sn_DHAR, no ARP
#define SCR_RECV        0x40
#define SIR_SEND_OK     0x10
#define SIR_TIMEOUT     0x08
#define SN_SR_UDP       0x22

#define MOTION_SOCK     1                 // socket 0 belongs to esp_eth (MACRAW)
#define SOCK_BUF_KB     8                 // 8 kB RX + 8 kB TX each for socket 0 and 1
#define POLL_PERIOD_US  250               // socket-1 poll cadence when idle
#define PEER_TIMEOUT_US (5 * 1000000LL)   // stop mirroring replies to a silent peer after 5 s
#ifndef ETH_LWIP_ONLY_FOR_DHCP
#define ETH_LWIP_ONLY_FOR_DHCP 1          // 1: esp_eth/lwIP run only until the DHCP lease, then stop
#endif
#define W5_PHYCFGR      0x002E

// ── shared SPI device ───────────────────────────────────────────────
static spi_device_handle_t s_spi = NULL;
static SemaphoreHandle_t   s_spi_mutex = NULL;
static volatile uint32_t s_spi_calls = 0, s_eth_spi_calls = 0, s_spi_wait_max_us = 0, s_spi_xfer_max_us = 0, s_spi_wait_sum_us = 0, s_spi_slow = 0;

static esp_err_t spi_xfer(uint16_t addr, uint8_t ctrl, bool write, void *buf, size_t len)
{
    spi_transaction_t t = {};
    t.cmd       = addr;
    t.addr      = ctrl;
    t.length    = len * 8;
    t.rxlength  = write ? 0 : len * 8;
    t.tx_buffer = write ? buf : NULL;
    t.rx_buffer = write ? NULL : buf;
    int64_t t0 = esp_timer_get_time();
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    int64_t t1 = esp_timer_get_time();
    esp_err_t r = spi_device_polling_transmit(s_spi, &t);
    int64_t t2 = esp_timer_get_time();
    xSemaphoreGive(s_spi_mutex);
    s_spi_calls++;
    uint32_t w = (uint32_t)(t1 - t0), x = (uint32_t)(t2 - t1);
    s_spi_wait_sum_us += w;
    if (w > s_spi_wait_max_us) s_spi_wait_max_us = w;
    if (x > s_spi_xfer_max_us) s_spi_xfer_max_us = x;
    if (x > 200) s_spi_slow++;
    return r;
}

// esp_eth custom SPI driver hooks: cmd = 16-bit address, addr = control byte (BSB|RWB|OM).
static void *eth_spi_init(const void *cfg) { (void)cfg; return (void *)1; }
static esp_err_t eth_spi_deinit(void *ctx) { (void)ctx; return ESP_OK; }
static esp_err_t eth_spi_read(void *ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t len)
{ (void)ctx; s_eth_spi_calls++; return spi_xfer((uint16_t)cmd, (uint8_t)addr, false, data, len); }
static esp_err_t eth_spi_write(void *ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t len)
{ (void)ctx; s_eth_spi_calls++; return spi_xfer((uint16_t)cmd, (uint8_t)addr, true, (void *)data, len); }

// register helpers for our own socket work
static inline uint8_t  r8 (uint16_t a, uint8_t bsb) { uint8_t v = 0; spi_xfer(a, (uint8_t)(bsb << 3), false, &v, 1); return v; }
static inline void     w8 (uint16_t a, uint8_t bsb, uint8_t v) { spi_xfer(a, (uint8_t)((bsb << 3) | 0x04), true, &v, 1); }
static inline uint16_t r16(uint16_t a, uint8_t bsb) { uint8_t v[2] = {0, 0}; spi_xfer(a, (uint8_t)(bsb << 3), false, v, 2); return (uint16_t)((v[0] << 8) | v[1]); }
static inline void     w16(uint16_t a, uint8_t bsb, uint16_t v) { uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v}; spi_xfer(a, (uint8_t)((bsb << 3) | 0x04), true, b, 2); }
static inline void     rbuf(uint16_t a, uint8_t bsb, void *d, size_t n) { spi_xfer(a, (uint8_t)(bsb << 3), false, d, n); }
static inline void     wbuf(uint16_t a, uint8_t bsb, const void *d, size_t n) { spi_xfer(a, (uint8_t)((bsb << 3) | 0x04), true, (void *)d, n); }

static bool sock_cmd(int s, uint8_t c)
{
    w8(SN_CR, W5_BSB_REG(s), c);
    for (int i = 0; i < 1000; i++) { if (r8(SN_CR, W5_BSB_REG(s)) == 0) return true; }
    return false;
}

// ── state ───────────────────────────────────────────────────────────
#define ETH_LINK_BIT BIT0
#define ETH_IP_BIT   BIT1
static EventGroupHandle_t s_events = NULL;
static esp_eth_handle_t   s_eth = NULL;
static esp_netif_ip_info_t s_ip = {};
static volatile bool      s_sock_open = false;
static volatile bool      s_lwip_stopped = false;
static bool               s_send_pending = false;   // a hardware SEND was issued and not yet confirmed
// Peer MAC hint (PEERMAC command): hosts behind Wi-Fi APs often never answer the W5500's ARP.
static uint8_t  s_peer_mac[6] = {0};
static uint8_t  s_peer_mac_ip[4] = {0};
static bool     s_peer_mac_valid = false;
static SemaphoreHandle_t  s_sock_mutex = NULL;   // serialises socket-1 TX against the RX task
static TaskHandle_t       s_rx_task = NULL;
static esp_timer_handle_t s_poll_timer = NULL;

// last peer (for mirrored RESP/LOG/TEL)
static uint8_t  s_peer_ip[4] = {0};
static uint16_t s_peer_port = 0;
static volatile int64_t s_peer_last_us = 0;

// stats
static volatile uint32_t s_rx_frames = 0, s_rx_bad = 0, s_tx_frames = 0, s_tx_fail = 0, s_sock_reopens = 0;
static volatile uint32_t s_polls = 0, s_tx_max_us = 0, s_rx_max_us = 0, s_tx_timeout = 0, s_tx_dropped = 0;
static volatile int64_t  s_rx_last_us = 0;

// ── socket 1 bring-up (needs the lease: W5500 does ARP/IP itself in UDP mode) ──
static void motion_socket_open(void)
{
    xSemaphoreTake(s_sock_mutex, portMAX_DELAY);
    // give the hardware stack our identity (does not affect MACRAW socket 0)
    uint8_t ip[4]  = { (uint8_t)(s_ip.ip.addr), (uint8_t)(s_ip.ip.addr >> 8), (uint8_t)(s_ip.ip.addr >> 16), (uint8_t)(s_ip.ip.addr >> 24) };
    uint8_t gw[4]  = { (uint8_t)(s_ip.gw.addr), (uint8_t)(s_ip.gw.addr >> 8), (uint8_t)(s_ip.gw.addr >> 16), (uint8_t)(s_ip.gw.addr >> 24) };
    uint8_t sn[4]  = { (uint8_t)(s_ip.netmask.addr), (uint8_t)(s_ip.netmask.addr >> 8), (uint8_t)(s_ip.netmask.addr >> 16), (uint8_t)(s_ip.netmask.addr >> 24) };
    wbuf(W5_SIPR, W5_BSB_COMMON, ip, 4);
    wbuf(W5_GAR,  W5_BSB_COMMON, gw, 4);
    wbuf(W5_SUBR, W5_BSB_COMMON, sn, 4);
    // socket 1: UDP on the motion port (buffer sizes were split before esp_eth opened socket 0)
    sock_cmd(MOTION_SOCK, SCR_CLOSE);
    w8(SN_MR, W5_BSB_REG(MOTION_SOCK), 0x02);
    w16(SN_PORT, W5_BSB_REG(MOTION_SOCK), ETH_UDP_PORT);
    sock_cmd(MOTION_SOCK, SCR_OPEN);
    uint8_t sr = r8(SN_SR, W5_BSB_REG(MOTION_SOCK));
    s_sock_open = (sr == SN_SR_UDP);
    s_send_pending = false;
    s_sock_reopens++;
    xSemaphoreGive(s_sock_mutex);
    ESP_LOGI(TAG, "motion socket %d: UDP port %d %s (ip %d.%d.%d.%d)", MOTION_SOCK, ETH_UDP_PORT,
             s_sock_open ? "open" : "FAILED", ip[0], ip[1], ip[2], ip[3]);
}

// ── events ──────────────────────────────────────────────────────────
static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    switch (id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "link up");
            xEventGroupSetBits(s_events, ETH_LINK_BIT);
            // if we already hold a lease the socket config survives a link flap; re-open anyway
            if (xEventGroupGetBits(s_events) & ETH_IP_BIT) motion_socket_open();
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            if (s_lwip_stopped) break;   // our own esp_eth_stop after the lease, not a cable event
            ESP_LOGW(TAG, "link down");
            xEventGroupClearBits(s_events, ETH_LINK_BIT);
            s_sock_open = false;
            break;
        default: break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base;
    if (id == IP_EVENT_ETH_GOT_IP) {
        s_ip = ((ip_event_got_ip_t *)data)->ip_info;
        ESP_LOGI(TAG, "ip " IPSTR, IP2STR(&s_ip.ip));
        xEventGroupSetBits(s_events, ETH_IP_BIT);
        motion_socket_open();
#if ETH_LWIP_ONLY_FOR_DHCP
        // Hand the chip to its own IP stack: with socket 0 in MACRAW mode and SIPR set, the
        // W5500 splits ARP/ICMP between lwIP and its hardware stack and every hardware SEND
        // stalled for tens of ms (measured). lwIP's job (DHCP) is done; stop the driver so
        // socket 0 closes. Link loss is watched in the RX task via PHYCFGR and restarts it.
        s_lwip_stopped = true;            // before stop: its DISCONNECTED event must be ignored
        esp_eth_stop(s_eth);
        w8(W5_MR, W5_BSB_COMMON, 0x00);   // clear PB: let the W5500 answer pings itself
        ESP_LOGI(TAG, "lwIP stopped after lease; hardware UDP stack owns " IPSTR, IP2STR(&s_ip.ip));
#endif
    }
}

// ── TX: mirror a COBS-encoded frame to the last peer ────────────────
// A hardware SEND needs the peer's MAC; if the W5500 has to ARP a host that answers slowly
// (a Wi-Fi laptop dozing through broadcasts) the SEND can sit for hundreds of ms. The RX path
// must never wait on that, so a SEND is issued and left pending; the next call checks whether it
// completed and drops its own frame if not. Motion frames keep flowing either way.
static void eth_tx_sink(const uint8_t *enc, int len)
{
    if (!s_sock_open || s_peer_port == 0) return;
    if (esp_timer_get_time() - s_peer_last_us > PEER_TIMEOUT_US) return;
    if (len <= 0 || len > 1400) return;
    int64_t t_tx0 = esp_timer_get_time();
    if (xSemaphoreTake(s_sock_mutex, pdMS_TO_TICKS(2)) != pdTRUE) { s_tx_fail++; return; }
    if (s_send_pending) {
        uint8_t ir = r8(SN_IR, W5_BSB_REG(MOTION_SOCK));
        if (ir & (SIR_SEND_OK | SIR_TIMEOUT)) {
            w8(SN_IR, W5_BSB_REG(MOTION_SOCK), ir);
            if (ir & SIR_SEND_OK) s_tx_frames++; else { s_tx_fail++; s_tx_timeout++; }
            s_send_pending = false;
        } else {
            xSemaphoreGive(s_sock_mutex); s_tx_fail++; s_tx_dropped++; return;   // previous SEND still waiting (ARP)
        }
    }
    uint16_t fsr = r16(SN_TX_FSR, W5_BSB_REG(MOTION_SOCK));
    if (fsr < len) { xSemaphoreGive(s_sock_mutex); s_tx_fail++; return; }
    wbuf(SN_DIPR, W5_BSB_REG(MOTION_SOCK), s_peer_ip, 4);
    w16(SN_DPORT, W5_BSB_REG(MOTION_SOCK), s_peer_port);
    bool use_mac = s_peer_mac_valid && memcmp(s_peer_mac_ip, s_peer_ip, 4) == 0;
    if (use_mac) wbuf(SN_DHAR, W5_BSB_REG(MOTION_SOCK), s_peer_mac, 6);
    uint16_t wr = r16(SN_TX_WR, W5_BSB_REG(MOTION_SOCK));
    wbuf(wr, W5_BSB_TX(MOTION_SOCK), enc, len);
    w16(SN_TX_WR, W5_BSB_REG(MOTION_SOCK), (uint16_t)(wr + len));
    sock_cmd(MOTION_SOCK, use_mac ? SCR_SEND_MAC : SCR_SEND);
    // a LAN peer with a cached MAC completes in tens of us: give it ~20 polls, then move on
    uint8_t ir = 0;
    for (int i = 0; i < 20; i++) {
        ir = r8(SN_IR, W5_BSB_REG(MOTION_SOCK));
        if (ir & (SIR_SEND_OK | SIR_TIMEOUT)) break;
    }
    if (ir & (SIR_SEND_OK | SIR_TIMEOUT)) {
        w8(SN_IR, W5_BSB_REG(MOTION_SOCK), ir);
        if (ir & SIR_SEND_OK) s_tx_frames++; else { s_tx_fail++; s_tx_timeout++; }
    } else {
        s_send_pending = true;
    }
    xSemaphoreGive(s_sock_mutex);
    { uint32_t d = (uint32_t)(esp_timer_get_time() - t_tx0); if (d > s_tx_max_us) s_tx_max_us = d; }
}

// ── RX task: poll socket 1, decode COBS, dispatch ───────────────────
static void poll_timer_cb(void *arg) { (void)arg; if (s_rx_task) xTaskNotifyGive(s_rx_task); }

static void eth_rx_task(void *arg)
{
    (void)arg;
    static uint8_t enc[1500], dec[COBS_MAX_FRAME];
    int64_t next_link_check = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        if (s_lwip_stopped && esp_timer_get_time() > next_link_check) {
            next_link_check = esp_timer_get_time() + 1000000;
            if ((r8(W5_PHYCFGR, W5_BSB_COMMON) & 0x01) == 0) {
                ESP_LOGW(TAG, "link lost: closing motion socket, restarting lwIP for a new lease");
                s_sock_open = false;
                s_lwip_stopped = false;
                xEventGroupClearBits(s_events, ETH_IP_BIT | ETH_LINK_BIT);
                esp_eth_start(s_eth);
            }
        }
        if (!s_sock_open) continue;
        s_polls++;
        for (;;) {
            int64_t t_rx0 = esp_timer_get_time();
            if (xSemaphoreTake(s_sock_mutex, pdMS_TO_TICKS(5)) != pdTRUE) break;
            uint16_t rsr = r16(SN_RX_RSR, W5_BSB_REG(MOTION_SOCK));
            if (rsr < 8) { xSemaphoreGive(s_sock_mutex); break; }
            uint16_t rd = r16(SN_RX_RD, W5_BSB_REG(MOTION_SOCK));
            uint8_t hdr[8];
            rbuf(rd, W5_BSB_RX(MOTION_SOCK), hdr, 8);
            uint16_t len = (uint16_t)((hdr[6] << 8) | hdr[7]);
            uint16_t take = len > sizeof(enc) ? sizeof(enc) : len;
            rbuf((uint16_t)(rd + 8), W5_BSB_RX(MOTION_SOCK), enc, take);
            w16(SN_RX_RD, W5_BSB_REG(MOTION_SOCK), (uint16_t)(rd + 8 + len));
            sock_cmd(MOTION_SOCK, SCR_RECV);
            xSemaphoreGive(s_sock_mutex);
            { uint32_t d = (uint32_t)(esp_timer_get_time() - t_rx0); if (d > s_rx_max_us) s_rx_max_us = d; }

            memcpy(s_peer_ip, hdr, 4);
            s_peer_port = (uint16_t)((hdr[4] << 8) | hdr[5]);
            s_peer_last_us = s_rx_last_us = esp_timer_get_time();

            // one datagram may carry several delimited frames (the app sends one per datagram)
            int start = 0;
            for (int i = 0; i < take; i++) {
                if (enc[i] != 0x00) continue;
                int n = i - start;
                if (n > 0 && n <= COBS_MAX_FRAME) {
                    int dl = cobs_decode(enc + start, n, dec);
                    if (dl > 0) { s_rx_frames++; cobs_dispatch_frame(dec, dl, COBS_SRC_ETHERNET); }
                    else s_rx_bad++;
                }
                start = i + 1;
            }
            if (start == 0 && take > 0) {
                // no delimiter at all: treat the whole datagram as one frame
                int dl = cobs_decode(enc, take, dec);
                if (dl > 0) { s_rx_frames++; cobs_dispatch_frame(dec, dl, COBS_SRC_ETHERNET); }
                else s_rx_bad++;
            }
        }
    }
}

// ── public API ──────────────────────────────────────────────────────
bool ethernet_transport_init(void)
{
    s_events = xEventGroupCreate();
    s_spi_mutex = xSemaphoreCreateMutex();
    s_sock_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t r = esp_event_loop_create_default();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "event loop %s", esp_err_to_name(r)); return false; }

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = ETH_SPI_MOSI;
    buscfg.miso_io_num = ETH_SPI_MISO;
    buscfg.sclk_io_num = ETH_SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;
    r = spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "spi bus %s", esp_err_to_name(r)); return false; }

    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits   = 16;          // W5500 address phase
    devcfg.address_bits   = 8;           // W5500 control phase
    devcfg.mode           = 0;
    devcfg.clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000;
    devcfg.spics_io_num   = ETH_SPI_CS;
    devcfg.queue_size     = 4;
    r = spi_bus_add_device(ETH_SPI_HOST, &devcfg, &s_spi);
    if (r != ESP_OK) { ESP_LOGE(TAG, "spi device %s", esp_err_to_name(r)); return false; }
    // Hold the bus for good: with the device acquired, spi_device_polling_transmit takes the
    // lock-free fast path (~15 us per transaction); without it each transaction went through
    // the bus-lock machinery and measured up to 1.2 ms on this build. Nothing else shares the
    // bus, and s_spi_mutex serialises our two users (esp_eth's custom driver and socket 1).
    spi_device_acquire_bus(s_spi, portMAX_DELAY);

    // esp_eth on socket 0 through our SPI device
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = ETH_SPI_INT;
    w5500_config.custom_spi_driver.config = NULL;
    w5500_config.custom_spi_driver.init   = eth_spi_init;
    w5500_config.custom_spi_driver.deinit = eth_spi_deinit;
    w5500_config.custom_spi_driver.read   = eth_spi_read;
    w5500_config.custom_spi_driver.write  = eth_spi_write;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = -1;      // RSTn is an RC on the board
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    gpio_install_isr_service(0);
    r = esp_eth_driver_install(&eth_config, &s_eth);
    if (r != ESP_OK) { ESP_LOGW(TAG, "W5500 driver install failed (%s)", esp_err_to_name(r)); return false; }

    // the W5500 has no MAC of its own: use the chip's Ethernet MAC (F08)
    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_ETH);
    esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "mac %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    // esp_eth's init gave all 16 kB RX/TX to socket 0 and has not opened it yet (that happens
    // in esp_eth_start). Buffer sizes must be set while sockets are closed: split 8/8 now.
    w8(SN_RXBUF_SIZE, W5_BSB_REG(0), SOCK_BUF_KB); w8(SN_TXBUF_SIZE, W5_BSB_REG(0), SOCK_BUF_KB);
    w8(SN_RXBUF_SIZE, W5_BSB_REG(MOTION_SOCK), SOCK_BUF_KB); w8(SN_TXBUF_SIZE, W5_BSB_REG(MOTION_SOCK), SOCK_BUF_KB);
    for (int s = 2; s < 8; s++) { w8(SN_RXBUF_SIZE, W5_BSB_REG(s), 0); w8(SN_TXBUF_SIZE, W5_BSB_REG(s), 0); }

    r = esp_netif_attach(eth_netif, esp_eth_new_netif_glue(s_eth));
    if (r != ESP_OK) { ESP_LOGW(TAG, "netif attach %s", esp_err_to_name(r)); return false; }
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler, NULL);
    r = esp_eth_start(s_eth);
    if (r != ESP_OK) { ESP_LOGW(TAG, "eth start %s", esp_err_to_name(r)); return false; }

    // RX task on core 1 (core 0 carries USB/lwIP/esp_eth), above the step engine's tasks
    xTaskCreatePinnedToCore(eth_rx_task, "EthRx", 6144, NULL, 6, &s_rx_task, 0);   // experiment: core 0 (core 1 carries the step-engine ISR)
    esp_timer_create_args_t ta = {};
    ta.callback = poll_timer_cb;
    ta.name = "eth_poll";
    esp_timer_create(&ta, &s_poll_timer);
    esp_timer_start_periodic(s_poll_timer, POLL_PERIOD_US);

    cobs_set_tx_sink(eth_tx_sink);
    ESP_LOGI(TAG, "W5500 up: SPI %d MHz, MACRAW on socket 0 for lwIP, UDP %d on socket %d",
             ETH_SPI_CLOCK_MHZ, ETH_UDP_PORT, MOTION_SOCK);
    return true;
}

void ethernet_transport_set_peer_mac(const uint8_t mac[6])
{
    memcpy(s_peer_mac, mac, 6);
    memcpy(s_peer_mac_ip, s_peer_ip, 4);     // valid for whoever is the current peer
    s_peer_mac_valid = true;
    // a SEND stuck in ARP for this peer is now pointless: forget it
    if (xSemaphoreTake(s_sock_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint8_t ir = r8(SN_IR, W5_BSB_REG(MOTION_SOCK));
        if (ir) w8(SN_IR, W5_BSB_REG(MOTION_SOCK), ir);
        s_send_pending = false;
        xSemaphoreGive(s_sock_mutex);
    }
    ESP_LOGI(TAG, "peer mac %02x:%02x:%02x:%02x:%02x:%02x for %d.%d.%d.%d", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             s_peer_mac_ip[0], s_peer_mac_ip[1], s_peer_mac_ip[2], s_peer_mac_ip[3]);
}

bool ethernet_transport_connected(void)
{
    return s_events && (xEventGroupGetBits(s_events) & ETH_IP_BIT) && s_sock_open;
}

void ethernet_transport_stats(eth_stats_t *out)
{
    if (!out) return;
    out->link = s_events ? (xEventGroupGetBits(s_events) & ETH_LINK_BIT) != 0 : false;
    out->sock_open = s_sock_open;
    out->ip = s_ip.ip.addr;
    out->rx_frames = s_rx_frames;
    out->rx_bad = s_rx_bad;
    out->tx_frames = s_tx_frames;
    out->tx_fail = s_tx_fail;
    out->sock_reopens = s_sock_reopens;
    out->rx_age_ms = s_rx_last_us ? (uint32_t)((esp_timer_get_time() - s_rx_last_us) / 1000) : 0xFFFFFFFFu;
    memcpy(out->peer_ip, s_peer_ip, 4);
    out->peer_port = s_peer_port;
    out->polls = s_polls;
    out->rx_max_us = s_rx_max_us;
    out->tx_max_us = s_tx_max_us;
    out->spi_calls = s_spi_calls;
    out->eth_spi_calls = s_eth_spi_calls;
    out->spi_wait_max_us = s_spi_wait_max_us;
    out->spi_xfer_max_us = s_spi_xfer_max_us;
    out->spi_wait_sum_us = s_spi_wait_sum_us;
    out->spi_slow = s_spi_slow;
    out->tx_timeout = s_tx_timeout;
    out->tx_dropped = s_tx_dropped;
    rbuf(SN_DHAR, W5_BSB_REG(MOTION_SOCK), out->dhar, 6);
    out->sn_ir = r8(SN_IR, W5_BSB_REG(MOTION_SOCK));
    out->send_pending = s_send_pending;
    out->phycfgr = r8(W5_PHYCFGR, W5_BSB_COMMON);
    out->lwip_stopped = s_lwip_stopped;
    s_rx_max_us = s_tx_max_us = s_spi_wait_max_us = s_spi_xfer_max_us = 0;   // max since last query
}

#endif // ENABLE_ETHERNET
