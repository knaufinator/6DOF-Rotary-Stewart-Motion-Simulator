/*
 * 6DOF 2 r13 board-bringup probe firmware.
 *
 * Purpose: exercise every subsystem of the r13 controller PCB that is reachable from
 * the ESP32-S3 over USB, one command at a time, with deterministic text output a host
 * script can parse. This is NOT the motion controller and never will be.
 *
 * Pin contract (r13 RC4 netlist, hardware/pcb/6dof2_r13/r13-rc4-2026-09-23/source/schematic_netlist.json):
 *   STEP  M0..M5 : GPIO 4 5 6 7 8 9      -> 33R series -> AM26LS31 input, 1k pull-down (R46-R57)
 *   DIR   M0..M5 : GPIO 10 11 12 13 14 17
 *   ALARM M0..M5 : GPIO 1 2 15 16 18 40   <- LTV-847 collector, 10k pull-up to 3V3 (R36-R41); LOW = alarm
 *   ESTOP_MCU    : GPIO 21                <- J2 loop, 10k pull-up (R1), 1k/100n filter; HIGH = loop open = STOP
 *   MCU_INHIBIT  : GPIO 47 -> U12 NOR, 1k pull-up (R61).  HIGH = inhibit (safe default)
 *   ARM_PULSE    : GPIO 48 -> U11 D-flop CLK, 1k pull-down (R62). Rising edge arms if permitted.
 *   W5500 SPI    : MOSI 35, MISO 37, SCLK 36, CS 38, INTn 39. RSTn is an RC on the board, not an MCU pin.
 *   UART0 (J1)   : TX 43, RX 44
 *   Native USB   : GPIO 19/20 (console) - never touched.
 *   BOOT / RESET : GPIO0 / EN (SW2 / SW1)
 *
 * Rules baked in:
 *   - GPIO47 goes HIGH and GPIO48 LOW before anything else runs.
 *   - GPIO48 is never driven as part of any automatic test; only the explicit `arm` command pulses it.
 *   - Every command ends with a line "OK" or "ERR <reason>".
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/temperature_sensor.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_netif_glue.h"
#include "lwip/sockets.h"

#define PROBE_VERSION "r13probe 0.8 (2026-10-08)"

/* ---- pin tables -------------------------------------------------------- */
static const int STEP_PIN[6]  = {4, 5, 6, 7, 8, 9};
static const int DIR_PIN[6]   = {10, 11, 12, 13, 14, 17};
static const int ALARM_PIN[6] = {1, 2, 15, 16, 18, 40};
#define PIN_ESTOP   21
#define PIN_INHIBIT 47
#define PIN_ARM     48
#define PIN_IO0     0
#define PIN_MOSI    35
#define PIN_SCLK    36
#define PIN_MISO    37
#define PIN_CS      38
#define PIN_INTN    39
#define PIN_U0TX    43
#define PIN_U0RX    44

static spi_device_handle_t s_w5500 = NULL;
static bool s_spi_bus_up = false;
static esp_eth_handle_t s_eth;
static temperature_sensor_handle_t s_tsens = NULL;
static bool s_wifi_inited = false;

/* ---- tiny helpers ------------------------------------------------------- */
static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
}
/* Inside selftest, sub-commands do not emit their own OK/ERR; failures are tallied. */
static int s_nest = 0, s_nest_fails = 0;
#define OK()       do { if (!s_nest) out("OK\n"); } while (0)
#define ERR(...)   do { if (s_nest) { s_nest_fails++; out("FAIL "); } else out("ERR "); out(__VA_ARGS__); out("\n"); } while (0)

static void cfg_out(int pin, int level)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT_OUTPUT,   /* readback always available */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_set_level(pin, level);          /* preload before direction switch */
    gpio_config(&c);
    gpio_set_level(pin, level);
}

static void cfg_in(int pin, bool pu, bool pd)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pu ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pd ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
}

/* Safe output state: everything low, inhibit asserted. Called at boot and after any test. */
static void safe_outputs(void)
{
    cfg_out(PIN_INHIBIT, 1);
    cfg_out(PIN_ARM, 0);
    for (int i = 0; i < 6; i++) { cfg_out(STEP_PIN[i], 0); cfg_out(DIR_PIN[i], 0); }
}

static void cfg_inputs(void)
{
    for (int i = 0; i < 6; i++) cfg_in(ALARM_PIN[i], false, false);
    cfg_in(PIN_ESTOP, false, false);
    cfg_in(PIN_IO0, false, false);     /* external 10k pull-up R5 */
    cfg_in(PIN_INTN, false, false);
}

/* ---- info ---------------------------------------------------------------- */
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    default: return "UNKNOWN";
    }
}

static void cmd_info(void)
{
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    uint32_t flash = 0;
    esp_flash_get_size(NULL, &flash);
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    out("probe.version=%s\n", PROBE_VERSION);
    out("app.idf=%s\n", esp_get_idf_version());
    out("chip.model=%d rev=%d.%d cores=%d features=0x%" PRIx32 "\n", ci.model, ci.revision / 100, ci.revision % 100, ci.cores, ci.features);
    out("chip.mac=%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    out("flash.bytes=%" PRIu32 "\n", flash);
    out("psram.init=%d psram.bytes=%u\n", esp_psram_is_initialized(), (unsigned)esp_psram_get_size());
    out("heap.free=%u heap.free.spiram=%u heap.free.internal=%u\n",
        (unsigned)esp_get_free_heap_size(),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    out("reset.reason=%s\n", reset_reason_str(esp_reset_reason()));
    out("uptime.ms=%lld\n", esp_timer_get_time() / 1000);
    OK();
}

/* ---- PSRAM pattern test --------------------------------------------------- */
static void cmd_psram(void)
{
    if (!esp_psram_is_initialized()) { ERR("psram not initialised"); return; }
    size_t sz = 1024 * 1024;  /* 1 MiB block, leaves room for heap */
    uint32_t *p = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    if (!p) { ERR("malloc spiram failed"); return; }
    size_t n = sz / 4, bad = 0;
    int64_t t0 = esp_timer_get_time();
    for (size_t i = 0; i < n; i++) p[i] = (uint32_t)(i * 2654435761u);
    for (size_t i = 0; i < n; i++) if (p[i] != (uint32_t)(i * 2654435761u)) bad++;
    for (size_t i = 0; i < n; i++) p[i] = ~(uint32_t)(i * 2654435761u);
    for (size_t i = 0; i < n; i++) if (p[i] != ~(uint32_t)(i * 2654435761u)) bad++;
    int64_t t1 = esp_timer_get_time();
    heap_caps_free(p);
    out("psram.test.bytes=%u bad=%u ms=%lld\n", (unsigned)sz, (unsigned)bad, (t1 - t0) / 1000);
    if (bad) ERR("psram pattern mismatches"); else OK();
}

/* ---- inputs ----------------------------------------------------------------- */
static void print_inputs(void)
{
    int e = gpio_get_level(PIN_ESTOP);
    out("estop.gpio21=%d  (%s)\n", e, e ? "LOOP OPEN -> STOP" : "loop closed -> run permitted");
    out("alarm.gpio[1,2,15,16,18,40]=");
    for (int i = 0; i < 6; i++) out("%d%s", gpio_get_level(ALARM_PIN[i]), i < 5 ? "," : "");
    out("  (0 = alarm asserted; 1 = idle/no drive)\n");
    out("io0=%d w5500.intn=%d\n", gpio_get_level(PIN_IO0), gpio_get_level(PIN_INTN));
    out("inhibit.gpio47=%d arm.gpio48=%d (readback)\n", gpio_get_level(PIN_INHIBIT), gpio_get_level(PIN_ARM));
    out("step.readback=");
    for (int i = 0; i < 6; i++) out("%d", gpio_get_level(STEP_PIN[i]));
    out(" dir.readback=");
    for (int i = 0; i < 6; i++) out("%d", gpio_get_level(DIR_PIN[i]));
    out("\n");
}

static void cmd_in(void) { print_inputs(); OK(); }

/* ---- bias (external pull resistor presence) --------------------------------- */
/* Internal pulls are ~45k. An external 10k/1k pull wins against them. If a pin
 * follows the internal pull, its external bias resistor is missing/open. */
typedef struct { int pin; const char *name; int expect; } bias_t;

static int bias_check(const bias_t *b, int *fails)
{
    cfg_in(b->pin, true, false);
    vTaskDelay(pdMS_TO_TICKS(30));
    int with_pu = gpio_get_level(b->pin);
    cfg_in(b->pin, false, true);
    vTaskDelay(pdMS_TO_TICKS(30));
    int with_pd = gpio_get_level(b->pin);
    cfg_in(b->pin, false, false);
    bool ok = (with_pu == b->expect) && (with_pd == b->expect);
    out("bias.%-14s gpio%-2d expect=%d pu=%d pd=%d %s\n", b->name, b->pin, b->expect, with_pu, with_pd,
        ok ? "PASS" : (with_pu == 1 && with_pd == 0 ? "FAIL(FLOATING)" : "FAIL(STUCK)"));
    if (!ok) (*fails)++;
    return ok;
}

static void cmd_bias(void)
{
    int fails = 0;
    static bias_t list[32]; int n = 0;
    static char names[32][16];
    for (int i = 0; i < 6; i++) { snprintf(names[n], 16, "STEP%d_1kPD", i); list[n] = (bias_t){STEP_PIN[i], names[n], 0}; n++; }
    for (int i = 0; i < 6; i++) { snprintf(names[n], 16, "DIR%d_1kPD", i);  list[n] = (bias_t){DIR_PIN[i], names[n], 0}; n++; }
    for (int i = 0; i < 6; i++) { snprintf(names[n], 16, "ALARM%d_10kPU", i); list[n] = (bias_t){ALARM_PIN[i], names[n], 1}; n++; }
    list[n++] = (bias_t){PIN_ESTOP, "ESTOP_10kPU", 1};
    list[n++] = (bias_t){PIN_INHIBIT, "INHIBIT_1kPU", 1};
    list[n++] = (bias_t){PIN_ARM, "ARM_1kPD", 0};
    list[n++] = (bias_t){PIN_IO0, "IO0_10kPU", 1};
    for (int i = 0; i < n; i++) bias_check(&list[i], &fails);
    safe_outputs();
    cfg_inputs();
    out("bias.fails=%d\n", fails);
    if (fails) ERR("%d bias failures", fails); else OK();
}

/* ---- pin-to-pin short walk ---------------------------------------------------- */
static void cmd_short(void)
{
    int fails = 0;
    static int drv[12]; static const char *dname[12]; static char nb[12][8];
    for (int i = 0; i < 6; i++) { drv[i] = STEP_PIN[i]; snprintf(nb[i], 8, "STEP%d", i); dname[i] = nb[i]; }
    for (int i = 0; i < 6; i++) { drv[6+i] = DIR_PIN[i]; snprintf(nb[6+i], 8, "DIR%d", i); dname[6+i] = nb[6+i]; }
    /* pins with external pull-down: expected LOW while a neighbour is HIGH */
    /* pins with external pull-up: expected HIGH while a neighbour is LOW */
    for (int d = 0; d < 12; d++) {
        int p = drv[d];
        /* phase HIGH */
        gpio_set_level(p, 1);
        esp_rom_delay_us(500);
        if (gpio_get_level(p) != 1) { out("short.%s drive=1 readback=0 -> SHORT TO GND or no drive\n", dname[d]); fails++; }
        for (int o = 0; o < 12; o++) if (o != d && gpio_get_level(drv[o]) != 0) { out("short.%s=1 sees %s=1 -> SHORT\n", dname[d], dname[o]); fails++; }
        if (gpio_get_level(PIN_ARM) != 0) { out("short.%s=1 sees ARM(48)=1 -> SHORT\n", dname[d]); fails++; }
        /* phase LOW */
        gpio_set_level(p, 0);
        esp_rom_delay_us(500);
        if (gpio_get_level(p) != 0) { out("short.%s drive=0 readback=1 -> SHORT TO 3V3\n", dname[d]); fails++; }
        if (gpio_get_level(PIN_INHIBIT) != 1) { out("short.%s=0 sees INHIBIT(47)=0 -> SHORT\n", dname[d]); fails++; }
        if (gpio_get_level(PIN_ESTOP) != 1) { out("short.%s=0 sees ESTOP(21)=0 -> SHORT (or loop closed)\n", dname[d]); fails++; }
        for (int a = 0; a < 6; a++) if (gpio_get_level(ALARM_PIN[a]) != 1) { out("short.%s=0 sees ALARM%d=0 -> SHORT (or alarm asserted)\n", dname[d], a); fails++; }
    }
    safe_outputs();
    out("short.walked=12 fails=%d\n", fails);
    if (fails) ERR("%d short-walk anomalies", fails); else OK();
}

/* ---- monitor ------------------------------------------------------------------- */
static void cmd_mon(int seconds)
{
    int64_t end = esp_timer_get_time() + (int64_t)seconds * 1000000;
    int last_e = -1, last_a[6] = {-1,-1,-1,-1,-1,-1}, last_int = -1;
    out("mon.start seconds=%d (short J2 to see estop fall)\n", seconds);
    while (esp_timer_get_time() < end) {
        int e = gpio_get_level(PIN_ESTOP), in = gpio_get_level(PIN_INTN);
        bool chg = (e != last_e) || (in != last_int);
        int a[6];
        for (int i = 0; i < 6; i++) { a[i] = gpio_get_level(ALARM_PIN[i]); if (a[i] != last_a[i]) chg = true; }
        if (chg) {
            out("mon.t=%lld estop=%d alarm=%d%d%d%d%d%d intn=%d\n", esp_timer_get_time() / 1000, e,
                a[0], a[1], a[2], a[3], a[4], a[5], in);
            last_e = e; last_int = in; memcpy(last_a, a, sizeof a);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    OK();
}

/* ---- W5500 over raw SPI ---------------------------------------------------------- */
static esp_err_t w5500_init(void)
{
    if (s_w5500) return ESP_OK;
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64,
    };
    esp_err_t r = ESP_OK;
    if (!s_spi_bus_up) { r = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO); if (r == ESP_OK) s_spi_bus_up = true; }
    if (r != ESP_OK) return r;
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000, .mode = 0, .spics_io_num = PIN_CS, .queue_size = 2,
    };
    return spi_bus_add_device(SPI2_HOST, &dev, &s_w5500);
}

/* W5500 VDM frame: addr[15:8] addr[7:0] control(BSB<<3 | RWB<<2 | OM) data... */
static esp_err_t w5500_xfer(uint16_t addr, uint8_t bsb, bool write, uint8_t *buf, size_t len)
{
    uint8_t tx[3 + 16] = { addr >> 8, addr & 0xff, (uint8_t)((bsb << 3) | (write ? 0x04 : 0x00)) };
    uint8_t rx[3 + 16] = {0};
    if (len > 16) return ESP_ERR_INVALID_SIZE;
    if (write) memcpy(tx + 3, buf, len);
    spi_transaction_t t = { .length = (3 + len) * 8, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t r = spi_device_transmit(s_w5500, &t);
    if (r == ESP_OK && !write) memcpy(buf, rx + 3, len);
    return r;
}

static void cmd_w5500(void)
{
    if (s_eth) { ERR("eth driver owns the W5500 now; reboot for raw register access"); return; }
    esp_err_t r = w5500_init();
    if (r != ESP_OK) { ERR("spi init %s", esp_err_to_name(r)); return; }
    uint8_t ver = 0, mr = 0, phy = 0;
    w5500_xfer(0x0039, 0, false, &ver, 1);
    w5500_xfer(0x0000, 0, false, &mr, 1);
    w5500_xfer(0x002E, 0, false, &phy, 1);
    out("w5500.versionr=0x%02x (expect 0x04)\n", ver);
    out("w5500.mr=0x%02x\n", mr);
    out("w5500.phycfgr=0x%02x link=%d speed=%s duplex=%s opmd=%d opmdc=%d\n", phy, phy & 1,
        (phy & 2) ? "100M" : "10M", (phy & 4) ? "full" : "half", (phy >> 6) & 1, (phy >> 3) & 7);
    /* write/readback round trip on SIPR (source IP) then restore */
    uint8_t ip[4] = {10, 20, 30, 40}, rb[4] = {0}, zero[4] = {0};
    w5500_xfer(0x000F, 0, true, ip, 4);
    w5500_xfer(0x000F, 0, false, rb, 4);
    w5500_xfer(0x000F, 0, true, zero, 4);
    bool rt = memcmp(ip, rb, 4) == 0;
    out("w5500.sipr.roundtrip=%s (%d.%d.%d.%d)\n", rt ? "PASS" : "FAIL", rb[0], rb[1], rb[2], rb[3]);
    out("w5500.intn.gpio39=%d\n", gpio_get_level(PIN_INTN));
    if (ver != 0x04) ERR("VERSIONR mismatch: SPI/W5500 not responding"); else if (!rt) ERR("register write/readback failed"); else OK();
}

/* ---- W5500 through the real esp_eth driver: link + DHCP ---------------------------- */
static esp_netif_t *s_eth_netif = NULL;
static volatile int s_eth_link = 0;
static volatile bool s_eth_got_ip = false;
static esp_netif_ip_info_t s_eth_ip;

static void eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == ETH_EVENT) {
        if (id == ETHERNET_EVENT_CONNECTED) s_eth_link = 1;
        else if (id == ETHERNET_EVENT_DISCONNECTED) s_eth_link = 0;
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        s_eth_ip = ((ip_event_got_ip_t *)data)->ip_info;
        s_eth_got_ip = true;
    }
}

static void cmd_eth(const char *secs_s)
{
    int secs = secs_s ? atoi(secs_s) : 15;
    if (secs < 1 || secs > 120) secs = 15;
    if (!s_eth) {
        /* the raw probe device and the driver must not share CS 38 */
        if (s_w5500) { spi_bus_remove_device(s_w5500); s_w5500 = NULL; }
        spi_bus_config_t bus = { .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_SCLK,
                                 .quadwp_io_num = -1, .quadhd_io_num = -1 };
        esp_err_t r = ESP_OK;
        if (!s_spi_bus_up) { r = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO); if (r == ESP_OK) s_spi_bus_up = true; }
        if (r != ESP_OK) { ERR("spi bus %s", esp_err_to_name(r)); return; }
        esp_netif_init();
        esp_event_loop_create_default();   /* harmless if wifi already made it */
        gpio_install_isr_service(0);       /* W5500 INTn (GPIO39) is interrupt driven */
        spi_device_interface_config_t dev = { .command_bits = 16, .address_bits = 8, .mode = 0,
                                              .clock_speed_hz = 26 * 1000 * 1000, .spics_io_num = PIN_CS, .queue_size = 20 };  /* GPIO-matrix full-duplex limit */
        eth_w5500_config_t w5500 = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &dev);
        w5500.int_gpio_num = PIN_INTN;
        eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
        eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
        phy_cfg.reset_gpio_num = -1;        /* RSTn is an RC on the board */
        esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500, &mac_cfg);
        esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);
        esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
        r = esp_eth_driver_install(&cfg, &s_eth);
        if (r != ESP_OK) { ERR("eth driver %s", esp_err_to_name(r)); s_eth = NULL; return; }
        uint8_t mac_addr[6];
        esp_read_mac(mac_addr, ESP_MAC_ETH);   /* W5500 has no MAC of its own: use the chip's Ethernet MAC */
        esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, mac_addr);
        out("eth.mac=%02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        esp_netif_config_t ncfg = ESP_NETIF_DEFAULT_ETH();
        s_eth_netif = esp_netif_new(&ncfg);
        esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth));
        esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, eth_event, NULL);
        r = esp_eth_start(s_eth);
        if (r != ESP_OK) { ERR("eth start %s", esp_err_to_name(r)); return; }
    }
    out("eth.wait secs=%d (plug J10 into a switch with DHCP)\n", secs);
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    int last = -1;
    while (esp_timer_get_time() < end) {
        if (s_eth_link != last) { out("eth.link=%d t=%lld\n", s_eth_link, esp_timer_get_time() / 1000); last = s_eth_link; }
        if (s_eth_got_ip) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_eth_got_ip) {
        out("eth.ip=" IPSTR " mask=" IPSTR " gw=" IPSTR "\n", IP2STR(&s_eth_ip.ip), IP2STR(&s_eth_ip.netmask), IP2STR(&s_eth_ip.gw));
        OK();
    } else if (s_eth_link) ERR("link up but no DHCP lease in %d s", secs);
    else ERR("no link in %d s", secs);
}

/* ---- link benchmarks: UDP echo and raw USB echo ------------------------------------- */
static void cmd_udpecho(const char *secs_s, const char *port_s)
{
    int secs = secs_s ? atoi(secs_s) : 20, port = port_s ? atoi(port_s) : 5005;
    if (secs < 1 || secs > 600) secs = 20;
    if (!s_eth) { ERR("run `eth` first"); return; }
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ERR("socket"); return; }
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(sock, (struct sockaddr *)&me, sizeof me) < 0) { close(sock); ERR("bind %d", port); return; }
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    static uint8_t buf[1500];
    uint32_t pkts = 0, bytes = 0, maxlen = 0;
    out("udpecho.start port=%d secs=%d ip=" IPSTR "\n", port, secs, IP2STR(&s_eth_ip.ip));
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        struct sockaddr_in from; socklen_t fl = sizeof from;
        int n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
        if (n <= 0) continue;
        sendto(sock, buf, n, 0, (struct sockaddr *)&from, fl);
        pkts++; bytes += n; if ((uint32_t)n > maxlen) maxlen = n;
    }
    close(sock);
    out("udpecho.done pkts=%" PRIu32 " bytes=%" PRIu32 " maxlen=%" PRIu32 "\n", pkts, bytes, maxlen);
    OK();
}

/* One-way ingest tests: the host streams packets, the board only counts them (the app's real
 * traffic is host -> board target frames). Reports count, bytes, and the largest inter-packet gap. */
static void cmd_udpsink(const char *secs_s, const char *port_s)
{
    int secs = secs_s ? atoi(secs_s) : 20, port = port_s ? atoi(port_s) : 5006;
    if (secs < 1 || secs > 600) secs = 20;
    if (!s_eth) { ERR("run `eth` first"); return; }
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ERR("socket"); return; }
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(sock, (struct sockaddr *)&me, sizeof me) < 0) { close(sock); ERR("bind %d", port); return; }
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    static uint8_t buf[1500];
    uint32_t pkts = 0, bytes = 0, seq_gaps = 0, expect = 0;
    int64_t first = 0, last = 0, maxgap = 0;
    out("udpsink.start port=%d secs=%d ip=" IPSTR "  (first 4 bytes = big-endian seq)\n", port, secs, IP2STR(&s_eth_ip.ip));
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        int n = recv(sock, buf, sizeof buf, 0);
        if (n <= 0) continue;
        int64_t now = esp_timer_get_time();
        if (!first) first = now; else if (now - last > maxgap) maxgap = now - last;
        last = now;
        if (n >= 4) {
            uint32_t seq = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
            if (pkts && seq != expect) seq_gaps++;
            expect = seq + 1;
        }
        pkts++; bytes += n;
    }
    close(sock);
    int64_t span = last - first;
    out("udpsink.done pkts=%" PRIu32 " bytes=%" PRIu32 " seqgaps=%" PRIu32 " span_ms=%lld rate_pps=%lld maxgap_us=%lld\n",
        pkts, bytes, seq_gaps, span / 1000, span > 0 ? (int64_t)pkts * 1000000 / span : 0, maxgap);
    OK();
}

static void cmd_usbsink(const char *secs_s)
{
    int secs = secs_s ? atoi(secs_s) : 20;
    if (secs < 1 || secs > 600) secs = 20;
    out("usbsink.start secs=%d\n", secs);
    static uint8_t buf[4096];
    uint32_t bytes = 0, reads = 0;
    int64_t first = 0, last = 0, maxgap = 0;
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, pdMS_TO_TICKS(20));
        if (n <= 0) continue;
        int64_t now = esp_timer_get_time();
        if (!first) first = now; else if (now - last > maxgap) maxgap = now - last;
        last = now;
        bytes += n; reads++;
    }
    int64_t span = last - first;
    out("usbsink.done bytes=%" PRIu32 " reads=%" PRIu32 " span_ms=%lld rate_kBps=%lld maxgap_us=%lld\n",
        bytes, reads, span / 1000, span > 0 ? (int64_t)bytes * 1000 / span : 0, maxgap);
    OK();
}

/* Raw echo on the USB console itself: everything received is written straight back.
 * The host must stop sending before `secs` elapse; the summary follows the echoed bytes. */
static void cmd_usbecho(const char *secs_s)
{
    int secs = secs_s ? atoi(secs_s) : 20;
    if (secs < 1 || secs > 600) secs = 20;
    out("usbecho.start secs=%d\n", secs);
    static uint8_t buf[2048];
    uint32_t bytes = 0, reads = 0;
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, pdMS_TO_TICKS(20));
        if (n <= 0) continue;
        int w = 0;
        while (w < n) {
            int k = usb_serial_jtag_write_bytes(buf + w, n - w, pdMS_TO_TICKS(100));
            if (k <= 0) break;
            w += k;
        }
        bytes += w; reads++;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    out("\nusbecho.done bytes=%" PRIu32 " reads=%" PRIu32 "\n", bytes, reads);
    OK();
}

/* ---- W5500 hardware-UDP experiment (no lwIP, no esp_eth) ------------------------------
 * The W5500 carries its own IP/UDP stack. Socket 0 is opened in UDP mode on `port`; every
 * datagram is read with one header+payload transfer and echoed (or only counted). This measures
 * the floor of per-packet cost on this board: SPI transactions + our own loop, nothing else. */
#define W5_BSB_COMMON 0
#define W5_BSB_S0_REG 1
#define W5_BSB_S0_TX  2
#define W5_BSB_S0_RX  3
#define W5_MR 0x0000
#define W5_GAR 0x0001
#define W5_SUBR 0x0005
#define W5_SHAR 0x0009
#define W5_SIPR 0x000F
#define W5_PHYCFGR 0x002E
#define S0_MR 0x0000
#define S0_CR 0x0001
#define S0_IR 0x0002
#define S0_SR 0x0003
#define S0_PORT 0x0004
#define S0_DIPR 0x000C
#define S0_DPORT 0x0010
#define S0_RXBUF_SIZE 0x001E
#define S0_TXBUF_SIZE 0x001F
#define S0_TX_FSR 0x0020
#define S0_TX_WR 0x0024
#define S0_RX_RSR 0x0026
#define S0_RX_RD 0x0028
#define SCR_OPEN 0x01
#define SCR_CLOSE 0x10
#define SCR_SEND 0x20
#define SCR_RECV 0x40
#define SIR_SEND_OK 0x10
#define SIR_TIMEOUT 0x08

static spi_device_handle_t s_w5fast = NULL;

static esp_err_t w5_xfer(uint16_t addr, uint8_t bsb, bool write, uint8_t *buf, size_t len)
{
    /* command phase = 16-bit address, address phase = 8-bit control byte; data in the data phase */
    spi_transaction_t t = {
        .cmd = addr, .addr = (uint32_t)((bsb << 3) | (write ? 0x04 : 0x00)),
        .length = len * 8, .rxlength = write ? 0 : len * 8,
        .tx_buffer = write ? buf : NULL, .rx_buffer = write ? NULL : buf,
    };
    return spi_device_polling_transmit(s_w5fast, &t);
}
static inline uint8_t w5_r8(uint16_t a, uint8_t bsb) { uint8_t v = 0; w5_xfer(a, bsb, false, &v, 1); return v; }
static inline void w5_w8(uint16_t a, uint8_t bsb, uint8_t v) { w5_xfer(a, bsb, true, &v, 1); }
static inline uint16_t w5_r16(uint16_t a, uint8_t bsb) { uint8_t v[2] = {0}; w5_xfer(a, bsb, false, v, 2); return (v[0] << 8) | v[1]; }
static inline void w5_w16(uint16_t a, uint8_t bsb, uint16_t v) { uint8_t b[2] = { v >> 8, v & 0xff }; w5_xfer(a, bsb, true, b, 2); }
static void w5_cmd(uint8_t c) { w5_w8(S0_CR, W5_BSB_S0_REG, c); while (w5_r8(S0_CR, W5_BSB_S0_REG)) {} }

static void cmd_w5udp(const char *secs_s, const char *mhz_s, const char *mode_s, const char *port_s)
{
    int secs = secs_s ? atoi(secs_s) : 20, mhz = mhz_s ? atoi(mhz_s) : 26, port = port_s ? atoi(port_s) : 5007;
    bool echo = !(mode_s && !strcmp(mode_s, "sink"));
    if (secs < 1 || secs > 600) secs = 20;
    if (mhz < 1 || mhz > 80) mhz = 26;
    if (s_eth) { ERR("esp_eth owns the W5500; reboot and run w5udp without `eth`"); return; }
    if (s_w5500) { spi_bus_remove_device(s_w5500); s_w5500 = NULL; }
    if (s_w5fast) { spi_bus_remove_device(s_w5fast); s_w5fast = NULL; }
    spi_bus_config_t bus = { .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_SCLK,
                             .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096 };
    if (!s_spi_bus_up) { if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) { ERR("spi bus"); return; } s_spi_bus_up = true; }
    spi_device_interface_config_t dev = { .command_bits = 16, .address_bits = 8, .mode = 0,
                                          .clock_speed_hz = mhz * 1000 * 1000, .spics_io_num = PIN_CS, .queue_size = 4 };
    esp_err_t r = spi_bus_add_device(SPI2_HOST, &dev, &s_w5fast);
    if (r != ESP_OK) { ERR("spi add device at %d MHz: %s", mhz, esp_err_to_name(r)); s_w5fast = NULL; return; }
    int actual_khz = 0; spi_device_get_actual_freq(s_w5fast, &actual_khz);
    spi_device_acquire_bus(s_w5fast, portMAX_DELAY);

    /* bring the chip to a known state: software reset, static IP = the DHCP lease we already hold */
    w5_w8(W5_MR, W5_BSB_COMMON, 0x80);
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_ETH);
    w5_xfer(W5_SHAR, W5_BSB_COMMON, true, mac, 6);
    uint8_t ip[4] = {192, 168, 1, 198}, gw[4] = {192, 168, 1, 254}, sn[4] = {255, 255, 255, 0};
    w5_xfer(W5_SIPR, W5_BSB_COMMON, true, ip, 4);
    w5_xfer(W5_GAR, W5_BSB_COMMON, true, gw, 4);
    w5_xfer(W5_SUBR, W5_BSB_COMMON, true, sn, 4);
    /* all 16 kB RX and TX to socket 0 */
    for (int s = 1; s < 8; s++) { w5_w8(S0_RXBUF_SIZE, 1 + 4 * s, 0); w5_w8(S0_TXBUF_SIZE, 1 + 4 * s, 0); }
    w5_w8(S0_RXBUF_SIZE, W5_BSB_S0_REG, 16); w5_w8(S0_TXBUF_SIZE, W5_BSB_S0_REG, 16);
    w5_w8(S0_MR, W5_BSB_S0_REG, 0x02);              /* UDP */
    w5_w16(S0_PORT, W5_BSB_S0_REG, port);
    w5_cmd(SCR_OPEN);
    uint8_t sr = w5_r8(S0_SR, W5_BSB_S0_REG);
    uint8_t ver = w5_r8(0x0039, W5_BSB_COMMON), phy = w5_r8(W5_PHYCFGR, W5_BSB_COMMON);
    out("w5udp.start mode=%s port=%d spi_khz=%d versionr=0x%02x phycfgr=0x%02x s0_sr=0x%02x(expect 0x22) ip=192.168.1.198 secs=%d\n",
        echo ? "echo" : "sink", port, actual_khz, ver, phy, sr, secs);
    if (sr != 0x22) { spi_device_release_bus(s_w5fast); ERR("socket did not open in UDP mode"); return; }

    static uint8_t pkt[1600];
    uint32_t pkts = 0, bytes = 0, seq_gaps = 0, expect = 0, tx_fail = 0;
    int64_t busy_us = 0, first = 0, last = 0, maxgap = 0;
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        uint16_t rsr = w5_r16(S0_RX_RSR, W5_BSB_S0_REG);
        if (rsr < 8) { continue; }
        int64_t t0 = esp_timer_get_time();
        uint16_t rd = w5_r16(S0_RX_RD, W5_BSB_S0_REG);
        uint8_t hdr[8]; w5_xfer(rd, W5_BSB_S0_RX, false, hdr, 8);
        uint16_t len = (hdr[6] << 8) | hdr[7];
        if (len > sizeof pkt) len = sizeof pkt;
        w5_xfer(rd + 8, W5_BSB_S0_RX, false, pkt, len);
        w5_w16(S0_RX_RD, W5_BSB_S0_REG, rd + 8 + len);
        w5_cmd(SCR_RECV);
        if (len >= 4) {
            uint32_t seq = ((uint32_t)pkt[0] << 24) | ((uint32_t)pkt[1] << 16) | ((uint32_t)pkt[2] << 8) | pkt[3];
            if (pkts && seq != expect) seq_gaps++;
            expect = seq + 1;
        }
        if (echo) {
            w5_xfer(S0_DIPR, W5_BSB_S0_REG, true, hdr, 4);       /* reply to sender */
            w5_xfer(S0_DPORT, W5_BSB_S0_REG, true, hdr + 4, 2);
            uint16_t wr = w5_r16(S0_TX_WR, W5_BSB_S0_REG);
            w5_xfer(wr, W5_BSB_S0_TX, true, pkt, len);
            w5_w16(S0_TX_WR, W5_BSB_S0_REG, wr + len);
            w5_cmd(SCR_SEND);
            uint8_t ir;
            do { ir = w5_r8(S0_IR, W5_BSB_S0_REG); } while (!(ir & (SIR_SEND_OK | SIR_TIMEOUT)));
            w5_w8(S0_IR, W5_BSB_S0_REG, ir);
            if (ir & SIR_TIMEOUT) tx_fail++;
        }
        int64_t t1 = esp_timer_get_time();
        busy_us += t1 - t0;
        if (!first) first = t1; else if (t1 - last > maxgap) maxgap = t1 - last;
        last = t1;
        pkts++; bytes += len;
    }
    w5_cmd(SCR_CLOSE);
    spi_device_release_bus(s_w5fast);
    int64_t span = last - first;
    out("w5udp.done pkts=%" PRIu32 " bytes=%" PRIu32 " seqgaps=%" PRIu32 " txfail=%" PRIu32 " span_ms=%lld rate_pps=%lld maxgap_us=%lld busy_us_per_pkt=%lld cpu_busy_pct=%lld\n",
        pkts, bytes, seq_gaps, tx_fail, span / 1000, span > 0 ? (int64_t)pkts * 1000000 / span : 0, maxgap,
        pkts ? busy_us / pkts : 0, span > 0 ? busy_us * 100 / span : 0);
    OK();
}

/* ---- temperature ---------------------------------------------------------------- */
static void cmd_temp(void)
{
    if (!s_tsens) {
        temperature_sensor_config_t c = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        if (temperature_sensor_install(&c, &s_tsens) != ESP_OK) { ERR("tsens install"); return; }
        temperature_sensor_enable(s_tsens);
    }
    float t = 0;
    temperature_sensor_get_celsius(s_tsens, &t);
    out("die.temp.c=%.1f\n", t);
    OK();
}

/* ---- STEP / DIR control ----------------------------------------------------------- */
static int parse_axis(const char *s)
{
    if (!s) return -1;
    int a = atoi(s);
    return (a >= 0 && a < 6 && isdigit((unsigned char)s[0])) ? a : -1;
}

static void cmd_level(const char *what, const char *axis_s, const char *lvl_s)
{
    int a = parse_axis(axis_s);
    if (a < 0 || !lvl_s) { ERR("usage: %s <0-5> <0|1>", what); return; }
    int pin = (what[0] == 's') ? STEP_PIN[a] : DIR_PIN[a];
    int lvl = atoi(lvl_s) ? 1 : 0;
    gpio_set_level(pin, lvl);
    esp_rom_delay_us(100);
    out("%s%d gpio%d set=%d readback=%d\n", what, a, pin, lvl, gpio_get_level(pin));
    OK();
}

/* bit-banged pulse train: n pulses at hz on STEP axis; counts readback edges */
static void cmd_pulse(const char *axis_s, const char *hz_s, const char *n_s)
{
    int a = parse_axis(axis_s);
    int hz = hz_s ? atoi(hz_s) : 0, n = n_s ? atoi(n_s) : 0;
    if (a < 0 || hz < 1 || hz > 200000 || n < 1 || n > 1000000) { ERR("usage: pulse <0-5> <hz 1..200000> <n 1..1e6>"); return; }
    int pin = STEP_PIN[a];
    uint32_t half = 500000 / hz;
    if (half < 2) half = 2;
    int seen = 0;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < n; i++) {
        gpio_set_level(pin, 1);
        esp_rom_delay_us(half);
        if (gpio_get_level(pin)) seen++;
        gpio_set_level(pin, 0);
        esp_rom_delay_us(half);
        if ((i & 0x3ff) == 0x3ff) vTaskDelay(1);
    }
    int64_t t1 = esp_timer_get_time();
    out("pulse.axis=%d gpio=%d n=%d hz=%d highs_seen=%d ms=%lld\n", a, pin, n, hz, seen, (t1 - t0) / 1000);
    if (seen != n) ERR("readback missed %d highs", n - seen); else OK();
}

/* hardware-timed square wave via LEDC for scope work */
static void cmd_tone(const char *axis_s, const char *hz_s, const char *ms_s)
{
    int a = parse_axis(axis_s);
    int hz = hz_s ? atoi(hz_s) : 0, ms = ms_s ? atoi(ms_s) : 0;
    if (a < 0 || hz < 10 || hz > 2000000 || ms < 1 || ms > 60000) { ERR("usage: tone <0-5> <hz 10..2e6> <ms 1..60000>"); return; }
    int pin = STEP_PIN[a];
    ledc_timer_config_t tc = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_4_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = hz, .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&tc) != ESP_OK) { ERR("ledc timer (freq unreachable)"); return; }
    ledc_channel_config_t cc = {
        .gpio_num = pin, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .duty = 8, .hpoint = 0,
    };
    ledc_channel_config(&cc);
    uint32_t actual = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    out("tone.axis=%d gpio=%d hz.req=%d hz.actual=%" PRIu32 " ms=%d\n", a, pin, hz, actual, ms);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    gpio_reset_pin(pin);
    cfg_out(pin, 0);
    out("tone.done readback=%d\n", gpio_get_level(pin));
    OK();
}

/* walk each STEP/DIR high for hold_ms so a meter can follow it on the DB25s */
static void cmd_walk(const char *ms_s)
{
    int ms = ms_s ? atoi(ms_s) : 500;
    if (ms < 10 || ms > 10000) { ERR("usage: walk [hold_ms 10..10000]"); return; }
    for (int i = 0; i < 6; i++) {
        gpio_set_level(STEP_PIN[i], 1); out("walk.STEP%d gpio%d HIGH (J%d pin3 vs pin14)\n", i, STEP_PIN[i], 3 + i);
        vTaskDelay(pdMS_TO_TICKS(ms)); gpio_set_level(STEP_PIN[i], 0);
        gpio_set_level(DIR_PIN[i], 1);  out("walk.DIR%d gpio%d HIGH (J%d pin4 vs pin5)\n", i, DIR_PIN[i], 3 + i);
        vTaskDelay(pdMS_TO_TICKS(ms)); gpio_set_level(DIR_PIN[i], 0);
    }
    safe_outputs();
    OK();
}

/* ---- inhibit / arm --------------------------------------------------------------- */
static void cmd_inhibit(const char *lvl_s)
{
    if (!lvl_s) { ERR("usage: inhibit <0|1>  (1 = inhibit, safe)"); return; }
    int l = atoi(lvl_s) ? 1 : 0;
    gpio_set_level(PIN_INHIBIT, l);
    esp_rom_delay_us(100);
    out("inhibit.gpio47 set=%d readback=%d\n", l, gpio_get_level(PIN_INHIBIT));
    OK();
}

static void cmd_arm(void)
{
    /* r13 contract: INHIBIT low >= 250 ms, loop closed, VCC5 valid, then a clean ARM rising edge. */
    if (gpio_get_level(PIN_ESTOP)) { ERR("stop loop open (gpio21=1): arm refused"); return; }
    if (gpio_get_level(PIN_INHIBIT)) { ERR("inhibit still asserted: run `inhibit 0` first"); return; }
    out("arm: waiting 300 ms for supervisor release, then 50 us ARM pulse on gpio48\n");
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(PIN_ARM, 1);
    esp_rom_delay_us(50);
    gpio_set_level(PIN_ARM, 0);
    out("arm.pulsed=1 arm.readback=%d  NOTE: DRIVE_ENABLE (U11 pin5) has no MCU readback; meter it.\n", gpio_get_level(PIN_ARM));
    OK();
}

/* ---- UART0 loopback on J1 ------------------------------------------------------------- */
static void cmd_uartloop(void)
{
    uart_config_t uc = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
    if (!uart_is_driver_installed(UART_NUM_0)) {
        uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
        uart_param_config(UART_NUM_0, &uc);
        uart_set_pin(UART_NUM_0, PIN_U0TX, PIN_U0RX, -1, -1);
    }
    const char *msg = "r13-J1-loop-0123456789";
    uart_flush_input(UART_NUM_0);
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
    char rb[64] = {0};
    int n = uart_read_bytes(UART_NUM_0, rb, sizeof rb - 1, pdMS_TO_TICKS(200));
    out("uart0.tx=%s rx.n=%d rx=%s\n", msg, n, n > 0 ? rb : "");
    if (n == (int)strlen(msg) && memcmp(msg, rb, n) == 0) OK();
    else ERR("loopback mismatch (jumper J1 pin2 to pin3?)");
}

/* ---- Wi-Fi scan (RF / antenna sanity) ----------------------------------------------- */
static void cmd_wifi(void)
{
    if (!s_wifi_inited) {
        esp_err_t r = nvs_flash_init();
        if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); r = nvs_flash_init(); }
        if (r != ESP_OK) { ERR("nvs %s", esp_err_to_name(r)); return; }
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        if (esp_wifi_init(&cfg) != ESP_OK) { ERR("wifi init"); return; }
        esp_wifi_set_mode(WIFI_MODE_STA);
        if (esp_wifi_start() != ESP_OK) { ERR("wifi start"); return; }
        s_wifi_inited = true;
    }
    wifi_scan_config_t sc = { .show_hidden = false };
    esp_err_t r = esp_wifi_scan_start(&sc, true);
    if (r != ESP_OK) { ERR("scan %s", esp_err_to_name(r)); return; }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t *recs = calloc(n ? n : 1, sizeof(*recs));
    esp_wifi_scan_get_ap_records(&n, recs);
    out("wifi.ap.count=%u\n", n);
    int best = -127;
    for (int i = 0; i < n && i < 12; i++) {
        out("wifi.ap[%d] rssi=%d ch=%d ssid=%s\n", i, recs[i].rssi, recs[i].primary, (const char *)recs[i].ssid);
        if (recs[i].rssi > best) best = recs[i].rssi;
    }
    out("wifi.best.rssi=%d\n", best);
    free(recs);
    if (n == 0) ERR("no APs seen: check antenna / RF"); else OK();
}

/* ---- selftest ------------------------------------------------------------------------- */
static void cmd_selftest(void)
{
    out("---- selftest begin ----\n");
    s_nest = 1; s_nest_fails = 0;
    cmd_info();
    cmd_psram();
    cmd_temp();
    print_inputs();
    cmd_bias();
    cmd_short();
    cmd_w5500();
    s_nest = 0;
    out("---- selftest end ---- fails=%d\n", s_nest_fails);
    if (s_nest_fails) ERR("selftest %d failures", s_nest_fails); else OK();
}

static void cmd_help(void)
{
    out("commands:\n"
        "  info | psram | temp | in | bias | short | selftest\n"
        "  mon <sec>                 watch estop/alarms/intn for changes\n"
        "  w5500                     SPI identity, PHY link, register round-trip\n"
        "  step <ax> <0|1> | dir <ax> <0|1>\n"
        "  pulse <ax> <hz> <n>       bit-banged pulse train with readback count\n"
        "  tone <ax> <hz> <ms>       LEDC square wave for the scope\n"
        "  walk [ms]                 raise each STEP/DIR in turn for meter probing\n"
        "  inhibit <0|1>             GPIO47 (1 = inhibit, default)\n"
        "  arm                       qualified ARM pulse on GPIO48 (needs loop closed, inhibit 0)\n"
        "  uartloop                  J1 TX->RX loopback\n"
        "  wifi                      scan APs (RF sanity)\n"
        "  safe                      all outputs low, inhibit high\n"
        "  reboot\n");
    OK();
}

/* ---- console loop ------------------------------------------------------------------------ */
static void dispatch(char *line)
{
    char *argv[6] = {0}; int argc = 0;
    for (char *tok = strtok(line, " \t\r\n"); tok && argc < 6; tok = strtok(NULL, " \t\r\n")) argv[argc++] = tok;
    if (argc == 0) { OK(); return; }
    const char *c = argv[0];
    if      (!strcmp(c, "help"))     cmd_help();
    else if (!strcmp(c, "info"))     cmd_info();
    else if (!strcmp(c, "psram"))    cmd_psram();
    else if (!strcmp(c, "temp"))     cmd_temp();
    else if (!strcmp(c, "in"))       cmd_in();
    else if (!strcmp(c, "bias"))     cmd_bias();
    else if (!strcmp(c, "short"))    cmd_short();
    else if (!strcmp(c, "selftest")) cmd_selftest();
    else if (!strcmp(c, "mon"))      cmd_mon(argc > 1 ? atoi(argv[1]) : 10);
    else if (!strcmp(c, "w5500"))    cmd_w5500();
    else if (!strcmp(c, "eth"))      cmd_eth(argv[1]);
    else if (!strcmp(c, "udpecho"))  cmd_udpecho(argv[1], argv[2]);
    else if (!strcmp(c, "usbecho"))  cmd_usbecho(argv[1]);
    else if (!strcmp(c, "udpsink"))  cmd_udpsink(argv[1], argv[2]);
    else if (!strcmp(c, "usbsink"))  cmd_usbsink(argv[1]);
    else if (!strcmp(c, "w5udp"))    cmd_w5udp(argv[1], argv[2], argv[3], argv[4]);
    else if (!strcmp(c, "step"))     cmd_level("step", argv[1], argv[2]);
    else if (!strcmp(c, "dir"))      cmd_level("dir", argv[1], argv[2]);
    else if (!strcmp(c, "pulse"))    cmd_pulse(argv[1], argv[2], argv[3]);
    else if (!strcmp(c, "tone"))     cmd_tone(argv[1], argv[2], argv[3]);
    else if (!strcmp(c, "walk"))     cmd_walk(argv[1]);
    else if (!strcmp(c, "inhibit"))  cmd_inhibit(argv[1]);
    else if (!strcmp(c, "arm"))      cmd_arm();
    else if (!strcmp(c, "uartloop")) cmd_uartloop();
    else if (!strcmp(c, "wifi"))     cmd_wifi();
    else if (!strcmp(c, "safe"))     { safe_outputs(); OK(); }
    else if (!strcmp(c, "reboot"))   { out("rebooting\n"); OK(); vTaskDelay(pdMS_TO_TICKS(50)); esp_restart(); }
    else ERR("unknown command '%s' (try help)", c);
}

void app_main(void)
{
    /* 1. safety first, before the console or any peripheral */
    safe_outputs();
    cfg_inputs();

    /* 2. console over native USB serial/JTAG */
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.rx_buffer_size = 4096;   /* link benchmarks keep up to 2 kB in flight */
    ucfg.tx_buffer_size = 4096;
    usb_serial_jtag_driver_install(&ucfg);
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    esp_log_level_set("*", ESP_LOG_WARN);

    vTaskDelay(pdMS_TO_TICKS(300));
    out("\n==== %s ====\n", PROBE_VERSION);
    out("boot: gpio47=1 (inhibit) gpio48=0 (arm idle) step/dir=0\n");
    cmd_selftest();
    out("ready\n> ");

    char line[128];
    while (true) {
        if (fgets(line, sizeof line, stdin)) {
            dispatch(line);
            out("> ");
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
