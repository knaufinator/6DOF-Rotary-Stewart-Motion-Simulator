// MCP23S17 SPI GPIO Expander — ESP-IDF native implementation
// See MCP23S17.h for usage notes.

#include "MCP23S17.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "MCP23S17";

// Static member: track per-host bus init state
bool MCP23S17::_busInitialized[3] = { false, false, false };

bool MCP23S17::begin(int mosi, int miso, int sclk, int clock_hz) {
    // Initialize SPI bus if this host hasn't been set up yet
    if (!_busInitialized[_host]) {
        spi_bus_config_t bus_cfg = {};
        bus_cfg.mosi_io_num = mosi;
        bus_cfg.miso_io_num = miso;
        bus_cfg.sclk_io_num = sclk;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = 4;

        esp_err_t ret = spi_bus_initialize(_host, &bus_cfg, SPI_DMA_DISABLED);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
            return false;
        }
        _busInitialized[_host] = true;
    }

    // Add this MCP23S17 as a device on the bus
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = clock_hz;
    dev_cfg.mode = 0;              // SPI Mode 0 (CPOL=0, CPHA=0)
    dev_cfg.spics_io_num = _cs;
    dev_cfg.queue_size = 1;

    esp_err_t ret = spi_bus_add_device(_host, &dev_cfg, &_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Enable Hardware Address Enable (HAEN) so multiple chips can share CS
    // Write to address 0 first (all chips respond when HAEN is disabled)
    _writeReg(MCP_IOCON, MCP_IOCON_HAEN);

    // Default: all pins as output (for motor step/dir control)
    _writeReg(MCP_IODIRA, 0x00);
    _writeReg(MCP_IODIRB, 0x00);

    // Clear all outputs
    _writeReg(MCP_OLATA, 0x00);
    _writeReg(MCP_OLATB, 0x00);

    ESP_LOGI(TAG, "MCP23S17 addr=%d on SPI%d CS=%d @ %dHz OK",
        _addr, _host, _cs, clock_hz);
    return true;
}

void MCP23S17::setDirection(uint16_t dir) {
    _writeReg(MCP_IODIRA, dir & 0xFF);
    _writeReg(MCP_IODIRB, (dir >> 8) & 0xFF);
}

void MCP23S17::writeAll(uint16_t value) {
    // Write both ports in two register writes (port A = low byte, port B = high byte)
    _writeReg(MCP_OLATA, value & 0xFF);
    _writeReg(MCP_OLATB, (value >> 8) & 0xFF);
}

uint16_t MCP23S17::readAll() {
    uint8_t a = _readReg(MCP_GPIOA);
    uint8_t b = _readReg(MCP_GPIOB);
    return (uint16_t)a | ((uint16_t)b << 8);
}

void MCP23S17::_writeReg(uint8_t reg, uint8_t val) {
    uint8_t tx[3] = { MCP_OPCODE_WRITE(_addr), reg, val };
    spi_transaction_t t = {};
    t.length = 24;  // 3 bytes × 8 bits
    t.tx_buffer = tx;
    spi_device_polling_transmit(_dev, &t);
}

uint8_t MCP23S17::_readReg(uint8_t reg) {
    uint8_t tx[3] = { MCP_OPCODE_READ(_addr), reg, 0x00 };
    uint8_t rx[3] = {0};
    spi_transaction_t t = {};
    t.length = 24;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.rxlength = 24;
    spi_device_polling_transmit(_dev, &t);
    return rx[2];
}
