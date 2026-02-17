#pragma once

// MCP23S17 SPI GPIO Expander — ESP-IDF native driver
// Used on PCBv1 (original ESP32 DevKit board) for step/dir motor control.
// The original PCB routes all 6 step + 6 direction signals through a single
// MCP23S17 on the HSPI bus (SPI2_HOST), allowing batch writes for all motors
// in a single SPI transaction.

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

// MCP23S17 register addresses (IOCON.BANK = 0, sequential mode)
#define MCP_IODIRA   0x00   // Port A direction (1=input, 0=output)
#define MCP_IODIRB   0x01   // Port B direction
#define MCP_IOCON    0x0A   // Configuration register
#define MCP_GPIOA    0x12   // Port A GPIO read
#define MCP_GPIOB    0x13   // Port B GPIO read
#define MCP_OLATA    0x14   // Port A output latch
#define MCP_OLATB    0x15   // Port B output latch

// SPI command byte: 0b0100 AAA W  (A=hw address, W=0 write, 1 read)
#define MCP_OPCODE_WRITE(addr) ((uint8_t)(0x40 | (((addr) & 0x07) << 1)))
#define MCP_OPCODE_READ(addr)  ((uint8_t)(0x41 | (((addr) & 0x07) << 1)))

// IOCON bits
#define MCP_IOCON_HAEN   0x08   // Hardware Address Enable

class MCP23S17 {
public:
    /**
     * @param host     SPI host (SPI2_HOST = HSPI on ESP32)
     * @param cs       Chip-select GPIO pin
     * @param hw_addr  Hardware address (0-7, matches A0-A2 pins on chip)
     */
    MCP23S17(spi_host_device_t host, gpio_num_t cs, uint8_t hw_addr = 0)
        : _host(host), _cs(cs), _addr(hw_addr), _dev(nullptr) {}

    /**
     * Initialize SPI bus (if not already) and configure the MCP23S17.
     * Enables hardware addressing (HAEN) so multiple chips can share one CS.
     * @param mosi  MOSI pin (default 13 = HSPI default on ESP32)
     * @param miso  MISO pin (default 12 = HSPI default on ESP32)
     * @param sclk  SCLK pin (default 14 = HSPI default on ESP32)
     * @param clock_hz  SPI clock frequency (default 8MHz, max 10MHz for MCP23S17)
     */
    bool begin(int mosi = 13, int miso = 12, int sclk = 14, int clock_hz = 8000000);

    /** Set all 16 pin directions at once. bit=1 → input, bit=0 → output. */
    void setDirection(uint16_t dir);

    /** Set all pin directions to output (convenience for motor control). */
    void allOutput() { setDirection(0x0000); }

    /** Write all 16 output pins at once (pins 0-7 = port A, 8-15 = port B). */
    void writeAll(uint16_t value);

    /** Read all 16 GPIO pins at once. */
    uint16_t readAll();

private:
    void _writeReg(uint8_t reg, uint8_t val);
    uint8_t _readReg(uint8_t reg);

    spi_host_device_t _host;
    gpio_num_t _cs;
    uint8_t _addr;
    spi_device_handle_t _dev;

    // Track whether the SPI bus has been initialized (shared across instances)
    static bool _busInitialized[3];  // indexed by spi_host_device_t
};
