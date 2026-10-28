#include "R14Hardware.h"
#include "R14BoardIo.h"
#include "CobsTransport.h"
#include "debug_uart.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <cstring>
#include <atomic>

extern "C" void r14_report_normalized_drive_alarm(int axis);
extern "C" void r14_report_transport_reconnect(void);

namespace {
const gpio_num_t kAlarmPins[] = {GPIO_NUM_1,GPIO_NUM_2,GPIO_NUM_15,GPIO_NUM_16,GPIO_NUM_18,GPIO_NUM_40};
constexpr int kTimeoutMs = 3;
class IdfBus : public r14::RegisterBus {
public:
    bool initialize() {
        i2c_master_bus_config_t config = {};
        config.i2c_port = I2C_NUM_0;
        config.sda_io_num = GPIO_NUM_41;
        config.scl_io_num = GPIO_NUM_42;
        config.clk_source = I2C_CLK_SRC_DEFAULT;
        config.glitch_ignore_cnt = 7;
        if (i2c_new_master_bus(&config, &bus_) != ESP_OK) return false;
        i2c_device_config_t device = {};
        device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        device.scl_speed_hz = 100000; // external board pullups; qualify 400 kHz separately
        device.device_address = 0x20;
        if (i2c_master_bus_add_device(bus_, &device, &iox_) != ESP_OK) return false;
        device.device_address = 0x40;
        return i2c_master_bus_add_device(bus_, &device, &adc_) == ESP_OK;
    }
    bool write(uint8_t address, uint8_t reg, const uint8_t* data, size_t n) override {
        if (n > 2 || !handle(address)) return false;
        uint8_t bytes[3] = {reg, 0, 0};
        std::memcpy(bytes + 1, data, n);
        return i2c_master_transmit(handle(address), bytes, n + 1, kTimeoutMs) == ESP_OK;
    }
    bool read(uint8_t address, uint8_t reg, uint8_t* data, size_t n) override {
        return handle(address) && i2c_master_transmit_receive(handle(address), &reg, 1, data, n, kTimeoutMs) == ESP_OK;
    }
private:
    i2c_master_dev_handle_t handle(uint8_t address) { return address == 0x20 ? iox_ : address == 0x40 ? adc_ : nullptr; }
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t iox_ = nullptr, adc_ = nullptr;
};
IdfBus bus;
r14::BoardIo io(bus);
r14::BoardInputs latest;
SemaphoreHandle_t mutex = nullptr;
std::atomic<int> field_request{-1};

uint8_t alarms() {
    uint8_t bits = 0;
    for (int i = 0; i < 6; ++i) if (gpio_get_level(kAlarmPins[i])) bits |= 1u << i;
    return bits;
}
void task(void*) {
    gpio_config_t pins = {};
    for (gpio_num_t pin : kAlarmPins) pins.pin_bit_mask |= 1ULL << pin;
    pins.mode = GPIO_MODE_INPUT; pins.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pins); // external pullups; HIGH means absent/fault
    const bool initialized = bus.initialize() && io.initialize();
    vTaskDelay(pdMS_TO_TICKS(3) ? pdMS_TO_TICKS(3) : 1); // first bus conversion
    uint8_t reported = 0;
    bool io_fault_reported = false;
    for (;;) {
        const uint8_t mask = alarms(); // independent of I2C transaction success
        for (int i = 0; i < 6; ++i) if ((mask & (1u << i)) && !(reported & (1u << i)))
            r14_report_normalized_drive_alarm(i);
        reported |= mask;
        const int request = field_request.exchange(-1);
        bool write_ok = request < 0 || io.set_field_feed(request != 0);
        r14::BoardInputs in = io.sample(esp_timer_get_time(), mask);
        const bool healthy = initialized && write_ok && in.expander_ok && in.adc_ok;
        // Any I/O failure invalidates pending admission; never automatically retry
        // initialization or release torque clear on a recovered bus.
        if (!healthy && !io_fault_reported) {
            r14_report_transport_reconnect();
            io_fault_reported = true;
        }
        if (mutex && xSemaphoreTake(mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            latest = in;
            xSemaphoreGive(mutex);
        }
        TickType_t delay = pdMS_TO_TICKS(5);
        vTaskDelay(delay ? delay : 1);
    }
}
}

void r14_hardware_start() {
    mutex = xSemaphoreCreateMutex();
    if (!mutex || xTaskCreatePinnedToCore(task,"R14IO",4096,nullptr,4,nullptr,0) != pdPASS)
        r14_report_transport_reconnect();
}

bool r14_hardware_command(const char* command, int source) {
    if (std::strncmp(command,"R14:",4)) return false;
    if (source != COBS_SRC_SERIAL) { serial_printf("R14:ERR local console required\r\n"); return true; }
    if (!std::strcmp(command,"R14:STATUS")) {
        r14::BoardInputs in;
        if (!mutex || xSemaphoreTake(mutex,pdMS_TO_TICKS(5)) != pdTRUE) {
            serial_printf("R14:ERR monitor unavailable\r\n"); return true;
        }
        in = latest; xSemaphoreGive(mutex);
        serial_printf("R14:iox=%d adc=%d fresh=%d hall_nc=0x%02x alarms=0x%02x field_good_local=%d field_fault=%d pulse_enable=%d torque_q_all=%d vcc5_mv=%u motion_qualified=0\r\n",
                      in.expander_ok,in.adc_ok,in.fresh(esp_timer_get_time(),50000),in.hall_closed_mask,in.alarm_mask,
                      in.field_good,in.field_fault,in.pulse_enable,in.torque_q_all,in.driver_5v_mv);
    } else if (!std::strcmp(command,"R14:FIELD=0") || !std::strcmp(command,"R14:FIELD=1")) {
        // Current firmware always inhibits pulses and asserts torque clear.
        field_request.store(command[10] == '1' ? 1 : 0);
        serial_printf("R14:FIELD request queued; verify R14:STATUS and remote voltage\r\n");
    } else {
        serial_printf("R14:ERR STATUS / FIELD=0|1 only; arm/home blocked pending physical qualification\r\n");
    }
    return true;
}
