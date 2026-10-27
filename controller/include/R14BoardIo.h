#pragma once
#include <cstddef>
#include <cstdint>

namespace r14 {

// Current 319-part C4 master. GPB4 is FIELD_EFUSE_FLT_N, NOT boost BAD.
// AUX15V_BAD clears the six torque latches in hardware; there is no dedicated
// MCU readback of that signal. GPB6 reports aggregate logic-Q, not actual torque.
struct BoardInputs {
    uint64_t sampled_us = 0;
    uint8_t hall_closed_mask = 0;
    uint8_t alarm_mask = 0x3f; // GPIO HIGH = alarm/off/open on the measured R13 interface
    uint16_t driver_5v_mv = 0;
    bool expander_ok = false;
    bool adc_ok = false;
    bool field_good = false;
    bool field_fault = true;
    bool pulse_enable = false;
    bool torque_q_all = false;
    bool fresh(uint64_t now_us, uint64_t max_age_us) const {
        return expander_ok && adc_ok && now_us >= sampled_us &&
               now_us - sampled_us <= max_age_us;
    }
};

class RegisterBus {
public:
    virtual ~RegisterBus() {}
    virtual bool write(uint8_t address, uint8_t reg, const uint8_t* data, size_t size) = 0;
    virtual bool read(uint8_t address, uint8_t reg, uint8_t* data, size_t size) = 0;
};

class BoardIo {
public:
    static const uint8_t kExpander = 0x20;
    static const uint8_t kAdc = 0x40;
    explicit BoardIo(RegisterBus& bus) : bus_(bus) {}

    bool initialize() {
        initialized_ = false;
        // Recover BANK=0 even after an MCU-only reset. At BANK=0, 0x05 is
        // GPINTENB; at BANK=1 it is IOCON. No output is driven by this write.
        if (!put(0x05, 0) || !put(0x0a, 0) || !put(0x0b, 0)) return false;
        // Establish arm-clock LOW, field OFF, clear asserted BEFORE directions.
        // GPA7 and GPB7 are output-only on current MCP23017 silicon.
        if (!put(0x15, 0) || !put(0x14, 0) || !put(0x01, 0x7d) ||
            !put(0x00, 0x7f) || !put(0x02, 0) || !put(0x03, 0) ||
            !put(0x0c, 0) || !put(0x0d, 0)) return false;
        uint8_t dirs[2], latches[2];
        if (!bus_.read(kExpander, 0x00, dirs, 2) || dirs[0] != 0x7f || dirs[1] != 0x7d ||
            !bus_.read(kExpander, 0x14, latches, 2) || latches[0] || latches[1]) return false;
        uint8_t id[2], die[2];
        if (!bus_.read(kAdc, 0xfe, id, 2) || id[0] != 0x54 || id[1] != 0x49 ||
            !bus_.read(kAdc, 0xff, die, 2) || die[0] != 0x22 || (die[1] & 0xf0) != 0x60)
            return false;
        // Bus-voltage continuous, 1.1 ms conversion, AVG=1. IN+/IN- are GND:
        // this board cannot measure motor/board current with its INA226.
        const uint8_t config[] = {0x41, 0x26};
        initialized_ = bus_.write(kAdc, 0, config, 2);
        return initialized_;
    }

    BoardInputs sample(uint64_t now_us, uint8_t alarm_high_mask) {
        BoardInputs in;
        in.sampled_us = now_us;
        in.alarm_mask = alarm_high_mask & 0x3f;
        if (!initialized_) return in;
        uint8_t pins[2], voltage[2], ready[2];
        in.expander_ok = bus_.read(kExpander, 0x12, pins, 2);
        in.adc_ok = bus_.read(kAdc, 0x06, ready, 2) && (ready[1] & 0x08) &&
                    bus_.read(kAdc, 0x02, voltage, 2);
        if (in.expander_ok) {
            in.hall_closed_mask = static_cast<uint8_t>(~pins[0]) & 0x3f;
            in.field_good = (pins[0] & 0x40) != 0;
            in.field_fault = (pins[1] & 0x10) == 0;
            in.pulse_enable = (pins[1] & 0x20) != 0;
            in.torque_q_all = (pins[1] & 0x40) != 0;
        }
        if (in.adc_ok) {
            const uint32_t raw = (uint32_t(voltage[0]) << 8) | voltage[1];
            const uint32_t mv = (raw * 5u + 2u) / 4u;
            if (mv > 36000) in.adc_ok = false;
            else in.driver_5v_mv = static_cast<uint16_t>(mv);
        }
        return in;
    }

    bool set_field_feed(bool enabled) {
        // Does not release torque clear or pulse the common hardware arm.
        return initialized_ && put(0x15, enabled ? 0x02 : 0);
    }

private:
    bool put(uint8_t reg, uint8_t value) { return bus_.write(kExpander, reg, &value, 1); }
    RegisterBus& bus_;
    bool initialized_ = false;
};
} // namespace r14
