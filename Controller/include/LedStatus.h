#pragma once

#include <stdint.h>

// ── LED Status Indicator System ─────────────────────────────────────
// Single RGB WS2812 on GPIO48 encodes system state via color + pattern.
// Priority-based: higher states override lower.
// Thread-safe: any task can call led_status_set() at any time.

// Blink patterns (timing defined in implementation)
enum LedPattern {
    LED_PAT_SOLID,          // Always on
    LED_PAT_BREATHE,        // Smooth fade in/out, ~2s cycle
    LED_PAT_SLOW_BLINK,     // 500ms on / 500ms off  (1Hz)
    LED_PAT_FAST_BLINK,     // 100ms on / 100ms off  (5Hz)
    LED_PAT_DOUBLE_PULSE,   // 2 quick flashes, pause
    LED_PAT_TRIPLE_PULSE,   // 3 quick flashes, pause
};

// System states — ordered by priority (highest = most urgent)
enum LedState {
    LED_STATE_OFF = 0,          // LED off
    LED_STATE_BOOT,             // White solid       — initializing
    LED_STATE_READY,            // Green breathe     — idle, waiting for commands
    LED_STATE_ENTITY,           // App color breathe — connected, showing entity color
    LED_STATE_COMMS_ACTIVE,     // Blue slow blink   — receiving serial/WiFi/BLE data
    LED_STATE_MOTORS_ACTIVE,    // Cyan solid        — motors stepping
    LED_STATE_CONFIG,           // Purple breathe    — configuration/setup mode
    LED_STATE_WARN_POSITION,    // Yellow double pulse — position error / missed steps
    LED_STATE_WARN_COMMS,       // Blue fast blink   — comms timeout
    LED_STATE_ESTOP,            // Red fast blink    — emergency stop active
    LED_STATE_ERROR,            // Red triple pulse  — hardware/fatal error

    LED_STATE_COUNT             // must be last
};

// Initialize the LED status system (call once from app_main)
void led_status_init(int gpio_num);

// Set the current LED state (thread-safe, highest priority wins)
void led_status_set(LedState state);

// Clear a specific state (e.g., motors stopped → clear MOTORS_ACTIVE)
void led_status_clear(LedState state);

// Clear all states and return to READY
void led_status_reset(void);

// Set custom entity color (from app handshake). Activates LED_STATE_ENTITY.
void led_status_set_entity_color(uint8_t r, uint8_t g, uint8_t b);
