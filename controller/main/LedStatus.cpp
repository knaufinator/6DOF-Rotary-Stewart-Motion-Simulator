#include "LedStatus.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "led_status";

// ── Color definitions (R, G, B) — kept dim to avoid blinding ────────
typedef struct { uint8_t r, g, b; } rgb_t;

// Entity color — set at runtime by app handshake via LED:R,G,B command
static rgb_t entity_color = {  0,  40,   0};  // default: same as READY green

static const rgb_t state_colors[LED_STATE_COUNT] = {
    [LED_STATE_OFF]            = {  0,   0,   0},
    [LED_STATE_BOOT]           = { 40,  40,  40},  // white
    [LED_STATE_READY]          = {  0,  40,   0},  // green
    [LED_STATE_ENTITY]         = {  0,  40,   0},  // placeholder — overridden by entity_color
    [LED_STATE_COMMS_ACTIVE]   = {  0,   0,  40},  // blue
    [LED_STATE_MOTORS_ACTIVE]  = {  0,  30,  30},  // cyan
    [LED_STATE_CONFIG]         = { 30,   0,  30},  // purple
    [LED_STATE_WARN_POSITION]  = { 40,  30,   0},  // yellow
    [LED_STATE_WARN_COMMS]     = {  0,   0,  50},  // bright blue
    [LED_STATE_ESTOP]          = { 60,   0,   0},  // red
    [LED_STATE_ERROR]          = { 80,   0,   0},  // bright red
};

static const LedPattern state_patterns[LED_STATE_COUNT] = {
    [LED_STATE_OFF]            = LED_PAT_SOLID,
    [LED_STATE_BOOT]           = LED_PAT_SOLID,
    [LED_STATE_READY]          = LED_PAT_BREATHE,
    [LED_STATE_ENTITY]         = LED_PAT_BREATHE,
    [LED_STATE_COMMS_ACTIVE]   = LED_PAT_SLOW_BLINK,
    [LED_STATE_MOTORS_ACTIVE]  = LED_PAT_SOLID,
    [LED_STATE_CONFIG]         = LED_PAT_BREATHE,
    [LED_STATE_WARN_POSITION]  = LED_PAT_DOUBLE_PULSE,
    [LED_STATE_WARN_COMMS]     = LED_PAT_FAST_BLINK,
    [LED_STATE_ESTOP]          = LED_PAT_FAST_BLINK,
    [LED_STATE_ERROR]          = LED_PAT_TRIPLE_PULSE,
};

// ── State tracking ──────────────────────────────────────────────────
static volatile uint32_t active_states = 0;  // bitmask of active LedState flags
static led_strip_handle_t strip = NULL;

// Get highest priority active state
static LedState get_current_state(void) {
    uint32_t bits = active_states;
    if (bits == 0) return LED_STATE_OFF;
    // Highest bit = highest priority (states ordered by priority in enum)
    for (int i = LED_STATE_COUNT - 1; i >= 0; i--) {
        if (bits & (1 << i)) return (LedState)i;
    }
    return LED_STATE_OFF;
}

// ── Pattern rendering ───────────────────────────────────────────────

static void set_pixel(rgb_t c, float brightness) {
    uint8_t r = (uint8_t)(c.r * brightness);
    uint8_t g = (uint8_t)(c.g * brightness);
    uint8_t b = (uint8_t)(c.b * brightness);
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}

static void set_off(void) {
    led_strip_clear(strip);
    led_strip_refresh(strip);
}

// Render one cycle of the current pattern. Returns when cycle completes
// so we can re-check state between cycles.
static void render_cycle(LedState state) {
    rgb_t color = (state == LED_STATE_ENTITY) ? entity_color : state_colors[state];
    LedPattern pat = state_patterns[state];

    switch (pat) {
    case LED_PAT_SOLID:
        set_pixel(color, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;

    case LED_PAT_BREATHE:
        // Sine-wave breathe over ~2 seconds (40 steps x 50ms)
        for (int i = 0; i < 40; i++) {
            float b = (sinf((float)i / 40.0f * 2.0f * M_PI - M_PI / 2.0f) + 1.0f) / 2.0f;
            if (b < 0.05f) b = 0.05f;  // never fully off
            set_pixel(color, b);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        break;

    case LED_PAT_SLOW_BLINK:
        set_pixel(color, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(500));
        set_off();
        vTaskDelay(pdMS_TO_TICKS(500));
        break;

    case LED_PAT_FAST_BLINK:
        set_pixel(color, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_off();
        vTaskDelay(pdMS_TO_TICKS(100));
        break;

    case LED_PAT_DOUBLE_PULSE:
        set_pixel(color, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_off();
        vTaskDelay(pdMS_TO_TICKS(100));
        set_pixel(color, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_off();
        vTaskDelay(pdMS_TO_TICKS(700));
        break;

    case LED_PAT_TRIPLE_PULSE:
        for (int p = 0; p < 3; p++) {
            set_pixel(color, 1.0f);
            vTaskDelay(pdMS_TO_TICKS(80));
            set_off();
            vTaskDelay(pdMS_TO_TICKS(80));
        }
        vTaskDelay(pdMS_TO_TICKS(700));
        break;
    }
}

// ── LED task ────────────────────────────────────────────────────────
static void LedStatusTask(void *pvParameters) {
    ESP_LOGI(TAG, "LED status task started on GPIO%d", (int)(intptr_t)pvParameters);

    for (;;) {
        LedState state = get_current_state();
        if (state == LED_STATE_OFF) {
            set_off();
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            render_cycle(state);
        }
    }
}

// ── Public API ──────────────────────────────────────────────────────

void led_status_init(int gpio_num) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_num,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags = { .with_dma = false },
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed on GPIO%d (0x%x)", gpio_num, ret);
        return;
    }
    led_strip_clear(strip);

    // Start with BOOT state
    active_states = (1 << LED_STATE_BOOT);

    xTaskCreatePinnedToCore(LedStatusTask, "LedStatus", 2048,
                            (void*)(intptr_t)gpio_num, 1, NULL, 0);
}

void led_status_set(LedState state) {
    if (state > LED_STATE_OFF && state < LED_STATE_COUNT) {
        active_states |= (1 << state);
    }
}

void led_status_clear(LedState state) {
    if (state > LED_STATE_OFF && state < LED_STATE_COUNT) {
        active_states &= ~(1 << state);
    }
}

void led_status_reset(void) {
    active_states = (1 << LED_STATE_READY);
}

void led_status_set_entity_color(uint8_t r, uint8_t g, uint8_t b) {
    // Scale to dim range (max ~60) to avoid blinding, preserve hue
    float max_ch = (float)(r > g ? (r > b ? r : b) : (g > b ? g : b));
    float scale = (max_ch > 0) ? 50.0f / max_ch : 1.0f;
    if (scale > 1.0f) scale = 1.0f;  // don't amplify dark colors
    entity_color.r = (uint8_t)(r * scale);
    entity_color.g = (uint8_t)(g * scale);
    entity_color.b = (uint8_t)(b * scale);
    // Activate entity state (overrides READY breathe)
    led_status_set(LED_STATE_ENTITY);
}
