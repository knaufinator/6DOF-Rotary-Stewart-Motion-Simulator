/*
 * Step/Dir Signal Analyzer — Test Harness Firmware
 * 
 * Runs on a second ESP32-S3-DevKitC to monitor and analyze step/dir output
 * from the 6DOF Stewart Platform controller board.
 *
 * Features:
 *   - GPIO ISR on all 6 STEP input pins (rising edge)
 *   - Per-motor: net position, total steps, step rate, timing stats, direction changes
 *   - JSON output over USB serial
 *   - ASCII command interface: STATUS, RESET, STREAM:1/0, MONITOR:N, HELP
 *
 * Pin mapping (from SN75175N receiver outputs or direct single-ended tap):
 *   STEP_IN[0..5] = GPIO 4, 5, 6, 7, 8, 9
 *   DIR_IN[0..5]  = GPIO 10, 11, 12, 13, 14, 17
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "led_strip.h"
#include <math.h>

static const char *TAG = "ANALYZER";

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

#define NUM_MOTORS          6
#define PCNT_MOTORS         4        /* ESP32-S3 has 4 PCNT units (M0-M3) */
#define REPORT_INTERVAL_MS  150      /* ~7 Hz streaming rate */
#define PCNT_POLL_MS        5        /* 200 Hz PCNT polling (prevents 16-bit overflow) */
#define IDLE_TIMEOUT_MS     1000     /* report idle after 1s of no steps */
#define MAX_CMD_LEN         64
#define LED_GPIO            48       /* WS2812 RGB on DevKitC */

/* GPIO pin assignments — match controller pin numbering for clarity */
static const gpio_num_t step_pins[NUM_MOTORS] = {
    GPIO_NUM_4,  GPIO_NUM_5,  GPIO_NUM_6,
    GPIO_NUM_7,  GPIO_NUM_8,  GPIO_NUM_9
};
static const gpio_num_t dir_pins[NUM_MOTORS] = {
    GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12,
    GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_17
};

/* -------------------------------------------------------------------------- */
/*  Per-motor statistics (updated in ISR)                                     */
/* -------------------------------------------------------------------------- */

#define RATE_HISTORY_LEN  8  /* rolling window samples (~1.2s at 150ms) */

typedef struct {
    volatile int32_t  position;          /* net position (direction-aware) */
    volatile uint32_t total_steps;       /* absolute step count */
    volatile uint32_t steps_in_window;   /* steps since last report (for rate calc) */
    volatile uint64_t last_step_time_us; /* timestamp of most recent step */
    volatile uint32_t min_interval_us;   /* shortest step-to-step interval */
    volatile uint32_t max_interval_us;   /* longest step-to-step interval */
    volatile uint64_t sum_interval_us;   /* running sum for average */
    volatile uint32_t interval_count;    /* number of interval samples */
    volatile uint32_t dir_changes;       /* direction reversals */
    volatile bool     last_dir;          /* previous direction level */
    /* Rolling rate history for smooth Hz display */
    uint32_t rate_history[RATE_HISTORY_LEN]; /* steps per window */
    int      rate_head;                     /* write index */
    int      rate_count;                    /* filled slots */
} motor_stats_t;

static motor_stats_t stats[NUM_MOTORS];

/* -------------------------------------------------------------------------- */
/*  Global state                                                              */
/* -------------------------------------------------------------------------- */

static volatile bool streaming = false;
static volatile int  monitor_motor = -1;   /* -1 = all, 0..5 = focused */

/* -------------------------------------------------------------------------- */
/*  PCNT hardware pulse counters (M0-M3)                                      */
/* -------------------------------------------------------------------------- */

static pcnt_unit_handle_t pcnt_units[PCNT_MOTORS] = {NULL};
static pcnt_channel_handle_t pcnt_channels[PCNT_MOTORS] = {NULL};

/* PCNT accumulator — polled at high rate to prevent 16-bit overflow.
 * PCNT counts direction-aware (STEP edge + DIR level), giving net position.
 * We accumulate abs(delta) for total_steps. */
static volatile int32_t  pcnt_position[PCNT_MOTORS];
static volatile uint32_t pcnt_total_steps[PCNT_MOTORS];
static volatile uint32_t pcnt_steps_in_window[PCNT_MOTORS];
static volatile uint64_t pcnt_last_active_us[PCNT_MOTORS];
static int16_t           pcnt_prev_count[PCNT_MOTORS];

static void init_pcnt(void)
{
    for (int i = 0; i < PCNT_MOTORS; i++) {
        pcnt_unit_config_t unit_cfg = {
            .low_limit  = -32768,
            .high_limit =  32767,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_units[i]));

        pcnt_chan_config_t chan_cfg = {
            .edge_gpio_num  = step_pins[i],
            .level_gpio_num = dir_pins[i],
        };
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[i], &chan_cfg, &pcnt_channels[i]));

        /* Rising edge: count +1 when DIR HIGH, -1 when DIR LOW */
        pcnt_channel_set_edge_action(pcnt_channels[i],
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
        pcnt_channel_set_level_action(pcnt_channels[i],
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        /* Enable glitch filter (1us = reject noise shorter than 1us) */
        pcnt_glitch_filter_config_t filt = { .max_glitch_ns = 1000 };
        pcnt_unit_set_glitch_filter(pcnt_units[i], &filt);

        pcnt_unit_enable(pcnt_units[i]);
        pcnt_unit_clear_count(pcnt_units[i]);
        pcnt_unit_start(pcnt_units[i]);

        pcnt_prev_count[i] = 0;
        pcnt_position[i] = 0;
        pcnt_total_steps[i] = 0;
        pcnt_steps_in_window[i] = 0;
        pcnt_last_active_us[i] = 0;

        ESP_LOGI(TAG, "PCNT unit %d: STEP=GPIO%d DIR=GPIO%d", i, step_pins[i], dir_pins[i]);
    }
}

/* High-frequency poll task — accumulates 16-bit PCNT into 32-bit counters */
static void pcnt_poll_task(void *arg)
{
    while (1) {
        for (int i = 0; i < PCNT_MOTORS; i++) {
            int count_val = 0;
            pcnt_unit_get_count(pcnt_units[i], &count_val);
            int16_t count = (int16_t)count_val;
            int16_t delta = count - pcnt_prev_count[i];
            pcnt_prev_count[i] = count;

            if (delta != 0) {
                pcnt_position[i] += delta;
                uint32_t abs_delta = (delta > 0) ? (uint32_t)delta : (uint32_t)(-delta);
                pcnt_total_steps[i] += abs_delta;
                pcnt_steps_in_window[i] += abs_delta;
                pcnt_last_active_us[i] = esp_timer_get_time();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(PCNT_POLL_MS));
    }
}

/* -------------------------------------------------------------------------- */
/*  ISR — Lightweight step handler for M4-M5 (no PCNT available)              */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR step_isr_handler(void *arg)
{
    uint32_t idx = (uint32_t)(uintptr_t)arg;
    motor_stats_t *s = &stats[idx];

    bool dir = gpio_get_level(dir_pins[idx]);
    if (s->total_steps > 0 && dir != s->last_dir)
        s->dir_changes++;
    s->last_dir = dir;

    if (dir) s->position++;
    else     s->position--;
    s->total_steps++;
    s->steps_in_window++;
    s->last_step_time_us = esp_timer_get_time();
}

/* -------------------------------------------------------------------------- */
/*  GPIO Initialization                                                       */
/* -------------------------------------------------------------------------- */

static void init_gpio(void)
{
    /* M0-M3: PCNT handles step/dir pins — configure DIR as input for PCNT level */
    for (int i = 0; i < PCNT_MOTORS; i++) {
        gpio_config_t dir_cfg = {
            .pin_bit_mask  = (1ULL << dir_pins[i]),
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_ENABLE,
            .intr_type     = GPIO_INTR_DISABLE
        };
        gpio_config(&dir_cfg);
    }

    /* M4-M5: GPIO ISR (no PCNT units left) */
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    for (int i = PCNT_MOTORS; i < NUM_MOTORS; i++) {
        gpio_config_t step_cfg = {
            .pin_bit_mask  = (1ULL << step_pins[i]),
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_ENABLE,
            .intr_type     = GPIO_INTR_POSEDGE
        };
        gpio_config(&step_cfg);

        gpio_config_t dir_cfg = {
            .pin_bit_mask  = (1ULL << dir_pins[i]),
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_ENABLE,
            .intr_type     = GPIO_INTR_DISABLE
        };
        gpio_config(&dir_cfg);

        gpio_isr_handler_add(step_pins[i], step_isr_handler, (void *)(uintptr_t)i);
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        memset((void *)&stats[i], 0, sizeof(motor_stats_t));
        stats[i].min_interval_us = UINT32_MAX;
    }

    ESP_LOGI(TAG, "GPIO initialized: M0-M3=PCNT hw, M4-M5=GPIO ISR");
}

/* -------------------------------------------------------------------------- */
/*  Stats helpers                                                             */
/* -------------------------------------------------------------------------- */

static void reset_all_stats(void)
{
    /* Reset PCNT motors (M0-M3) */
    for (int i = 0; i < PCNT_MOTORS; i++) {
        pcnt_unit_clear_count(pcnt_units[i]);
        pcnt_prev_count[i] = 0;
        pcnt_position[i] = 0;
        pcnt_total_steps[i] = 0;
        pcnt_steps_in_window[i] = 0;
        pcnt_last_active_us[i] = 0;
    }
    /* Reset ISR motors (M4-M5) + all stats structs */
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i >= PCNT_MOTORS) gpio_intr_disable(step_pins[i]);
        memset((void *)&stats[i], 0, sizeof(motor_stats_t));
        stats[i].min_interval_us = UINT32_MAX;
        if (i >= PCNT_MOTORS) gpio_intr_enable(step_pins[i]);
    }
}

/* -------------------------------------------------------------------------- */
/*  JSON Output                                                               */
/* -------------------------------------------------------------------------- */

static void print_motor_json(int idx, float report_period_sec)
{
    int32_t  pos;
    uint32_t steps;
    float    rate_hz;
    uint32_t min_us = 0, max_us = 0, avg_us = 0, dir_chg = 0;
    uint32_t idle_ms = 0;
    bool dir_level = gpio_get_level(dir_pins[idx]);
    uint64_t now = esp_timer_get_time();

    if (idx < PCNT_MOTORS) {
        /* PCNT motor — data from hardware accumulator */
        pos   = pcnt_position[idx];
        steps = pcnt_total_steps[idx];

        /* Rolling average rate from PCNT window data */
        motor_stats_t *s = &stats[idx];
        uint32_t rate_sum = 0;
        int rate_n = s->rate_count < RATE_HISTORY_LEN ? s->rate_count : RATE_HISTORY_LEN;
        for (int k = 0; k < rate_n; k++)
            rate_sum += s->rate_history[k];
        rate_hz = (rate_n > 0 && report_period_sec > 0.0f)
            ? (float)rate_sum / ((float)rate_n * report_period_sec) : 0.0f;

        /* Estimate avg interval from rate */
        if (rate_hz > 0) avg_us = (uint32_t)(1000000.0f / rate_hz);

        idle_ms = (pcnt_last_active_us[idx] > 0)
            ? (uint32_t)((now - pcnt_last_active_us[idx]) / 1000) : 0;
    } else {
        /* ISR motor (M4-M5) */
        motor_stats_t *s = &stats[idx];
        pos   = s->position;
        steps = s->total_steps;
        dir_chg = s->dir_changes;
        min_us = (s->min_interval_us == UINT32_MAX) ? 0 : s->min_interval_us;
        max_us = s->max_interval_us;
        avg_us = s->interval_count > 0
            ? (uint32_t)(s->sum_interval_us / s->interval_count) : 0;

        uint32_t rate_sum = 0;
        int rate_n = s->rate_count < RATE_HISTORY_LEN ? s->rate_count : RATE_HISTORY_LEN;
        for (int k = 0; k < rate_n; k++)
            rate_sum += s->rate_history[k];
        rate_hz = (rate_n > 0 && report_period_sec > 0.0f)
            ? (float)rate_sum / ((float)rate_n * report_period_sec) : 0.0f;

        idle_ms = (s->last_step_time_us > 0)
            ? (uint32_t)((now - s->last_step_time_us) / 1000) : 0;
    }

    printf("{\"id\":%d"
           ",\"pos\":%"PRId32
           ",\"steps\":%"PRIu32
           ",\"rate\":%.1f"
           ",\"dir\":%d"
           ",\"min_us\":%"PRIu32
           ",\"max_us\":%"PRIu32
           ",\"avg_us\":%"PRIu32
           ",\"dir_chg\":%"PRIu32
           ",\"idle_ms\":%"PRIu32
           "}",
           idx, pos, steps,
           rate_hz, dir_level ? 1 : 0,
           min_us, max_us, avg_us,
           dir_chg, idle_ms);
}

static void print_status(float report_period_sec)
{
    uint32_t t_ms = (uint32_t)(esp_timer_get_time() / 1000);
    printf("{\"t_ms\":%"PRIu32",\"motors\":[", t_ms);

    if (monitor_motor >= 0 && monitor_motor < NUM_MOTORS) {
        /* Single motor focus mode */
        print_motor_json(monitor_motor, report_period_sec);
    } else {
        /* All motors */
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i > 0) printf(",");
            print_motor_json(i, report_period_sec);
        }
    }

    printf("]}\n");
    fflush(stdout);

    /* Push window counts into rolling history, then reset */
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stats_t *s = &stats[i];
        uint32_t window;
        if (i < PCNT_MOTORS) {
            window = pcnt_steps_in_window[i];
            pcnt_steps_in_window[i] = 0;
        } else {
            window = s->steps_in_window;
            s->steps_in_window = 0;
        }
        s->rate_history[s->rate_head] = window;
        s->rate_head = (s->rate_head + 1) % RATE_HISTORY_LEN;
        if (s->rate_count < RATE_HISTORY_LEN) s->rate_count++;
    }
}

/* -------------------------------------------------------------------------- */
/*  Summary — compact one-line per motor (for quick visual check)             */
/* -------------------------------------------------------------------------- */

static void print_summary(void)
{
    printf("\n--- Analyzer Summary ---\n");
    printf("Motor | Position | Steps  | Rate(Hz) | MinUs | MaxUs | AvgUs | DirChg\n");
    printf("------+----------+--------+----------+-------+-------+-------+-------\n");

    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stats_t *s = &stats[i];
        uint32_t avg_us = s->interval_count > 0
            ? (uint32_t)(s->sum_interval_us / s->interval_count) : 0;
        uint32_t min_us = (s->min_interval_us == UINT32_MAX) ? 0 : s->min_interval_us;

        printf("  %d   | %8"PRId32" | %6"PRIu32" |          | %5"PRIu32" | %5"PRIu32" | %5"PRIu32" | %"PRIu32"\n",
               i, s->position, s->total_steps,
               min_us, s->max_interval_us, avg_us, s->dir_changes);
    }
    printf("\n");
    fflush(stdout);
}

/* -------------------------------------------------------------------------- */
/*  Command Processing                                                        */
/* -------------------------------------------------------------------------- */

static void process_command(const char *cmd)
{
    if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "STATUS?") == 0) {
        print_status(0.0f);
    }
    else if (strcmp(cmd, "SUMMARY") == 0) {
        print_summary();
    }
    else if (strcmp(cmd, "RESET") == 0) {
        reset_all_stats();
        printf("{\"ok\":\"stats_reset\"}\n");
        fflush(stdout);
    }
    else if (strcmp(cmd, "STREAM:1") == 0) {
        streaming = true;
        printf("{\"ok\":\"streaming_on\"}\n");
        fflush(stdout);
    }
    else if (strcmp(cmd, "STREAM:0") == 0) {
        streaming = false;
        printf("{\"ok\":\"streaming_off\"}\n");
        fflush(stdout);
    }
    else if (strncmp(cmd, "MONITOR:", 8) == 0) {
        int m = atoi(cmd + 8);
        if (m < 0 || m >= NUM_MOTORS) {
            monitor_motor = -1;  /* all motors */
            printf("{\"ok\":\"monitor_all\"}\n");
        } else {
            monitor_motor = m;
            printf("{\"ok\":\"monitor_%d\"}\n", m);
        }
        fflush(stdout);
    }
    else if (strcmp(cmd, "PINS") == 0 || strcmp(cmd, "PINS?") == 0) {
        /* Read raw GPIO levels for all STEP and DIR inputs — static wire test */
        printf("{\"pins\":{\"step\":[");
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i > 0) printf(",");
            printf("%d", gpio_get_level(step_pins[i]));
        }
        printf("],\"dir\":[");
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i > 0) printf(",");
            printf("%d", gpio_get_level(dir_pins[i]));
        }
        printf("]}}\n");
        fflush(stdout);
    }
    else if (strcmp(cmd, "HELP") == 0) {
        printf("Commands:\n");
        printf("  STATUS    — JSON dump of all motor stats\n");
        printf("  SUMMARY   — Human-readable table\n");
        printf("  PINS      — Read raw GPIO levels (STEP + DIR inputs)\n");
        printf("  RESET     — Clear all counters\n");
        printf("  STREAM:1  — Enable continuous JSON output (2 Hz)\n");
        printf("  STREAM:0  — Disable streaming\n");
        printf("  MONITOR:N — Focus on motor N (0-5), or -1 for all\n");
        printf("  HELP      — This message\n");
        fflush(stdout);
    }
    else if (strlen(cmd) > 0) {
        printf("{\"error\":\"unknown_command\",\"cmd\":\"%s\"}\n", cmd);
        fflush(stdout);
    }
}

/* -------------------------------------------------------------------------- */
/*  Serial Input Task                                                         */
/* -------------------------------------------------------------------------- */

static void serial_task(void *arg)
{
    char buf[MAX_CMD_LEN];
    int pos = 0;

    /* Set stdin to non-blocking */
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

    while (1) {
        uint8_t byte;
        int n = read(fileno(stdin), &byte, 1);

        if (n > 0) {
            if (byte == '\n' || byte == '\r' || byte == 'X') {
                if (pos > 0) {
                    buf[pos] = '\0';
                    process_command(buf);
                    pos = 0;
                }
            } else if (pos < MAX_CMD_LEN - 1) {
                buf[pos++] = (char)byte;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Report Task — Periodic streaming + idle detection                         */
/* -------------------------------------------------------------------------- */

static void report_task(void *arg)
{
    const float report_period_sec = REPORT_INTERVAL_MS / 1000.0f;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_INTERVAL_MS));

        if (streaming) {
            print_status(report_period_sec);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Activity Monitor Task — Detects ghost pulses and idle transitions         */
/* -------------------------------------------------------------------------- */

static void activity_task(void *arg)
{
    bool was_active[NUM_MOTORS] = {false};

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));  /* 10 Hz check */

        uint64_t now = esp_timer_get_time();

        for (int i = 0; i < NUM_MOTORS; i++) {
            bool is_active = false;

            if (stats[i].last_step_time_us > 0) {
                uint32_t idle_ms = (uint32_t)((now - stats[i].last_step_time_us) / 1000);
                is_active = (idle_ms < IDLE_TIMEOUT_MS);
            }

            /* Transition detection */
            if (was_active[i] && !is_active) {
                /* Motor just went idle — report final stats */
                if (streaming) {
                    printf("{\"event\":\"idle\",\"motor\":%d"
                           ",\"pos\":%"PRId32
                           ",\"steps\":%"PRIu32"}\n",
                           i, stats[i].position, stats[i].total_steps);
                    fflush(stdout);
                }
            }
            else if (!was_active[i] && is_active) {
                /* Motor just started stepping */
                if (streaming) {
                    printf("{\"event\":\"active\",\"motor\":%d}\n", i);
                    fflush(stdout);
                }
            }

            was_active[i] = is_active;
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  LED — Orange breathe to distinguish from Controller (green)               */
/* -------------------------------------------------------------------------- */

static led_strip_handle_t led_strip = NULL;

static void init_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags = { .with_dma = false },
    };
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed (0x%x)", ret);
    } else {
        led_strip_clear(led_strip);
        ESP_LOGI(TAG, "LED initialized on GPIO%d (orange = test harness)", LED_GPIO);
    }
}

static void led_task(void *arg)
{
    /* Orange breathe: visually distinct from controller's green breathe */
    for (;;) {
        if (!led_strip) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        for (int i = 0; i < 40; i++) {
            float b = (sinf((float)i / 40.0f * 2.0f * M_PI - M_PI / 2.0f) + 1.0f) / 2.0f;
            if (b < 0.05f) b = 0.05f;
            /* Orange = R:50, G:20, B:0 (warm amber) */
            led_strip_set_pixel(led_strip, 0,
                (uint8_t)(50 * b), (uint8_t)(20 * b), 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Entry Point                                                               */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║   Step/Dir Signal Analyzer v2.0         ║\n");
    printf("║   6DOF Stewart Platform Test Harness    ║\n");
    printf("║   M0-M3: PCNT hw  |  M4-M5: GPIO ISR   ║\n");
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");
    printf("Channels: %d motors (%d PCNT + %d ISR)\n",
           NUM_MOTORS, PCNT_MOTORS, NUM_MOTORS - PCNT_MOTORS);
    printf("STEP pins: GPIO %d, %d, %d, %d, %d, %d\n",
           step_pins[0], step_pins[1], step_pins[2],
           step_pins[3], step_pins[4], step_pins[5]);
    printf("DIR  pins: GPIO %d, %d, %d, %d, %d, %d\n",
           dir_pins[0], dir_pins[1], dir_pins[2],
           dir_pins[3], dir_pins[4], dir_pins[5]);
    printf("\n");
    printf("Commands: STATUS, SUMMARY, RESET, STREAM:1/0, MONITOR:N, HELP\n");
    printf("\n");
    fflush(stdout);

    init_pcnt();
    init_gpio();
    init_led();

    /* Launch tasks */
    xTaskCreatePinnedToCore(pcnt_poll_task, "pcnt_poll", 2048, NULL, 6, NULL, 1);  /* highest prio */
    xTaskCreatePinnedToCore(serial_task,    "serial",    4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(report_task,    "report",    4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(activity_task,  "activity",  2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(led_task,       "led",       2048, NULL, 1, NULL, 0);

    printf("Analyzer ready. Waiting for step/dir signals...\n\n");
    fflush(stdout);
}
