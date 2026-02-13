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
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "ANALYZER";

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

#define NUM_MOTORS          6
#define REPORT_INTERVAL_MS  500      /* 2 Hz streaming rate */
#define IDLE_TIMEOUT_MS     1000     /* report idle after 1s of no steps */
#define MAX_CMD_LEN         64

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
} motor_stats_t;

static motor_stats_t stats[NUM_MOTORS];

/* -------------------------------------------------------------------------- */
/*  Global state                                                              */
/* -------------------------------------------------------------------------- */

static volatile bool streaming = false;
static volatile int  monitor_motor = -1;   /* -1 = all, 0..5 = focused */

/* -------------------------------------------------------------------------- */
/*  ISR — Step pulse handler                                                  */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR step_isr_handler(void *arg)
{
    uint32_t idx = (uint32_t)(uintptr_t)arg;
    motor_stats_t *s = &stats[idx];
    uint64_t now = esp_timer_get_time();

    /* Read direction pin level */
    bool dir = gpio_get_level(dir_pins[idx]);

    /* Track direction changes */
    if (s->total_steps > 0 && dir != s->last_dir) {
        s->dir_changes++;
    }
    s->last_dir = dir;

    /* Update position (DIR HIGH = forward, LOW = reverse) */
    if (dir) {
        s->position++;
    } else {
        s->position--;
    }
    s->total_steps++;
    s->steps_in_window++;

    /* Timing statistics */
    if (s->last_step_time_us > 0) {
        uint32_t interval = (uint32_t)(now - s->last_step_time_us);
        if (interval < s->min_interval_us) s->min_interval_us = interval;
        if (interval > s->max_interval_us) s->max_interval_us = interval;
        s->sum_interval_us += interval;
        s->interval_count++;
    }
    s->last_step_time_us = now;
}

/* -------------------------------------------------------------------------- */
/*  GPIO Initialization                                                       */
/* -------------------------------------------------------------------------- */

static void init_gpio(void)
{
    /* Configure STEP pins: input, pull-down, rising-edge interrupt */
    for (int i = 0; i < NUM_MOTORS; i++) {
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
    }

    /* Install ISR service with IRAM allocation for low latency */
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_isr_handler_add(step_pins[i], step_isr_handler, (void *)(uintptr_t)i);

        /* Initialize stats */
        memset((void *)&stats[i], 0, sizeof(motor_stats_t));
        stats[i].min_interval_us = UINT32_MAX;
    }

    ESP_LOGI(TAG, "GPIO initialized: %d STEP + %d DIR inputs", NUM_MOTORS, NUM_MOTORS);
}

/* -------------------------------------------------------------------------- */
/*  Stats helpers                                                             */
/* -------------------------------------------------------------------------- */

static void reset_all_stats(void)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_intr_disable(step_pins[i]);
        memset((void *)&stats[i], 0, sizeof(motor_stats_t));
        stats[i].min_interval_us = UINT32_MAX;
        gpio_intr_enable(step_pins[i]);
    }
}

/* -------------------------------------------------------------------------- */
/*  JSON Output                                                               */
/* -------------------------------------------------------------------------- */

static void print_motor_json(int idx, float report_period_sec)
{
    motor_stats_t *s = &stats[idx];

    uint32_t avg_us = s->interval_count > 0
        ? (uint32_t)(s->sum_interval_us / s->interval_count) : 0;

    float rate_hz = (report_period_sec > 0.0f)
        ? (float)s->steps_in_window / report_period_sec : 0.0f;

    uint32_t min_us = (s->min_interval_us == UINT32_MAX) ? 0 : s->min_interval_us;

    /* Idle time: ms since last step */
    uint64_t now = esp_timer_get_time();
    uint32_t idle_ms = (s->last_step_time_us > 0)
        ? (uint32_t)((now - s->last_step_time_us) / 1000) : 0;

    bool dir_level = gpio_get_level(dir_pins[idx]);

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
           idx, s->position, s->total_steps,
           rate_hz, dir_level ? 1 : 0,
           min_us, s->max_interval_us, avg_us,
           s->dir_changes, idle_ms);
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

    /* Reset windowed step counts for next rate calculation */
    for (int i = 0; i < NUM_MOTORS; i++) {
        stats[i].steps_in_window = 0;
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
    else if (strcmp(cmd, "HELP") == 0) {
        printf("Commands:\n");
        printf("  STATUS    — JSON dump of all motor stats\n");
        printf("  SUMMARY   — Human-readable table\n");
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
/*  Entry Point                                                               */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║   Step/Dir Signal Analyzer v1.0         ║\n");
    printf("║   6DOF Stewart Platform Test Harness    ║\n");
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");
    printf("Channels: %d motors\n", NUM_MOTORS);
    printf("STEP pins: GPIO %d, %d, %d, %d, %d, %d\n",
           step_pins[0], step_pins[1], step_pins[2],
           step_pins[3], step_pins[4], step_pins[5]);
    printf("DIR  pins: GPIO %d, %d, %d, %d, %d, %d\n",
           dir_pins[0], dir_pins[1], dir_pins[2],
           dir_pins[3], dir_pins[4], dir_pins[5]);
    printf("\n");
    printf("Commands: STATUS, SUMMARY, RESET, STREAM:1/0, MONITOR:N, HELP\n");
    printf("Append 'X' as terminator (e.g., STATUS?X) for compatibility.\n");
    printf("\n");
    fflush(stdout);

    init_gpio();

    /* Launch tasks */
    xTaskCreatePinnedToCore(serial_task,   "serial",   4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(report_task,   "report",   4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(activity_task, "activity",  2048, NULL, 2, NULL, 1);

    printf("Analyzer ready. Waiting for step/dir signals...\n\n");
    fflush(stdout);
}
