#include "MotionCueing.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── NVS includes (ESP32 only) ─────────────────────────────────────── */
#ifdef ESP_PLATFORM
#include "nvs_flash.h"
#include "nvs.h"
#include "debug_uart.h"
#else
#define DEBUG_PRINTF(...)
#define DEBUG_PRINTLN(...)
#endif

/* ── Biquad primitives ─────────────────────────────────────────────── */

void biquadReset(BiquadFilter* f) {
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

float biquadProcess(BiquadFilter* f, float x) {
    float y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

void biquadSetHighpass(BiquadFilter* f, float fc, float fs, float Q) {
    f->fc = fc;
    f->Q  = Q;
    if (fc <= 0.0f || fs <= 0.0f || fc >= fs * 0.499f || Q <= 0.0f) {
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }
    float omega = 2.0f * (float)M_PI * fc / fs;
    float sinw  = sinf(omega);
    float cosw  = cosf(omega);
    float alpha = sinw / (2.0f * Q);
    float a0    = 1.0f + alpha;
    f->b0 = ((1.0f + cosw) / 2.0f) / a0;
    f->b1 = (-(1.0f + cosw))       / a0;
    f->b2 = ((1.0f + cosw) / 2.0f) / a0;
    f->a1 = (-2.0f * cosw)         / a0;
    f->a2 = (1.0f - alpha)         / a0;
}

void biquadSetLowpass(BiquadFilter* f, float fc, float fs, float Q) {
    f->fc = fc;
    f->Q  = Q;
    if (fc <= 0.0f || fs <= 0.0f || fc >= fs * 0.499f || Q <= 0.0f) {
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }
    float omega = 2.0f * (float)M_PI * fc / fs;
    float sinw  = sinf(omega);
    float cosw  = cosf(omega);
    float alpha = sinw / (2.0f * Q);
    float a0    = 1.0f + alpha;
    f->b0 = ((1.0f - cosw) / 2.0f) / a0;
    f->b1 = (1.0f - cosw)          / a0;
    f->b2 = ((1.0f - cosw) / 2.0f) / a0;
    f->a1 = (-2.0f * cosw)         / a0;
    f->a2 = (1.0f - alpha)         / a0;
}

void biquadSetNotch(BiquadFilter* f, float fc, float fs, float Q) {
    f->fc = fc;
    f->Q  = Q;
    if (fc <= 0.0f || fs <= 0.0f || fc >= fs * 0.499f || Q <= 0.0f) {
        f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
        f->a1 = 0.0f; f->a2 = 0.0f;
        return;
    }
    float omega = 2.0f * (float)M_PI * fc / fs;
    float sinw  = sinf(omega);
    float cosw  = cosf(omega);
    float alpha = sinw / (2.0f * Q);
    float a0    = 1.0f + alpha;
    f->b0 = 1.0f             / a0;
    f->b1 = (-2.0f * cosw)   / a0;
    f->b2 = 1.0f             / a0;
    f->a1 = (-2.0f * cosw)   / a0;
    f->a2 = (1.0f - alpha)   / a0;
}

/* ── Input Filter (pre-MCA signal conditioning) ──────────────────── */

void initInputFilter(InputFilterConfig* cfg, float sample_rate) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->sample_rate = sample_rate;
    cfg->enabled = 0;
    for (int i = 0; i < 6; i++) {
        cfg->axes[i].lp_enabled = 0;
        cfg->axes[i].notch_enabled = 0;
        cfg->axes[i].lp.fc = 20.0f;
        cfg->axes[i].lp.Q  = 0.707f;
        cfg->axes[i].notch.fc = 0.0f;
        cfg->axes[i].notch.Q  = 5.0f;
    }
}

void resetInputFilter(InputFilterConfig* cfg) {
    for (int i = 0; i < 6; i++) {
        biquadReset(&cfg->axes[i].lp);
        biquadReset(&cfg->axes[i].notch);
    }
}

void processInputFilter(InputFilterConfig* cfg, const float in[6], float out[6]) {
    if (!cfg->enabled) {
        for (int i = 0; i < 6; i++)
            out[i] = in[i];
        return;
    }
    for (int i = 0; i < 6; i++) {
        float y = in[i];
        if (cfg->axes[i].lp_enabled)
            y = biquadProcess(&cfg->axes[i].lp, y);
        if (cfg->axes[i].notch_enabled)
            y = biquadProcess(&cfg->axes[i].notch, y);
        out[i] = y;
    }
}

void inputFilterUpdateSampleRate(InputFilterConfig* cfg, float new_sr) {
    cfg->sample_rate = new_sr;
    for (int i = 0; i < 6; i++) {
        InputAxisFilter* ax = &cfg->axes[i];
        if (ax->lp_enabled && ax->lp.fc > 0)
            biquadSetLowpass(&ax->lp, ax->lp.fc, new_sr, ax->lp.Q > 0 ? ax->lp.Q : 0.707f);
        if (ax->notch_enabled && ax->notch.fc > 0)
            biquadSetNotch(&ax->notch, ax->notch.fc, new_sr, ax->notch.Q > 0 ? ax->notch.Q : 5.0f);
    }
}

/* ── Axis channel processing ───────────────────────────────────────── */

static float processAxisChannel(AxisChannelFilter* ch, float x) {
    float y = x;
    if (ch->hp_enabled)
        y = biquadProcess(&ch->hp, y);
    if (ch->lp_enabled)
        y = biquadProcess(&ch->lp, y);
    y *= ch->gain;
    if (ch->rate_limit > 0.0f) {
        float delta = y - ch->last_output;
        if (delta > ch->rate_limit)
            y = ch->last_output + ch->rate_limit;
        else if (delta < -ch->rate_limit)
            y = ch->last_output - ch->rate_limit;
    }
    ch->last_output = y;
    return y;
}

/* ── Preset definitions ────────────────────────────────────────────── */
/* Axis order: surge=0, sway=1, heave=2, roll=3, pitch=4, yaw=5        */

typedef struct {
    const char* name;
    float hp_fc[6];
    float lp_fc[6];
    float gain[6];
    float hp_Q[6];
    float lp_Q[6];
    int   tilt_enabled;
    float tilt_fc;
    float tilt_Q;
    float tilt_surge_gain;
    float tilt_sway_gain;
} PresetDef;

static const PresetDef PRESET_TABLE[MCA_PRESET_COUNT] = {
    /* MCA_OFF */
    { "off",
      {0,0,0,0,0,0}, {0,0,0,0,0,0},
      {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f},
      {0.707f,0.707f,0.707f,0.707f,0.707f,0.707f},
      {0.707f,0.707f,0.707f,0.707f,0.707f,0.707f},
      0, 0.5f, 0.707f, 0.0f, 0.0f },

    /* MCA_GENTLE — light washout, wide bandwidth (truck, flight) */
    { "gentle",
      {0.3f,0.3f,0.4f, 0.3f,0.3f,0.3f},
      {8.0f,8.0f,8.0f, 6.0f,6.0f,6.0f},
      {1.0f,1.0f,1.0f, 1.0f,1.0f,1.0f},
      {0.5f,0.5f,0.5f, 0.5f,0.5f,0.5f},
      {0.707f,0.707f,0.707f,0.707f,0.707f,0.707f},
      1, 0.3f, 0.707f, 0.08f, 0.08f },

    /* MCA_MODERATE — balanced washout (general racing/driving) */
    { "moderate",
      {0.8f,0.8f,1.0f, 0.5f,0.5f,0.8f},
      {12.0f,12.0f,10.0f, 8.0f,8.0f,10.0f},
      {1.0f,1.0f,1.2f, 1.0f,1.0f,0.8f},
      {0.6f,0.6f,0.6f, 0.5f,0.5f,0.6f},
      {0.707f,0.707f,0.707f,0.707f,0.707f,0.707f},
      1, 0.5f, 0.707f, 0.15f, 0.15f },

    /* MCA_AGGRESSIVE — tight washout, fast return (high-speed racing) */
    { "aggressive",
      {1.5f,1.5f,2.0f, 1.0f,1.0f,1.5f},
      {15.0f,15.0f,12.0f, 10.0f,10.0f,12.0f},
      {1.2f,1.2f,1.5f, 1.0f,1.0f,0.7f},
      {0.707f,0.707f,0.707f,0.707f,0.707f,0.707f},
      {0.707f,0.707f,0.707f,0.707f,0.707f,0.707f},
      1, 0.8f, 0.707f, 0.25f, 0.25f },

    /* MCA_RACE_PRO — strong onset, aggressive washout */
    { "race_pro",
      {2.0f,2.0f,2.5f, 1.5f,1.5f,2.0f},
      {20.0f,20.0f,15.0f, 12.0f,12.0f,15.0f},
      {1.5f,1.5f,1.8f, 1.2f,1.2f,0.6f},
      {0.8f,0.8f,0.8f, 0.707f,0.707f,0.8f},
      {0.6f,0.6f,0.6f, 0.707f,0.707f,0.6f},
      1, 1.0f, 0.6f, 0.35f, 0.35f },
};

/* ── Core API ──────────────────────────────────────────────────────── */

const char* mcaPresetName(int preset) {
    if (preset < 0 || preset >= MCA_PRESET_COUNT)
        return "unknown";
    return PRESET_TABLE[preset].name;
}

void initMotionCueing(MotionCueingConfig* cfg, float sample_rate) {
    memset(cfg, 0, sizeof(MotionCueingConfig));
    cfg->schema_version = MCA_SCHEMA_VERSION;
    cfg->sample_rate = sample_rate;
    cfg->enabled = 0;
    cfg->preset = MCA_OFF;
    for (int i = 0; i < 6; i++) {
        cfg->channels[i].gain = 1.0f;
        cfg->channels[i].hp.Q = 0.707f;
        cfg->channels[i].lp.Q = 0.707f;
    }
    cfg->tilt.fc = 0.5f;
    cfg->tilt.Q  = 0.707f;
    cfg->tilt.hp_fc = 0.3f;
    cfg->tilt.hp_Q  = 0.707f;
    cfg->tilt.hp_enabled = 1;
    cfg->tilt.surge_hp_enabled = 1;
    cfg->tilt.sway_hp_enabled = 1;
    cfg->tilt.sway_hp_fc = 0.3f;
    cfg->tilt.sway_hp_Q  = 0.707f;
    cfg->tilt.hp_linked = 1;
}

void setMotionCueingPreset(MotionCueingConfig* cfg, int preset) {
    if (preset < 0 || preset >= MCA_PRESET_COUNT)
        return;

    const PresetDef* p = &PRESET_TABLE[preset];
    float fs = cfg->sample_rate;
    cfg->preset = preset;

    for (int i = 0; i < 6; i++) {
        AxisChannelFilter* ch = &cfg->channels[i];

        ch->hp_enabled = (p->hp_fc[i] > 0.0f) ? 1 : 0;
        ch->lp_enabled = (p->lp_fc[i] > 0.0f) ? 1 : 0;
        ch->gain = p->gain[i];
        ch->rate_limit = 0.0f;

        if (ch->hp_enabled)
            biquadSetHighpass(&ch->hp, p->hp_fc[i], fs, p->hp_Q[i]);
        else {
            ch->hp.fc = 0.0f;
            ch->hp.Q = p->hp_Q[i];
        }
        if (ch->lp_enabled)
            biquadSetLowpass(&ch->lp, p->lp_fc[i], fs, p->lp_Q[i]);
        else {
            ch->lp.fc = 0.0f;
            ch->lp.Q = p->lp_Q[i];
        }
    }

    cfg->tilt.enabled = p->tilt_enabled;
    cfg->tilt.surge_gain = p->tilt_surge_gain;
    cfg->tilt.sway_gain  = p->tilt_sway_gain;
    cfg->tilt.fc = p->tilt_fc;
    cfg->tilt.Q  = p->tilt_Q;
    if (p->tilt_fc > 0.0f) {
        biquadSetLowpass(&cfg->tilt.surge_lp, p->tilt_fc, fs, p->tilt_Q);
        biquadSetLowpass(&cfg->tilt.sway_lp,  p->tilt_fc, fs, p->tilt_Q);
    }
    /* Tilt HP washout defaults for presets with tilt enabled */
    if (p->tilt_enabled) {
        if (cfg->tilt.hp_fc <= 0.0f) cfg->tilt.hp_fc = 0.3f;
        if (cfg->tilt.hp_Q  <= 0.0f) cfg->tilt.hp_Q  = 0.707f;
        cfg->tilt.hp_enabled = 1;
        cfg->tilt.surge_hp_enabled = 1;
        cfg->tilt.sway_hp_enabled = 1;
        cfg->tilt.sway_hp_fc = cfg->tilt.hp_fc;
        cfg->tilt.sway_hp_Q  = cfg->tilt.hp_Q;
        cfg->tilt.hp_linked = 1;
        biquadSetHighpass(&cfg->tilt.surge_hp, cfg->tilt.hp_fc, fs, cfg->tilt.hp_Q);
        biquadSetHighpass(&cfg->tilt.sway_hp,  cfg->tilt.sway_hp_fc, fs, cfg->tilt.sway_hp_Q);
    }

    cfg->enabled = (preset != MCA_OFF) ? 1 : 0;
}

void resetMotionCueing(MotionCueingConfig* cfg) {
    for (int i = 0; i < 6; i++) {
        biquadReset(&cfg->channels[i].hp);
        biquadReset(&cfg->channels[i].lp);
        cfg->channels[i].last_output = 0.0f;
    }
    biquadReset(&cfg->tilt.surge_lp);
    biquadReset(&cfg->tilt.sway_lp);
    biquadReset(&cfg->tilt.surge_hp);
    biquadReset(&cfg->tilt.sway_hp);
}

void processMotionCueing(MotionCueingConfig* cfg, const float in[6], float out[6]) {
    if (!cfg->enabled) {
        for (int i = 0; i < 6; i++)
            out[i] = in[i];
        return;
    }

    for (int i = 0; i < 6; i++)
        out[i] = processAxisChannel(&cfg->channels[i], in[i]);

    /* Tilt coordination: sustained surge -> pitch, sustained sway -> roll */
    if (cfg->tilt.enabled) {
        float pitch_add = biquadProcess(&cfg->tilt.surge_lp, in[0]) * cfg->tilt.surge_gain;
        float roll_add  = biquadProcess(&cfg->tilt.sway_lp,  in[1]) * cfg->tilt.sway_gain;
        /* HP washout on tilt output: tilt returns to center over time */
        if (cfg->tilt.surge_hp_enabled)
            pitch_add = biquadProcess(&cfg->tilt.surge_hp, pitch_add);
        if (cfg->tilt.sway_hp_enabled)
            roll_add  = biquadProcess(&cfg->tilt.sway_hp,  roll_add);
        out[4] += pitch_add;   /* pitch = index 4 */
        out[3] += roll_add;    /* roll  = index 3 */
    }
}

/* ── Sample rate update (recalculates all active biquads) ──────────── */

void mcaUpdateSampleRate(MotionCueingConfig* cfg, float new_sr) {
    if (new_sr <= 0.0f || new_sr == cfg->sample_rate) return;
    cfg->sample_rate = new_sr;
    for (int i = 0; i < 6; i++) {
        AxisChannelFilter* ch = &cfg->channels[i];
        if (ch->hp.fc > 0.0f)
            biquadSetHighpass(&ch->hp, ch->hp.fc, new_sr, ch->hp.Q > 0 ? ch->hp.Q : 0.707f);
        if (ch->lp.fc > 0.0f)
            biquadSetLowpass(&ch->lp, ch->lp.fc, new_sr, ch->lp.Q > 0 ? ch->lp.Q : 0.707f);
    }
    if (cfg->tilt.fc > 0.0f) {
        float q = cfg->tilt.Q > 0 ? cfg->tilt.Q : 0.707f;
        biquadSetLowpass(&cfg->tilt.surge_lp, cfg->tilt.fc, new_sr, q);
        biquadSetLowpass(&cfg->tilt.sway_lp,  cfg->tilt.fc, new_sr, q);
    }
    if (cfg->tilt.surge_hp_enabled && cfg->tilt.hp_fc > 0.0f) {
        float q = cfg->tilt.hp_Q > 0 ? cfg->tilt.hp_Q : 0.707f;
        biquadSetHighpass(&cfg->tilt.surge_hp, cfg->tilt.hp_fc, new_sr, q);
    }
    if (cfg->tilt.sway_hp_enabled && cfg->tilt.sway_hp_fc > 0.0f) {
        float q = cfg->tilt.sway_hp_Q > 0 ? cfg->tilt.sway_hp_Q : 0.707f;
        biquadSetHighpass(&cfg->tilt.sway_hp, cfg->tilt.sway_hp_fc, new_sr, q);
    }
}

/* ── Per-channel parameter setters ─────────────────────────────────── */

void mcaSetChannelHpFc(MotionCueingConfig* cfg, int axis, float fc) {
    if (axis < 0 || axis >= 6) return;
    AxisChannelFilter* ch = &cfg->channels[axis];
    if (fc > 0.0f) {
        biquadSetHighpass(&ch->hp, fc, cfg->sample_rate, ch->hp.Q > 0 ? ch->hp.Q : 0.707f);
        biquadReset(&ch->hp);
    } else {
        ch->hp.fc = 0.0f;
    }
}

void mcaSetChannelHpQ(MotionCueingConfig* cfg, int axis, float Q) {
    if (axis < 0 || axis >= 6 || Q <= 0.0f) return;
    AxisChannelFilter* ch = &cfg->channels[axis];
    if (ch->hp.fc > 0.0f) {
        biquadSetHighpass(&ch->hp, ch->hp.fc, cfg->sample_rate, Q);
        biquadReset(&ch->hp);
    } else {
        ch->hp.Q = Q;
    }
}

void mcaSetChannelLpFc(MotionCueingConfig* cfg, int axis, float fc) {
    if (axis < 0 || axis >= 6) return;
    AxisChannelFilter* ch = &cfg->channels[axis];
    if (fc > 0.0f) {
        biquadSetLowpass(&ch->lp, fc, cfg->sample_rate, ch->lp.Q > 0 ? ch->lp.Q : 0.707f);
        biquadReset(&ch->lp);
    } else {
        ch->lp.fc = 0.0f;
    }
}

void mcaSetChannelLpQ(MotionCueingConfig* cfg, int axis, float Q) {
    if (axis < 0 || axis >= 6 || Q <= 0.0f) return;
    AxisChannelFilter* ch = &cfg->channels[axis];
    if (ch->lp.fc > 0.0f) {
        biquadSetLowpass(&ch->lp, ch->lp.fc, cfg->sample_rate, Q);
        biquadReset(&ch->lp);
    } else {
        ch->lp.Q = Q;
    }
}

void mcaSetChannelGain(MotionCueingConfig* cfg, int axis, float gain) {
    if (axis < 0 || axis >= 6) return;
    cfg->channels[axis].gain = gain;
}

void mcaSetChannelRateLimit(MotionCueingConfig* cfg, int axis, float limit) {
    if (axis < 0 || axis >= 6) return;
    cfg->channels[axis].rate_limit = (limit >= 0.0f) ? limit : 0.0f;
}

void mcaSetChannelHpEnabled(MotionCueingConfig* cfg, int axis, int enabled) {
    if (axis < 0 || axis >= 6) return;
    cfg->channels[axis].hp_enabled = enabled ? 1 : 0;
}

void mcaSetChannelLpEnabled(MotionCueingConfig* cfg, int axis, int enabled) {
    if (axis < 0 || axis >= 6) return;
    cfg->channels[axis].lp_enabled = enabled ? 1 : 0;
}

/* ── Tilt parameter setters ────────────────────────────────────────── */

void mcaSetTiltEnabled(MotionCueingConfig* cfg, int enabled) {
    cfg->tilt.enabled = enabled ? 1 : 0;
}

void mcaSetTiltSurgeGain(MotionCueingConfig* cfg, float gain) {
    cfg->tilt.surge_gain = gain;
}

void mcaSetTiltSwayGain(MotionCueingConfig* cfg, float gain) {
    cfg->tilt.sway_gain = gain;
}

void mcaSetTiltFc(MotionCueingConfig* cfg, float fc) {
    cfg->tilt.fc = fc;
    if (fc > 0.0f) {
        float Q = cfg->tilt.Q > 0 ? cfg->tilt.Q : 0.707f;
        biquadSetLowpass(&cfg->tilt.surge_lp, fc, cfg->sample_rate, Q);
        biquadSetLowpass(&cfg->tilt.sway_lp,  fc, cfg->sample_rate, Q);
        biquadReset(&cfg->tilt.surge_lp);
        biquadReset(&cfg->tilt.sway_lp);
    }
}

void mcaSetTiltQ(MotionCueingConfig* cfg, float Q) {
    if (Q <= 0.0f) return;
    cfg->tilt.Q = Q;
    if (cfg->tilt.fc > 0.0f) {
        biquadSetLowpass(&cfg->tilt.surge_lp, cfg->tilt.fc, cfg->sample_rate, Q);
        biquadSetLowpass(&cfg->tilt.sway_lp,  cfg->tilt.fc, cfg->sample_rate, Q);
        biquadReset(&cfg->tilt.surge_lp);
        biquadReset(&cfg->tilt.sway_lp);
    }
}

/* ── Config validation ─────────────────────────────────────────────── */

int mcaValidateConfig(const MotionCueingConfig* cfg) {
    if (!cfg) return 0;
    if (cfg->schema_version != MCA_SCHEMA_VERSION) return 0;
    if (cfg->sample_rate <= 0.0f || cfg->sample_rate > 100000.0f) return 0;
    if (cfg->preset < 0 || cfg->preset >= MCA_PRESET_COUNT) return 0;
    for (int i = 0; i < 6; i++) {
        if (cfg->channels[i].gain < -100.0f || cfg->channels[i].gain > 100.0f) return 0;
        if (cfg->channels[i].hp.Q < 0.0f || cfg->channels[i].hp.Q > 100.0f) return 0;
        if (cfg->channels[i].lp.Q < 0.0f || cfg->channels[i].lp.Q > 100.0f) return 0;
    }
    return 1;
}

/* ── NVS Persistence ───────────────────────────────────────────────── */

#ifdef ESP_PLATFORM

int mcaSaveToNVS(const MotionCueingConfig* cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MCA_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        DEBUG_PRINTF("MCA: NVS open failed: %s\n", esp_err_to_name(err));
        return -1;
    }
    err = nvs_set_blob(handle, MCA_NVS_KEY, cfg, sizeof(MotionCueingConfig));
    if (err != ESP_OK) {
        DEBUG_PRINTF("MCA: NVS write failed: %s\n", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }
    err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        DEBUG_PRINTF("MCA: NVS commit failed: %s\n", esp_err_to_name(err));
        return -1;
    }
    DEBUG_PRINTLN("MCA: Config saved to NVS");
    return 0;
}

int mcaLoadFromNVS(MotionCueingConfig* cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MCA_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        DEBUG_PRINTF("MCA: NVS open failed (no saved config?): %s\n", esp_err_to_name(err));
        return -1;
    }
    MotionCueingConfig tmp;
    size_t required_size = sizeof(MotionCueingConfig);
    err = nvs_get_blob(handle, MCA_NVS_KEY, &tmp, &required_size);
    nvs_close(handle);
    if (err != ESP_OK || required_size != sizeof(MotionCueingConfig)) {
        DEBUG_PRINTF("MCA: NVS read failed or size mismatch (%d vs %d)\n",
                     (int)required_size, (int)sizeof(MotionCueingConfig));
        return -1;
    }
    /* Schema validation — reject if version doesn't match */
    if (!mcaValidateConfig(&tmp)) {
        DEBUG_PRINTF("MCA: Saved config invalid (schema %u, expected %u) — using defaults\n",
                     tmp.schema_version, MCA_SCHEMA_VERSION);
        return -1;
    }
    /* Copy validated config, then recalculate all biquad coefficients
       from stored fc/Q (coefficients aren't saved, only tuning params) */
    float saved_sr = tmp.sample_rate;
    memcpy(cfg, &tmp, sizeof(MotionCueingConfig));
    cfg->sample_rate = saved_sr;
    /* Recompute coefficients from stored fc/Q */
    for (int i = 0; i < 6; i++) {
        AxisChannelFilter* ch = &cfg->channels[i];
        if (ch->hp_enabled && ch->hp.fc > 0.0f)
            biquadSetHighpass(&ch->hp, ch->hp.fc, saved_sr, ch->hp.Q);
        if (ch->lp_enabled && ch->lp.fc > 0.0f)
            biquadSetLowpass(&ch->lp, ch->lp.fc, saved_sr, ch->lp.Q);
        ch->last_output = 0.0f;
    }
    if (cfg->tilt.fc > 0.0f) {
        biquadSetLowpass(&cfg->tilt.surge_lp, cfg->tilt.fc, saved_sr, cfg->tilt.Q);
        biquadSetLowpass(&cfg->tilt.sway_lp,  cfg->tilt.fc, saved_sr, cfg->tilt.Q);
    }
    resetMotionCueing(cfg);
    DEBUG_PRINTF("MCA: Loaded from NVS (preset=%s, sr=%.0f)\n",
                 mcaPresetName(cfg->preset), cfg->sample_rate);
    return 0;
}

#else
/* ── Stubs for non-ESP32 (SIL) ─────────────────────────────────────── */
int mcaSaveToNVS(const MotionCueingConfig* cfg) { (void)cfg; return -1; }
int mcaLoadFromNVS(MotionCueingConfig* cfg) { (void)cfg; return -1; }
#endif
