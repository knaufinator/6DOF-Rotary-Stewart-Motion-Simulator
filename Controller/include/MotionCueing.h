#ifndef MOTION_CUEING_H
#define MOTION_CUEING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Schema version — bump when struct layout changes ────────────── */
#define MCA_SCHEMA_VERSION  5
#define MCA_NVS_KEY         "mca_cfg"
#define MCA_NVS_NAMESPACE   "mca"

/* ── Biquad Filter (2nd-order IIR, direct form II transposed) ───────── */

typedef struct {
    /* Computed coefficients (set by biquadSetHighpass/Lowpass) */
    float b0, b1, b2;
    float a1, a2;
    /* Filter state */
    float z1, z2;
    /* Tuning parameters (stored for readback / NVS persistence) */
    float fc;                 /* cutoff frequency (Hz), 0 = not configured */
    float Q;                  /* quality factor                            */
} BiquadFilter;

/* ── Per-axis channel: HP washout -> LP smoothing -> gain -> rate limit  */

typedef struct {
    BiquadFilter hp;          /* high-pass washout                        */
    BiquadFilter lp;          /* low-pass smoothing                       */
    float gain;               /* output scaling                           */
    float rate_limit;         /* max change per sample (0 = disabled)     */
    float last_output;        /* previous output for rate limiting        */
    int   hp_enabled;         /* 1 = HP active                            */
    int   lp_enabled;         /* 1 = LP active                            */
} AxisChannelFilter;

/* ── Pre-MCA Input Filter (per-axis LP + notch) ───────────────────── */

typedef struct {
    BiquadFilter lp;          /* low-pass: remove high-freq noise/jitter  */
    BiquadFilter notch;       /* notch: remove specific resonance freq    */
    int   lp_enabled;         /* 1 = LP active                            */
    int   notch_enabled;      /* 1 = notch active                         */
} InputAxisFilter;

typedef struct {
    InputAxisFilter axes[6];
    int   enabled;            /* master enable for input filtering         */
    float sample_rate;        /* Hz — kept in sync with MCA sample rate    */
} InputFilterConfig;

/* ── Tilt Coordination ─────────────────────────────────────────────── */

typedef struct {
    BiquadFilter surge_lp;    /* LP filter: sustained surge -> pitch      */
    BiquadFilter sway_lp;     /* LP filter: sustained sway  -> roll       */
    BiquadFilter surge_hp;    /* HP washout: tilt pitch returns to center */
    BiquadFilter sway_hp;     /* HP washout: tilt roll returns to center  */
    float surge_gain;         /* rad per unit of sustained surge          */
    float sway_gain;          /* rad per unit of sustained sway           */
    float fc;                 /* LP cutoff for tilt filters               */
    float Q;                  /* LP Q for tilt filters                    */
    float hp_fc;              /* surge HP washout cutoff (Hz)             */
    float hp_Q;               /* surge HP washout Q                       */
    int   hp_enabled;         /* legacy: 1 = both active (compat)         */
    int   enabled;
    /* Independent per-channel HP washout (v5) */
    int   surge_hp_enabled;   /* 1 = surge->pitch HP washout active       */
    int   sway_hp_enabled;    /* 1 = sway->roll HP washout active         */
    float sway_hp_fc;         /* sway HP washout cutoff (Hz)              */
    float sway_hp_Q;          /* sway HP washout Q                        */
    int   hp_linked;          /* 1 = sway mirrors surge fc/Q              */
} TiltCoordination;

/* ── Motion Cueing Presets ─────────────────────────────────────────── */

enum McaPreset {
    MCA_OFF         = 0,
    MCA_GENTLE      = 1,
    MCA_MODERATE    = 2,
    MCA_AGGRESSIVE  = 3,
    MCA_RACE_PRO    = 4,
    MCA_PRESET_COUNT = 5,
};

/* ── Full Motion Cueing Config ─────────────────────────────────────── */

typedef struct {
    uint32_t          schema_version; /* must == MCA_SCHEMA_VERSION       */
    AxisChannelFilter channels[6];
    TiltCoordination  tilt;
    float sample_rate;        /* Hz — updated adaptively from packet rate */
    int   enabled;            /* master enable                            */
    int   preset;             /* current McaPreset enum value             */
} MotionCueingConfig;

/* ── Biquad primitives ─────────────────────────────────────────────── */

void  biquadReset(BiquadFilter* f);
float biquadProcess(BiquadFilter* f, float x);
void  biquadSetHighpass(BiquadFilter* f, float fc, float fs, float Q);
void  biquadSetLowpass(BiquadFilter* f, float fc, float fs, float Q);

/* ── Core API ──────────────────────────────────────────────────────── */

void initMotionCueing(MotionCueingConfig* cfg, float sample_rate);
void setMotionCueingPreset(MotionCueingConfig* cfg, int preset);
void resetMotionCueing(MotionCueingConfig* cfg);
void processMotionCueing(MotionCueingConfig* cfg, const float in[6], float out[6]);

/* ── Input Filter API ─────────────────────────────────────────────── */

void initInputFilter(InputFilterConfig* cfg, float sample_rate);
void resetInputFilter(InputFilterConfig* cfg);
void processInputFilter(InputFilterConfig* cfg, const float in[6], float out[6]);
void inputFilterUpdateSampleRate(InputFilterConfig* cfg, float new_sr);

/* Biquad notch filter design */
void biquadSetNotch(BiquadFilter* f, float fc, float fs, float Q);

/* Preset name lookup (returns static string) */
const char* mcaPresetName(int preset);

/* Update sample rate and recalculate all active biquad coefficients */
void mcaUpdateSampleRate(MotionCueingConfig* cfg, float new_sr);

/* ── Per-channel parameter setters (recalculate coefficients) ──────── */

void mcaSetChannelHpFc(MotionCueingConfig* cfg, int axis, float fc);
void mcaSetChannelHpQ(MotionCueingConfig* cfg, int axis, float Q);
void mcaSetChannelLpFc(MotionCueingConfig* cfg, int axis, float fc);
void mcaSetChannelLpQ(MotionCueingConfig* cfg, int axis, float Q);
void mcaSetChannelGain(MotionCueingConfig* cfg, int axis, float gain);
void mcaSetChannelRateLimit(MotionCueingConfig* cfg, int axis, float limit);
void mcaSetChannelHpEnabled(MotionCueingConfig* cfg, int axis, int enabled);
void mcaSetChannelLpEnabled(MotionCueingConfig* cfg, int axis, int enabled);

/* ── Tilt parameter setters ────────────────────────────────────────── */

void mcaSetTiltEnabled(MotionCueingConfig* cfg, int enabled);
void mcaSetTiltSurgeGain(MotionCueingConfig* cfg, float gain);
void mcaSetTiltSwayGain(MotionCueingConfig* cfg, float gain);
void mcaSetTiltFc(MotionCueingConfig* cfg, float fc);
void mcaSetTiltQ(MotionCueingConfig* cfg, float Q);

/* ── NVS Persistence (ESP32 only; stubs on other platforms) ────────── */

/* Returns 0 on success, -1 on failure */
int mcaSaveToNVS(const MotionCueingConfig* cfg);
int mcaLoadFromNVS(MotionCueingConfig* cfg);

/* Validate a loaded config; returns 1 if valid, 0 if schema mismatch */
int mcaValidateConfig(const MotionCueingConfig* cfg);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CUEING_H */
