/*
 * Assetto Corsa Shared Memory Plugin
 * ====================================
 * Reads AC physics telemetry via Windows shared memory and maps it
 * to 6-DOF platform axes.
 *
 * Build (Windows only):
 *   cl /LD /I ..\src plugin_assetto_corsa.c /Fe:plugin_assetto_corsa.dll
 *
 * Default axis mapping:
 *   Surge  <- accG.z  (frontal G-force)
 *   Sway   <- accG.x  (lateral G-force)
 *   Heave  <- accG.y  (vertical G-force)
 *   Roll   <- roll    (body roll in radians)
 *   Pitch  <- pitch   (body pitch in radians)
 *   Yaw    <- angVel.y (yaw rate from physics, rad/s)
 *
 * Channel, min/max range, and invert are all configurable via plugin parameters.
 */

#include "plugin_api.h"
#include <math.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── AC shared memory struct (must match AC's layout exactly) ──────── */

#pragma pack(push, 4)
typedef struct { float x, y, z; } ACVec3;

typedef struct {
    int     packetId;
    float   gas;
    float   brake;
    float   fuel;
    int     gear;
    int     rpms;
    float   steerAngle;
    float   speedKmh;
    ACVec3  velocity;
    ACVec3  accG;           /* x=lateral, y=vertical, z=frontal */
    float   wheelSlip[4];
    float   wheelLoad[4];
    float   wheelsPressure[4];
    float   wheelAngularSpeed[4];
    float   tyreWear[4];
    float   tyreDirtyLevel[4];
    float   tyreCoreTemperature[4];
    float   camberRAD[4];
    float   suspensionTravel[4];
    float   drs;
    float   tc;
    float   heading;
    float   pitch;
    float   roll;
    float   cgHeight;
    float   carDamage[5];
    int     numberOfTyresOut;
    int     pitLimiterOn;
    float   abs_;
    float   kersCharge;
    float   kersInput;
    int     autoShifterOn;
    float   rideHeight[2];
    float   turboBoost;
    float   ballast;
    float   airDensity;
    float   airTemp;
    float   roadTemp;
    ACVec3  localAngularVel;
    float   finalFF;
    float   performanceMeter;
    int     engineBrake;
    int     ersRecoveryLevel;
    int     ersPowerLevel;
    int     ersHeatCharging;
    int     ersIsCharging;
    float   kersCurrentKJ;
    int     drsAvailable;
    int     drsEnabled;
    float   brakeTemp[4];
    float   clutch;
    float   tyreTempI[4];
    float   tyreTempM[4];
    float   tyreTempO[4];
    int     isAIControlled;
    float   tyreContactPoint[12];
    float   tyreContactNormal[12];
    float   tyreContactHeading[12];
    float   brakeBias;
    ACVec3  localVelocity;
} ACPhysicsData;
#pragma pack(pop)

/* ── Channel enum ─────────────────────────────────────────────────── */

#define CH_NONE            0
#define CH_SURGE_G         1
#define CH_SWAY_G          2
#define CH_HEAVE_G         3
#define CH_ROLL            4
#define CH_PITCH           5
#define CH_YAW_RATE        6
#define CH_LOCAL_VEL_X     7
#define CH_LOCAL_VEL_Z     8
#define CH_ANG_VEL_X       9
#define CH_ANG_VEL_Y       10
#define CH_ANG_VEL_Z       11
#define CH_TRACTION_LOSS   12
#define CH_TRACTION_LOSS_AVG 13
#define CH_SUSP_FL         14
#define CH_SUSP_FR         15
#define CH_SUSP_RL         16
#define CH_SUSP_RR         17
#define CH_G_LAT_ABS       18
#define CH_G_LON_ABS       19
#define CH_COUNT           20

#define CHANNEL_ENUM_LABELS \
    "None\0" \
    "Surge G (accG.z)\0" \
    "Sway G (accG.x)\0" \
    "Heave G (accG.y)\0" \
    "Roll (rad)\0" \
    "Pitch (rad)\0" \
    "Yaw Rate (heading)\0" \
    "Local Vel X\0" \
    "Local Vel Z\0" \
    "Ang Vel X (roll)\0" \
    "Ang Vel Y (yaw)\0" \
    "Ang Vel Z (pitch)\0" \
    "Traction Loss (max)\0" \
    "Traction Loss (avg)\0" \
    "Susp FL\0" \
    "Susp FR\0" \
    "Susp RL\0" \
    "Susp RR\0" \
    "G Lateral (abs)\0" \
    "G Longitudinal (abs)\0" \
    "\0"

/* ── Plugin state ─────────────────────────────────────────────────── */

#ifdef _WIN32
static HANDLE s_hMap = NULL;
static const ACPhysicsData* s_phys = NULL;
#endif

static int   s_connected = 0;
static int   s_last_pid = -1;
static float s_sample_rate = 60.0f;

/* Yaw rate from heading delta */
static float s_last_heading = 0.0f;
static int   s_heading_init = 0;
static double s_last_time = 0.0;
static float s_yaw_rate_filtered = 0.0f;

/* Per-axis config: channel selection + asymmetric min/max + invert */
static int   s_axis_ch[6]     = { CH_SURGE_G, CH_SWAY_G, CH_HEAVE_G, CH_ROLL, CH_PITCH, CH_ANG_VEL_Y };
static float s_axis_min[6]    = { -3.0f, -3.0f, -2.0f, -0.5f, -0.5f, -3.0f };
static float s_axis_max[6]    = {  1.5f,  3.0f,  2.0f,  0.5f,  0.5f,  3.0f };
static float s_axis_invert[6] = {  0, 0, 0, 0, 0, 0 };

/* ── Parameter declarations ───────────────────────────────────────── */

static const StewartParamDef s_params[] = {
    /* Surge axis */
    { "surge_ch",  "Surge Channel",  "Telemetry channel for surge axis",     STEWART_PARAM_ENUM,  CH_SURGE_G, 0, CH_COUNT-1, CHANNEL_ENUM_LABELS },
    { "surge_min", "Surge Min",      "Raw value mapping to -100%",           STEWART_PARAM_FLOAT, -3.0f, -50.0f, 0.0f, NULL },
    { "surge_max", "Surge Max",      "Raw value mapping to +100%",           STEWART_PARAM_FLOAT,  1.5f,  0.0f, 50.0f, NULL },
    { "surge_inv", "Surge Invert",   "Flip surge axis direction",            STEWART_PARAM_BOOL,   0, 0, 1, NULL },
    /* Sway axis */
    { "sway_ch",   "Sway Channel",   "Telemetry channel for sway axis",     STEWART_PARAM_ENUM,  CH_SWAY_G, 0, CH_COUNT-1, CHANNEL_ENUM_LABELS },
    { "sway_min",  "Sway Min",       "Raw value mapping to -100%",           STEWART_PARAM_FLOAT, -3.0f, -50.0f, 0.0f, NULL },
    { "sway_max",  "Sway Max",       "Raw value mapping to +100%",           STEWART_PARAM_FLOAT,  3.0f,  0.0f, 50.0f, NULL },
    { "sway_inv",  "Sway Invert",    "Flip sway axis direction",             STEWART_PARAM_BOOL,   0, 0, 1, NULL },
    /* Heave axis */
    { "heave_ch",  "Heave Channel",  "Telemetry channel for heave axis",    STEWART_PARAM_ENUM,  CH_HEAVE_G, 0, CH_COUNT-1, CHANNEL_ENUM_LABELS },
    { "heave_min", "Heave Min",      "Raw value mapping to -100%",           STEWART_PARAM_FLOAT, -2.0f, -50.0f, 0.0f, NULL },
    { "heave_max", "Heave Max",      "Raw value mapping to +100%",           STEWART_PARAM_FLOAT,  2.0f,  0.0f, 50.0f, NULL },
    { "heave_inv", "Heave Invert",   "Flip heave axis direction",            STEWART_PARAM_BOOL,   0, 0, 1, NULL },
    /* Roll axis */
    { "roll_ch",   "Roll Channel",   "Telemetry channel for roll axis",     STEWART_PARAM_ENUM,  CH_ROLL, 0, CH_COUNT-1, CHANNEL_ENUM_LABELS },
    { "roll_min",  "Roll Min",       "Raw value mapping to -100%",           STEWART_PARAM_FLOAT, -0.5f, -10.0f, 0.0f, NULL },
    { "roll_max",  "Roll Max",       "Raw value mapping to +100%",           STEWART_PARAM_FLOAT,  0.5f,  0.0f, 10.0f, NULL },
    { "roll_inv",  "Roll Invert",    "Flip roll axis direction",              STEWART_PARAM_BOOL,   0, 0, 1, NULL },
    /* Pitch axis */
    { "pitch_ch",  "Pitch Channel",  "Telemetry channel for pitch axis",    STEWART_PARAM_ENUM,  CH_PITCH, 0, CH_COUNT-1, CHANNEL_ENUM_LABELS },
    { "pitch_min", "Pitch Min",      "Raw value mapping to -100%",           STEWART_PARAM_FLOAT, -0.5f, -10.0f, 0.0f, NULL },
    { "pitch_max", "Pitch Max",      "Raw value mapping to +100%",           STEWART_PARAM_FLOAT,  0.5f,  0.0f, 10.0f, NULL },
    { "pitch_inv", "Pitch Invert",   "Flip pitch axis direction",            STEWART_PARAM_BOOL,   0, 0, 1, NULL },
    /* Yaw axis */
    { "yaw_ch",    "Yaw Channel",    "Telemetry channel for yaw axis",      STEWART_PARAM_ENUM,  CH_ANG_VEL_Y, 0, CH_COUNT-1, CHANNEL_ENUM_LABELS },
    { "yaw_min",   "Yaw Min",        "Raw value mapping to -100%",           STEWART_PARAM_FLOAT, -3.0f, -50.0f, 0.0f, NULL },
    { "yaw_max",   "Yaw Max",        "Raw value mapping to +100%",           STEWART_PARAM_FLOAT,  3.0f,  0.0f, 50.0f, NULL },
    { "yaw_inv",   "Yaw Invert",     "Flip yaw axis direction",              STEWART_PARAM_BOOL,   0, 0, 1, NULL },
};

/* ── Plugin info ──────────────────────────────────────────────────── */

static const StewartPluginInfo s_info = {
    STEWART_PLUGIN_API_VERSION,
    "Assetto Corsa",
    "Stewart Platform Project",
    "1.0.0",
    "Reads Assetto Corsa physics via Windows shared memory.\n"
    "Configurable per-axis channel mapping with asymmetric scaling.",
    100,   /* preferred_rate_hz — AC physics updates at ~100 Hz */
    6,
    { "Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw" },
    sizeof(s_params) / sizeof(s_params[0]),
    s_params
};

/* ── Helpers ──────────────────────────────────────────────────────── */

static float extract_channel(const ACPhysicsData* p, int ch, float yaw_rate) {
    switch (ch) {
        case CH_SURGE_G:          return p->accG.z;
        case CH_SWAY_G:           return p->accG.x;
        case CH_HEAVE_G:          return p->accG.y;
        case CH_ROLL:             return p->roll;
        case CH_PITCH:            return p->pitch;
        case CH_YAW_RATE:         return yaw_rate;
        case CH_LOCAL_VEL_X:      return p->localVelocity.x;
        case CH_LOCAL_VEL_Z:      return p->localVelocity.z;
        case CH_ANG_VEL_X:        return p->localAngularVel.x;
        case CH_ANG_VEL_Y:        return p->localAngularVel.y;
        case CH_ANG_VEL_Z:        return p->localAngularVel.z;
        case CH_TRACTION_LOSS: {
            float mx = 0;
            for (int w = 0; w < 4; w++) {
                float s = p->wheelSlip[w] < 0 ? -p->wheelSlip[w] : p->wheelSlip[w];
                if (s > mx) mx = s;
            }
            return mx;
        }
        case CH_TRACTION_LOSS_AVG: {
            float sum = 0;
            for (int w = 0; w < 4; w++) {
                float s = p->wheelSlip[w] < 0 ? -p->wheelSlip[w] : p->wheelSlip[w];
                sum += s;
            }
            return sum / 4.0f;
        }
        case CH_SUSP_FL: return p->suspensionTravel[0];
        case CH_SUSP_FR: return p->suspensionTravel[1];
        case CH_SUSP_RL: return p->suspensionTravel[2];
        case CH_SUSP_RR: return p->suspensionTravel[3];
        case CH_G_LAT_ABS: return p->accG.x < 0 ? -p->accG.x : p->accG.x;
        case CH_G_LON_ABS: return p->accG.z < 0 ? -p->accG.z : p->accG.z;
        default: return 0.0f;
    }
}

static float scale_asymmetric(float raw, float min_v, float max_v) {
    float pct;
    if (raw < 0.0f) {
        float denom = (min_v < -0.001f) ? min_v : -1.0f;
        pct = (raw / denom) * -100.0f;
    } else {
        float denom = (max_v > 0.001f) ? max_v : 1.0f;
        pct = (raw / denom) * 100.0f;
    }
    if (pct > 100.0f) pct = 100.0f;
    if (pct < -100.0f) pct = -100.0f;
    return pct;
}

/* ── Entry points ─────────────────────────────────────────────────── */

STEWART_EXPORT const StewartPluginInfo* stewart_plugin_info(void) {
    return &s_info;
}

STEWART_EXPORT int stewart_plugin_init(float sample_rate) {
    s_sample_rate = sample_rate;
    s_connected = 0;
    s_last_pid = -1;
    s_last_heading = 0.0f;
    s_heading_init = 0;
    s_last_time = 0.0;
    s_yaw_rate_filtered = 0.0f;

    /* Reset to default axis mapping */
    s_axis_ch[0] = CH_SURGE_G; s_axis_ch[1] = CH_SWAY_G;  s_axis_ch[2] = CH_HEAVE_G;
    s_axis_ch[3] = CH_ROLL;    s_axis_ch[4] = CH_PITCH;    s_axis_ch[5] = CH_ANG_VEL_Y;
    s_axis_min[0] = -3.0f; s_axis_min[1] = -3.0f; s_axis_min[2] = -2.0f;
    s_axis_min[3] = -0.5f; s_axis_min[4] = -0.5f; s_axis_min[5] = -3.0f;
    s_axis_max[0] =  1.5f; s_axis_max[1] =  3.0f; s_axis_max[2] =  2.0f;
    s_axis_max[3] =  0.5f; s_axis_max[4] =  0.5f; s_axis_max[5] =  3.0f;
    for (int i = 0; i < 6; i++) s_axis_invert[i] = 0;

#ifdef _WIN32
    s_hMap = OpenFileMappingA(FILE_MAP_READ, FALSE, "Local\\acpmf_physics");
    if (!s_hMap) return -1;  /* AC not running */

    s_phys = (const ACPhysicsData*)MapViewOfFile(s_hMap, FILE_MAP_READ, 0, 0, sizeof(ACPhysicsData));
    if (!s_phys) {
        CloseHandle(s_hMap);
        s_hMap = NULL;
        return -1;
    }
    s_connected = 1;
    s_last_pid = s_phys->packetId;
    return 0;
#else
    return -1;  /* not available on non-Windows */
#endif
}

STEWART_EXPORT int stewart_plugin_process(StewartPluginContext* ctx) {
#ifdef _WIN32
    if (!s_connected || !s_phys) {
        for (int i = 0; i < 6; i++) ctx->output[i] = 0.0f;
        return -1;
    }

    /* Snapshot the shared memory struct locally to avoid torn reads.
     * AC writes the struct non-atomically, so we read packetId before
     * and after copying. If it changed, a write occurred mid-copy. */
    static ACPhysicsData s_snapshot;
    static int s_snapshot_valid = 0;

    volatile const int* pid_ptr = &s_phys->packetId;
    int pid1 = *pid_ptr;

    if (pid1 == s_last_pid) {
        /* No new data — return non-zero so host keeps previous output */
        return 1;
    }

    /* Copy entire struct to local buffer */
    memcpy(&s_snapshot, (const void*)s_phys, sizeof(ACPhysicsData));

    /* Re-read packetId — if it changed during our copy, data is torn */
    int pid2 = *pid_ptr;
    if (pid1 != pid2) {
        /* Torn read detected — discard, host keeps previous values */
        return 1;
    }

    s_last_pid = pid1;
    s_snapshot_valid = 1;
    const ACPhysicsData* p = &s_snapshot;

    /* Compute yaw rate from heading delta */
    float yaw_rate = 0.0f;
    {
        float heading = p->heading;
        if (s_heading_init && s_last_time > 0.0) {
            float dh = heading - s_last_heading;
            if (dh > (float)M_PI) dh -= 2.0f * (float)M_PI;
            if (dh < -(float)M_PI) dh += 2.0f * (float)M_PI;
            float dt = (float)(ctx->timestamp - s_last_time);
            if (dt > 0.002f && dt < 0.5f) {
                float rate = dh / dt;
                if (rate > -10.0f && rate < 10.0f)
                    yaw_rate = rate;
            }
        }
        s_last_heading = heading;
        s_heading_init = 1;
        s_last_time = ctx->timestamp;

        /* Heavy LP filter: alpha=0.05 at 100Hz → ~0.8Hz cutoff */
        s_yaw_rate_filtered += 0.05f * (yaw_rate - s_yaw_rate_filtered);
    }

    /* Map channels to output axes */
    for (int i = 0; i < 6; i++) {
        int ch = s_axis_ch[i];
        if (ch <= CH_NONE || ch >= CH_COUNT) {
            ctx->output[i] = 0.0f;
            ctx->raw_input[i] = 0.0f;
            continue;
        }
        float raw = extract_channel(p, ch, s_yaw_rate_filtered);
        ctx->raw_input[i] = raw;  /* pre-scaling value for profiling */
        if (s_axis_invert[i] != 0.0f) raw = -raw;
        ctx->output[i] = scale_asymmetric(raw, s_axis_min[i], s_axis_max[i]);
    }

    return 0;
#else
    for (int i = 0; i < 6; i++) ctx->output[i] = 0.0f;
    return -1;
#endif
}

STEWART_EXPORT void stewart_plugin_shutdown(void) {
#ifdef _WIN32
    if (s_phys) { UnmapViewOfFile(s_phys); s_phys = NULL; }
    if (s_hMap) { CloseHandle(s_hMap); s_hMap = NULL; }
#endif
    s_connected = 0;
    s_heading_init = 0;
    s_yaw_rate_filtered = 0.0f;
}

STEWART_EXPORT void stewart_plugin_set_param(const char* name, float value) {
    /* Axis channel assignments */
    if      (strcmp(name, "surge_ch") == 0)  s_axis_ch[0] = (int)value;
    else if (strcmp(name, "sway_ch") == 0)   s_axis_ch[1] = (int)value;
    else if (strcmp(name, "heave_ch") == 0)  s_axis_ch[2] = (int)value;
    else if (strcmp(name, "roll_ch") == 0)   s_axis_ch[3] = (int)value;
    else if (strcmp(name, "pitch_ch") == 0)  s_axis_ch[4] = (int)value;
    else if (strcmp(name, "yaw_ch") == 0)    s_axis_ch[5] = (int)value;
    /* Min values */
    else if (strcmp(name, "surge_min") == 0) s_axis_min[0] = value;
    else if (strcmp(name, "sway_min") == 0)  s_axis_min[1] = value;
    else if (strcmp(name, "heave_min") == 0) s_axis_min[2] = value;
    else if (strcmp(name, "roll_min") == 0)  s_axis_min[3] = value;
    else if (strcmp(name, "pitch_min") == 0) s_axis_min[4] = value;
    else if (strcmp(name, "yaw_min") == 0)   s_axis_min[5] = value;
    /* Max values */
    else if (strcmp(name, "surge_max") == 0) s_axis_max[0] = value;
    else if (strcmp(name, "sway_max") == 0)  s_axis_max[1] = value;
    else if (strcmp(name, "heave_max") == 0) s_axis_max[2] = value;
    else if (strcmp(name, "roll_max") == 0)  s_axis_max[3] = value;
    else if (strcmp(name, "pitch_max") == 0) s_axis_max[4] = value;
    else if (strcmp(name, "yaw_max") == 0)   s_axis_max[5] = value;
    /* Invert flags */
    else if (strcmp(name, "surge_inv") == 0) s_axis_invert[0] = value;
    else if (strcmp(name, "sway_inv") == 0)  s_axis_invert[1] = value;
    else if (strcmp(name, "heave_inv") == 0) s_axis_invert[2] = value;
    else if (strcmp(name, "roll_inv") == 0)  s_axis_invert[3] = value;
    else if (strcmp(name, "pitch_inv") == 0) s_axis_invert[4] = value;
    else if (strcmp(name, "yaw_inv") == 0)   s_axis_invert[5] = value;
}
