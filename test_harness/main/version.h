#ifndef VERSION_H
#define VERSION_H

// ── Test Harness Firmware Version ────────────────────────────────────
// Bump MAJOR for breaking changes (pin map, JSON format, command set)
// Bump MINOR for new features
// Bump PATCH for bug fixes
// FW_BUILD_HASH is injected by CMake from `git rev-parse --short HEAD`

#define FW_VERSION_MAJOR  3
#define FW_VERSION_MINOR  0
#define FW_VERSION_PATCH  0

// Firmware identity
#define FW_PLATFORM_ID  "step-analyzer"

#define FW_BUILD_DATE  __DATE__
#define FW_BUILD_TIME  __TIME__

// FW_BUILD_HASH injected by CMake; fallback if built outside CMake
#ifndef FW_BUILD_HASH
#define FW_BUILD_HASH  "unknown"
#endif

// String helper macros
#define _FW_STR(x)  #x
#define _FW_XSTR(x) _FW_STR(x)

#define FW_VERSION_STRING \
    _FW_XSTR(FW_VERSION_MAJOR) "." \
    _FW_XSTR(FW_VERSION_MINOR) "." \
    _FW_XSTR(FW_VERSION_PATCH)

// Full build ID: version + git hash
#define FW_BUILD_ID \
    FW_VERSION_STRING "+" FW_BUILD_HASH

#endif // VERSION_H
