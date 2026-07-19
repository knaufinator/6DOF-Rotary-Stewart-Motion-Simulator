#ifndef VERSION_H
#define VERSION_H

// ── Firmware Version ─────────────────────────────────────────────────
// Bump MAJOR for breaking changes (protocol, config format)
// Bump MINOR for new features
// Bump PATCH for bug fixes
// Build date/time is appended automatically by the compiler

#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  0
#define FW_VERSION_PATCH  0

// Protocol version — must match main controller protocol for app compatibility
// App checks this on connect to ensure compatibility
#define FW_PROTOCOL_VERSION  1

#define FW_BUILD_DATE     __DATE__
#define FW_BUILD_TIME     __TIME__

// String helper macros
#define _FW_STR(x) #x
#define _FW_XSTR(x) _FW_STR(x)
#define FW_VERSION_STRING \
    _FW_XSTR(FW_VERSION_MAJOR) "." _FW_XSTR(FW_VERSION_MINOR) "." \
    _FW_XSTR(FW_VERSION_PATCH)

// Platform variant identifier
#define FW_PLATFORM_ID    "mini-6dof"

#endif // VERSION_H
