/*
 * SimTools UDP Plugin
 * ===================
 * Receives 6-axis motion data via UDP from SimTools or compatible software.
 * Supports binary (8/10/12/14/16-bit) and CSV text formats.
 *
 * Build:
 *   Windows:  cl /LD /I ../src plugin_simtools_udp.c ws2_32.lib
 *   Linux:    gcc -shared -fPIC -I ../src -o plugin_simtools_udp.so plugin_simtools_udp.c
 */

#include "plugin_api.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef SOCKET sock_t;
  #define SOCK_INVALID INVALID_SOCKET
  #define SOCK_ERROR   SOCKET_ERROR
  #define sock_close   closesocket
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <errno.h>
  #include <fcntl.h>
  typedef int sock_t;
  #define SOCK_INVALID (-1)
  #define SOCK_ERROR   (-1)
  #define sock_close   close
#endif

/* ── Plugin state ─────────────────────────────────────────────────── */

static int   s_port       = 4123;
static int   s_bit_depth  = 12;
static float s_values[6]  = {0};
static sock_t s_sock      = SOCK_INVALID;
static int   s_running    = 0;
static int   s_wsa_init   = 0;

/* Stats */
static int   s_packets_rx = 0;
static int   s_packets_bad = 0;

/* ── Parameter declarations ───────────────────────────────────────── */

static const char* s_bit_labels = "8-bit\00010-bit\00012-bit\00014-bit\00016-bit\000";

static const StewartParamDef s_params[] = {
    {
        "port", "UDP Port", "Port to listen for SimTools data (default 4123)",
        STEWART_PARAM_INT, 4123.0f, 1024.0f, 65535.0f, NULL
    },
    {
        "bit_depth", "Bit Depth", "Resolution of incoming data values",
        STEWART_PARAM_ENUM, 2.0f, 0.0f, 4.0f, NULL  /* index: 0=8, 1=10, 2=12, 3=14, 4=16 */
    },
};

/* ── Plugin info ──────────────────────────────────────────────────── */

static const StewartPluginInfo s_info = {
    STEWART_PLUGIN_API_VERSION,
    "SimTools UDP",
    "Stewart Platform Project",
    "1.0.0",
    "Receives 6-axis motion data via UDP from SimTools or compatible software. "
    "Supports binary (8-16 bit) and CSV text formats.",
    0,   /* preferred_rate_hz */
    6,   /* axis_count */
    { NULL, NULL, NULL, NULL, NULL, NULL },
    sizeof(s_params) / sizeof(s_params[0]),
    s_params
};

/* ── Helpers ──────────────────────────────────────────────────────── */

static int bit_depth_from_index(int idx) {
    const int opts[] = {8, 10, 12, 14, 16};
    if (idx < 0) idx = 0;
    if (idx > 4) idx = 4;
    return opts[idx];
}

static int open_socket(void) {
    sock_t sock;
    struct sockaddr_in addr;
    int reuse = 1;

#ifdef _WIN32
    if (!s_wsa_init) {
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return -1;
        s_wsa_init = 1;
    }
#endif

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == SOCK_INVALID) return -1;

    /* Non-blocking so process() doesn't stall the app */
#ifdef _WIN32
    {
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);
    }
#else
    {
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }
#endif

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)s_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCK_ERROR) {
        sock_close(sock);
        return -1;
    }

    s_sock = sock;
    s_running = 1;
    s_packets_rx = 0;
    s_packets_bad = 0;
    return 0;
}

static void close_socket(void) {
    if (s_sock != SOCK_INVALID) {
        sock_close(s_sock);
        s_sock = SOCK_INVALID;
    }
    s_running = 0;
}

/* Drain all pending UDP packets and parse the most recent one */
static void drain_udp(void) {
    char buf[256];
    struct sockaddr_in from;
    int n;
    int got_any = 0;
    float last_values[6] = {0};

    if (s_sock == SOCK_INVALID) return;

    for (;;) {
#ifdef _WIN32
        int from_len = sizeof(from);
#else
        socklen_t from_len = sizeof(from);
#endif
        n = recvfrom(s_sock, buf, sizeof(buf) - 1, 0,
                     (struct sockaddr*)&from, &from_len);
        if (n <= 0) break;

        s_packets_rx++;

        int bd = bit_depth_from_index(s_bit_depth);
        float values[6] = {0};
        int parsed = 0;

        /* Binary parsing: 8-bit = 6 bytes, 10-16 bit = 12 bytes LE */
        if (bd <= 8 && n >= 6 && n < 20) {
            unsigned char* ub = (unsigned char*)buf;
            int i;
            for (i = 0; i < 6; i++)
                values[i] = ((float)ub[i] - 128.0f) / 128.0f * 100.0f;
            parsed = 1;
        } else if (n >= 12 && n < 20) {
            unsigned char* ub = (unsigned char*)buf;
            float max_val = (float)((1 << bd) - 1);
            float center = max_val * 0.5f;
            int i;
            for (i = 0; i < 6; i++) {
                unsigned short raw = (unsigned short)(ub[i*2] | (ub[i*2+1] << 8));
                values[i] = ((raw - center) / center) * 100.0f;
                if (values[i] > 100.0f) values[i] = 100.0f;
                if (values[i] < -100.0f) values[i] = -100.0f;
            }
            parsed = 1;
        }

        /* Fallback: CSV text */
        if (!parsed) {
            buf[n] = '\0';
            if (sscanf(buf, "%f,%f,%f,%f,%f,%f",
                       &values[0], &values[1], &values[2],
                       &values[3], &values[4], &values[5]) >= 6) {
                parsed = 1;
                /* Normalize if values look like raw integers */
                {
                    int needs_norm = 0, i;
                    for (i = 0; i < 6; i++) {
                        if (values[i] > 100.5f || values[i] < -100.5f) {
                            needs_norm = 1; break;
                        }
                    }
                    if (needs_norm) {
                        float max_val = (float)((1 << bd) - 1);
                        float center = max_val * 0.5f;
                        for (i = 0; i < 6; i++) {
                            values[i] = ((values[i] - center) / center) * 100.0f;
                            if (values[i] > 100.0f) values[i] = 100.0f;
                            if (values[i] < -100.0f) values[i] = -100.0f;
                        }
                    }
                }
            }
        }

        if (parsed) {
            memcpy(last_values, values, sizeof(last_values));
            got_any = 1;
        } else {
            s_packets_bad++;
        }
    }

    if (got_any) {
        memcpy(s_values, last_values, sizeof(s_values));
    }
}

/* ── Entry points ─────────────────────────────────────────────────── */

STEWART_EXPORT const StewartPluginInfo* stewart_plugin_info(void) {
    return &s_info;
}

STEWART_EXPORT int stewart_plugin_init(float sample_rate) {
    (void)sample_rate;
    memset(s_values, 0, sizeof(s_values));
    s_packets_rx = 0;
    s_packets_bad = 0;

    if (open_socket() != 0) {
        return -1;  /* bind failed */
    }
    return 0;
}

STEWART_EXPORT int stewart_plugin_process(StewartPluginContext* ctx) {
    drain_udp();
    memcpy(ctx->output, s_values, sizeof(s_values));
    return 0;
}

STEWART_EXPORT void stewart_plugin_shutdown(void) {
    close_socket();
    memset(s_values, 0, sizeof(s_values));
}

STEWART_EXPORT void stewart_plugin_set_param(const char* name, float value) {
    if (strcmp(name, "port") == 0) {
        int new_port = (int)value;
        if (new_port != s_port) {
            s_port = new_port;
            /* Rebind if running */
            if (s_running) {
                close_socket();
                open_socket();
            }
        }
    } else if (strcmp(name, "bit_depth") == 0) {
        s_bit_depth = (int)value;
    }
}
