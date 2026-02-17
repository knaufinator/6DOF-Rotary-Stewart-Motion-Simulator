#pragma once
/*
 * Dev Logger — lightweight file-based debug logging for development.
 *
 * Usage:
 *   DEV_LOG("hil", "Fingerprint received: %s", mac);
 *   DEV_LOG("serial", "RX %d bytes", n);
 *   DEV_WARN("hil", "Handshake timeout on %s", port);
 *   DEV_ERR("serial", "Port disconnected: %s", port);
 *
 * Output goes to app_debug.log (auto-created next to executable).
 * Set DEV_LOG_ENABLED=0 to compile out all logging at zero cost.
 */

#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <mutex>

#ifndef DEV_LOG_ENABLED
#define DEV_LOG_ENABLED 1
#endif

#if DEV_LOG_ENABLED

class DevLogger {
public:
    static DevLogger& instance() {
        static DevLogger s;
        return s;
    }

    void write(const char* level, const char* category, const char* fmt, ...) {
        std::lock_guard<std::mutex> lock(m_mutex);
        ensureOpen();
        if (!m_fp) return;

        // Timestamp
        time_t now = time(nullptr);
        struct tm t;
#ifdef _WIN32
        localtime_s(&t, &now);
#else
        localtime_r(&now, &t);
#endif
        fprintf(m_fp, "%02d:%02d:%02d [%-5s] [%-8s] ",
            t.tm_hour, t.tm_min, t.tm_sec, level, category);

        va_list args;
        va_start(args, fmt);
        vfprintf(m_fp, fmt, args);
        va_end(args);

        fprintf(m_fp, "\n");
        fflush(m_fp);
    }

    void close() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_fp) { fclose(m_fp); m_fp = nullptr; }
    }

private:
    DevLogger() : m_fp(nullptr) {}
    ~DevLogger() { close(); }

    void ensureOpen() {
        if (!m_fp) {
            m_fp = fopen("app_debug.log", "a");
            if (m_fp) {
                fprintf(m_fp, "\n========== Session started ==========\n");
                fflush(m_fp);
            }
        }
    }

    FILE* m_fp;
    std::mutex m_mutex;
};

#define DEV_LOG(cat, ...)  DevLogger::instance().write("INFO",  cat, __VA_ARGS__)
#define DEV_WARN(cat, ...) DevLogger::instance().write("WARN",  cat, __VA_ARGS__)
#define DEV_ERR(cat, ...)  DevLogger::instance().write("ERROR", cat, __VA_ARGS__)

#else

#define DEV_LOG(cat, ...)  ((void)0)
#define DEV_WARN(cat, ...) ((void)0)
#define DEV_ERR(cat, ...)  ((void)0)

#endif
