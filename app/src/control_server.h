#pragma once
/*
 * Control Server — localhost TCP command API for programmatic / MCP driving.
 *
 * Protocol: newline-delimited JSON over TCP (127.0.0.1:<port>, default 8770).
 *   request : {"id":<int>,"cmd":"<name>","args":{...}}\n
 *   response: {"id":<int>,"ok":true,"result":{...}}\n
 *          or {"id":<int>,"ok":false,"error":"<msg>"}\n
 *
 * A background accept/reader thread parses requests and enqueues them; drain()
 * — called once per frame on the render thread — executes every queued command
 * against g_app and completes the caller's response. All g_app access therefore
 * stays single-threaded (same thread as update()), so no extra locking is needed
 * beyond the request queue itself.
 */
#include <atomic>

class ControlServer {
public:
    bool start(int port);   // spin up listener thread; false if bind fails
    void stop();            // stop threads, close sockets, unblock waiters
    void drain();           // execute queued commands (render thread only)
    bool running() const { return m_running.load(); }
    int  port()    const { return m_port; }
private:
    std::atomic<bool> m_running{false};
    int m_port = 0;
};

extern ControlServer g_ctrl;

// Implemented in main.cpp (needs the GL loader + GLFW window handle).
// Reads the just-rendered back buffer into a PNG. Must be called on the
// render thread with the GL context current (drain() satisfies this).
bool CaptureWindowPNG(const char* path, int* out_w, int* out_h);
