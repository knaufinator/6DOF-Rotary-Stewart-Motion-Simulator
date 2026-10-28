#include "app.h"
#include "udp_transport.h"
#include "dev_log.h"
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cctype>

namespace {
const char* field(const char* text,const char* key) {
    const char* result=nullptr;size_t length=std::strlen(key);
    for(const char* p=text;(p=std::strstr(p,key));p+=length) {
        if(p!=text && p[-1]!=',')continue;
        if(result)return nullptr; // ambiguous duplicate key
        result=p+length;
    }
    return result;
}
bool number(const char* text,const char* key,float& value) {
    const char* p=field(text,key);if(!p)return false;
    char* end=nullptr;float parsed=std::strtof(p,&end);
    if(end==p || (*end && *end!=',') || !std::isfinite(parsed))return false;
    value=parsed;return true;
}
void failHandshake(App& app,Entity& e,const char* reason) {
    e.hil_handshake_ok=false;e.hil_handshake_pending=false;
    e.hil_handshake_phase=HandshakePhase::Failed;e.hil_cmd_queue.clear();
    std::snprintf(e.hil_last_error,sizeof(e.hil_last_error),"%s",reason);
    std::snprintf(e.hil_handshake_msg,sizeof(e.hil_handshake_msg),"%s",reason);
    app.log(e.id,"hil","Handshake failed: %s",reason);
}
bool parseBits(const char* line,int& bits,float& max_raw) {
    const char* p=line+5;char* end=nullptr;long parsed=std::strtol(p,&end,10);
    if(end==p || *end!=',' || parsed<8 || parsed>18 || !number(end+1,"max_raw=",max_raw))return false;
    bits=static_cast<int>(parsed);
    // Original firmware may report the historical -2 default before BITS:N.
    const float top=static_cast<float>((1u<<bits)-1u);
    return max_raw==top || max_raw==top-1;
}
}

void App::beginNetworkHil(Entity& e, std::shared_ptr<UdpTransport> up) {
    e.hil_cmd_queue.clear();
    if(!up || !up->isOpen()) {failHandshake(*this,e,"Transport is closed");return;}
    up->setCobsMode(true);
    e.serial = up;
    e.hil_network = true;
    e.enabled = true;
    e.transport.udp_connected = true;
    snprintf(e.transport.udp_ip, sizeof(e.transport.udp_ip), "%s", e.hil_host);
    e.hil_tel_seq = 0;
    e.hil_last_error[0] = '\0';
    if (up->controlConnected()) {
        // Bridge mode (Voron relay): the bridge/mini own identity + cueing. Raw-HIL path,
        // FINGERPRINT handshake bypassed, source driven over the TCP control plane.
        e.hil_cap_raw = true;
        e.hil_raw_mode = true;
        e.hil_handshake_ok = true;
        e.hil_handshake_pending = false;
        e.hil_handshake_phase = HandshakePhase::Ready;
        snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Bridge connected");
        log(e.id, "hil", "Network HIL (bridge): UDP %s:%d, control %s:%d",
            e.hil_host, e.hil_udp_port, e.hil_host, e.hil_tcp_port);
    } else {
        // Direct mode: the peer is the controller's own Ethernet. Same handshake as USB,
        // carried as COBS frames in UDP datagrams; motion is baked DATA18_SEQ.
        e.hil_cap_raw = false;
        e.hil_raw_mode = false;
        e.hil_handshake_ok = false;
        e.cue_sync_valid = false;
        e.hil_handshake_pending = true;
        e.hil_handshake_phase = HandshakePhase::WaitFingerprint;
        e.hil_device_params.clear();
        e.hil_handshake_start = frame_time;
        e.hil_hs_attempts = 1;
        e.hil_hs_last_send = frame_time;
        snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Requesting fingerprint (direct)...");
        log(e.id, "hil", "Network HIL (direct ESP32): UDP %s:%d — handshaking...", e.hil_host, e.hil_udp_port);
        up->write((const uint8_t*)"\0\0\0\0", 4);
        if (!up->localMac().empty()) {
            // Let the board answer us without ARP (Wi-Fi hosts frequently ignore the W5500's ARP).
            std::string pm = "PEERMAC:" + up->localMac();
            up->sendCommand(pm.c_str());
            log(e.id, "hil", "Sent %s", pm.c_str());
        }
        up->sendCommand("FINGERPRINT?");
    }
    settings_dirty = true;
}

Entity* App::findEntity(int id) {
    for (auto& e : entities) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

void App::log(int entity_id, const char* source, const char* fmt, ...) {
    LogEntry entry = {};
    entry.timestamp = frame_time;
    entry.entity_id = entity_id;
    snprintf(entry.source, sizeof(entry.source), "%s", source);

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.message, sizeof(entry.message), fmt, args);
    va_end(args);

    console_log.push_back(entry);
    if ((int)console_log.size() > console_max) {
        console_log.erase(console_log.begin(),
                          console_log.begin() + ((int)console_log.size() - console_max));
    }

    // Also print to stdout
    printf("[E%d/%s] %s\n", entity_id, source, entry.message);
}

// ── HIL Serial Line Handler (multi-step handshake + logging) ─────────
//
// Handshake sequence:
//   1. FINGERPRINT? → verify device identity (MAC, fw version, protocol)
//   2. CONFIG?      → read geometry (RD, PD, L1, L2, height, theta_r, theta_p, servo config)
//   3. BITS?        → read input bit depth
//   4. Validate     → compare device params against app expectations
//   5. Sync         → push any corrections (BITS:N) then enable motion
//
// Motion packets are BLOCKED until phase reaches Ready.

static void advanceHandshake(App& app, Entity& e) {
    if (!e.serial) return;
    switch (e.hil_handshake_phase) {
        case HandshakePhase::WaitFingerprint:
            // Fingerprint received — query geometry next
            e.hil_handshake_phase = HandshakePhase::WaitConfig;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Querying geometry...");
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("CONFIG?");
            break;

        case HandshakePhase::WaitConfig:
            // Geometry received — query bit depth next
            e.hil_handshake_phase = HandshakePhase::WaitBits;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Querying bit depth...");
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("BITS?");
            break;

        case HandshakePhase::WaitBits: {
            // BITS? response received — now push our desired bit depth and wait for echo.
            // Motion stays BLOCKED until firmware confirms the correct BITS:N back.
            auto& dp = e.hil_device_params;

            // Check protocol version compatibility
            if (dp.proto_ver != 1 && dp.proto_ver != 2) {
                app.log(e.id, "hil", "WARNING: Unknown protocol version %d (expected 1 or 2)", dp.proto_ver);
            }

            // Send BITS:N immediately — firmware will echo it back as "BITS:N,max_raw=..."
            {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "BITS:%d", e.config.bit_depth);
                e.serial->sendCommand(cmd);
                app.log(e.id, "hil", "BITS:%d sent — waiting for firmware echo...", e.config.bit_depth);
            }
            e.hil_handshake_phase = HandshakePhase::WaitBitsConfirm;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Confirming bit depth...");
            break;
        }

        case HandshakePhase::WaitBitsConfirm: {
            // Firmware echoed BITS:N confirmation — now validate and finalize.
            e.hil_handshake_phase = HandshakePhase::Validating;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Validating parameters...");

            auto& dp = e.hil_device_params;

            // Check geometry sync between app settings and device
            e.hil_geo_synced = true;
            if (dp.config_received) {
                const auto& geo = e.config.geometry;
                float diffs[] = {
                    fabsf(geo.RD - dp.RD),
                    fabsf(geo.PD - dp.PD),
                    fabsf(geo.ServoArmLengthL1 - dp.L1),
                    fabsf(geo.ConnectingArmLengthL2 - dp.L2),
                    fabsf(geo.platformHeight - dp.height),
                    fabsf(geo.theta_r - dp.theta_r),
                    fabsf(geo.theta_p - dp.theta_p),
                };
                for (int i = 0; i < 7; i++) {
                    if (diffs[i] > 0.05f) { e.hil_geo_synced = false; break; }
                }

                if (!e.hil_geo_synced) {
                    app.log(e.id, "hil", "Geometry out of sync — pushing app settings to device.");
                    app.log(e.id, "hil", "  App:    RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f theta_r=%.2f theta_p=%.2f",
                        geo.RD, geo.PD, geo.ServoArmLengthL1, geo.ConnectingArmLengthL2,
                        geo.platformHeight, geo.theta_r, geo.theta_p);
                    app.log(e.id, "hil", "  Device: RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f theta_r=%.2f theta_p=%.2f",
                        dp.RD, dp.PD, dp.L1, dp.L2, dp.height, dp.theta_r, dp.theta_p);
                    // Push app geometry to device (app is authoritative)
                    char cmd[128];
                    snprintf(cmd, sizeof(cmd), "CONFIG:RD=%.4f", geo.RD);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:PD=%.4f", geo.PD);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:L1=%.4f", geo.ServoArmLengthL1);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:L2=%.4f", geo.ConnectingArmLengthL2);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:height=%.4f", geo.platformHeight);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:theta_r=%.4f", geo.theta_r);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:theta_p=%.4f", geo.theta_p);
                    e.hil_cmd_queue.push_back(cmd);
                } else {
                    app.log(e.id, "hil", "Geometry in sync with device.");
                }
                // Import theta_s from device (not configurable via app UI, firmware-authoritative)
                if (dp.config_received) {
                    bool has_theta_s = false;
                    for (int i = 0; i < 6; i++) if (dp.theta_s[i] != 0.0f) { has_theta_s = true; break; }
                    if (has_theta_s) {
                        for (int i = 0; i < 6; i++) e.config.geometry.theta_s[i] = dp.theta_s[i];
                        e.config.rebuildPlatform();
                    }
                }
            }

            {
                e.hil_handshake_phase = HandshakePhase::Ready;
                e.hil_handshake_ok = true;
                // Reset FK pose so stale pose from prior connection doesn't
                // corrupt the visualization on the new connection.
                memset(e.hil_fk_pose, 0, sizeof(e.hil_fk_pose));
                e.hil_tel_seq = -1;
                e.state.ik_seq = 0;
                if (e.hil_geo_synced) {
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                        "Ready — fw %s, proto %d", dp.fw_version, dp.proto_ver);
                } else {
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                        "Ready — GEOMETRY OUT OF SYNC");
                }

                // Auto-detect platform type from firmware platform_id
                if (strstr(dp.platform_id, "mini") != nullptr) {
                    e.config.platform_type = PlatformType::PWMServo;
                    // Import servo params from device if available
                    if (dp.config_received) {
                        for (int i = 0; i < 6; i++)
                            e.config.servo.center_us[i] = dp.servo_center[i];
                        if (dp.pulse_per_rad > 0.0f)
                            e.config.servo.pulse_per_rad = dp.pulse_per_rad;
                    }
                    app.log(e.id, "hil", "Platform type: PWM Servo (Mini-6DOF)");
                } else {
                    e.config.platform_type = PlatformType::Stepper;
                    app.log(e.id, "hil", "Platform type: Stepper");
                }

                // Send entity color to RGB LED on controller
                {
                    int r = (int)(e.color[0] * 255.0f);
                    int g = (int)(e.color[1] * 255.0f);
                    int b = (int)(e.color[2] * 255.0f);
                    char ledcmd[32];
                    snprintf(ledcmd, sizeof(ledcmd), "LED:%d,%d,%d", r, g, b);
                    e.hil_cmd_queue.push_back(ledcmd);
                }

                // Lock input source to serial, query stats, set telemetry rate + tick rate
                e.hil_cmd_queue.push_back(e.hil_network ? "INPUT:ETHERNET" : "INPUT:SERIAL");
                e.hil_cmd_queue.push_back("INPUT_STAT");
                {
                    char telcmd[32];
                    snprintf(telcmd, sizeof(telcmd), "TELRATE:%d", e.hil_tel_target_hz);
                    e.hil_cmd_queue.push_back(telcmd);
                }
                {
                    char tickcmd[32];
                    snprintf(tickcmd, sizeof(tickcmd), "TICKRATE:%d", e.hil_tick_rate_us);
                    e.hil_cmd_queue.push_back(tickcmd);
                }

                app.log(e.id, "hil", "Handshake complete — transport ready (fw %s, proto %d, bits %d, tel %dHz, tick %dµs)",
                    dp.fw_version, dp.proto_ver,
                    dp.bits_received ? dp.bit_depth : e.config.bit_depth,
                    e.hil_tel_target_hz, e.hil_tick_rate_us);
            }
            e.hil_handshake_pending = false;
            break;
        }
        default:
            break;
    }
}

void App::handleHilLine(int entity_id, const char* line) {
    // Always log the line to console
    log(entity_id, "esp32", "%s", line);

    Entity* e = findEntity(entity_id);
    if (!e) return;

    // ── Phase 1: FINGERPRINT response ────────────────────────────────
    // Format: "FINGERPRINT:AABBCCDDEEFF,fw=1.0.0,proto=1,platform=mini6dof"
    if (strncmp(line, "FINGERPRINT:", 12) == 0) {
        if(e->hil_handshake_phase!=HandshakePhase::WaitFingerprint)return;
        const char* payload = line + 12;

        // Extract MAC (12 hex chars before first comma)
        char mac[16] = {};
        const char* comma1 = strchr(payload, ',');
        if (comma1) {
            int len = (int)(comma1 - payload);
            if (len > 0 && len < (int)sizeof(mac)) {
                memcpy(mac, payload, len);
                mac[len] = '\0';
            }
        }

        // Extract fw version
        char fw[16] = {};
        const char* fw_start = field(payload, "fw=");
        if (fw_start) {
            const char* fw_end = strchr(fw_start, ',');
            int len = fw_end ? (int)(fw_end - fw_start) : (int)strlen(fw_start);
            if (len > 0 && len < (int)sizeof(fw)) {
                memcpy(fw, fw_start, len);
                fw[len] = '\0';
            }
        }

        // Extract protocol version
        int proto = 0;
        const char* proto_start = field(payload, "proto=");
        char* proto_end=nullptr;
        if (proto_start) {
            long value=std::strtol(proto_start,&proto_end,10);
            if(proto_end!=proto_start && (!*proto_end || *proto_end==',') && (value==1 || value==2))proto=int(value);
        }
        bool valid_mac=std::strlen(mac)==12;
        for(const char* p=mac;*p;++p)valid_mac=valid_mac && std::isxdigit(static_cast<unsigned char>(*p));
        if(!valid_mac || !fw[0] || !proto) {
            failHandshake(*this,*e,"Invalid device identity or unsupported protocol");return;
        }

        // Extract platform ID
        char plat[16] = {};
        const char* plat_start = strstr(payload, "platform=");
        if (plat_start) {
            plat_start += 9;
            const char* plat_end = strchr(plat_start, ',');
            int len = plat_end ? (int)(plat_end - plat_start) : (int)strlen(plat_start);
            if (len > 0 && len < (int)sizeof(plat)) {
                memcpy(plat, plat_start, len);
                plat[len] = '\0';
            }
        }

        // Store in device params
        auto& dp = e->hil_device_params;
        snprintf(dp.fingerprint, sizeof(dp.fingerprint), "%s", mac);
        snprintf(dp.fw_version, sizeof(dp.fw_version), "%s", fw);
        snprintf(dp.platform_id, sizeof(dp.platform_id), "%s", plat);
        dp.proto_ver = proto;

        // Also store in legacy fields for UI/save compat
        snprintf(e->hil_fw_version, sizeof(e->hil_fw_version), "%s", fw);
        e->hil_proto_ver = proto;

        // Raw-HIL capability gate: newer firmware advertises "rawhil=1" or a
        // caps token containing "raw" (e.g. caps=raw). Old firmware omits it →
        // hil_cap_raw stays false → baked path unchanged.
        {
            bool cap = false;
            if (strstr(payload, "rawhil=1")) cap = true;
            else { const char* caps = strstr(payload, "caps=");
                   if (caps && strstr(caps, "raw")) cap = true; }
            e->hil_cap_raw = cap;
            if (!cap) e->hil_raw_mode = false;  // can't stream raw to a baked-only device
        }

        // Fingerprint verification — ONLY advance handshake during WaitFingerprint.
        // Stale FINGERPRINT responses (from manual button clicks or previous sessions)
        // must not corrupt the state machine when we're at a different phase.
        bool during_fp_phase = (e->hil_handshake_phase == HandshakePhase::WaitFingerprint);

        if (e->hil_fingerprint[0] == '\0') {
            // First connect — store fingerprint
            snprintf(e->hil_fingerprint, sizeof(e->hil_fingerprint), "%s", mac);
            log(e->id, "hil", "Device paired: %s (fw %s, proto %d, platform %s)", mac, fw, proto, plat);
            settings_dirty = true;
            if (during_fp_phase) advanceHandshake(*this, *e);
        } else if (strcmp(e->hil_fingerprint, mac) == 0) {
            // Same device
            if (during_fp_phase) {
                log(e->id, "hil", "Device verified: %s (fw %s, proto %d, platform %s)", mac, fw, proto, plat);
                advanceHandshake(*this, *e);
            }
            // else: stale duplicate — ignore silently
        } else {
            // DIFFERENT device — reject!
            e->hil_handshake_ok = false;
            e->hil_handshake_pending = false;
            e->hil_cmd_queue.clear();
            e->hil_handshake_phase = HandshakePhase::Failed;
            snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg),
                "FINGERPRINT MISMATCH — expected %s", e->hil_fingerprint);
            snprintf(e->hil_last_error, sizeof(e->hil_last_error),
                "FINGERPRINT MISMATCH\nConnected device: %s\nExpected: %s\nUse \"Clear Fingerprint\" to pair a new device.",
                mac, e->hil_fingerprint);
            log(e->id, "hil", "FINGERPRINT MISMATCH! Expected %s, got %s — disconnecting",
                e->hil_fingerprint, mac);
            log(e->id, "hil", "Clear fingerprint in Settings to pair a new device");
            if (e->serial) {
                e->serial->close();
                e->serial.reset();
            }
            e->transport.usb_connected = false;
            e->hil_auto_connect = false;
        }
        return;
    }

    // ── Phase 2: CONFIG response ─────────────────────────────────────
    // Format: "CONFIG:RD=15.75,PD=16.00,L1=7.25,L2=28.50,height=25.52,theta_r=10.00,theta_p=30.00"
    // Also accept "CONFIG:OK key=value" confirmations after push
    if (strncmp(line, "CONFIG:", 7) == 0) {
        // Skip CONFIG:OK and CONFIG:ERR confirmations (just log them)
        if (strncmp(line + 7, "OK ", 3) == 0 || strncmp(line + 7, "ERR", 3) == 0) {
            // Already logged via general serial output
            return;
        }

        // Accept CONFIG response during handshake or after (for re-query)
        bool during_handshake = (e->hil_handshake_phase == HandshakePhase::WaitConfig);
        if(!during_handshake && e->hil_handshake_phase!=HandshakePhase::Ready)return;
        auto& dp = e->hil_device_params;
        const char* p = line + 7;

        // Parse key=value pairs
        DeviceParams parsed=dp;
        bool valid=number(p,"RD=",parsed.RD) && number(p,"PD=",parsed.PD) &&
            number(p,"L1=",parsed.L1) && number(p,"L2=",parsed.L2) && number(p,"height=",parsed.height) &&
            number(p,"theta_r=",parsed.theta_r) && number(p,"theta_p=",parsed.theta_p);
        valid=valid && parsed.RD>0 && parsed.PD>0 && parsed.L1>0 && parsed.L2>0 && parsed.height>0;
        if(const char* v=field(p,"theta_s=")) {
            for(int i=0;i<6;++i) {
                char* end=nullptr;float angle=std::strtof(v,&end);
                if(end==v || !std::isfinite(angle) || (i<5 ? *end!=',' : (*end && *end!=','))) {valid=false;break;}
                parsed.theta_s[i]=angle;v=end+(*end?1:0);
            }
        }
        if(!valid) {if(during_handshake)failHandshake(*this,*e,"Invalid or incomplete geometry response");return;}
        dp=parsed;
        dp.config_received = true;

        // Re-check geometry sync after any CONFIG update
        if (!during_handshake) {
            const auto& geo = e->config.geometry;
            e->hil_geo_synced = true;
            float diffs[] = {
                fabsf(geo.RD - dp.RD), fabsf(geo.PD - dp.PD),
                fabsf(geo.ServoArmLengthL1 - dp.L1), fabsf(geo.ConnectingArmLengthL2 - dp.L2),
                fabsf(geo.platformHeight - dp.height),
                fabsf(geo.theta_r - dp.theta_r), fabsf(geo.theta_p - dp.theta_p),
            };
            for (int i = 0; i < 7; i++) {
                if (diffs[i] > 0.05f) { e->hil_geo_synced = false; break; }
            }
            if (e->hil_geo_synced) {
                log(e->id, "hil", "Geometry sync confirmed — device matches app settings.");
                snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg),
                    "Ready — fw %s, proto %d", dp.fw_version, dp.proto_ver);
            } else {
                log(e->id, "hil", "Geometry still out of sync after push.");
            }
        }
        // Advance handshake after CONFIG (stepper platforms don't send SERVO:)
        if (during_handshake) {
            advanceHandshake(*this, *e);
        }
        return;
    }

    // Format: "SERVO:center=1500,1500,1500,1500,1500,1500,pulse_per_rad=1018.6"
    if (strncmp(line, "SERVO:", 6) == 0 &&
        (e->hil_handshake_phase == HandshakePhase::WaitConfig || e->hil_handshake_phase == HandshakePhase::WaitBits || e->hil_handshake_phase == HandshakePhase::Ready)) {
        auto& dp = e->hil_device_params;
        const char* p = line + 6;

        const char* center = strstr(p, "center=");
        if (center) {
            sscanf(center + 7, "%d,%d,%d,%d,%d,%d",
                &dp.servo_center[0], &dp.servo_center[1], &dp.servo_center[2],
                &dp.servo_center[3], &dp.servo_center[4], &dp.servo_center[5]);
        }
        const char* ppr = strstr(p, "pulse_per_rad=");
        if (ppr) dp.pulse_per_rad = (float)atof(ppr + 14);

        // CONFIG alone advances the handshake. An early SERVO cannot replace it.
        return;
    }

    // ── Phase 3: BITS? response ─────────────────────────────────────
    // Format: "BITS:12,max_raw=4095" — firmware reports current bit depth
    if (strncmp(line, "BITS:", 5) == 0 &&
        e->hil_handshake_phase == HandshakePhase::WaitBits) {
        auto& dp = e->hil_device_params;
        int bits;float max_raw;
        if(!parseBits(line,bits,max_raw)) {failHandshake(*this,*e,"Invalid bit-depth response");return;}
        dp.bit_depth=bits;dp.max_raw=max_raw;
        dp.bits_received = true;
        // Sends BITS:N and transitions to WaitBitsConfirm
        advanceHandshake(*this, *e);
        return;
    }

    // ── Phase 3b: BITS:N echo confirmation ──────────────────────────
    // Firmware echoes "BITS:N,max_raw=..." after we send BITS:N — confirms
    // the correct bit depth is active before motion TX begins.
    if (strncmp(line, "BITS:", 5) == 0 &&
        e->hil_handshake_phase == HandshakePhase::WaitBitsConfirm) {
        auto& dp = e->hil_device_params;
        int confirmed;float max_raw;
        if(!parseBits(line,confirmed,max_raw) || confirmed!=e->config.bit_depth ||
                max_raw!=static_cast<float>((1u<<confirmed)-1u)) {
            // A delayed BITS? reply can arrive after BITS:N was sent. Keep the
            // gate closed and let the bounded confirmation retry finish/fail.
            log(e->id,"hil","Ignoring unmatched bit-depth confirmation");return;
        }
        dp.max_raw=max_raw;
        dp.bit_depth = confirmed;
        dp.bits_received = true;
        log(e->id, "hil", "BITS:%d confirmed by firmware (max_raw=%.0f)",
            confirmed, dp.max_raw);
        advanceHandshake(*this, *e);
        return;
    }
}

void App::pollHilHandshake(Entity& e) {
    bool disconnected=!e.serial || !e.serial->isOpen();
    if(!disconnected && e.hil_network) {
        auto* network=dynamic_cast<UdpTransport*>(e.serial.get());
        if(network && !network->direct() && !network->controlConnected())disconnected=true;
    }
    if(disconnected) {
        e.hil_handshake_ok=false;e.hil_handshake_pending=false;
        e.hil_handshake_phase=HandshakePhase::Idle;e.cue_sync_valid=false;
        e.hil_cmd_queue.clear();return;
    }
    // Connection setup sends immediately; retry without waiting for telemetry.
    if (e.serial && e.serial->isOpen()
        && e.hil_handshake_phase == HandshakePhase::WaitFingerprint
        && e.hil_handshake_pending) {

        double since_last = frame_time - e.hil_hs_last_send;

        // Retry every 0.5s — first attempt was sent on connect
        bool should_try = since_last > 0.5;

        const int hs_max_attempts = e.hil_network ? 30 : 10;   // Wi-Fi hosts can take seconds to answer the first ARP
        if (should_try && e.hil_hs_attempts < hs_max_attempts) {
            e.hil_hs_attempts++;
            e.hil_hs_last_send = frame_time;
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("FINGERPRINT?");
            DEV_LOG("hil", "Sending FINGERPRINT? (attempt %d/10)", e.hil_hs_attempts);
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                     "Requesting fingerprint...%s",
                     e.hil_hs_attempts > 1 ? " (retrying)" : "");
        }

        // Give up after the transport-specific attempt budget
        if (e.hil_hs_attempts >= hs_max_attempts && since_last > 1.0) {
            e.hil_handshake_phase = HandshakePhase::Failed;
            e.hil_handshake_ok = false;
            e.hil_handshake_pending = false;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                     "No response after %d attempts", e.hil_hs_attempts);
            snprintf(e.hil_last_error, sizeof(e.hil_last_error),
                     "No FINGERPRINT response after %d attempts.\nDevice may be offline, crashed, or wrong firmware.",
                     e.hil_hs_attempts);
            log(e.id, "hil", "Handshake failed — no FINGERPRINT response after %d attempts",
                e.hil_hs_attempts);
        }
    }

    // ── Retry for CONFIG? / BITS? / BITS:N confirm phases ───────
    // If the response is lost, resend with flush.
    if (e.serial && e.serial->isOpen() && e.hil_handshake_pending
        && (e.hil_handshake_phase == HandshakePhase::WaitConfig
         || e.hil_handshake_phase == HandshakePhase::WaitBits
         || e.hil_handshake_phase == HandshakePhase::WaitBitsConfirm)) {
        bool is_config  = (e.hil_handshake_phase == HandshakePhase::WaitConfig);
        bool is_confirm = (e.hil_handshake_phase == HandshakePhase::WaitBitsConfirm);
        char bits_set_cmd[32];
        snprintf(bits_set_cmd, sizeof(bits_set_cmd), "BITS:%d", e.config.bit_depth);
        const char* cmd = is_config ? "CONFIG?" : (is_confirm ? bits_set_cmd : "BITS?");
        double since_last = frame_time - e.hil_hs_last_send;
        if (since_last > 0.5 && e.hil_hs_attempts < 5) {
            e.hil_hs_attempts++;
            e.hil_hs_last_send = frame_time;
            // Full flush to drain any residual COBS state
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand(cmd);
            DEV_LOG("hil", "Retrying %s (attempt %d/5)", cmd, e.hil_hs_attempts);
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                     "%s (retry %d)...",
                     is_config ? "Querying geometry" : "Querying bit depth",
                     e.hil_hs_attempts);
        }
        if (e.hil_hs_attempts >= 5 && since_last > 2.0) {
            // Capture label BEFORE changing phase
            char fail_msg[128];
            snprintf(fail_msg, sizeof(fail_msg),
                     "No response to %s after 5 attempts", cmd);
            e.hil_handshake_phase = HandshakePhase::Failed;
            e.hil_handshake_ok = false;
            e.hil_handshake_pending = false;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "%s", fail_msg);
            snprintf(e.hil_last_error, sizeof(e.hil_last_error), "%s", fail_msg);
            log(e.id, "hil", "Handshake failed — %s", fail_msg);
        }
    }

}
