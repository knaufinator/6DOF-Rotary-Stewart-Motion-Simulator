#ifndef WIFI_TRANSPORT_H
#define WIFI_TRANSPORT_H

#ifdef ENABLE_WIFI

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize WiFi STA mode and start UDP listener task.
 *
 * Requires ENABLE_WIFI to be defined at compile time.
 * Loads saved SSID/password from NVS and auto-connects.
 *
 * @param process_packet  Callback invoked for each valid 12-byte binary payload
 *                        (6 x uint16_t LE, same format as serial binary protocol).
 * @return true on success
 */
bool wifi_transport_init(void (*process_packet)(const uint8_t *payload));

/**
 * Returns true once WiFi is connected and an IP address has been acquired.
 */
bool wifi_transport_connected(void);

/**
 * Get the current WiFi IP address as a string.
 * Returns "0.0.0.0" if not connected.
 */
const char* wifi_transport_get_ip(void);

/**
 * Get the current WiFi RSSI (signal strength in dBm).
 * Returns 0 if not connected.
 */
int wifi_transport_get_rssi(void);

/**
 * Get the SSID currently connected to (or attempting).
 */
const char* wifi_transport_get_ssid(void);

/**
 * Set WiFi credentials (does not connect yet — call wifi_transport_connect).
 */
void wifi_transport_set_credentials(const char *ssid, const char *password);

/**
 * Connect to WiFi with the current credentials.
 * Non-blocking — connection happens in background.
 */
void wifi_transport_connect(void);

/**
 * Disconnect from WiFi.
 */
void wifi_transport_disconnect(void);

/**
 * Save current WiFi credentials to NVS for auto-connect on boot.
 * @return 0 on success, non-zero on failure
 */
int wifi_transport_save_credentials(void);

/**
 * Load WiFi credentials from NVS.
 * @return 0 on success (credentials loaded), non-zero if no saved credentials
 */
int wifi_transport_load_credentials(void);

/**
 * Get WiFi connection state as a string: "connected", "connecting", "disconnected", "failed"
 */
const char* wifi_transport_state_str(void);

#ifdef __cplusplus
}
#endif

#endif // ENABLE_WIFI
#endif // WIFI_TRANSPORT_H
