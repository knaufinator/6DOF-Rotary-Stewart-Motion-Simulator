#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#ifdef ENABLE_BLE

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize BLE GATT server with a custom motion data service.
 *
 * Service UUID:  4210xxxx-0001-1000-8000-00805f9b34fb
 * Motion RX characteristic: writable, receives 12-byte binary payloads
 * Status TX characteristic: notify, sends status/telemetry
 *
 * @param process_packet  Callback for each valid 12-byte binary payload
 * @return true on success
 */
bool ble_transport_init(void (*process_packet)(const uint8_t *payload));

/**
 * Returns true when a BLE client is connected.
 */
bool ble_transport_connected(void);

/**
 * Get BLE connection state as string.
 */
const char* ble_transport_state_str(void);

/**
 * Send notification data to connected BLE client (for telemetry).
 * @param data  Pointer to data bytes
 * @param len   Length of data
 * @return 0 on success
 */
int ble_transport_notify(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // ENABLE_BLE
#endif // BLE_TRANSPORT_H
