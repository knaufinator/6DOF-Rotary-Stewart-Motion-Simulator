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
 * Motion RX characteristic (0xFF01): writable, receives 12-byte binary payloads
 * Status TX characteristic (0xFF02): notify, sends status/telemetry
 * Accel  RX characteristic (0xFF03): writable, receives 24-byte accel packets
 *
 * @param process_packet  Callback for each valid 12-byte binary payload
 * @return true on success
 */
bool ble_transport_init(void (*process_packet)(const uint8_t *payload));

/**
 * Register callback for accelerometer/gyro data packets.
 * Called when 24 bytes (6 x float32 LE) arrive on the accel characteristic.
 * Data order: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
 * Units: m/s² for accel, rad/s for gyro (Android TYPE_ACCELEROMETER + TYPE_GYROSCOPE)
 *
 * @param process_accel  Callback receiving pointer to 6 floats
 */
void ble_transport_set_accel_callback(void (*process_accel)(const float *data));

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
