#ifndef ETHERNET_TRANSPORT_H
#define ETHERNET_TRANSPORT_H

#ifdef ENABLE_ETHERNET

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize W5500 SPI Ethernet and start UDP listener task.
 *
 * Requires ENABLE_ETHERNET to be defined at compile time.
 * Uses pin definitions from helpers.h (ETH_SPI_*).
 *
 * @param process_packet  Callback invoked for each valid 12-byte binary payload
 *                        (6 x uint16_t LE, same format as serial binary protocol).
 * @return true on success
 */
bool ethernet_transport_init(void (*process_packet)(const uint8_t *payload));

/**
 * Returns true once the Ethernet link is up and an IP address has been acquired.
 */
bool ethernet_transport_connected(void);

#ifdef __cplusplus
}
#endif

#endif // ENABLE_ETHERNET
#endif // ETHERNET_TRANSPORT_H
