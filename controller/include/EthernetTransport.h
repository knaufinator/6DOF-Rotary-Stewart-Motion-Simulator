#ifndef ETHERNET_TRANSPORT_H
#define ETHERNET_TRANSPORT_H

#ifdef ENABLE_ETHERNET

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bring up the W5500: esp_eth + lwIP (DHCP) on hardware socket 0, and a hardware UDP socket
 * on ETH_UDP_PORT (socket 1) that carries COBS frames exactly like the USB console does.
 * Received frames are dispatched through cobs_dispatch_frame(..., COBS_SRC_ETHERNET), so the
 * handlers registered with cobs_set_data_handler / cobs_set_cmd_handler see them.
 * RESP / LOG / TEL frames sent with cobs_send() are mirrored to the last UDP peer.
 *
 * Pins and clock come from helpers.h (ETH_SPI_*, ETH_UDP_PORT).
 * @return true on success
 */
bool ethernet_transport_init(void);

/** true once the link is up, a lease is held and the motion socket is open. */
bool ethernet_transport_connected(void);

typedef struct {
    bool     link;
    bool     sock_open;
    uint32_t ip;            // network byte order (lwIP u32)
    uint32_t rx_frames;     // COBS frames decoded from the motion socket
    uint32_t rx_bad;        // datagrams that did not decode
    uint32_t tx_frames;     // frames mirrored to the peer
    uint32_t tx_fail;       // mirrored frames dropped (no room / timeout)
    uint32_t sock_reopens;
    uint32_t rx_age_ms;     // time since the last received frame (0xFFFFFFFF = never)
    uint8_t  peer_ip[4];
    uint16_t peer_port;
    uint32_t polls;         // socket-1 poll iterations
    uint32_t rx_max_us;     // worst datagram read time since last query
    uint32_t tx_max_us;     // worst mirrored send time since last query
    uint32_t spi_calls;     // all SPI transactions (ours + esp_eth)
    uint32_t eth_spi_calls; // transactions issued by esp_eth
    uint32_t spi_wait_max_us;  // worst wait for the SPI mutex since last query
    uint32_t spi_xfer_max_us;  // worst single transaction since last query
    uint32_t spi_wait_sum_us;  // cumulative mutex wait
    uint32_t spi_slow;      // transactions slower than 200 us
    uint8_t  phycfgr;       // W5500 PHYCFGR snapshot
    bool     lwip_stopped;
    uint32_t tx_timeout;    // SENDs that ended in W5500 TIMEOUT (ARP failed)
    uint32_t tx_dropped;    // frames dropped because the previous SEND was still pending
    uint8_t  dhar[6];       // W5500 destination MAC cache for the motion socket
    uint8_t  sn_ir;         // socket interrupt register snapshot
    bool     send_pending;
} eth_stats_t;

void ethernet_transport_stats(eth_stats_t *out);

/** Destination MAC for the current peer (PEERMAC command). Replies then use SEND_MAC, no ARP. */
void ethernet_transport_set_peer_mac(const uint8_t mac[6]);

#ifdef __cplusplus
}
#endif

#endif // ENABLE_ETHERNET
#endif // ETHERNET_TRANSPORT_H
