// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#pragma once

// #define DEBUG_RTP 1

#include <stdint.h>
#include <sys/types.h>

#include "aes67_internal.h"
#include "nettypes.h"

#if !AES67_XMOS
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#endif

// ------- Network Sockets ---------

typedef struct {
    int fd;
    int joined_group;

#if AES67_XMOS
    xtcp_ipaddr_t dest_addr;
    uint16_t dest_port;
    xtcp_ipaddr_t src_addr;
    uint16_t src_port;
#else
    size_t if_index;
    struct sockaddr_storage dest_addr;
    struct sockaddr_storage src_addr;

    union {
        struct ipv6_mreq imr6;
        struct ip_mreq imr;
    };
#endif
} aes67_socket_t;

// ------- RTP packet handling ---------

#if AES67_VLAN_ID != 0
#define ETH_HEADER_LENGTH (18)  // 14 + 4 for VLAN tag
#else
#define ETH_HEADER_LENGTH (14)
#endif
#define IP_HEADER_LENGTH (200)
#define UDP_HEADER_LENGTH (8)
#define RTP_HEADER_LENGTH (12)
#define RTP_MAX_PAYLOAD (1440)

#define ETH_MTU (ETH_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + RTP_HEADER_LENGTH + RTP_MAX_PAYLOAD)

#define RTP_VERSION (2)

typedef struct {
    uint8_t
        version_p_x_cc; // Version(2), Padding(1), Extension(1), CSRC count(4)
    uint8_t m_pt;       // Marker(1), Payload Type(7)
    uint16_t sequence;
    uint32_t timestamp;
    uint32_t ssrc;
} aes67_rtp_header_t;

// RFC 3550 RTP header field access macros - version_p_x_cc byte
#define RTP_VERSION_GET(hdr) (((hdr)->version_p_x_cc >> 6) & 0x3)
#define RTP_VERSION_SET(hdr, v)                                                \
    ((hdr)->version_p_x_cc = ((hdr)->version_p_x_cc & 0x3F) | (((v)&0x3) << 6))

#define RTP_PADDING_GET(hdr) (((hdr)->version_p_x_cc >> 5) & 0x1)
#define RTP_PADDING_SET(hdr, p)                                                \
    ((hdr)->version_p_x_cc = ((hdr)->version_p_x_cc & 0xDF) | (((p)&0x1) << 5))

#define RTP_EXTENSION_GET(hdr) (((hdr)->version_p_x_cc >> 4) & 0x1)
#define RTP_EXTENSION_SET(hdr, x)                                              \
    ((hdr)->version_p_x_cc = ((hdr)->version_p_x_cc & 0xEF) | (((x)&0x1) << 4))

#define RTP_CSRC_COUNT_GET(hdr) ((hdr)->version_p_x_cc & 0xF)
#define RTP_CSRC_COUNT_SET(hdr, cc)                                            \
    ((hdr)->version_p_x_cc = ((hdr)->version_p_x_cc & 0xF0) | ((cc)&0xF))

// RFC 3550 RTP header field access macros - m_pt byte
#define RTP_MARKER_GET(hdr) (((hdr)->m_pt >> 7) & 0x1)
#define RTP_MARKER_SET(hdr, m)                                                 \
    ((hdr)->m_pt = ((hdr)->m_pt & 0x7F) | (((m)&0x1) << 7))

#define RTP_PAYLOAD_TYPE_GET(hdr) ((hdr)->m_pt & 0x7F)
#define RTP_PAYLOAD_TYPE_SET(hdr, pt)                                          \
    ((hdr)->m_pt = ((hdr)->m_pt & 0x80) | ((pt)&0x7F))

// RFC 3550 RTP extension header access macros (when extension bit is set)
// Extension header format: [16-bit profile ID][16-bit length][extension
// data...] Note: ext_ptr should point to the extension header (after main RTP
// header + CSRC list)
#define RTP_EXT_PROFILE_GET(ext_ptr) ntohs(((uint16_t *)(ext_ptr))[0])
#define RTP_EXT_PROFILE_SET(ext_ptr, id)                                       \
    (((uint16_t *)(ext_ptr))[0] = htons(id))

#define RTP_EXT_LENGTH_GET(ext_ptr) ntohs(((uint16_t *)(ext_ptr))[1])
#define RTP_EXT_LENGTH_SET(ext_ptr, len)                                       \
    (((uint16_t *)(ext_ptr))[1] = htons(len))

// Get pointer to extension data (after 4-byte extension header)
#define RTP_EXT_DATA_PTR(ext_ptr) ((uint8_t *)(ext_ptr) + 4)

// Raw Ethernet packet support
// Protocol constants
#define ETH_TYPE_IP 0x0800
#define IP_VERSION_4 4
#define IP_PROTO_UDP 17


// IP header structure
typedef struct _aes67_ip_header {
#if AES67_VLAN_ID != 0
    tagged_ethernet_hdr_t eth;
#else
    ethernet_hdr_t eth;
#endif
    uint8_t version_ihl;
    uint8_t tos;
    uint16_t total_length;
    uint16_t identification;
    uint16_t flags_fragment;
    uint8_t ttl;
    uint8_t protocol;
    uint16_t checksum;
    uint32_t src_ip;
    uint32_t dest_ip;
} aes67_ip_header_t;

// UDP header structure
typedef struct _aes67_udp_header {
    aes67_ip_header_t ip;
    uint16_t src_port;
    uint16_t dest_port;
    uint16_t length;
    uint16_t checksum;
} aes67_udp_header_t;

typedef struct _aes67_rtp_packet {
    // native byte order length of the RTP packet including the RTP header
    // (but no other header lengths). On receipt, padding is removed from
    // this length.
    //
    // this field also aligns the rest of the headers (excepting the Ethernet
    // header) on 32-bit boundaries.
    uint16_t rtp_length;
    aes67_udp_header_t header;
    aes67_rtp_header_t rtp_header;
    uint8_t payload[RTP_MAX_PAYLOAD];
} aes67_rtp_packet_t;

// return a pointer to the start of the Ethernet header
static inline uint8_t *alias
aes67_rtp_packet_start_raw(REFERENCE_PARAM(aes67_rtp_packet_t, packet)) {
#ifdef __XC__
    unsafe { return (uint8_t * alias) &packet.header; }
#else
    return (uint8_t *)&packet->header;
#endif
}

// return the length of the RTP packet including all packet headers
static inline size_t
aes67_rtp_packet_length_raw(REFERENCE_PARAM(const aes67_rtp_packet_t, packet)) {
#ifdef __XC__
    return ETH_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH +
           packet.rtp_length;
#else
    return ETH_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH +
           packet->rtp_length;
#endif
}

// return a pointer to the start of the RTP header
static inline const uint8_t *alias
aes67_const_rtp_packet_start_rtp(REFERENCE_PARAM(const aes67_rtp_packet_t, packet)) {
#ifdef __XC__
    unsafe { return (const uint8_t * alias) &packet.rtp_header; }
#else
    return (const uint8_t *)&packet->rtp_header;
#endif
}

// return a pointer to the start of the RTP header
static inline uint8_t *alias
aes67_rtp_packet_start_rtp(REFERENCE_PARAM(aes67_rtp_packet_t, packet)) {
#ifdef __XC__
    unsafe { return (uint8_t * alias) &packet.rtp_header; }
#else
    return (uint8_t *)&packet->rtp_header;
#endif
}

// return the length of the RTP packet alone
static inline size_t
aes67_rtp_packet_length_rtp(REFERENCE_PARAM(const aes67_rtp_packet_t, packet)) {
#ifdef __XC__
    return packet.rtp_length;
#else
    return packet->rtp_length;
#endif
}

aes67_status_t aes67_rtp_parse(aes67_rtp_packet_t *unsafe packet);
#if AES67_XMOS
aes67_status_t aes67_rtp_recv(CLIENT_INTERFACE(xtcp_if, xtcp),
                              aes67_socket_t *unsafe socket,
                              aes67_rtp_packet_t *unsafe packet);
#else
aes67_status_t aes67_rtp_recv(aes67_socket_t *socket,
                              aes67_rtp_packet_t *packet);
#endif

#define RTP_SEQ_MOD (1 << 16)
#define RTP_MIN_SEQUENTIAL 2

typedef struct {
    uint16_t max_seq;
    uint32_t cycles;
    uint32_t base_seq;
    uint32_t bad_seq;
    uint32_t probation;
    uint32_t received;
    uint32_t expected_prior;
    uint32_t received_prior;
    uint32_t transit;
    uint32_t jitter;
} aes67_rtp_sequence_state_t;

void aes67_rtp_init_sequence(aes67_rtp_sequence_state_t *s, uint16_t seq);

int aes67_rtp_update_sequence(aes67_rtp_sequence_state_t *s, uint16_t seq);

#if AES67_XMOS
aes67_status_t aes67_socket_recv(CLIENT_INTERFACE(xtcp_if, xtcp),
                                 REFERENCE_PARAM(const aes67_socket_t, sock),
                                 ARRAY_OF_SIZE(uint8_t, buffer, buffer_size),
                                 size_t buffer_size, REFERENCE_PARAM(size_t, received_len));
aes67_status_t aes67_socket_send(CLIENT_INTERFACE(xtcp_if, xtcp),
                                 REFERENCE_PARAM(const aes67_socket_t, sock),
                                 ARRAY_OF_SIZE(const uint8_t, buffer, len),
                                 size_t len);

#if AES67_XMOS
aes67_status_t aes67_socket_recv_rtp(chanend xtcp, const aes67_socket_t *sock, aes67_rtp_packet_t *packet);
aes67_status_t aes67_socket_send_rtp(chanend xtcp, const aes67_socket_t *sock, const aes67_rtp_packet_t *packet);
#else
aes67_status_t aes67_socket_recv_rtp(const aes67_socket_t *sock, aes67_rtp_packet_t *packet);
aes67_status_t aes67_socket_send_rtp(const aes67_socket_t *sock, const aes67_rtp_packet_t *packet);
#endif

void aes67_socket_close(CLIENT_INTERFACE(xtcp_if, xtcp),
                        REFERENCE_PARAM(aes67_socket_t, sock));
#else
aes67_status_t aes67_socket_recv(const aes67_socket_t *sock, uint8_t *buffer, size_t buffer_size, size_t *received_len);
aes67_status_t aes67_socket_send(const aes67_socket_t *sock, const uint8_t *buffer, size_t len);
// Note: _rtp functions have unified signatures for both platforms, declared above
void aes67_socket_close(aes67_socket_t *sock);
#endif // AES67_XMOS

// Raw packet send/receive functions
aes67_status_t aes67_raw_send_rtp(const uint8_t src_mac_addr[MACADDR_NUM_BYTES],
                                  streaming chanend c_eth_tx_hp,
                                  REFERENCE_PARAM(const aes67_socket_t, sock),
                                  REFERENCE_PARAM(aes67_rtp_packet_t, packet));

aes67_status_t aes67_raw_recv_rtp(streaming unsafe chanend c_eth_rx_hp,
                                  REFERENCE_PARAM(const aes67_socket_t, sock),
                                  REFERENCE_PARAM(const ethernet_packet_info_t,
                                                  packet_info),
                                  REFERENCE_PARAM(aes67_rtp_packet_t, packet));

void ipv4_to_multicast_mac(xtcp_ipaddr_t ipv4_addr,
                           uint8_t dest_mac[MACADDR_NUM_BYTES]);

// Platform-specific multicast address detection
#if AES67_XMOS
static inline int _is_multicast(const xtcp_ipaddr_t addr) {
    return (addr[0] & 0xF0) == 0xE0;
}
#else
static inline int _is_multicast(struct sockaddr_storage *addr) {
    switch (addr->ss_family) {
    case AF_INET: {
        struct sockaddr_in *addr4 = (struct sockaddr_in *)addr;
        return IN_MULTICAST(ntohl(addr4->sin_addr.s_addr));
    } break;

    case AF_INET6: {
        struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)addr;
        return IN6_IS_ADDR_MULTICAST(&addr6->sin6_addr);
    } break;

    default: {
        return -1;
    }
    }
}
#endif
