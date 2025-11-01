// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#pragma once

// #define DEBUG_RTP 1

#include <stdint.h>
#include <sys/types.h>

#include "aes67_internal.h"

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

// ------- RTCP packet handling ---------

// RTCP packet types (RFC 3550)
#define RTCP_SR   200  // Sender Report
#define RTCP_RR   201  // Receiver Report
#define RTCP_SDES 202  // Source Description
#define RTCP_BYE  203  // Goodbye
#define RTCP_APP  204  // Application-defined

// RTCP SDES item types
#define RTCP_SDES_END   0
#define RTCP_SDES_CNAME 1
#define RTCP_SDES_NAME  2
#define RTCP_SDES_EMAIL 3
#define RTCP_SDES_PHONE 4
#define RTCP_SDES_LOC   5
#define RTCP_SDES_TOOL  6
#define RTCP_SDES_NOTE  7
#define RTCP_SDES_PRIV  8

// RTCP common header (first 32 bits of all RTCP packets)
typedef struct {
    uint8_t version_p_rc;  // Version(2), Padding(1), Reception report count(5) or Source count(5)
    uint8_t packet_type;   // RTCP packet type
    uint16_t length;       // Length in 32-bit words minus 1
} aes67_rtcp_header_t;

// RTCP header field access macros
#define RTCP_VERSION_GET(hdr) (((hdr)->version_p_rc >> 6) & 0x3)
#define RTCP_VERSION_SET(hdr, v) \
    ((hdr)->version_p_rc = ((hdr)->version_p_rc & 0x3F) | (((v) & 0x3) << 6))

#define RTCP_PADDING_GET(hdr) (((hdr)->version_p_rc >> 5) & 0x1)
#define RTCP_PADDING_SET(hdr, p) \
    ((hdr)->version_p_rc = ((hdr)->version_p_rc & 0xDF) | (((p) & 0x1) << 5))

#define RTCP_RC_GET(hdr) ((hdr)->version_p_rc & 0x1F)
#define RTCP_RC_SET(hdr, rc) \
    ((hdr)->version_p_rc = ((hdr)->version_p_rc & 0xE0) | ((rc) & 0x1F))

// RTCP Sender Report
typedef struct {
    uint32_t ntp_timestamp_msw;  // NTP timestamp, most significant word
    uint32_t ntp_timestamp_lsw;  // NTP timestamp, least significant word
    uint32_t rtp_timestamp;      // RTP timestamp
    uint32_t sender_packet_count; // Sender's packet count
    uint32_t sender_octet_count;  // Sender's octet count
} aes67_rtcp_sender_info_t;

// RTCP Reception Report Block
typedef struct {
    uint32_t ssrc;              // SSRC of source
    uint32_t fraction_lost;     // Fraction lost since last SR/RR (8 bits) + cumulative lost (24 bits)
    uint32_t ext_highest_seq;   // Extended highest sequence number received
    uint32_t interarrival_jitter; // Interarrival jitter
    uint32_t lsr;               // Last SR timestamp
    uint32_t dlsr;              // Delay since last SR
} aes67_rtcp_reception_report_t;

// Reception report field access macros
#define RTCP_RR_FRACTION_LOST_GET(rr) (((rr)->fraction_lost >> 24) & 0xFF)
#define RTCP_RR_FRACTION_LOST_SET(rr, f) \
    ((rr)->fraction_lost = ((rr)->fraction_lost & 0x00FFFFFF) | (((f) & 0xFF) << 24))

#define RTCP_RR_CUMULATIVE_LOST_GET(rr) ((rr)->fraction_lost & 0x00FFFFFF)
#define RTCP_RR_CUMULATIVE_LOST_SET(rr, c) \
    ((rr)->fraction_lost = ((rr)->fraction_lost & 0xFF000000) | ((c) & 0x00FFFFFF))

// RTCP Sender Report packet
typedef struct {
    aes67_rtcp_header_t header;
    uint32_t ssrc;
    aes67_rtcp_sender_info_t sender_info;
    // Variable number of reception reports follow
    // aes67_rtcp_reception_report_t reports[RTCP_RC_GET(&header)];
} aes67_rtcp_sr_t;

// RTCP Receiver Report packet
typedef struct {
    aes67_rtcp_header_t header;
    uint32_t ssrc;
    // Variable number of reception reports follow
    // aes67_rtcp_reception_report_t reports[RTCP_RC_GET(&header)];
} aes67_rtcp_rr_t;

// RTCP SDES item
typedef struct {
    uint8_t type;
    uint8_t length;
    // Variable length data follows
    // uint8_t data[length];
} aes67_rtcp_sdes_item_t;

// RTCP SDES chunk
typedef struct {
    uint32_t ssrc;
    // Variable number of SDES items follow, terminated by RTCP_SDES_END
    // aes67_rtcp_sdes_item_t items[];
} aes67_rtcp_sdes_chunk_t;

// RTCP Source Description packet
typedef struct {
    aes67_rtcp_header_t header;
    // Variable number of SDES chunks follow
    // aes67_rtcp_sdes_chunk_t chunks[RTCP_RC_GET(&header)];
} aes67_rtcp_sdes_t;

// RTCP BYE packet
typedef struct {
    aes67_rtcp_header_t header;
    // Variable number of SSRCs follow
    // uint32_t ssrcs[RTCP_RC_GET(&header)];
    // Optional reason string may follow (length byte + string)
} aes67_rtcp_bye_t;

// RTCP APP packet
typedef struct {
    aes67_rtcp_header_t header;
    uint32_t ssrc;
    uint32_t name;  // 4 ASCII characters
    // Variable application-dependent data follows
    // uint8_t data[];
} aes67_rtcp_app_t;

// Generic RTCP packet union
typedef union {
    aes67_rtcp_header_t header;
    aes67_rtcp_sr_t sr;
    aes67_rtcp_rr_t rr;
    aes67_rtcp_sdes_t sdes;
    aes67_rtcp_bye_t bye;
    aes67_rtcp_app_t app;
} aes67_rtcp_packet_t;

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
