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

#if AES67_XMOS
aes67_status_t aes67_socket_open_recv(CLIENT_INTERFACE(xtcp_if, xtcp),
                                      REFERENCE_PARAM(aes67_socket_t, sock),
                                      const char address[],
                                      const char _port[]);
aes67_status_t aes67_socket_open_send(CLIENT_INTERFACE(xtcp_if, xtcp),
                                      REFERENCE_PARAM(aes67_socket_t, sock),
                                      const char address[],
                                      const char _port[]);
int aes67_socket_recv(CLIENT_INTERFACE(xtcp_if, xtcp),
                      REFERENCE_PARAM(const aes67_socket_t, sock),
                      ARRAY_OF_SIZE(uint8_t, data, len),
                      size_t len);
int aes67_socket_send(CLIENT_INTERFACE(xtcp_if, xtcp),
                      REFERENCE_PARAM(const aes67_socket_t, sock),
                      ARRAY_OF_SIZE(uint8_t, data, len),
                      size_t len);
void aes67_socket_close(CLIENT_INTERFACE(xtcp_if, xtcp),
                        REFERENCE_PARAM(aes67_socket_t, sock));
#else
aes67_status_t aes67_socket_open_recv(aes67_socket_t *sock,
                                      const char *address,
                                      const char *,
                                      const char *ifname);
aes67_status_t aes67_socket_open_send(aes67_socket_t *sock,
                                      const char *address,
                                      const char *,
                                      const char *ifname);
int aes67_socket_recv(const aes67_socket_t *sock, uint8_t *data, size_t len);
int aes67_socket_send(const aes67_socket_t *sock, uint8_t *data, size_t len);
void aes67_socket_close(aes67_socket_t *sock);
#endif // AES67_XMOS

// ------- RTP packet handling ---------

#define RTP_MAX_PAYLOAD (1440)
#define RTP_HEADER_LENGTH (12)
#define RTP_VERSION (2)

typedef struct {
    uint8_t version;
    uint8_t padding;
    uint8_t extension;
    uint8_t csrc_count;
    uint8_t marker;
    uint8_t payload_type;

    uint16_t sequence;
    uint32_t timestamp;
    uint32_t ssrc;

    uint16_t payload_length;
#ifdef __XC__
    uint8_t *unsafe payload;
#else
    uint8_t *payload;
#endif

    uint16_t length;
    uint8_t buffer[1500];
} aes67_rtp_packet_t;

int aes67_rtp_parse(aes67_rtp_packet_t *unsafe packet);
#if AES67_XMOS
int aes67_rtp_recv(CLIENT_INTERFACE(xtcp_if, xtcp),
                   aes67_socket_t *unsafe socket,
                   aes67_rtp_packet_t *unsafe packet);
#else
int aes67_rtp_recv(aes67_socket_t *socket, aes67_rtp_packet_t *packet);
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
