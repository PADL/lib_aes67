// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#pragma once

#include <stdint.h>
#include <sys/types.h>

#include "aes67_internal.h"

#if AES67_XMOS
#define NI_MAXHOST 256
#define NI_MAXSERV 32
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#endif // AES67_XMOS

// ------- SAP packet handling ---------

#define AES67_SAP_ADDRESS_LOCAL "239.255.255.255"
#define AES67_SAP_ADDRESS_ORG "239.195.255.255"
#define AES67_SAP_ADDRESS_GLOBAL "224.2.127.254"
#define AES67_SAP_PORT 9875
#define AES67_SAP_PORT_STRING STRINGIFY(9875)
#define AES67_SAP_VERSION_1 1

#define AES67_SDP_MIME_TYPE "application/sdp"
#define AES67_SDP_MIME_TYPE_LEN (sizeof("application/sdp") - 1)
#define AES67_SDP_MAX_LEN (2048)

#define AES67_SAP_MAX_HEADER (36)
#define AES67_SAP_MAX_LEN (AES67_SAP_MAX_HEADER + AES67_SDP_MAX_LEN)

#define AES67_SAP_PERIODIC_TIME (XS1_TIMER_HZ * 10) // 10 seconds
#define AES67_SAP_TIMEOUT (60 * 6) // expire after this if not readvertisement, in units of AES67_SAP_PERIODIC_TIME

typedef struct _aes67_sap {
    aes67_sap_message_type_t message_type;
    uint16_t message_id_hash;
#if AES67_XMOS
    char message_source[sizeof("255.255.255.255")];
#else
    char message_source[INET6_ADDRSTRLEN];
#endif
    char sdp[AES67_SDP_MAX_LEN];
} aes67_sap_t;

aes67_status_t aes67_sap_parse(ARRAY_OF_SIZE(const uint8_t, data, data_len),
                               size_t data_len,
                               REFERENCE_PARAM(aes67_sap_t, sap));

// Include the RTP header for socket type
#include "rtp_protocol.h"

int aes67_sap_generate(REFERENCE_PARAM(const aes67_socket_t, sock),
                       const char sdp[],
                       uint8_t message_type,
                       ARRAY_OF_SIZE(uint8_t, buffer, buffer_len),
                       size_t buffer_len);
#if AES67_XMOS
int aes67_sap_send_sdp_string(CLIENT_INTERFACE(xtcp_if, xtcp),
                              REFERENCE_PARAM(const aes67_socket_t, sock),
                              const char sdp[],
                              uint8_t message_type);
#else
int aes67_sap_send_sdp_string(const aes67_socket_t *sock,
                              const char *sdp,
                              uint8_t message_type);
#endif

// ------- SDP handling ---------

typedef struct _aes67_sdp {
    char address[NI_MAXHOST]; // c=
#ifdef __XC__
    char __port[NI_MAXSERV]; // m=
#else
    char port[NI_MAXSERV]; // m=
#endif

    char session_id[256];              // o=
    char session_origin[256];          // o=
    aes67_session_name_t session_name; // s=
    char information[256];             // i=

    int payload_type;       // m=
    int encoding;           // a=rtpmap
    int sample_size;        // Size of the samples based on the encoding (16/24)
    int sample_rate;        // a=rtpmap
    int channel_count;      // a=rtpmap
    double packet_duration; // a=ptime

    char ptp_gmid[24];      // a=ts-refclk
    int ptp_domain;         // a=ts-refclk
    uint64_t clock_offset;  // a=mediaclk

    uint32_t framecount;    // a=framecount
    uint32_t sync_time;     // a=sync-time

#define AES67_SDP_FLAG_PERMANENT 0x1
    uint32_t flags;
    uint32_t timestamp; // this is in units of AES67_SAP_PERIODIC_TIME
} aes67_sdp_t;

#if AES67_FAST_CONNECT_ENABLED
typedef struct _aes67_sdp_fast_connect {
#define AES67_FAST_CONNECT_MAGIC ((uint32_t)0xAE5675DB)
    uint32_t magic;
    uint32_t valid;
    aes67_sdp_t sdp[NUM_AES67_RECEIVERS];
} aes67_sdp_fast_connect_t;
#endif

aes67_status_t aes67_sdp_parse_string(const char str[],
                                      REFERENCE_PARAM(aes67_sdp_t, sdp));
aes67_status_t aes67_sdp_to_string(REFERENCE_PARAM(const aes67_sdp_t, sdp),
                                   ARRAY_OF_SIZE(char, buffer, buflen),
                                   size_t buflen);

void aes67_sdp_set_defaults(REFERENCE_PARAM(aes67_sdp_t, sdp));

int aes67_sdp_is_valid(REFERENCE_PARAM(const aes67_sdp_t, sdp));

void aes67_sdp_set_payload_type(REFERENCE_PARAM(aes67_sdp_t, sdp),
                                int payload_type);

void aes67_sdp_set_port(REFERENCE_PARAM(aes67_sdp_t, sdp),
                        const char port_string[]);

void aes67_sdp_set_address(REFERENCE_PARAM(aes67_sdp_t, sdp),
                           const char address[]);

void aes67_sdp_set_ipv4_address(REFERENCE_PARAM(aes67_sdp_t, sdp),
                                const uint8_t ip_addr[4]);

void aes67_sdp_set_ipv4_session_origin(REFERENCE_PARAM(aes67_sdp_t, sdp),
                                       const uint8_t ip_addr[4]);

void aes67_sdp_set_ipv4_port(REFERENCE_PARAM(aes67_sdp_t, sdp), uint16_t __port);

void aes67_sdp_set_ptp_gmid(REFERENCE_PARAM(aes67_sdp_t, sdp),
                            const uint8_t ptp_gmid[8]);

void aes67_sdp_set_ptp_domain(REFERENCE_PARAM(aes67_sdp_t, sdp), int domain);

void aes67_sdp_set_encoding(REFERENCE_PARAM(aes67_sdp_t, sdp), int encoding);

void aes67_sdp_set_encoding_name(REFERENCE_PARAM(aes67_sdp_t, sdp),
                                 const char encoding_name[]);

void aes67_sdp_set_session_id(REFERENCE_PARAM(aes67_sdp_t, sdp),
                              uint64_t session_id);

aes67_status_t aes67_sdp_get_ipv4_address(REFERENCE_PARAM(const aes67_sdp_t, sdp),
                                          uint8_t ip_addr[4]);

aes67_status_t aes67_sdp_get_ipv4_session_origin(REFERENCE_PARAM(const aes67_sdp_t, sdp),
                                                 uint8_t ip_addr[4]);

aes67_status_t aes67_sdp_get_ptp_gmid(REFERENCE_PARAM(const aes67_sdp_t, sdp),
                                      uint8_t ptp_gmid[8]);

int aes67_sdp_get_ptp_domain(REFERENCE_PARAM(const aes67_sdp_t, sdp));

aes67_status_t aes67_sdp_get_ipv4_port(REFERENCE_PARAM(const aes67_sdp_t, sdp),
                                       REFERENCE_PARAM(uint16_t, __port));

#ifndef __XC__
// RTP packet utility function that needs SDP
uint32_t aes67_rtp_packet_duration(REFERENCE_PARAM(const aes67_rtp_packet_t, packet),
                                   REFERENCE_PARAM(const aes67_sdp_t, sdp));
#endif
