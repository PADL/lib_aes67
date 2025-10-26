// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#pragma once

#ifdef __XC__
#define AES67_XMOS 1
#else
#if __has_include(<xtcp.h>) // __has_include() not available on XC
#define AES67_XMOS 1
#endif
#endif // __XC__

#if AES67_XMOS
#include <debug_print.h>
#include <xtcp.h>
#include <ethernet.h>
#include <xccompat.h>
#else
#define REFERENCE_PARAM(type, name) type *name
#define NULLABLE_REFERENCE_PARAM(type, name) type *name
#define ARRAY_OF_SIZE(type, name, size) type *name
#endif

#include <stdint.h>

#include "aes67.h"
#include "nettypes.h"

#define AES67_DEFAULT_PORT 5004

#ifdef __XC__
#define unsafe unsafe
#else
#define unsafe
#endif

#ifdef __XC__
#define alias alias
#else
#define alias
#endif

#ifdef __XC__
#define streaming streaming
#else
#define streaming
#endif

#define COMPILER_BARRIER() asm volatile("" ::: "memory")

// from lib_tsn ethernet_conf.h
#define NUM_ETHERNET_PORTS 1
#define NUM_ETHERNET_MASTER_PORTS 1
#define ETHERNET_SUPPORT_HP_QUEUES 1
#define ETHERNET_MAX_ETHERTYPE_FILTERS 3

#define AES67_MAX_AUDIO_SAMPLE_RATE 48000

#define AES67_MIN_PACKET_RATE 1000 // Table 4
#define AES67_MAX_PACKET_RATE 8000 // Table 4

#ifndef MAX_SAMPLES_PER_RTP_PACKET
#define MAX_SAMPLES_PER_RTP_PACKET                                             \
    (AES67_MAX_AUDIO_SAMPLE_RATE / AES67_MIN_PACKET_RATE)
#endif

#ifndef AUDIO_OUTPUT_FIFO_WORD_SIZE
#define AUDIO_OUTPUT_FIFO_WORD_SIZE                                            \
    (AES67_MAX_AUDIO_SAMPLE_RATE / 50) // 20ms buffer per AES67 spec
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#define DEFAULT_PAGE_SIZE (256)

#define BIT(_x) (1 << (_x))

static inline int popcount(uint32_t x) {
    uint8_t count;

    for (count = 0; x; count++)
        x &= x - 1;

    return count;
}

#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#define AES67_DEFAULT_PORT_STR STRINGIFY(AES67_DEFAULT_PORT)
#define AES67_DEFAULT_SAMPLE_RATE 48000
#define AES67_DEFAULT_ENCODING AES67_ENCODING_L24

#ifndef AES67_VLAN_ID
#define AES67_VLAN_ID 0
#endif

#define NANOSECONDS_PER_SECOND (1000000000)
#define MAX_REASONABLE_DELAY_NS (1000000000UL) // 1 second max delay

#define DSCP_AF41 34 // media traffic
#define DSCP_EF 46 // PTPv2 (AES67), audio (Dante)
#define DSCP_CS7 56 // PTPv2 (Dante)

#if AES67_DANTE_DSCP_COMPAT
#define DSCP_PTP_EVENT DSCP_CS7
#define DSCP_PTP_GENERAL DSCP_EF
#define DSCP_RTP DSCP_EF
#else
#define DSCP_PTP_EVENT DSCP_EF
#define DSCP_PTP_GENERAL DSCP_EF
#define DSCP_RTP DSCP_AF41
#endif // AES67_DANTE_DSCP_COMPAT

typedef char aes67_session_name_t[256];

static inline int is_valid_receiver_id(int32_t id) {
    return id >= 0 && id < NUM_AES67_RECEIVERS;
}

static inline int is_valid_sender_id(int32_t id) {
    return id >= 0 && id < NUM_AES67_SENDERS;
}

typedef enum _aes67_stream_state {
    // the stream is not sending or receiving
    AES67_STREAM_STATE_DISABLED = 0,
    // the stream info is being updated; the rest of the fields are invalid
    AES67_STREAM_STATE_UPDATING,
    // a receiver stream is configured but not yet streaming
    AES67_STREAM_STATE_POTENTIAL,
    // the stream is sending or receiving
    AES67_STREAM_STATE_ENABLED,
} aes67_stream_state_t;

typedef enum _aes67_media_clock_state {
    AES67_MEDIA_CLOCK_STATE_DISABLED = 0,
    AES67_MEDIA_CLOCK_STATE_ENABLED,
} aes67_media_clock_state_t;

typedef enum _aes67_media_control_command {
    AES67_MEDIA_CONTROL_COMMAND_NOOP = 0,
    AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE,
    AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE,
    AES67_MEDIA_CONTROL_COMMAND_UNSUBSCRIBE,
    AES67_MEDIA_CONTROL_COMMAND_GET_CLOCK_INFO,
    AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO,
    AES67_MEDIA_CONTROL_COMMAND_START_STREAMING,
    AES67_MEDIA_CONTROL_COMMAND_STOP_STREAMING,
    AES67_MEDIA_CONTROL_COMMAND_SET_SAMPLE_RATE,
} aes67_media_control_command_t;

typedef struct _aes67_stream_info {
    uint32_t state; // word so atomic write/read
    uint8_t sample_size; // this MUST be the first field
    uint8_t payload_type;
    uint8_t channel_count;
    uint8_t stream_id;
    uint32_t encoding;
    uint32_t sample_rate;
    uint32_t packet_time_us;
    xtcp_ipaddr_t src_addr; // owner, creator
    xtcp_ipaddr_t dest_addr;
    uint16_t dest_port;
    uint16_t gm_port;
    n64_t gm_id;
    uint64_t clock_offset;
} aes67_stream_info_t;

extern aes67_stream_info_t receiver_streams[NUM_AES67_RECEIVERS];
extern aes67_stream_info_t sender_streams[NUM_AES67_SENDERS];

extern uint32_t receiver_stream_state[NUM_AES67_RECEIVERS];
extern uint32_t sender_stream_state[NUM_AES67_SENDERS];

// IO task select handlers
void aes67_media_control_init(void);

#ifdef __XC__
#pragma select handler
void aes67_media_control(chanend media_control,
                         client ethernet_cfg_if i_eth_cfg,
                         client xtcp_if i_xtcp,
                         uint32_t flags);
#endif

// LWIP things that aren't exposed
extern char *unsafe ip4addr_ntoa_r(const ip4_addr_t *unsafe,
                                   char *unsafe,
                                   int buflen);
extern int ip4addr_aton(const char *unsafe cp, ip4_addr_t *unsafe addr);

// Media clock select handler functions
void aes67_register_buf_fifo(uint32_t i, uintptr_t fifo);
void aes67_register_clock(uint32_t i);
aes67_media_clock_info_t aes67_get_clock_info(void);

const char *unsafe aes67_status_to_string(aes67_status_t status);
