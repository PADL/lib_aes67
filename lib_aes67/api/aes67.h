// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#pragma once

/*
 * AES67 stack for XMOS
 */

#include <stdint.h>
#include <xtcp.h>
#include <ptp.h>

#ifndef AES67_MAX_CHANNELS_PER_RECEIVER
#define AES67_MAX_CHANNELS_PER_RECEIVER 8
#endif

#ifndef NUM_AES67_RECEIVERS
#define NUM_AES67_RECEIVERS 2
#endif

#ifndef AES67_MAX_CHANNELS_PER_SENDER
#define AES67_MAX_CHANNELS_PER_SENDER 8
#endif

#ifndef NUM_AES67_SENDERS
#define NUM_AES67_SENDERS 2
#endif

#ifndef AES67_NUM_MEDIA_OUTPUTS
#define AES67_NUM_MEDIA_OUTPUTS                                                \
    (AES67_MAX_CHANNELS_PER_SENDER * NUM_AES67_RECEIVERS)
#endif

#ifndef AES67_NUM_MEDIA_INPUTS
#define AES67_NUM_MEDIA_INPUTS                                                 \
    (AES67_MAX_CHANNELS_PER_RECEIVER * NUM_AES67_SENDERS)
#endif

typedef enum _aes67_status {
    AES67_STATUS_OK = 0,
    AES67_STATUS_INVALID_STREAM_ID,
    AES67_STATUS_ALREADY_SUBSCRIBED,
    AES67_STATUS_NOT_SUBSCRIBED,
    AES67_STATUS_NOT_IMPLEMENTED,
    AES67_STATUS_STREAM_NAME_MISMATCH,
    AES67_STATUS_SOCKET_ERROR,
    AES67_STATUS_UNSUPPORTED_RTP_VERSION,
    AES67_STATUS_UNKNOWN_RTP_ENCODING,
    AES67_STATUS_UNEXPECTED_RTP_PAYLOAD_TYPE,
    AES67_STATUS_BAD_PACKET_LENGTH,
    AES67_STATUS_RTP_PACKET_OUT_OF_SEQUENCE,
    AES67_STATUS_INVALID_SAMPLE_SIZE,
    AES67_STATUS_INVALID_CHANNEL_COUNT,
    AES67_STATUS_INVALID_SAMPLE_RATE,
    AES67_STATUS_INVALID_SDP_ADDRESS,
    AES67_STATUS_INVALID_SAP_PACKET,
    AES67_STATUS_INVALID_SESSION_DESCRIPTION,
    AES67_STATUS_OUT_OF_BUFFER_SPACE,
    AES67_STREAM_ALREADY_ADVERTISING,
    AES67_STREAM_NOT_ADVERTISED,
    AES67_STREAM_NOT_STREAMING,
    AES67_STREAM_INVALID_FRAME_COUNT,
    AES67_STATUS_INVALID_ETH_DEST_MAC,
    AES67_STATUS_INVALID_ETH_TYPE,
    AES67_STATUS_INVALID_IP_VERSION,
    AES67_STATUS_INVALID_IP_PROTOCOL,
    AES67_STATUS_INVALID_IP_SRC_ADDR,
    AES67_STATUS_INVALID_IP_DEST_ADDR,
    AES67_STATUS_INVALID_UDP_SRC_PORT,
    AES67_STATUS_INVALID_UDP_DEST_PORT,
    AES67_STATUS_INVALID_IP_CHECKSUM,
    AES67_STATUS_UNKNOWN_ERROR = 0xff
} aes67_status_t;

typedef struct _aes67_media_clock_pid_coefficients {
    uint16_t p_numerator;
    uint16_t p_denominator;
    uint16_t i_numerator;
    uint16_t i_denominator;
    uint16_t d_numerator;
    uint16_t d_denominator;
} aes67_media_clock_pid_coefficients_t;

extern aes67_media_clock_pid_coefficients_t cs2600_pid_coefficients;

#define AES67_TIME_SOURCE_INFO_PTP_LOCKED_FLAG 0x01

typedef struct _aes67_time_source_info {
    uint8_t ptp_id[8];
    uint16_t ptp_domain;
    uint8_t flags;
    uint8_t reserved;
} aes67_time_source_info_t;

typedef struct _aes67_media_clock_info {
    uint32_t active;
    uint32_t rate;
    uint32_t lock_counter;
    uint32_t unlock_counter;
} aes67_media_clock_info_t;

typedef enum _aes67_event_type {
    AES67_EVENT_NOOP = 0,
    AES67_EVENT_TIME_SOURCE_INFO,
    AES67_EVENT_MEDIA_CLOCK_INFO
} aes67_event_type_t;

typedef struct _aes67_event_info {
    aes67_event_type_t event_type;
    union {
        aes67_time_source_info_t time_source_info;
        aes67_media_clock_info_t media_clock_info;
    } u;
} aes67_event_info_t;

/*
 * aes67_manager() can run on any tile, usually the tile that is handling
 * non-realtime tasks such as UARTs.
 *
 * because of the use of shared memory, aes67_io_task() and the sender and
 * receiver tasks MUST run on the same tile for the stack to work.
 */

#ifdef __XC__
interface aes67_interface {
    // receiver
    aes67_status_t subscribe(int16_t id, const char session_name[]);
    aes67_status_t unsubscribe(int16_t id, const char session_name[]);

    // sender
    aes67_status_t advertise(int16_t id,
                             const char session_name[],
                             uint8_t ip_addr[4],
                             uint32_t sample_size,
                             uint32_t channel_count);
    aes67_status_t unadvertise(int16_t id);

    // control
    aes67_status_t get_time_source_info(aes67_time_source_info_t &info);
    aes67_status_t get_media_clock_info(aes67_media_clock_info_t &info);

    aes67_status_t set_sample_rate(uint32_t rate);

    // events
    [[notification]] slave void event_ready();
    [[clears_notification]] aes67_event_info_t get_event_info();
};

[[combinable]] void
aes67_manager(server interface aes67_interface i_aes67[num_aes67_clients],
              size_t num_aes67_clients,
              client xtcp_if i_xtcp,
              chanend media_control);

void aes67_io_task(chanend ?ptp_svr,
                   chanend buf_ctl[num_buf_ctl],
                   uint32_t num_buf_ctl,
                   out buffered port:32 p_fs,
                   REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t, pid_coefficients),
                   chanend media_control,
                   client interface ethernet_cfg_if i_eth_cfg,
                   client xtcp_if i_xtcp,
                   chanend c_ptp[num_ptp],
                   uint32_t num_ptp,
                   enum ptp_server_type server_type);

// depacketizer
void aes67_rtp_receiver(CLIENT_INTERFACE(xtcp_if, i_xtcp), chanend buf_ctl,
                        streaming chanend ?c_eth_rx_hp);
#endif // __XC__

void
aes67_get_receiver_samples(int32_t id,
                           ARRAY_OF_SIZE(uint32_t, samples, len),
                           size_t len,
                           uint32_t local_timestamp);

// this must be called from the same tile as aes67_rtp_receiver() as it uses
// shared memory
void
aes67_get_all_receiver_samples(ARRAY_OF_SIZE(uint32_t, samples, len),
                               size_t len,
                               uint32_t local_timestamp);

// packetizer
#ifdef __XC__
void aes67_rtp_sender(CLIENT_INTERFACE(xtcp_if, i_xtcp),
                      CLIENT_INTERFACE(ethernet_cfg_if?, i_eth_cfg),
                      chanend data_ready,
                      streaming chanend ?c_eth_tx_hp);
#endif

// call this before sending any samples
void aes67_init_sender_buffers(void);

// submit up to a complete packet's worth of samples
// can call multiple times to fill buffer
// samples should be ordered by channel first, then frame
void
aes67_submit_sender_samples(chanend media,
                            int32_t id,
                            ARRAY_OF_SIZE(uint32_t, samples, len),
                            size_t len,
                            uint32_t timestamp);
