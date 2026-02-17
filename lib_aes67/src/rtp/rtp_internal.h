#pragma once

#include <aes67_internal.h>

#include "rtp_protocol.h"
#include "rtp_fifo.h"

#ifdef __XC__
extern "C" {
#endif

typedef struct _aes67_receiver {
    aes67_socket_t socket;
    aes67_rtp_sequence_state_t sequence_state;
    aes67_audio_fifo_t fifos[AES67_MAX_CHANNELS_PER_RECEIVER];
    int buf_ctl_notified;
} aes67_receiver_t;

extern aes67_receiver_t receivers[NUM_AES67_RECEIVERS];

#ifndef __XC__
aes67_status_t
aes67_process_rtp_packet(chanend buf_ctl,
                         int32_t id,
                         REFERENCE_PARAM(aes67_receiver_t, receiver),
                         REFERENCE_PARAM(const aes67_rtp_packet_t, packet));
#endif

aes67_status_t
aes67_process_rtp_packet_opaque(chanend buf_ctl,
                                int32_t id,
                                REFERENCE_PARAM(aes67_receiver_t, receiver),
                                const uint32_t words[AES67_RTP_PACKET_STRUCT_SIZE / 4]);

typedef struct _aes67_sender {
    // immutable, shared file descriptor
    aes67_socket_t socket;

    // owned by packetizer
    aes67_rtp_sequence_state_t sequence_state;
    uint64_t media_clock;
    uint32_t frames_per_packet;
    uint32_t samples_per_packet;

    // owned by task pushing samples
    uint32_t pending_ts;
    uint32_t sample_count;
} aes67_sender_t;

extern aes67_sender_t senders[NUM_AES67_SENDERS];

aes67_status_t
aes67_send_rtp_packet(CLIENT_INTERFACE(xtcp_if, i_xtcp),
                      const uint8_t src_mac_addr[MACADDR_NUM_BYTES],
                      streaming_chanend_t c_eth_tx_hp,
                      int32_t id,
                      ARRAY_OF_SIZE(const uint32_t, samples, len),
                      size_t len,
                      uint32_t timestamp);

// Buffer management functions for double buffering
uint32_t *unsafe aes67_get_tx_buffer0(int32_t sender_id);
uint32_t *unsafe aes67_get_tx_buffer1(int32_t sender_id);

aes67_stream_info_t *unsafe aes67_get_receiver_stream(int32_t id);

#ifdef __XC__
}
#endif
