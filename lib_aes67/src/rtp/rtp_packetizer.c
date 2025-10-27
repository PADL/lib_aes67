// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <debug_print.h>
#include <string.h>

#include "aes67_internal.h"
#include "aes67_rtp.h"
#include "aes67_utils.h"
#include "ptp_internal.h"

// Convert local XMOS timestamp to RTP media clock
// Local XMOS timestamp -> PTP timestamp -> RTP media clock (based on sample
// rate)
static uint64_t
local_timestamp_to_rtp_media_clock(uint32_t local_timestamp,
                                   const aes67_stream_info_t *stream_info) {
    ptp_time_info_mod64 timeInfo;
    uint32_t ptp_timestamp_ns;
    uint64_t rtp_media_clock;

    // Get PTP time info and convert local timestamp to PTP nanoseconds
    ptp_get_local_time_info_mod64(&timeInfo);
    ptp_timestamp_ns = local_timestamp_to_ptp_mod32(local_timestamp, &timeInfo);

    // Convert PTP nanoseconds to RTP media clock units (sample rate)
    // PTP timestamp is in nanoseconds, we need to convert to sample rate
    // Formula: rtp_media_clock = (ptp_ns * sample_rate) /
    // NANOSECONDS_PER_SECOND
    rtp_media_clock = ((uint64_t)ptp_timestamp_ns * stream_info->sample_rate) /
                      NANOSECONDS_PER_SECOND;

    return rtp_media_clock;
}

static void rtp_packet_header_init(const aes67_stream_info_t *stream_info,
                                   aes67_sender_t *sender,
                                   aes67_rtp_header_t *rtp_header) {
    // Set version_p_x_cc byte using macros
    RTP_VERSION_SET(rtp_header, RTP_VERSION);
    RTP_PADDING_SET(rtp_header, 0);
    RTP_EXTENSION_SET(rtp_header, 0);
    RTP_CSRC_COUNT_SET(rtp_header, 0);

    // Set m_pt byte using macros
    RTP_MARKER_SET(rtp_header, 0);
    RTP_PAYLOAD_TYPE_SET(rtp_header, stream_info->payload_type);

    rtp_header->sequence = sender->sequence_state.max_seq++;
    rtp_header->timestamp =
        (sender->media_clock + stream_info->clock_offset) & 0xffffffff;
    rtp_header->ssrc = stream_info->stream_id; // XXX
}

static void encode_rtp_packet_header(aes67_rtp_header_t *rtp_header) {
    // Convert multi-byte fields to network byte order in place
    rtp_header->sequence = htons(rtp_header->sequence);
    rtp_header->timestamp = htonl(rtp_header->timestamp);
    rtp_header->ssrc = htonl(rtp_header->ssrc);
}

aes67_status_t
aes67_send_rtp_packet(CLIENT_INTERFACE(xtcp_if, i_xtcp),
                      const uint8_t src_mac_addr[MACADDR_NUM_BYTES],
                      streaming_chanend_t c_eth_tx_hp,
                      int32_t id,
                      ARRAY_OF_SIZE(const uint32_t, samples, len),
                      size_t len,
                      uint32_t timestamp) {
    aes67_stream_info_t *stream_info = &sender_streams[id];
    aes67_sender_t *sender = &senders[id];
    aes67_rtp_packet_t packet;
    aes67_status_t status;

    if (stream_info->state != AES67_STREAM_STATE_ENABLED ||
        sender->socket.fd == -1)
        return AES67_STREAM_NOT_STREAMING;

    COMPILER_BARRIER();

    if (len != stream_info->channel_count * sender->frames_per_packet)
        return AES67_STREAM_INVALID_FRAME_COUNT;

    // Convert local XMOS timestamp to RTP media clock and update sender
    sender->media_clock =
        local_timestamp_to_rtp_media_clock(timestamp, stream_info);

    rtp_packet_header_init(stream_info, sender, &packet.rtp_header);
    uint8_t *payload_ptr = packet.payload;

    for (size_t i = 0, sample_index = 0; i < sender->frames_per_packet; i++) {
        for (size_t ch = 0; ch < stream_info->channel_count; ch++) {
            uint32_t sample = samples[sample_index++];

            switch (stream_info->encoding) {
            case AES67_ENCODING_L16:
                *((uint16_t *)payload_ptr) = htons(sample);
                break;
            case AES67_ENCODING_L24:
                payload_ptr[0] = (sample & 0xff000000) >> 24;
                payload_ptr[1] = (sample & 0x00ff0000) >> 16;
                payload_ptr[2] = (sample & 0x0000ff00) >> 8;
                break;
            case AES67_ENCODING_L32:
                *((uint32_t *)payload_ptr) = htonl(sample);
                break;
            }

            payload_ptr += stream_info->sample_size;
        }
    }

    encode_rtp_packet_header(&packet.rtp_header);
    packet.rtp_length = RTP_HEADER_LENGTH + stream_info->sample_size * len;

    if (c_eth_tx_hp) {
        status = aes67_raw_send_rtp(src_mac_addr, c_eth_tx_hp,
                                    &sender->socket, &packet);
    } else {
        status = aes67_socket_send_rtp(i_xtcp, &sender->socket, &packet);
    }

    return status;
}
