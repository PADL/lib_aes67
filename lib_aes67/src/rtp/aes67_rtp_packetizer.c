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
                                   aes67_rtp_packet_t *rtp_packet) {
    rtp_packet->version = RTP_VERSION;
    rtp_packet->padding = 0;
    rtp_packet->extension = 0;
    rtp_packet->csrc_count = 0;
    rtp_packet->marker = 0;
    rtp_packet->payload_type = stream_info->payload_type;

    rtp_packet->sequence = sender->sequence_state.max_seq++;
    rtp_packet->timestamp =
        (sender->media_clock + stream_info->clock_offset) & 0xffffffff;
    rtp_packet->ssrc = stream_info->stream_id; // XXX
}

static void encode_rtp_packet_header(aes67_rtp_packet_t *rtp_packet) {
    // note: assumes little-endian
    uint16_t sequence_number = __builtin_bswap16(rtp_packet->sequence);
    uint32_t timestamp = __builtin_bswap32(rtp_packet->timestamp);
    uint32_t ssrc = __builtin_bswap32(rtp_packet->ssrc);

    rtp_packet->length = RTP_HEADER_LENGTH + rtp_packet->payload_length;

#define bitShift(byte, mask, shift) ((byte & mask) << shift)

    rtp_packet->buffer[0] = bitShift(rtp_packet->version, 0x02, 6) |
                            bitShift(rtp_packet->padding, 0x01, 5) |
                            bitShift(rtp_packet->extension, 0x01, 4) |
                            bitShift(rtp_packet->csrc_count, 0x0F, 0);
    rtp_packet->buffer[1] = bitShift(rtp_packet->marker, 0x01, 7) |
                            bitShift(rtp_packet->payload_type, 0x7F, 0);
    memcpy(&rtp_packet->buffer[2], &sequence_number, 2);
    memcpy(&rtp_packet->buffer[4], &timestamp, 4);
    memcpy(&rtp_packet->buffer[8], &ssrc, 4);
}

aes67_status_t aes67_send_rtp_packet(unsigned i_xtcp,
                                     int32_t id,
                                     ARRAY_OF_SIZE(const uint32_t, samples, len),
                                     size_t len,
                                     uint32_t timestamp) {
    aes67_stream_info_t *stream_info = &sender_streams[id];
    aes67_sender_t *sender = &senders[id];

    if (stream_info->state != AES67_STREAM_STATE_ENABLED ||
        sender->socket.fd == -1)
        return AES67_STREAM_NOT_STREAMING;

    COMPILER_BARRIER();

    if (len != stream_info->channel_count * sender->frames_per_packet)
        return AES67_STREAM_INVALID_FRAME_COUNT;

    // Convert local XMOS timestamp to RTP media clock and update sender
    sender->media_clock =
        local_timestamp_to_rtp_media_clock(timestamp, stream_info);

    aes67_rtp_packet_t rtp_packet;
    rtp_packet_header_init(stream_info, sender, &rtp_packet);
    rtp_packet.payload = &rtp_packet.buffer[RTP_HEADER_LENGTH];
    rtp_packet.payload_length = stream_info->sample_size * len;

    uint8_t *payload_ptr = rtp_packet.payload;

    for (size_t i = 0, sample_index = 0; i < sender->frames_per_packet; i++) {
        for (size_t ch = 0; ch < stream_info->channel_count; ch++) {
            uint32_t sample = samples[sample_index++];

            switch (stream_info->encoding) {
            case AES67_ENCODING_L16:
                *((uint16_t *)payload_ptr) = __builtin_bswap16(sample);
                break;
            case AES67_ENCODING_L24:
                payload_ptr[0] = (sample & 0xff000000) >> 24;
                payload_ptr[1] = (sample & 0x00ff0000) >> 16;
                payload_ptr[2] = (sample & 0x0000ff00) >> 8;
                break;
            case AES67_ENCODING_L32:
                *((uint32_t *)payload_ptr) = __builtin_bswap32(sample);
                break;
            }

            payload_ptr += stream_info->sample_size;
        }
    }

    assert(payload_ptr - rtp_packet.payload == rtp_packet.payload_length);
    assert(RTP_HEADER_LENGTH + rtp_packet.payload_length <=
           sizeof(rtp_packet.buffer));

    encode_rtp_packet_header(&rtp_packet);

    if (aes67_socket_send(i_xtcp, &sender->socket, rtp_packet.buffer,
                          rtp_packet.length) != rtp_packet.length)
        return AES67_STATUS_SOCKET_ERROR;

    return AES67_STATUS_OK;
}
