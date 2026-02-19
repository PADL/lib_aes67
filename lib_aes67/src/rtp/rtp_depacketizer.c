// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <debug_print.h>
#include <string.h>

#include "aes67_internal.h"
#include "rtp_internal.h"

aes67_stream_info_t *unsafe aes67_get_receiver_stream(int32_t id) {
    assert(is_valid_receiver_id(id));

    return &receiver_streams[id];
}

static void open_receiver_stream(aes67_stream_info_t *stream_info,
                                 aes67_receiver_t *receiver,
                                 const aes67_rtp_packet_t *packet) {
    uint16_t sequence = packet->rtp_header.sequence;

    aes67_rtp_init_sequence(&receiver->sequence_state, sequence);

    receiver->sequence_state.max_seq = sequence - 1;
    receiver->sequence_state.probation = RTP_MIN_SEQUENTIAL;

    // Initialize FIFOs for this receiver
    receiver->buf_ctl_notified = 0;

    for (size_t ch = 0; ch < stream_info->channel_count; ch++)
        aes67_audio_fifo_enable(&receiver->fifos[ch]);

    // word writes are atomic, so safe to update
    stream_info->state = AES67_STREAM_STATE_ENABLED;
}

aes67_status_t aes67_process_rtp_packet(chanend buf_ctl,
                                        int32_t id,
                                        aes67_receiver_t *receiver,
                                        const aes67_rtp_packet_t *packet) {
    aes67_stream_info_t *stream_info = aes67_get_receiver_stream(id);
    int need_open = 0;
    uint32_t state = stream_info->state; // atomic read

    if (state == AES67_STREAM_STATE_POTENTIAL)
        need_open = 1;
    else if (state != AES67_STREAM_STATE_ENABLED)
        return AES67_STATUS_OK;

    COMPILER_BARRIER();

    size_t frame_size = stream_info->sample_size * stream_info->channel_count;

    if (RTP_VERSION_GET(&packet->rtp_header) != RTP_VERSION)
        return AES67_STATUS_UNSUPPORTED_RTP_VERSION;
    else if (RTP_PAYLOAD_TYPE_GET(&packet->rtp_header) !=
             stream_info->payload_type)
        return AES67_STATUS_UNEXPECTED_RTP_PAYLOAD_TYPE;
    else if ((aes67_rtp_packet_length_rtp(packet) % frame_size) != 0)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    if (!aes67_rtp_update_sequence(&receiver->sequence_state,
                                   packet->rtp_header.sequence))
        return AES67_STATUS_RTP_PACKET_OUT_OF_SEQUENCE;

    if (need_open)
        open_receiver_stream(stream_info, receiver, packet);

    // follow ordering of 1722 listener: set timestamp, maintain FIFO, push
    for (size_t ch = 0; ch < stream_info->channel_count; ch++) {
        aes67_audio_fifo_maintain(&receiver->fifos[ch], buf_ctl,
                                  packet->rtp_header.timestamp,
                                  stream_info->clock_offset,
                                  stream_info->packet_time_us,
                                  &receiver->buf_ctl_notified);
    }

    // samples are laid out Frame0[C0C1...CN] ... FrameN[C0C1...CN]
    // payload_ptr points to the first sample for a channel
    // sample_size is used by aes67_audio_fifo_push_samples() to iterate frames
    const size_t frame_count = aes67_rtp_packet_length_rtp(packet) / frame_size;
    uint8_t *payload_ptr = (uint8_t *)&packet->payload[0];

    for (size_t ch = 0; ch < stream_info->channel_count; ch++) {
        aes67_audio_fifo_push_samples(
            &receiver->fifos[ch], payload_ptr, stream_info->sample_size,
            stream_info->channel_count, frame_count,
            stream_info->encoding);
        payload_ptr += stream_info->sample_size;
    }

    return AES67_STATUS_OK;
}

static void
pull_samples(int32_t id,
             uint32_t *output_buffer,
             size_t *output_index_p,
             size_t len,
             uint32_t local_timestamp) {
    aes67_stream_info_t *stream_info = aes67_get_receiver_stream(id);
    size_t used_channels = stream_info->channel_count;
    size_t output_index = *output_index_p;
    int valid;

    assert(len <= AES67_NUM_MEDIA_OUTPUTS);

    if (used_channels > AES67_MAX_CHANNELS_PER_RECEIVER)
        used_channels = AES67_MAX_CHANNELS_PER_RECEIVER;

    for (size_t ch = 0; ch < used_channels && output_index < len; ch++) {
        if (stream_info->state != AES67_STREAM_STATE_ENABLED)
            break;

        output_buffer[output_index] = aes67_audio_fifo_pull_sample(
            &receivers[id].fifos[ch], local_timestamp, &valid);
        if (!valid)
            output_buffer[output_index] = 0;

        output_index++;
    }

    if (output_index <= len) {
        size_t unused_channels =
            AES67_MAX_CHANNELS_PER_RECEIVER - used_channels;
        if (unused_channels)
            memset(&output_buffer[output_index], 0,
                   unused_channels * sizeof(uint32_t));
        output_index += unused_channels;
    }

    *output_index_p = output_index;
}

uint32_t
aes67_get_receiver_sample(int32_t id,
                          uint32_t ch,
                          uint32_t local_timestamp) {
    aes67_stream_info_t *stream_info = aes67_get_receiver_stream(id);
    uint32_t sample;
    int valid;

    if (stream_info->state != AES67_STREAM_STATE_ENABLED ||
        ch >= AES67_MAX_CHANNELS_PER_RECEIVER)
        return 0;

    sample = aes67_audio_fifo_pull_sample(&receivers[id].fifos[ch], local_timestamp, &valid);
    if (!valid)
        sample = 0;

    return sample;
}

void aes67_get_receiver_samples(int32_t id,
                                uint32_t *output_buffer,
                                size_t len,
                                uint32_t local_timestamp) {
    size_t output_index = 0;

    pull_samples(id, output_buffer, &output_index, len, local_timestamp);
}

// Public function that uses global receivers array (declared in
// aes67_rtp_receiver.xc)
void aes67_get_all_receiver_samples(uint32_t *output_buffer,
                                    size_t len,
                                    uint32_t local_timestamp) {
    size_t output_index = 0;

    memset(output_buffer, 0, len * sizeof(uint32_t));

    for (int32_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        pull_samples(id, output_buffer, &output_index, len, local_timestamp);
        if (output_index > len)
            break;
    }
}

#ifdef AES67_XMOS
aes67_status_t aes67_process_rtp_packet_opaque(chanend buf_ctl,
                                               int32_t id,
                                               aes67_receiver_t *receiver,
                                               const uint32_t words[AES67_RTP_PACKET_STRUCT_SIZE_WORDS]) {
    return aes67_process_rtp_packet(buf_ctl, id, receiver, (const aes67_rtp_packet_t *)words);
}
#endif
