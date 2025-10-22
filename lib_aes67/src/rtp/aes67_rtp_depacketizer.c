// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <debug_print.h>
#include <aes67_internal.h>
#include <string.h>

#include "aes67_rtp.h"

aes67_stream_info_t *unsafe aes67_get_receiver_stream(int32_t id) {
    assert(is_valid_receiver_id(id));

    return &receiver_streams[id];
}

static int get_absolute_media_clock(const aes67_stream_info_t *stream_info,
                                    aes67_receiver_t *receiver,
                                    uint32_t rtp_ts,
                                    uint64_t *media_clock_p) {
    uint64_t media_clock, delta;
    int can_update = 1;

    /* Basic algorithm derived from GStreamer */
    media_clock = (rtp_ts - stream_info->clock_offset) & 0xffffffff;
    media_clock += receiver->media_clock & 0xffffffff00000000;

    if (media_clock < receiver->media_clock) {
        delta = receiver->media_clock - media_clock;
        if (delta > 0xffffffff)
            media_clock += 1ULL << 32;
    } else {
        delta = media_clock - receiver->media_clock;
        if (delta > 0xffffffff) {
            if (media_clock < 1ULL << 32)
                media_clock = 0;
            else
                media_clock -= 1ULL << 32;
            can_update = 0;
        }
    }

    *media_clock_p = media_clock;

    return can_update;
}

static void media_clock_to_ptp_time_ns(const aes67_stream_info_t *stream_info,
                                       uint64_t media_clock,
                                       uint64_t *ptpTimeNS) {
    // Convert media clock samples to nanoseconds
    // Use 64-bit arithmetic to avoid overflow: media_clock * 1e9 / sample_rate
    *ptpTimeNS =
        (media_clock * NANOSECONDS_PER_SECOND) / stream_info->sample_rate;

    // offset by packet time to create a PTP presentation time
    ptpTimeNS += stream_info->packet_time_us * 1000 * 2;
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
    aes67_stream_info_t *stream_info = &receiver_streams[id];
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

    uint64_t media_clock;
    int update_media_clock = get_absolute_media_clock(
        stream_info, receiver, packet->rtp_header.timestamp,
        &media_clock);
    size_t frame_count = aes67_rtp_packet_length_rtp(packet) / frame_size;

    uint64_t ptp_time_ns;
    media_clock_to_ptp_time_ns(stream_info, media_clock, &ptp_time_ns);

    // Set presentation timestamp on first channel for timing control
    if (frame_count && stream_info->channel_count)
        aes67_audio_fifo_set_presentation_time(&receiver->fifos[0],
                                               (uint32_t)ptp_time_ns);

    // samples are laid out Frame0[C0C1...CN] ... FrameN[C0C1...CN]
    // payload_ptr points to the first sample for a channel
    // sample_size is used by aes67_audio_fifo_push_samples() to iterate frames
    uint8_t *payload_ptr = (uint8_t *)&packet->payload[0];

    for (size_t ch = 0; ch < stream_info->channel_count; ch++) {
        aes67_audio_fifo_push_samples(
            &receiver->fifos[ch], payload_ptr, stream_info->sample_size,
            stream_info->channel_count, frame_count, stream_info->encoding);
        payload_ptr += stream_info->sample_size;
    }

    if (update_media_clock)
        receiver->media_clock = media_clock + frame_count;

    for (size_t ch = 0; ch < stream_info->channel_count; ch++)
        aes67_audio_fifo_maintain(&receiver->fifos[ch], buf_ctl,
                                  &receiver->buf_ctl_notified);

    return AES67_STATUS_OK;
}

static void pull_samples(int32_t id,
                         uint32_t *output_buffer,
                         size_t *output_index_p,
                         size_t len,
                         uint32_t local_timestamp) {
    aes67_stream_info_t *stream_info = &receiver_streams[id];
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

