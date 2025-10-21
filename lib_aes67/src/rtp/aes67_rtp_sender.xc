// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <string.h>
#include <stdlib.h>

#include "aes67_internal.h"
#include "aes67_rtp.h"
#include "aes67_utils.h"
#include "nettypes.h"
#include "ptp_internal.h"

aes67_sender_t senders[NUM_AES67_SENDERS];

// Sender state check - atomic 32-bit read
static inline int aes67_is_sender_open(const aes67_sender_t &sender) {
    return sender.socket.fd != -1;
}

static void aes67_poll_stream_info_changed(uint32_t time, int rtp_tx_socket) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_SENDERS; id++) {
        aes67_stream_info_t stream_info = get_sender_stream(id);
        aes67_sender_t &sender = senders[id];

        switch (stream_info.state) {
        case AES67_STREAM_STATE_DISABLED:
            sender.socket.fd = -1;
            sender.sample_count = 0;  // Reset accumulator when disabling
            sender.pending_ts = 0;    // Reset pending timestamp
            break;
        case AES67_STREAM_STATE_ENABLED:
            if (sender.socket.fd != -1)
                break;
            sender.socket.fd = rtp_tx_socket;
            sender.sequence_state.max_seq = time & 0xffff;
            sender.media_clock = 0;
            sender.frames_per_packet = (stream_info.sample_rate / 1000000) * stream_info.packet_time_us;
            sender.samples_per_packet = sender.frames_per_packet * stream_info.channel_count;
            break;
        case AES67_STREAM_STATE_UPDATING:
            break;
        default:
            fail("invalid sender stream info state");
            break;
        }
    }
}

#define swapPointer(__dst, __src)        \
    do {                                 \
        uint32_t *movable tmp;           \
        tmp = move(__src);               \
        __src = move(__dst);             \
        __dst = move(tmp);               \
    } while (0)

void aes67_rtp_sender(CLIENT_INTERFACE(xtcp_if, i_xtcp), chanend data_ready) {
    uint32_t *movable tx_rdbuffer[NUM_AES67_SENDERS];
    int rtp_tx_socket;
    uint32_t time;
    timer t;

    // Initialize movable read buffers
    for (int32_t i = 0; i < NUM_AES67_SENDERS; i++) {
        unsafe {
            tx_rdbuffer[i] = (uint32_t *movable)aes67_get_tx_buffer0(i);
        }
    }

    rtp_tx_socket = i_xtcp.socket(XTCP_PROTOCOL_UDP);
    assert(rtp_tx_socket != -1);

    // TODO: we might need a FIFO or double buffer here if we can't send packets fast enough

    while (1) {
        [[ordered]]
        select {
            case data_ready :> int32_t id:
                aes67_stream_info_t stream_info = get_sender_stream(id);
                aes67_sender_t &sender = senders[id];
                uint32_t *movable &tmp;
                uint32_t timestamp;

                // synchronization point: swap double buffers
                data_ready :> timestamp;
                data_ready :> tmp;

                swapPointer(tx_rdbuffer[id], tmp);

                // send RTP packet
                aes67_send_rtp_packet(i_xtcp, stream_info, sender,
                                      tx_rdbuffer[id], sender.samples_per_packet,
                                      timestamp);
                break;
            case t when timerafter(time) :> void:
                aes67_poll_stream_info_changed(time, rtp_tx_socket);
                time += XS1_TIMER_HZ;
                break;
        }
    }

    i_xtcp.close(rtp_tx_socket);
}

static uint32_t *movable tx_wrbuffer[NUM_AES67_SENDERS];

void aes67_init_sender_buffers(void) {
    for (int32_t i = 0; i < NUM_AES67_SENDERS; i++) {
        unsafe {
            tx_wrbuffer[i] = (uint32_t *movable)aes67_get_tx_buffer1(i);
        }
    }
}

// Submit samples, ordered by channel and then frame (i.e. in network packet
// order) until enough samples are submitted to cause a RTP packet to be emitted
// the timestamp is the timestamp of the first sample in the packet
void aes67_submit_sender_samples(chanend data_ready,
                                 int32_t id,
                                 ARRAY_OF_SIZE(uint32_t, samples, len),
                                 size_t len,
                                 uint32_t timestamp) {
    assert(len <= AES67_MAX_CHANNELS_PER_SENDER);

    aes67_sender_t &sender = senders[id];

    // Check if sender is open (atomic 32-bit read) - avoid race with stream removal
    if (!aes67_is_sender_open(sender))
        return;

    uint32_t sample_count = sender.sample_count;

    if (sample_count == 0)
        sender.pending_ts = timestamp;

#pragma unsafe arrays
    for (size_t i = 0; i < len; i++)
        tx_wrbuffer[id][sample_count++] = samples[i];

    if (sample_count == sender.samples_per_packet) {
        // Check sender is still open before signaling (avoid race with stream removal)
        if (!aes67_is_sender_open(sender)) {
            sender.sample_count = 0;  // Reset on stream close
            return;
        }

        // we need to signal
        sample_count = 0;

        unsafe {
            uint32_t *unsafe &tmp = tx_wrbuffer[id];
            data_ready <: id;
            data_ready <: sender.pending_ts;
            data_ready <: tmp;
        }
    }

    sender.sample_count = sample_count;
}
