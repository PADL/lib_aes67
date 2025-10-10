// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>

#include "aes67_internal.h"

#define AES67_SENDER_BUFFER_LEN                                                \
    (AES67_MAX_CHANNELS_PER_SENDER * MAX_SAMPLES_PER_RTP_PACKET)

// Double buffers for RTP sender - moved here to avoid XC aliasing issues
static uint32_t tx_buffer0[NUM_AES67_SENDERS][AES67_SENDER_BUFFER_LEN];
static uint32_t tx_buffer1[NUM_AES67_SENDERS][AES67_SENDER_BUFFER_LEN];

uint32_t *aes67_get_tx_buffer0(int32_t sender_id) {
    assert(is_valid_sender_id(sender_id));
    return tx_buffer0[sender_id];
}

uint32_t *aes67_get_tx_buffer1(int32_t sender_id) {
    assert(is_valid_sender_id(sender_id));
    return tx_buffer1[sender_id];
}
