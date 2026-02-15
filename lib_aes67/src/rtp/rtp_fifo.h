// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025 PADL Software Pty Ltd, All rights reserved

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <aes67_internal.h>

#define START_OF_FIFO(s) ((uint32_t *)&((s)->fifo[0]))
#define END_OF_FIFO(s) ((uint32_t *)&((s)->fifo[AUDIO_OUTPUT_FIFO_WORD_SIZE]))

typedef enum aes67_fifo_state {
    AES67_FIFO_DISABLED = 0, // Not active
    AES67_FIFO_ZEROING,      // Pushing zeros through to fill
    AES67_FIFO_LOCKING, // Clock recovery trying to lock to the sample stream
    AES67_FIFO_LOCKED   // Clock recovery is locked and working
} aes67_fifo_state_t;

typedef struct aes67_audio_fifo {
    int zero_flag; // When set, the FIFO will output zero samples instead of its
                   // contents
    volatile uint32_t *unsafe dptr;  // The read pointer
    volatile uint32_t *unsafe wrptr; // The write pointer
    uint32_t *unsafe marker; // This indicates which sample is the one which the
                             // timestamps apply to
    int local_ts; // When a sample is played out, this contains the ref clock
                  // when it happened
    int ptp_ts;   // Contains the PTP timestamp of the marked sample
    uint32_t sample_count; // The count of samples that have passed through the
                           // buffer
    volatile uint32_t *unsafe zero_marker; // Used during zeroing phase
    aes67_fifo_state_t state;              // State of the FIFO
    int last_notification_time; // Last time that the clock recovery thread was
                                // informed of the timestamp info
    int pending_init_notification; // Flag for pending initialization
                                   // notification
    uint32_t fifo[AUDIO_OUTPUT_FIFO_WORD_SIZE]; // Circular buffer storage
} aes67_audio_fifo_t;

// FIFO management functions
void aes67_audio_fifo_init(aes67_audio_fifo_t *unsafe fifo);
void aes67_audio_fifo_disable(aes67_audio_fifo_t *unsafe fifo);
void aes67_audio_fifo_enable(aes67_audio_fifo_t *unsafe fifo);

// Sample operations
void aes67_audio_fifo_push_sample(aes67_audio_fifo_t *fifo, uint32_t sample);
uint32_t aes67_audio_fifo_pull_sample(aes67_audio_fifo_t *fifo,
                                      uint32_t timestamp,
                                      int *valid);

// Timestamp and clock recovery
void aes67_audio_fifo_maintain(aes67_audio_fifo_t *fifo,
                               chanend buf_ctl,
                               int *notified_buf_ctl);

// Buffer control message handling
void aes67_audio_fifo_handle_buf_ctl_unsafe(unsigned int buf_ctl,
                                            aes67_audio_fifo_t *unsafe fifo,
                                            int *unsafe buf_ctl_notified,
                                            unsigned int tmr);
aes67_fifo_state_t aes67_audio_fifo_get_state(aes67_audio_fifo_t *fifo);

// Multi-bit sample support
void aes67_audio_fifo_push_samples(aes67_audio_fifo_t *fifo,
                                   uint8_t *sample_ptr,
                                   size_t sample_size,
                                   size_t stride,
                                   size_t num_samples,
                                   uint32_t encoding,
                                   uint32_t ptp_timestamp_ns);
