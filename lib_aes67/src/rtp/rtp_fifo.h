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
    AES67_FIFO_LOCKING,      // Clock recovery trying to lock to the sample stream
    AES67_FIFO_LOCKED        // Clock recovery is locked and working
} aes67_fifo_state_t;

typedef struct aes67_audio_fifo {
    volatile uint32_t *unsafe dptr;         // The read pointer
    volatile uint32_t *unsafe wrptr;        // The write pointer
    volatile uint32_t *unsafe marker;       // This indicates which sample is the one which the
                                            // timestamps apply to
    uint32_t local_ts;                      // Local time at sample playout

    uint32_t media_clock;                   // RTP media clock
    uint32_t clock_offset;                  // SDP media clock offset
    uint32_t packet_time;                   // SDP packet time in ns
    uint32_t sample_count;                  // The count of samples that have passed through the buffer
    volatile uint32_t *unsafe zero_marker;
    aes67_fifo_state_t state;               // State of the FIFO
    uint32_t last_notification_time;        // Last time that the clock recovery thread was
                                            // informed of the timestamp info
    uint8_t pending_init_notification;      // Flag for pending initialization
    uint8_t zero_flag;                      // When set, the FIFO will output zero samples instead of its contrents
    uint32_t fifo[AUDIO_OUTPUT_FIFO_WORD_SIZE]; // Circular buffer storage
} aes67_audio_fifo_t;

// FIFO management functions
void aes67_audio_fifo_init(aes67_audio_fifo_t *unsafe fifo);
void aes67_audio_fifo_disable(aes67_audio_fifo_t *unsafe fifo);
void aes67_audio_fifo_enable(aes67_audio_fifo_t *unsafe fifo);

// Sample operations
void aes67_audio_fifo_push_sample(aes67_audio_fifo_t *fifo, uint32_t sample);

// Pull a sample from the FIFO
unsafe static inline uint32_t
aes67_audio_fifo_pull_sample(aes67_audio_fifo_t *unsafe fifo,
                             uint32_t timestamp) {
    volatile uint32_t *unsafe dptr = fifo->dptr;
    uint32_t sample;
    uint8_t valid;

    valid = (dptr != fifo->wrptr);

    if (dptr == fifo->wrptr)
        return 0; // underflow

    sample = *dptr;

    if (dptr == fifo->marker && fifo->local_ts == 0)
        fifo->local_ts = timestamp ? timestamp : 1;

    dptr++;
    if (dptr == END_OF_FIFO(fifo))
        dptr = START_OF_FIFO(fifo);

    fifo->dptr = dptr;

    return (valid && !fifo->zero_flag) ? sample : 0;
}

// Timestamp and clock recovery
void aes67_audio_fifo_maintain(aes67_audio_fifo_t *fifo,
                               chanend buf_ctl,
                               uint32_t media_clock,
                               uint32_t clock_offset,
                               uint32_t packet_time_us,
                               int *notified_buf_ctl);

// Buffer control message handling
void aes67_audio_fifo_handle_buf_ctl_unsafe(uint32_t buf_ctl,
                                            aes67_audio_fifo_t *unsafe fifo,
                                            int *unsafe buf_ctl_notified);
aes67_fifo_state_t aes67_audio_fifo_get_state(aes67_audio_fifo_t *fifo);

// Multi-bit sample support
void aes67_audio_fifo_push_samples(aes67_audio_fifo_t *fifo,
                                   uint8_t *sample_ptr,
                                   size_t sample_size,
                                   size_t stride,
                                   size_t num_samples,
                                   uint32_t encoding
#if AES67_METERING
                                   , size_t meter_channel
#endif
                                   );
