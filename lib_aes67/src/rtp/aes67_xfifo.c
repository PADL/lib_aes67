// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025 PADL Software Pty Ltd, All rights reserved

#include <string.h>
#include <xassert.h>

#include "aes67_xfifo.h"
#include "aes67_media_clock_client.h"
#include "aes67_utils.h"

#define OUTPUT_DURING_LOCK 0
#define NOTIFICATION_PERIOD 250

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// FIFO initialization
void aes67_audio_fifo_init(aes67_audio_fifo_t *fifo) {
    fifo->state = AES67_FIFO_DISABLED;
    fifo->dptr = START_OF_FIFO(fifo);
    fifo->wrptr = START_OF_FIFO(fifo);
    fifo->pending_init_notification = 0;
    fifo->last_notification_time = 0;
    fifo->zero_flag = 1;
    fifo->marker = NULL;
    fifo->ptp_ts = 0;
    fifo->sample_count = 0;
    fifo->zero_marker = NULL;
}

// Disable FIFO
void aes67_audio_fifo_disable(aes67_audio_fifo_t *fifo) {
    if (!fifo)
        return;

    fifo->state = AES67_FIFO_DISABLED;
    fifo->zero_flag = 1;
}

// Enable FIFO
void aes67_audio_fifo_enable(aes67_audio_fifo_t *fifo) {
    fifo->state = AES67_FIFO_ZEROING;
    fifo->dptr = START_OF_FIFO(fifo);
    fifo->wrptr = START_OF_FIFO(fifo);
    fifo->marker = NULL;
    fifo->ptp_ts = 0;
    fifo->zero_marker = END_OF_FIFO(fifo) - 1;
    fifo->zero_flag = 1;
    *fifo->zero_marker = 1;
    fifo->sample_count = 0;
    fifo->pending_init_notification = 1;
}

// Pull a sample from the FIFO
uint32_t aes67_audio_fifo_pull_sample(aes67_audio_fifo_t *fifo,
                                      uint32_t timestamp,
                                      int *valid) {
    volatile uint32_t *dptr = fifo->dptr;
    uint32_t sample;

    *valid = (dptr != fifo->wrptr);

    if (dptr == fifo->wrptr) {
        // Underflow
        return 0;
    }

    sample = *dptr;

    // Check if this is the marked sample for timestamp correlation
    if (dptr == fifo->marker) {
        // Marker reached - could be used for timing if needed
    }

    dptr++;
    if (dptr == END_OF_FIFO(fifo)) {
        dptr = START_OF_FIFO(fifo);
    }

    fifo->dptr = dptr;

    // Set local timestamp when sample is pulled for timing correlation
    if (timestamp == 0)
        timestamp = 1;
    fifo->local_ts = timestamp;

    if (fifo->zero_flag)
        sample = 0;

    return sample;
}

// Set presentation timestamp for packet timing
void aes67_audio_fifo_set_presentation_time(aes67_audio_fifo_t *fifo,
                                            uint32_t ptp_time_ns) {
    if (ptp_time_ns == 0)
        ptp_time_ns = 1;

    fifo->ptp_ts = ptp_time_ns;
}

// Maintain FIFO state and notify clock recovery
void aes67_audio_fifo_maintain(aes67_audio_fifo_t *fifo,
                               chanend buf_ctl,
                               int *notified_buf_ctl) {
    uint32_t time_since_last_notification;

    if (fifo->pending_init_notification && *notified_buf_ctl == 0) {
        aes67_notify_buf_ctl_stream_info(buf_ctl, (uintptr_t)fifo);
        *notified_buf_ctl = 1;
        fifo->pending_init_notification = 0;
    }

    switch (fifo->state) {
    case AES67_FIFO_DISABLED:
        break;

    case AES67_FIFO_ZEROING:
        if (*fifo->zero_marker == 0) {
            // We have zeroed the entire fifo
            // Set the wrptr so that the fifo size is 1/2 of the buffer size
            int buf_len = (END_OF_FIFO(fifo) - START_OF_FIFO(fifo));
            volatile uint32_t *new_wrptr;

            new_wrptr = fifo->dptr + (buf_len >> 1);
            while (new_wrptr >= END_OF_FIFO(fifo))
                new_wrptr -= buf_len;

            fifo->wrptr = new_wrptr;
            fifo->state = AES67_FIFO_LOCKING;
            fifo->ptp_ts = 0;
            fifo->marker = NULL;
#if (OUTPUT_DURING_LOCK == 0)
            fifo->zero_flag = 1;
#endif
        }
        break;

    case AES67_FIFO_LOCKING:
    case AES67_FIFO_LOCKED:
        time_since_last_notification =
            (int32_t)fifo->sample_count - (int32_t)fifo->last_notification_time;

        if (fifo->ptp_ts != 0 && *notified_buf_ctl == 0 &&
            (fifo->last_notification_time == 0 ||
             time_since_last_notification > NOTIFICATION_PERIOD)) {
            aes67_notify_buf_ctl_stream_info(buf_ctl, (uintptr_t)fifo);
            *notified_buf_ctl = 1;
            fifo->last_notification_time = fifo->sample_count;
        }
        break;
    }
}

// Push multiple samples with stride support for different bit depths
void aes67_audio_fifo_push_samples(aes67_audio_fifo_t *fifo,
                                   uint8_t *sample_ptr,
                                   size_t sample_size,
                                   size_t stride, // effectively channel count
                                   size_t num_frames,
                                   uint32_t encoding) {
    volatile uint32_t *wrptr = fifo->wrptr;
    uint32_t sample;
    size_t sample_count = 0;

    if (fifo->state == AES67_FIFO_DISABLED)
        return;

    for (size_t i = 0; i < num_frames; i++) {
        switch (encoding) {
        case AES67_ENCODING_L32: // 32-bit
            sample = __builtin_bswap32(*((uint32_t *)sample_ptr));
            break;
        case AES67_ENCODING_L24: // 24-bit
            sample = ((uint32_t)sample_ptr[0] << 16) |
                     ((uint32_t)sample_ptr[1] << 8) | ((uint32_t)sample_ptr[2]);
            sample <<= 8;
            break;
        case AES67_ENCODING_L16: // 16-bit
            sample =
                (uint32_t)(__builtin_bswap16(*((uint16_t *)sample_ptr)) << 16);
            break;
        case AES67_ENCODING_AM824: // 32-bit right justified, network byte order
            sample = __builtin_bswap32(*((uint32_t *)sample_ptr)) << 8;
            break;
        default:
            fail("unsupported RTP encoding");
            return;
        }

        if (unlikely(fifo->state == AES67_FIFO_ZEROING))
            sample = 0;

        volatile uint32_t *new_wrptr = wrptr + 1;

        if (new_wrptr == END_OF_FIFO(fifo))
            new_wrptr = START_OF_FIFO(fifo);

        if (new_wrptr != fifo->dptr) {
            *wrptr = sample;
            wrptr = new_wrptr;
        }

        sample_ptr += sample_size * stride;
        sample_count++;
    }

    fifo->wrptr = wrptr;
    fifo->sample_count += sample_count;
}

// Handle buffer control messages
void aes67_audio_fifo_handle_buf_ctl_unsafe(unsigned int buf_ctl,
                                            aes67_audio_fifo_t *fifo,
                                            int *buf_ctl_notified,
                                            unsigned int tmr) {
    int cmd = aes67_get_buf_ctl_cmd(buf_ctl);

    switch (cmd) {
    case BUF_CTL_REQUEST_INFO: {
        aes67_send_buf_ctl_info(buf_ctl, fifo->state == AES67_FIFO_LOCKED,
                                fifo->ptp_ts, 0,
                                fifo->dptr - START_OF_FIFO(fifo),
                                fifo->wrptr - START_OF_FIFO(fifo), tmr);
        fifo->ptp_ts = 0;
        fifo->marker = NULL;
        break;
    }

    case BUF_CTL_ADJUST_FILL: {
        int adjust = aes67_get_buf_ctl_adjust(buf_ctl);
        volatile uint32_t *new_wrptr = fifo->wrptr - adjust;

        while (new_wrptr < START_OF_FIFO(fifo))
            new_wrptr += (END_OF_FIFO(fifo) - START_OF_FIFO(fifo));

        while (new_wrptr >= END_OF_FIFO(fifo))
            new_wrptr -= (END_OF_FIFO(fifo) - START_OF_FIFO(fifo));

        fifo->wrptr = new_wrptr;
        fifo->state = AES67_FIFO_LOCKED;
        fifo->zero_flag = 0;
        fifo->ptp_ts = 0;
        fifo->marker = NULL;
        aes67_buf_ctl_ack(buf_ctl);
        *buf_ctl_notified = 0;
        break;
    }

    case BUF_CTL_RESET:
        fifo->state = AES67_FIFO_ZEROING;
        if (fifo->wrptr == START_OF_FIFO(fifo))
            fifo->zero_marker = END_OF_FIFO(fifo) - 1;
        else
            fifo->zero_marker = fifo->wrptr - 1;
        fifo->zero_flag = 1;
        *fifo->zero_marker = 1;
        aes67_buf_ctl_ack(buf_ctl);
        *buf_ctl_notified = 0;
        break;

    case BUF_CTL_ACK:
        aes67_buf_ctl_ack(buf_ctl);
        *buf_ctl_notified = 0;
        break;

    default:
        break;
    }
}

// Get FIFO state
aes67_fifo_state_t aes67_audio_fifo_get_state(aes67_audio_fifo_t *fifo) {
    if (fifo == NULL)
        return AES67_FIFO_DISABLED;
    else
        return fifo->state;
}

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
