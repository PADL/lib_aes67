// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025-2026, PADL Software Pty Ltd, All rights reserved

#include <stdlib.h>
#include <xccompat.h>
#include <xscope.h>
#include <xassert.h>

#include "aes67_internal.h"
#include "ptp.h"
#include "ptp_internal.h"
#include "debug_print.h"
#include "media_clock_internal.h"
#include "media_clock_client.h"
#include "misc_timer.h"

// The clock recovery internal representation of the worldlen.  More precision
// and range than the external worldlen representation. The max precision is 26
// bits before the PTP clock recovery multiplication overflows
#define WORDLEN_FRACTIONAL_BITS 24

#define MAX_ERROR_TOLERANCE 500

typedef struct _pid_error_t {
    int64_t p, i, d;
} pid_error_t;

static inline void pid_error_init(pid_error_t *error) {
    error->p = error->i = error->d = 0;
}

/**
 * \brief Records the state of the clock recovery for one media clock
 */
typedef struct _clock_info_t {
    uint32_t t1;                       // Previous local timestamp
    uint64_t wordlen_40_24;            // Sample period in ticks, as 40.24 fixed point integer
    pid_error_t error;                 // Accumulated/last error
    uint32_t rate;                     // Sample rate (Hz)
    uint64_t ptp1;                     // Actual PTP nanoseconds at t1
} clock_info_t;

static clock_info_t ptp_clock_info;

/**
 * Converts the internal 64 bit wordlen into an external 32 bit wordlen
 */
static inline uint32_t wordlen_40_24_to_wordlen_16_16(uint64_t w_40_24) {
    return (w_40_24 >> (WORDLEN_FRACTIONAL_BITS - WC_FRACTIONAL_BITS));
}

/**
 * Calculates expected local wordlen (sample period) for a given rate,
 * expressed as a 40.24 fixed point integer.
 */
static inline uint64_t calculate_wordlen_40_24(uint32_t sample_rate) {
    const uint64_t timer_ticks_per_second = ((uint64_t)TIMER_TICKS_PER_SEC << WORDLEN_FRACTIONAL_BITS);
    return (timer_ticks_per_second / sample_rate);
}

void aes67_init_media_clock_recovery(uint32_t clk_time, uint32_t rate) {
    clock_info_t *clock_info = &ptp_clock_info;
    ptp_time_info_mod64 timeInfo;

    clock_info->rate = rate;
    pid_error_init(&clock_info->error);
    clock_info->wordlen_40_24 = rate ? calculate_wordlen_40_24(clock_info->rate) : 0;

    clock_info->t1 = clk_time;

    ptp_get_local_time_info_mod64(&timeInfo);
    clock_info->ptp1 = local_timestamp_to_ptp_mod64(clk_time, &timeInfo);
}

uint32_t
aes67_update_media_clock(const uint32_t t2,
                         const aes67_media_clock_pid_coefficients_t *pid_coefficients) {
    clock_info_t *clock_info = &ptp_clock_info;
    const int64_t diff_local = (int64_t)t2 - (int64_t)clock_info->t1;

    if (abs64(diff_local) < 100)
      return wordlen_40_24_to_wordlen_16_16(clock_info->wordlen_40_24);

    ptp_time_info_mod64 timeInfo;
    pid_error_t error;
    uint64_t ptp2;

    pid_error_init(&error);
    ptp_get_local_time_info_mod64(&timeInfo);
    ptp2 = local_timestamp_to_ptp_mod64(t2, &timeInfo);

    const int64_t diff_ptp = (int64_t)ptp2 - (int64_t)clock_info->ptp1;

    // error expressed in _nanoseconds_ as a 40.24 fixed-point integer (10ns per tick)
    error.i = (diff_ptp * clock_info->wordlen_40_24) - (diff_local * clock_info->wordlen_40_24 * 10);
    error.i = (error.i << WORDLEN_FRACTIONAL_BITS) / clock_info->wordlen_40_24;

    if (abs64(error.i >> WORDLEN_FRACTIONAL_BITS) > MAX_ERROR_TOLERANCE) {
        debug_printf("aes67_update_media_clock: resetting PID, error %d!\n", error.i >> WORDLEN_FRACTIONAL_BITS);
        clock_info->wordlen_40_24 = calculate_wordlen_40_24(clock_info->rate);
        pid_error_init(&clock_info->error);
    } else {
        error.p = error.i - clock_info->error.i;
        error.d = pid_coefficients->d_numerator ? (error.p - clock_info->error.p) : 0;

        clock_info->error.p = error.p & ~(0xff); // mask scheduler noise
        clock_info->error.i = error.i;

        clock_info->wordlen_40_24 -=
            ((error.p / diff_local) * pid_coefficients->p_numerator) / pid_coefficients->p_denominator +
            ((error.i / diff_local) * pid_coefficients->i_numerator) / pid_coefficients->i_denominator +
            ((error.d / diff_local) * pid_coefficients->d_numerator) / pid_coefficients->d_denominator;
    }

    clock_info->t1 = t2;
    clock_info->ptp1 = ptp2;

    return wordlen_40_24_to_wordlen_16_16(clock_info->wordlen_40_24);
}

uint32_t fifo_uninit_count = AES67_NUM_MEDIA_OUTPUTS;

#if (AES67_NUM_MEDIA_OUTPUTS != 0)
buf_info_t buf_info[AES67_NUM_MEDIA_OUTPUTS];

void aes67_media_clock_init_buffers(void) {}

int aes67_media_clock_get_buf_info(uintptr_t fifo) {
    int stream_num = -1;

    for (int i = 0; i < AES67_NUM_MEDIA_OUTPUTS; i++) {
        if (buf_info[i].fifo == fifo)
            stream_num = i;
    }

    return stream_num;
}

void aes67_register_buf_fifo(uint32_t i, uintptr_t fifo) {
    buf_info[i].fifo = fifo;
    fifo_uninit_count--;
}
#endif

// PID coefficients for various clock chips

aes67_media_clock_pid_coefficients_t cs2100_pid_coefficients = {
    .p_numerator = 100,   // Increased P gain: 100/11 ≈ 9.09 for faster response
    .p_denominator = 11,
    .i_numerator = 1,     // Reduced I gain: 1/20 = 0.05 to prevent windup
    .i_denominator = 20,
    .d_numerator = 1,     // Added small D term: 1/10 = 0.1 for stability
    .d_denominator = 10
};

aes67_media_clock_pid_coefficients_t cs2300_pid_coefficients = {
    .p_numerator = 32,    // Original CS2300 P gain: 32/1 = 32
    .p_denominator = 1,
    .i_numerator = 1,     // Original CS2300 I gain: 1/4 = 0.25
    .i_denominator = 4,
    .d_numerator = 0,     // Original CS2300 had no D term
    .d_denominator = 1
};

aes67_media_clock_pid_coefficients_t cs2600_pid_coefficients = {
    .p_numerator = 512,
    .p_denominator = 1,
    .i_numerator = 1,
    .i_denominator = 4,
    .d_numerator = 0,
    .d_denominator = 1
};
