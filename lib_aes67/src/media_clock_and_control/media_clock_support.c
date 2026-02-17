// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025, PADL Software Pty Ltd, All rights reserved

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
// and range than the external worldlen representation.  The max percision is 26
// bits before the PTP clock recovery multiplication overflows
#define WORDLEN_FRACTIONAL_BITS 24

/**
 * \brief Records the state of the clock recovery for one media clock
 */
typedef struct clock_info_t {
    // see commit 3f775411 of lib_tsn which removed this support
    uint32_t t1;
    uint32_t ptp1;
    uint64_t wordlen;

    int64_t ierror;
    int64_t prev_perror; // For derivative term calculation
    uint32_t rate;

    int first;
} clock_info_t;

static clock_info_t ptp_clock_info;

/**
 * \brief Converts the internal 64 bit wordlen into an external 32 bit wordlen
 */
static inline uint32_t local_wordlen_to_external_wordlen(uint64_t w) {
    return (w >> (WORDLEN_FRACTIONAL_BITS - WC_FRACTIONAL_BITS));
}

static uint64_t calculate_wordlen(uint32_t sample_rate) {
    // Calculate local timer ticks per sample
    // Local timer runs at 100MHz (100,000,000 ticks/sec)
    // For a given sample rate, each sample represents 1/sample_rate seconds
    const uint64_t timer_ticks_per_second =
        (100000000LL << WORDLEN_FRACTIONAL_BITS);

    if ((sample_rate % 48000) == 0 || (sample_rate % 44100) == 0) {
        return (timer_ticks_per_second / sample_rate);
    } else {
        fail("Unsupported sample rate");
    }
}

void aes67_init_media_clock_recovery(uint32_t clk_time, uint32_t rate) {
    clock_info_t *clock_info = &ptp_clock_info;
    ptp_time_info_mod64 timeInfo;

    clock_info->first = 1;
    clock_info->rate = rate;
    clock_info->ierror = 0;
    clock_info->prev_perror = 0;

    if (rate != 0) {
        clock_info->wordlen = calculate_wordlen(clock_info->rate);
    } else {
        clock_info->wordlen = 0;
    }

    clock_info->t1 = clk_time;
    ptp_get_local_time_info_mod64(&timeInfo);

    clock_info->ptp1 = local_timestamp_to_ptp_mod32(clk_time, &timeInfo);
}

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
    .p_numerator = 50,
    .p_denominator = 1,
    .i_numerator = 0,
    .i_denominator = 50,
    .d_numerator = 0,
    .d_denominator = 1
};

#define MAX_ERROR_TOLERANCE 100

uint32_t aes67_update_media_clock(
    REFERENCE_PARAM(const aes67_media_clock_t, mclock),
    uint32_t t2,
    REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t,
                    pid_coefficients)) {
    clock_info_t *clock_info = &ptp_clock_info;
    int64_t perror, diff_ptp, diff_local;
    ptp_time_info_mod64 timeInfo;
    uint32_t ptp2;

    ptp_get_local_time_info_mod64(&timeInfo);
    ptp2 = local_timestamp_to_ptp_mod32(t2, &timeInfo);

    diff_local = (int32_t)t2 - (int32_t)clock_info->t1;
    diff_ptp = (int32_t)ptp2 - (int32_t)clock_info->ptp1;

    debug_printf("diff_local %d diff_ptp %d\n", diff_local, diff_ptp);

    if (abs64(diff_local) == 0)
      return local_wordlen_to_external_wordlen(clock_info->wordlen);

    perror = (diff_ptp * clock_info->wordlen) - (diff_local * clock_info->wordlen * 10);
    perror = ((perror << WORDLEN_FRACTIONAL_BITS) / (int64_t)clock_info->wordlen);

    if ((perror >> WORDLEN_FRACTIONAL_BITS) > MAX_ERROR_TOLERANCE ||
        (perror >> WORDLEN_FRACTIONAL_BITS) < -MAX_ERROR_TOLERANCE) {
        clock_info->wordlen =
            (((int64_t)TIMER_TICKS_PER_SEC << WORDLEN_FRACTIONAL_BITS) / clock_info->rate);
        clock_info->ierror = 0;
        clock_info->prev_perror = 0;
        clock_info->first = 1;
    } else {
        int64_t ierror, derror = 0;

        if (clock_info->first) {
            clock_info->ierror = perror;
            clock_info->first = 0;
        } else {
            clock_info->ierror += perror;
            derror = pid_coefficients->d_numerator ? (perror - clock_info->prev_perror) : 0;
        }
        ierror = clock_info->ierror;

        clock_info->prev_perror = perror;

        clock_info->wordlen -=
            ((perror / diff_local) * pid_coefficients->p_numerator) / pid_coefficients->p_denominator -
            ((ierror / diff_local) * pid_coefficients->i_numerator) / pid_coefficients->i_denominator -
            ((derror / diff_local) * pid_coefficients->d_numerator) / pid_coefficients->d_denominator;
    }

    clock_info->t1 = t2;
    clock_info->ptp1 = ptp2;

    return local_wordlen_to_external_wordlen(clock_info->wordlen);
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
