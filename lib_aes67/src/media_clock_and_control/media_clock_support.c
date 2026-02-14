// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025, PADL Software Pty Ltd, All rights reserved

/**
 * PID CONTROLLER TUNING GUIDE:
 *
 * To enable PID tuning debug output, add to your Makefile or build config:
 * -DDEBUG_PID_TUNING=1
 *
 * To tune PID coefficients, modify the constants below:
 * - pid_coefficients.p_numerator/pid_coefficients.p_denominator: Proportional
 * gain (default 80/11 ≈ 7.27)
 * - pid_coefficients->i_numerator/pid_coefficients->i_denominator: Integral
 * gain (default 1/5 = 0.2)
 * - pid_coefficients->d_numerator/pid_coefficients->d_denominator: Derivative
 * gain (default 0/1 = 0, disabled)
 *
 * Suggested tuning sequence:
 * 1. Set D=0, tune P for fastest response without overshoot
 * 2. Add I to eliminate steady-state error
 * 3. Add D if needed to reduce overshoot/oscillation
 */

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
static uint32_t local_wordlen_to_external_wordlen(uint64_t w) {
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

void aes67_init_media_clock_recovery(NULLABLE_RESOURCE(chanend, ptp_svr),
                                     uint32_t clk_time,
                                     uint32_t rate) {
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
    .p_numerator = 80,    // Original CS2100 P gain: 80/11 ≈ 7.27
    .p_denominator = 11,
    .i_numerator = 1,     // Original CS2100 I gain: 1/5 = 0.2
    .i_denominator = 5,
    .d_numerator = 0,     // Original CS2100 had no D term
    .d_denominator = 1
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
    .p_numerator = 5,
    .p_denominator = 1,
    .i_numerator = 1,
    .i_denominator = 50,
    .d_numerator = 0,
    .d_denominator = 1
};

#define MAX_ERROR_TOLERANCE 100

// Enhanced debug levels for PID tuning
#ifndef DEBUG_PID_TUNING
#define DEBUG_PID_TUNING 0
#endif

uint32_t aes67_update_media_clock(
    NULLABLE_RESOURCE(chanend, ptp_svr),
    REFERENCE_PARAM(const aes67_media_clock_t, mclock),
    uint32_t t2,
    int32_t period0,
    REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t,
                    pid_coefficients)) {
    clock_info_t *clock_info = &ptp_clock_info;
    int64_t ierror, perror, derror;
    int64_t diff_local;
    ptp_time_info_mod64 timeInfo;

    int64_t err, diff_ptp;
    uint32_t ptp2;

    ptp_get_local_time_info_mod64(&timeInfo);
    ptp2 = local_timestamp_to_ptp_mod32(t2, &timeInfo);

    diff_local = (int32_t)t2 - (int32_t)clock_info->t1;
    diff_ptp = (int32_t)ptp2 - (int32_t)clock_info->ptp1;

    // Skip PID update if insufficient time has passed to avoid division by zero
    int64_t abs_diff_local = diff_local < 0 ? -diff_local : diff_local;
    if (abs_diff_local < 100) { // Minimum 100 timer ticks (1us at 100MHz)
        return local_wordlen_to_external_wordlen(clock_info->wordlen);
    }

    // Calculate timing error using PTP-based approach
    // error in ns = diff_ptp - diff_local * wlptp / wl
    // error in ns * wl = dptp * wl - dlocal * wlptp
    // err = actual - expected

    err = (diff_ptp * clock_info->wordlen) -
          (diff_local * clock_info->wordlen * 10);
    err = ((err << WORDLEN_FRACTIONAL_BITS) / (int64_t)clock_info->wordlen);

    // Chop off bottom bits - thread scheduling causes noise here
    err &= ~(0xff);

    if ((err >> WORDLEN_FRACTIONAL_BITS) > MAX_ERROR_TOLERANCE ||
        (err >> WORDLEN_FRACTIONAL_BITS) < -MAX_ERROR_TOLERANCE) {
        // Reset to default wordlen if error is too large
        clock_info->wordlen =
            ((100000000LL << WORDLEN_FRACTIONAL_BITS) / clock_info->rate);
        clock_info->ierror = 0;
        clock_info->prev_perror = 0;
        clock_info->first = 1;
    } else {
        // Apply PID control using the calculated error
        // P term: current error
        perror = err;

        // I term: accumulate error over time
        if (clock_info->first) {
            clock_info->ierror = err;
            derror = 0; // No derivative on first call
            clock_info->first = 0;
        } else {
            clock_info->ierror += err;
            // D term: rate of change of error
            if (pid_coefficients->d_numerator != 0) {
                derror = perror - clock_info->prev_perror;
            } else {
                derror = 0;
            }
        }
        ierror = clock_info->ierror;

#if DEBUG_PID_TUNING
        if (clock_info->prev_perror != perror || abs((int32_t)err) > 1000) {
            debug_printf(
                "PID: P=%d/%d I=%d/%d D=%d/%d perror=%d ierror=%d derror=%d "
                "diff_local=%d wordlen=%x\n",
                pid_coefficients->p_numerator, pid_coefficients->p_denominator,
                pid_coefficients->i_numerator, pid_coefficients->i_denominator,
                pid_coefficients->d_numerator, pid_coefficients->d_denominator,
                perror, ierror, derror, diff_local, clock_info->wordlen);
        }
#endif

        // Update stored values for next iteration
        clock_info->prev_perror = perror;

        // Apply PID control to adjust wordlen
        // P term: proportional to current error
        // I term: proportional to accumulated error
        // D term: proportional to rate of error change
        if (diff_local) {
            int64_t p_adjustment = ((perror * pid_coefficients->p_numerator) /
                                    pid_coefficients->p_denominator) /
                                   diff_local;
            int64_t i_adjustment = ((ierror * pid_coefficients->i_numerator) /
                                    pid_coefficients->i_denominator) /
                                   diff_local;
            int64_t d_adjustment = ((derror * pid_coefficients->d_numerator) /
                                    pid_coefficients->d_denominator) /
                                   diff_local;

            clock_info->wordlen = clock_info->wordlen - p_adjustment -
                                  i_adjustment - d_adjustment;
        }
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
