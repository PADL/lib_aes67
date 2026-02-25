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

typedef struct _pid_error_t {
    int64_t p, i, d;
} pid_error_t;

static inline void
pid_error_init(pid_error_t *error) {
    error->p = error->i = error->d = 0;
}

typedef struct _timestamp_info_t {
    uint32_t local;                    // local timestamp in ticks
    uint32_t actual;                   // local PTP timestamp mod 32
    uint32_t expected;                 // media clock PTP timestamp mod 32
} timestamp_info_t;

static inline void
timestamp_info_init(timestamp_info_t *info) {
    info->local = info->actual = info->expected = 0;
}

static inline uint8_t
timestamp_info_valid(const timestamp_info_t *info) {
    return !!info->local;
}

typedef struct _clock_info_t {
    uint64_t wordlen_40_24;         // sample period in ticks, as 40.24 fixed point integer
    pid_error_t error;              // accumulated/last error
    uint32_t rate;                  // sample rate (Hz)

    timestamp_info_t t2;            // timestamp at t2
    timestamp_info_t t1;            // timestamp at t1

    unsigned int first;
    unsigned int locked;
} clock_info_t;

// the global PTP media clock
static clock_info_t ptp_clock_info;

// convert internal 64-bit wordlen to external 32-bit wordlen
static inline uint32_t
wordlen_40_24_to_wordlen_16_16(uint64_t w_40_24) {
    return (w_40_24 >> (WORDLEN_FRACTIONAL_BITS - WC_FRACTIONAL_BITS));
}

// calculates expected local wordlen (sample period) for a given rate,
// expressed as a 40.24 fixed point integer.
static inline uint64_t
calculate_wordlen_40_24(uint32_t sample_rate) {
    const uint64_t timer_ticks_per_second = ((uint64_t)TIMER_TICKS_PER_SEC << WORDLEN_FRACTIONAL_BITS);
    return (timer_ticks_per_second / sample_rate);
}

void
aes67_init_media_clock_recovery(uint32_t rate) {
    clock_info_t *clock_info = &ptp_clock_info;

    clock_info->rate = rate;
    pid_error_init(&clock_info->error);
    clock_info->wordlen_40_24 = rate ? calculate_wordlen_40_24(clock_info->rate) : 0;

    timestamp_info_init(&clock_info->t2);
    timestamp_info_init(&clock_info->t1);

    clock_info->first = TRUE;
    clock_info->locked = FALSE;
}

void
aes67_update_media_clock_info(uint8_t locked,
                              uint32_t local_ts,
                              uint32_t local_ptp_ts,
                              uint32_t media_clock_ptp_ts) {
    clock_info_t *clock_info = &ptp_clock_info;

    // because local is used to atomically test validity, ensure that it is
    // never zero (at the expenese of 10ns jitter on rollover)
    clock_info->t2.local = local_ts ? local_ts : 1;
    clock_info->t2.actual = local_ptp_ts;
    clock_info->t2.expected = media_clock_ptp_ts;

    clock_info->locked = locked;
}

//
// some notes on clock recovery
//
// lib_tsn performs stream-based clock recovery, where the phase error is the
// presentation time - the current time (expressed in the PTP time base). The
// current time is always sampled at the PLL clock edge (or as close as
// possible).
//
// AES67 uses sample time rather than presentation time, but the effect is the
// same (simply requiring the local time to be adjusted backwards by the
// expected presentation time offset).
//
// phase error = t2.expected - t2.actual
//
// the old PTP media clock recovery code (since deleted) calculated the phase
// error as the difference between the PTP and local deltas (expressed in PTP
// time base), where the delta is between PID invocations. This has the
// advantage that it can run in a sender-only configuration, but it does not
// incorporate any feedback from the PLL.
//
// PTP delta = t2.actual - t1.actual
// local delta = t2.local - t1.local
// phase error = ptp_delta - local delta * NANOSECONDS_PER_TICK
//
// note in both cases the phase error is divided by PID sample period
// (local delta) to decouple the PID coefficients from the sample period
//

#define DEBUG_INTERVAL 100

#if DEBUG_INTERVAL
static int pid_debug_counter;
#endif

#define PID_MAX_CORRECTION (1 << 20)

uint32_t
aes67_update_media_clock(const aes67_media_clock_pid_coefficients_t *pid_coefficients) {
    clock_info_t *clock_info = &ptp_clock_info;

    if (timestamp_info_valid(&clock_info->t2)) {
        if (timestamp_info_valid(&clock_info->t1)) {
            if (likely(clock_info->locked)) {
                pid_error_t error;
#if DEBUG_INTERVAL
                pid_debug_counter++;
#endif

                pid_error_init(&error);
                error.i = (int64_t)clock_info->t2.expected - (int64_t)clock_info->t2.actual;
                error.i <<= WORDLEN_FRACTIONAL_BITS;

                if (!clock_info->first) {
                    error.p = error.i - clock_info->error.i;
                    error.d = pid_coefficients->d_numerator ? (error.p - clock_info->error.p) : 0;

                    const int64_t pid_period = ((int64_t)clock_info->t2.local - (int64_t)clock_info->t1.local);
                    const int64_t wl_correction =
                        ((error.p / pid_period) * pid_coefficients->p_numerator) / pid_coefficients->p_denominator +
                        ((error.i / pid_period) * pid_coefficients->i_numerator) / pid_coefficients->i_denominator +
                        ((error.d / pid_period) * pid_coefficients->d_numerator) / pid_coefficients->d_denominator;

                    if (abs64(wl_correction) >= PID_MAX_CORRECTION) {
#if DEBUG_INTERVAL
                        if ((pid_debug_counter % DEBUG_INTERVAL) == 0)
                            debug_printf("PID: wl_correction %d %x out of range\n",
                                         (wl_correction << 32), (wl_correction & 0xffffffff));
#endif
                        goto reset_pid;
                    }
#if DEBUG_INTERVAL
                    if ((pid_debug_counter % DEBUG_INTERVAL) == 0) {
                        debug_printf("PID: phase_err %d freq_err %d wl_correction %d\n",
                                     (int32_t)error.i, (int32_t)error.p, (int32_t)wl_correction);
                    }
#endif
                    clock_info->wordlen_40_24 -= wl_correction;
                } else {
                    clock_info->first = FALSE;
                }

                clock_info->error.p = error.p;
                clock_info->error.i = error.i;
            } else {
reset_pid:
                clock_info->wordlen_40_24 = calculate_wordlen_40_24(clock_info->rate);
                pid_error_init(&clock_info->error);
                clock_info->first = TRUE;
            }
        }

        clock_info->t1 = clock_info->t2;
        clock_info->t2.local = 0; // atomically mark t2 invalid
    }

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
// TODO: re-add coefficients for CS2100/CS2300 when we have tested
aes67_media_clock_pid_coefficients_t cs2600_pid_coefficients = {
    .p_numerator = 2,
    .p_denominator = 1,
    .i_numerator = 0,
    .i_denominator = 4,
    .d_numerator = 0,
    .d_denominator = 1
};
