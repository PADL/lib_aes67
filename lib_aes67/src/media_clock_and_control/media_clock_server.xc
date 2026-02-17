// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025-2026, PADL Software Pty Ltd, All rights reserved
#include <xs1.h>
#include <xclib.h>
#include <debug_print.h>
#include <xscope.h>
#include <xassert.h>

#include "media_clock_client.h"
#include "media_clock_internal.h"
#include "aes67_utils.h"
#include "ptp.h"
#include "ptp_internal.h"
#include "rtp_internal.h"

#ifndef DEBUG_MEDIA_CLOCK
#define DEBUG_MEDIA_CLOCK 0
#endif

#ifndef PLL_OUTPUT_TIMING_CHECK
#define PLL_OUTPUT_TIMING_CHECK 1
#endif

#define STABLE_THRESHOLD 32
#define LOCK_COUNT_THRESHOLD 1000
#define ACCEPTABLE_FILL_ADJUST 50000
#define LOST_LOCK_THRESHOLD 1000
#define MIN_FILL_LEVEL 5

// Force unlocking if there is a large step change of word length during
// "debouncing" period (improve handling of grandmaster transitions)
#define UNLOCK_ON_LARGE_DIFF_CHANGE 0
#define LOST_LOCK_THRESHOLD_LARGE 10000

aes67_media_clock_t ptp_media_clock = {{0}};
static int registered[MAX_CLK_CTL_CLIENTS];

void aes67_clk_ctl_set_rate(chanend clk_ctl, int wordLength) {
    master {
        clk_ctl <: CLK_CTL_SET_RATE;
        clk_ctl <: wordLength;
    }
}

#if AES67_NUM_MEDIA_OUTPUTS != 0

static void manage_buffer(buf_info_t &b,
                          chanend buf_ctl,
                          int index,
                          timer tmr) {
#if DEBUG_MEDIA_CLOCK
    static int last_fill;
#endif
    uint32_t media_clock, clock_offset, packet_time;
    uint64_t ptp_ts, ptp_ts_samples;
    ptp_time_info_mod64 timeInfo;
    int fifo_locked;
    int32_t sample_diff;
    uint32_t wordLength;
    uintptr_t rdptr, wrptr;
    uint32_t local_ts;
    intptr_t fill;

    wordLength = ptp_media_clock.wordLength;

    buf_ctl <: index;
    buf_ctl <: BUF_CTL_REQUEST_INFO;
    master {
        buf_ctl <: 0;
        buf_ctl :> fifo_locked;
        buf_ctl :> media_clock;
        buf_ctl :> clock_offset;
        buf_ctl :> packet_time;
        buf_ctl :> local_ts;
        buf_ctl :> rdptr;
        buf_ctl :> wrptr;
    }

    fill = wrptr - rdptr;

    if (fill < 0)
        fill += AUDIO_OUTPUT_FIFO_WORD_SIZE;

#ifdef MEDIA_OUTPUT_FIFO_FILL
    xscope_int(MEDIA_OUTPUT_FIFO_FILL, fill);
#endif

    ptp_get_local_time_info_mod64(timeInfo);
    ptp_ts = local_timestamp_to_ptp_mod64(local_ts, timeInfo); // TODO: packet_time adjustment

    // convert computed PTP presentation to a media clock (sample count)
    ptp_ts_samples = ptp_ts * (ptp_media_clock.info.rate / 100);
    ptp_ts_samples /= (NANOSECONDS_PER_SECOND / 100);
    ptp_ts_samples += clock_offset;
    ptp_ts_samples &= 0xffffffff;

    sample_diff = (int32_t)ptp_ts_samples - (int32_t)media_clock;

#if DEBUG_MEDIA_CLOCK
    if (fifo_locked) {
        int32_t fill_diff = last_fill - fill;
        if (fill_diff > 1 || fill_diff < -1)
            debug_printf("last fill changed: last %d fill %d\n", last_fill,
                         fill);
        last_fill = fill;
    }
#endif // DEBUG_MEDIA_CLOCK

    if (wordLength == 0) {
        // clock not locked yet
        buf_ctl <: index;
        buf_ctl <: BUF_CTL_ACK;
        inct(buf_ctl);
        debug_printf("clock not locked yet\n");
        return;
    }

    if (fifo_locked && b.lock_count < LOCK_COUNT_THRESHOLD)
        b.lock_count++;

    if (sample_diff < ACCEPTABLE_FILL_ADJUST &&
        sample_diff > -ACCEPTABLE_FILL_ADJUST &&
        (sample_diff - b.prev_diff <= 1 &&
         sample_diff - b.prev_diff >= -1)) {
        b.stability_count++;
    } else {
        b.stability_count = 0;
    }

    if (!fifo_locked && b.stability_count > STABLE_THRESHOLD) {
        int max_adjust = AUDIO_OUTPUT_FIFO_WORD_SIZE - MAX_SAMPLES_PER_RTP_PACKET;

        if (fill - sample_diff > max_adjust ||
            fill - sample_diff < -max_adjust) {
#if DEBUG_MEDIA_CLOCK == 2
            debug_printf("Media output %d compensation too large: %d samples\n",
                         index, sample_diff);
#endif
            buf_ctl <: index;
            buf_ctl <: BUF_CTL_RESET;
            inct(buf_ctl);
        } else {
#if DEBUG_MEDIA_CLOCK
            debug_printf("Media output %d locked: %d samples shorter\n", index,
                         sample_diff);
#endif
            b.lock_count = 0;
            buf_ctl <: index;
            buf_ctl <: BUF_CTL_ADJUST_FILL;
            buf_ctl <: sample_diff;
            inct(buf_ctl);
            ptp_media_clock.info.lock_counter++;
        }
    } else if (fifo_locked &&
               ((b.lock_count == LOCK_COUNT_THRESHOLD &&
                (sample_diff > LOST_LOCK_THRESHOLD ||
                 sample_diff < -LOST_LOCK_THRESHOLD ||
                fill < MIN_FILL_LEVEL))
#if UNLOCK_ON_LARGE_DIFF_CHANGE
                || (sample_diff > LOST_LOCK_THRESHOLD_LARGE || sample_diff < -LOST_LOCK_THRESHOLD_LARGE)
#endif
    )) {
#if DEBUG_MEDIA_CLOCK
        if (b.lock_count == LOCK_COUNT_THRESHOLD)
            debug_printf("Media output %d lost lock (threshold reached) (%d)\n", index, sample_diff);
#if UNLOCK_ON_LARGE_DIFF_CHANGE
        else if (sample_diff > LOST_LOCK_THRESHOLD_LARGE ||
                 sample_diff < -LOST_LOCK_THRESHOLD_LARGE)
            debug_printf("Media output %d lost lock (large change) (%d)\n", index, sample_diff);
#endif
        else
            debug_printf("Media output %d lost lock (discontinuity) (%d)\n", index, sample_diff);
#endif // DEBUG_MEDIA_CLOCK
        buf_ctl <: index;
        buf_ctl <: BUF_CTL_RESET;
        inct(buf_ctl);
        ptp_media_clock.info.unlock_counter++;
    } else {
        buf_ctl <: index;
        buf_ctl <: BUF_CTL_ACK;
        inct(buf_ctl);
    }

    b.prev_diff = sample_diff;
}

#endif // (AES67_NUM_MEDIA_OUTPUTS != 0)

#ifndef INITIAL_MEDIA_CLOCK_OUTPUT_DELAY
#define INITIAL_MEDIA_CLOCK_OUTPUT_DELAY 100000
#endif
#define EVENT_AFTER_PORT_OUTPUT_DELAY 100

#define INTERNAL_CLOCK_DIVIDE 25

static inline void update_media_clock_divide(aes67_media_clock_t &clk) {
    uint64_t divWordLength =
        (uint64_t)clk.wordLength * INTERNAL_CLOCK_DIVIDE / 2;

    clk.baseLength = divWordLength >> (WC_FRACTIONAL_BITS);
    clk.baseLengthRemainder = divWordLength & ((1 << WC_FRACTIONAL_BITS) - 1);
}

static void init_media_clock(aes67_media_clock_t &clk,
                             timer tmr,
                             out buffered port:32 p) {
    int ptime, time;

    clk.info.active = 0;
    clk.count = 0;
    clk.wordLength = 0x8235556;
    update_media_clock_divide(clk);
    clk.lowBits = 0;
    clk.bit = 0;

    p <: 0 @ ptime;
    tmr :> time;

    clk.wordTime = ptime + INITIAL_MEDIA_CLOCK_OUTPUT_DELAY;
    clk.next_event =
        time + INITIAL_MEDIA_CLOCK_OUTPUT_DELAY + EVENT_AFTER_PORT_OUTPUT_DELAY;
}

static void do_media_clock_output(aes67_media_clock_t &clk,
                                  out buffered port:32 p) {
    const unsigned int one = (1 << WC_FRACTIONAL_BITS);
    const unsigned mult = PLL_TO_WORD_MULTIPLIER / (2 * INTERNAL_CLOCK_DIVIDE);

    clk.count++;
    if (clk.count == mult) {
        clk.bit = ~clk.bit;
        clk.count = 0;
    }

    clk.wordTime += clk.baseLength;
    clk.next_event += clk.baseLength;

    clk.lowBits = clk.lowBits + clk.baseLengthRemainder;
    if (clk.lowBits >= one) {
        clk.wordTime += 1;
        clk.next_event += 1;
        clk.lowBits -= one;
    }

    p @ clk.wordTime <: clk.bit;
}

static void update_media_clocks(uint32_t clk_time,
                                const aes67_media_clock_pid_coefficients_t &pid_coefficients) {
    if (!ptp_media_clock.info.active)
      return;

    ptp_media_clock.wordLength =
        aes67_update_media_clock(ptp_media_clock, clk_time, pid_coefficients);

    update_media_clock_divide(ptp_media_clock);
}

void aes67_register_clock(uint32_t i) { registered[i] = 0; }

aes67_media_clock_info_t aes67_get_clock_info(void) {
    return ptp_media_clock.info;
}

static void aes67_set_clock_info(const aes67_media_clock_info_t info,
                                 int clk_time) {
    int prev_active = ptp_media_clock.info.active;
    ptp_media_clock.info = info;

    debug_printf("set_clock_info: prev_active %d -> active %d\n", prev_active,
                 info.active);

    if (!prev_active && info.active)
        aes67_init_media_clock_recovery(clk_time - CLOCK_RECOVERY_PERIOD,
                                        ptp_media_clock.info.rate);
}

void aes67_io_task(chanend buf_ctl[num_buf_ctl],
                   uint32_t num_buf_ctl,
                   out buffered port:32 p_fs,
                   REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t, pid_coefficients),
                   chanend media_control,
                   client interface ethernet_cfg_if i_eth_cfg,
                   client xtcp_if i_xtcp,
                   uint32_t flags) {
    timer tmr;
    uint32_t ptp_timeout;
    uint32_t clk_time;
#if (AES67_NUM_MEDIA_OUTPUTS != 0)
    uint8_t buf_ctl_cmd;
#endif
    timer clk_timer;
    int last_ptp_sync_lock = 0;
    enum ptp_server_type ptp_server_type;

    if (flags & AES67_FLAG_PTP_SLAVE_ONLY)
        ptp_server_type = PTP_SLAVE_ONLY;
    else
        ptp_server_type = PTP_GRANDMASTER_CAPABLE;

    ptp_server_init(i_eth_cfg, null, i_xtcp, ptp_server_type, tmr, ptp_timeout);

#if (AES67_NUM_MEDIA_OUTPUTS != 0)
    aes67_media_clock_init_buffers();
#endif

    for (int i = 0; i < MAX_CLK_CTL_CLIENTS; i++)
        registered[i] = -1;

    ptp_media_clock.info.active = 0;

    tmr :> clk_time;

    clk_time += CLOCK_RECOVERY_PERIOD;

    init_media_clock(ptp_media_clock, tmr, p_fs);

    while (1) {
#pragma ordered
        select {
            case clk_timer when timerafter(ptp_media_clock.next_event) :> int now:
#if PLL_OUTPUT_TIMING_CHECK
                if ((now - ptp_media_clock.next_event) >
                    ptp_media_clock.baseLength) {
                    static int count = 0;

                    if (++count == 3) {
                        debug_printf(
                            "ERROR: failed to drive PLL freq signal in time\n");
                        count = 0;
                    }
                }
#endif
                do_media_clock_output(ptp_media_clock, p_fs);
                break;

            case !isnull(i_xtcp) => i_xtcp.event_ready():
                xtcp_event_type_t event;
                int32_t id;

                event = i_xtcp.get_event(id);
                ptp_l3_handle_event(i_xtcp, event, id);
                break;

            case tmr when timerafter(ptp_timeout) :> void:
                if (last_ptp_sync_lock != sync_lock) {
                    aes67_media_clock_info_t info = ptp_media_clock.info;

                    info.active = sync_lock;
                    if (info.active) {
                        unsafe {
                            aes67_stream_info_t *unsafe s0 = aes67_get_receiver_stream(0);
                            uint32_t state = s0->state; // atomic read

                            if (state == AES67_STREAM_STATE_POTENTIAL ||
                                state == AES67_STREAM_STATE_ENABLED) {
                                COMPILER_BARRIER();
                                info.rate = s0->sample_rate;
                            } else {
                                info.rate = AES67_DEFAULT_SAMPLE_RATE;
                            }
                        }
                    }

                    aes67_set_clock_info(info, clk_time);
                    last_ptp_sync_lock = sync_lock;
                }

                if (timeafter(ptp_timeout, clk_time)) {
                    update_media_clocks(clk_time, pid_coefficients);
                    clk_time += CLOCK_RECOVERY_PERIOD;
                }

                ptp_periodic(null, i_xtcp, ptp_timeout);
                ptp_timeout += PTP_PERIODIC_TIME;
                break;

#if (AES67_NUM_MEDIA_OUTPUTS != 0)
            case (uint32_t i = 0; i < num_buf_ctl;
                  i++)(fifo_uninit_count == 0) => inuchar_byref(buf_ctl[i], buf_ctl_cmd):
                int fifo, buf_index;

                fifo = inuint(buf_ctl[i]);
                (void)inct(buf_ctl[i]);

                buf_index = aes67_media_clock_get_buf_info(fifo);
                assert(buf_index >= 0);

                switch (buf_ctl_cmd) {
                case BUF_CTL_NOTIFY_STREAM_INFO:
                    manage_buffer(buf_info[buf_index], buf_ctl[i], buf_index, tmr);
                    break;

                default:
                    break;
                }
                break;
#endif // AES67_NUM_MEDIA_OUTPUTS != 0

            case aes67_media_control(media_control, i_eth_cfg, i_xtcp, flags):
                break;
            }
    }
}
