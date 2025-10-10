// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025, PADL Software Pty Ltd, All rights reserved
#include <xs1.h>
#include <xclib.h>
#include <debug_print.h>
#include <xscope.h>
#include <assert.h>

#include "aes67_media_clock_client.h"
#include "aes67_media_clock_internal.h"
#include "aes67_utils.h"
#include "ptp.h"
#include "ptp_internal.h"
#include "aes67_rtp.h"

#ifndef DEBUG_MEDIA_CLOCK
#define DEBUG_MEDIA_CLOCK 2
#endif

#ifndef PLL_OUTPUT_TIMING_CHECK
#define PLL_OUTPUT_TIMING_CHECK 1
#endif

#define STABLE_THRESHOLD 32
#define LOCK_COUNT_THRESHOLD 1000
#define ACCEPTABLE_FILL_ADJUST 50000
#define LOST_LOCK_THRESHOLD 24
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

#if DEBUG_MEDIA_CLOCK
static int last_fill;
#endif

static void manage_buffer(buf_info_t &b,
                          chanend ?ptp_svr,
                          chanend buf_ctl,
                          int index,
                          timer tmr) {
    int fifo_locked;
    int diff, sample_diff;
    unsigned int wordLength;
    int rdptr, wrptr, fill;
    int thiscore_now, othercore_now;
    unsigned server_tile_id;

    wordLength = ptp_media_clock.wordLength;

    buf_ctl <: index;
    buf_ctl <: BUF_CTL_REQUEST_INFO;
    master {
        buf_ctl <: 0;
        buf_ctl :> othercore_now;
        tmr :> thiscore_now;
        buf_ctl :> fifo_locked;
        // Skip ptp_ts and local_ts - not sent by simplified buffer management
        buf_ctl :> rdptr;
        buf_ctl :> wrptr;
        buf_ctl :> server_tile_id;
    }

    fill = wrptr - rdptr;

    if (fill < 0)
        fill += AUDIO_OUTPUT_FIFO_WORD_SIZE;

#ifdef MEDIA_OUTPUT_FIFO_FILL
    xscope_int(MEDIA_OUTPUT_FIFO_FILL, fill);
#endif

    // Simplified approach: trust PTP-based media clock recovery
    // Instead of timestamp comparison, use buffer fill level directly
    diff = 0;  // Disable timestamp-based error calculation

#if DEBUG_MEDIA_CLOCK
    if (fifo_locked && index == 0) {
        int diff = last_fill - fill;
        if (diff > 1 || diff < -1)
            debug_printf("last fill changed: last %d fill %d\n", last_fill,
                         fill);
        last_fill = fill;
    }
#endif

    if (wordLength == 0) {
        // clock not locked yet
        buf_ctl <: index;
        buf_ctl <: BUF_CTL_ACK;
        inct(buf_ctl);
        debug_printf("clock not locked yet\n");
        return;
    }

    sample_diff = diff / ((int) ((wordLength * 10) >> WC_FRACTIONAL_BITS));

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

    if (!fifo_locked && (b.stability_count > STABLE_THRESHOLD)) {
        int max_adjust =
            AUDIO_OUTPUT_FIFO_WORD_SIZE - MAX_SAMPLES_PER_RTP_PACKET;

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
            debug_printf("Media output %d lost lock\n", index);
#if UNLOCK_ON_LARGE_DIFF_CHANGE
        else if (sample_diff > LOST_LOCK_THRESHOLD_LARGE ||
                 sample_diff < -LOST_LOCK_THRESHOLD_LARGE)
            debug_printf("Media output %d lost lock (large change)\n", index);
#endif
        else
            debug_printf("Media output %d lost lock (discontinuity)\n", index);
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

#define INITIAL_MEDIA_CLOCK_OUTPUT_DELAY 100000
#define EVENT_AFTER_PORT_OUTPUT_DELAY 100

#define INTERNAL_CLOCK_DIVIDE 25

static void update_media_clock_divide(aes67_media_clock_t &clk) {
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

static void update_media_clocks(chanend ?ptp_svr,
                                int clk_time,
                                const aes67_media_clock_pid_coefficients_t &pid_coefficients) {
    if (ptp_media_clock.info.active) {
        ptp_media_clock.wordLength =
            aes67_update_media_clock(ptp_svr, ptp_media_clock, clk_time,
                                     CLOCK_RECOVERY_PERIOD, pid_coefficients);

        update_media_clock_divide(ptp_media_clock);
    }
}

void aes67_register_clock(uint32_t i) { registered[i] = 0; }

aes67_media_clock_info_t aes67_get_clock_info(void) {
    return ptp_media_clock.info;
}

static void aes67_set_clock_info(const aes67_media_clock_info_t info,
                                 chanend ?ptp_svr,
                                 int clk_time) {
    int prev_active = ptp_media_clock.info.active;
    ptp_media_clock.info = info;

    debug_printf("set_clock_info: prev_active %d -> active %d\n", prev_active,
                 info.active);

    if (!prev_active && info.active)
        aes67_init_media_clock_recovery(ptp_svr,
                                        clk_time - CLOCK_RECOVERY_PERIOD,
                                        ptp_media_clock.info.rate);
}

void aes67_io_task(chanend ?ptp_svr,
                   chanend buf_ctl[num_buf_ctl],
                   uint32_t num_buf_ctl,
                   out buffered port:32 p_fs,
                   REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t, pid_coefficients),
                   chanend media_control,
                   client interface ethernet_cfg_if i_eth_cfg,
                   client xtcp_if i_xtcp,
                   chanend c_ptp[num_ptp],
                   uint32_t num_ptp,
                   enum ptp_server_type server_type
) {
    timer tmr;
    int ptp_timeout;
    unsigned int clk_time;
#if (AES67_NUM_MEDIA_OUTPUTS != 0)
    uint8_t buf_ctl_cmd;
#endif
    timer clk_timer;
    int last_ptp_sync_lock = 0;

    ptp_server_init(i_eth_cfg, null, i_xtcp, c_ptp[0], server_type, tmr,
                    ptp_timeout);

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

            case (uint32_t i = 0; i < num_ptp; i++)
                ptp_process_client_request(c_ptp[i], tmr):
                break;

            case tmr when timerafter(ptp_timeout) :> void:
                if (last_ptp_sync_lock != sync_lock) {
                    aes67_media_clock_info_t info = ptp_media_clock.info;

                    info.active = sync_lock;
                    if (info.active) {
                        aes67_stream_info_t s0 = get_receiver_stream(0);
                        info.rate = s0.sample_rate ? s0.sample_rate
                                                   : AES67_DEFAULT_SAMPLE_RATE;
                    }

                    aes67_set_clock_info(info, ptp_svr, clk_time);
                    last_ptp_sync_lock = sync_lock;
                }

                if (timeafter(ptp_timeout, clk_time)) {
                    update_media_clocks(ptp_svr, clk_time, pid_coefficients);
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
                    manage_buffer(buf_info[buf_index], ptp_svr, buf_ctl[i],
                                  buf_index, tmr);
                    break;

                default:
                    break;
                }
                break;
#endif // AES67_NUM_MEDIA_OUTPUTS != 0

            case aes67_media_control(media_control, i_xtcp):
                break;
            }
    }
}
