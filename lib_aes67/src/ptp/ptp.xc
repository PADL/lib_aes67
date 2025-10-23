// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025, PADL Software Pty Ltd, All rights reserved
/* This module implements the 802.1AS gPTP and IEEE 1588 timing protocols.
   It is a restricted version of the protocol that can only handle
   endpoints with one port. As such it is optimized (particularly for
   memory usage) and combined the code for the port state machines and the site
   state machines into one. */
#include <string.h>
#include <limits.h>
#include <xassert.h>
#include <xclib.h>
#include "ptp.h"
#include "ptp_internal.h"
#include "ptp_config.h"
#include "ptp_pdu.h"
#include "ethernet.h"
#include "misc_timer.h"
#include "print.h"
#include "debug_print.h"
#include "util/nettypes.h"

// #define PTP_DEBUG 1

#define timeafter(A, B) ((int)((B) - (A)) < 0)

/* The adjust between local clock ticks and ptp clock ticks.
   This is the ratio between our clock speed and the grandmaster less 1.
   For example, if we are running 1% faster than the master clock then
   this value will be 0.01 */
#define PTP_ADJUST_WEIGHT 32
static int g_ptp_adjust_valid = 0;
signed g_ptp_adjust = 0;
signed g_inv_ptp_adjust = 0;

static const xtcp_ipaddr_t any_addr;
static int32_t g_ptp_l3_event_tx = -1;
static int32_t g_ptp_l3_general_tx = -1;

int32_t g_ptp_l3_event_rx = -1;
int32_t g_ptp_l3_general_rx = -1;

/* The average path delay (over the last PDELAY_AVG_WINDOW pdelay_reqs)
   between the foreign master port and our slave port in nanoseconds (ptp time)
*/
#define PTP_PATH_DELAY_WEIGHT 32

ptp_port_info_t ptp_port_info[PTP_NUM_PORTS];
static uint16_t steps_removed_from_gm;

/* These variables make up the state of the local clock/port */
unsigned ptp_reference_local_ts;
ptp_timestamp ptp_reference_ptp_ts;
static int ptp_last_gm_freq_change = 0;
static int ptp_gm_timebase_ind = 0;
static n64_t my_port_id;
static n80_t master_port_id;
static u8_t ptp_priority1;
static u8_t ptp_priority2 = PTP_DEFAULT_PRIORITY2;

/* Timing variables */
static unsigned last_received_announce_time_valid[PTP_NUM_PORTS];
static unsigned last_received_announce_time[PTP_NUM_PORTS];
static unsigned last_received_sync_time[PTP_NUM_PORTS];
static unsigned last_receive_sync_upstream_interval[PTP_NUM_PORTS];
static unsigned last_announce_time[PTP_NUM_PORTS];
static unsigned last_sync_time[PTP_NUM_PORTS];
static unsigned last_pdelay_req_time[PTP_NUM_PORTS];

static ptp_timestamp prev_adjust_master_ts;
static unsigned prev_adjust_local_ts;
static int prev_adjust_valid = 0;

static unsigned received_sync = 0;
static u16_t received_sync_id;
static unsigned received_sync_ts;

int sync_lock = 0;
static int sync_count = 0;

static gPTPAnnounceMessage best_announce_msg;

static uint64_t pdelay_epoch_timer;
static unsigned prev_pdelay_local_ts;

static int tile_timer_offset;
static int periodic_counter[PTP_NUM_PORTS];

#define DEBUG_PRINT 0
#define DEBUG_PRINT_ANNOUNCE 0
#define DEBUG_PRINT_AS_CAPABLE 0
#define DEBUG_PRINT_PDELAY_CLAMP 0

// FIXME: update to take a port number (but this function doesn't appear to be
// used anywhere?)
ptp_port_role_t ptp_current_state() {
#if PTP_NUM_PORTS == 1
    return ptp_port_info[0].role_state;
#else
    return 0;
#endif
}

[[dual_issue]] unsigned
local_timestamp_to_ptp_mod32(unsigned local_ts, ptp_time_info_mod64 &info) {
    long long local_diff = (signed)local_ts - (signed)info.local_ts;

    local_diff *= 10;
    local_diff =
        local_diff + ((local_diff * info.ptp_adjust) >> PTP_ADJUST_PREC);

    return (info.ptp_ts_lo + (int)local_diff);
}

void local_timestamp_to_ptp_mod64(unsigned local_ts,
                                  ptp_time_info_mod64 *info,
                                  unsigned *hi,
                                  unsigned *lo) {
    long long local_diff = (signed)local_ts - (signed)info->local_ts;
    uint64_t ptp_mod64 = ((uint64_t)info->ptp_ts_hi << 32) + info->ptp_ts_lo;

    local_diff *= 10;
    local_diff =
        local_diff + ((local_diff * info->ptp_adjust) >> PTP_ADJUST_PREC);

    ptp_mod64 += local_diff;

    *hi = ptp_mod64 >> 32;
    *lo = (unsigned)ptp_mod64;
}

void ptp_get_reference_ptp_ts_mod_64(unsigned &hi, unsigned &lo) {
    uint64_t t;
    t = ptp_reference_ptp_ts.seconds[0] +
        ((uint64_t)ptp_reference_ptp_ts.seconds[1] << 32);
    t = t * NANOSECONDS_PER_SECOND;
    t += ptp_reference_ptp_ts.nanoseconds;
    hi = (unsigned)(t >> 32);
    lo = (unsigned)t;
}

static long long local_time_to_ptp_time(unsigned t, int l_ptp_adjust) {
    long long ret = ((long long)t) * 10;

    ret = ret + ((ret * l_ptp_adjust) >> PTP_ADJUST_PREC);
    return ret;
}

static void ptp_timestamp_offset64(ptp_timestamp &alias dst,
                                   ptp_timestamp &alias ts,
                                   long long offset) {
    uint64_t sec = ts.seconds[0] | ((uint64_t)ts.seconds[1] << 32);

    uint64_t nanosec = ts.nanoseconds;

    nanosec = nanosec + offset;
    sec = sec + nanosec / NANOSECONDS_PER_SECOND;
    nanosec = nanosec % NANOSECONDS_PER_SECOND;

    dst.seconds[1] = (unsigned)(sec >> 32);
    dst.seconds[0] = (unsigned)sec;
    dst.nanoseconds = nanosec;
}

void ptp_timestamp_offset(ptp_timestamp &ts, int offset) {
    ptp_timestamp_offset64(ts, ts, offset);
}

static long long ptp_timestamp_diff(ptp_timestamp &a, ptp_timestamp &b) {
    uint64_t sec_a = a.seconds[0] | ((uint64_t)a.seconds[1] << 32);
    uint64_t sec_b = b.seconds[0] | ((uint64_t)b.seconds[1] << 32);
    uint64_t nanosec_a = a.nanoseconds;
    uint64_t nanosec_b = b.nanoseconds;

    long long sec_diff = sec_a - sec_b;
    long long nanosec_diff = nanosec_a - nanosec_b;

    nanosec_diff += sec_diff * NANOSECONDS_PER_SECOND;

    return nanosec_diff;
}

unsigned ptp_timestamp_to_local(ptp_timestamp &ts, ptp_time_info &info) {
    long long ptp_diff;
    long long local_diff;
    ptp_diff = ptp_timestamp_diff(ts, info.ptp_ts);

    local_diff =
        ptp_diff + ((ptp_diff * info.inv_ptp_adjust) >> PTP_ADJUST_PREC);
    local_diff = local_diff / 10;

    return (info.local_ts + local_diff);
}

unsigned ptp_mod32_timestamp_to_local(unsigned ts, ptp_time_info_mod64 &info) {
    long long ptp_diff;
    long long local_diff;
    ptp_diff = (signed)ts - (signed)info.ptp_ts_lo;

    local_diff =
        ptp_diff + ((ptp_diff * info.inv_ptp_adjust) >> PTP_ADJUST_PREC);
    local_diff = local_diff / 10;

    return (info.local_ts + local_diff);
}

static void _local_timestamp_to_ptp(ptp_timestamp &ptp_ts,
                                    unsigned local_ts,
                                    unsigned reference_local_ts,
                                    ptp_timestamp &reference_ptp_ts,
                                    unsigned ptp_adjust) {
    unsigned local_diff = (signed)local_ts - (signed)reference_local_ts;

    uint64_t diff = local_time_to_ptp_time(local_diff, ptp_adjust);

    ptp_timestamp_offset64(ptp_ts, reference_ptp_ts, diff);
}

void local_timestamp_to_ptp(ptp_timestamp &ptp_ts,
                            unsigned local_ts,
                            ptp_time_info &info) {
    _local_timestamp_to_ptp(ptp_ts, local_ts, info.local_ts, info.ptp_ts,
                            info.ptp_adjust);
}

#define local_to_ptp_ts(ptp_ts, local_ts)                                      \
    _local_timestamp_to_ptp(ptp_ts, local_ts, ptp_reference_local_ts,          \
                            ptp_reference_ptp_ts, g_ptp_adjust)

static void create_my_announce_msg(gPTPAnnounceMessage *pAnnounceMesg);

// Unified delay mechanism support
typedef enum {
    DELAY_MECHANISM_PDELAY, // IEEE 802.1AS peer delay
    DELAY_MECHANISM_E2E     // IEEE 1588 end-to-end delay
} delay_mechanism_t;

static void update_delay(delay_mechanism_t mechanism,
                         int port_num,
                         ptp_timestamp t1,
                         ptp_timestamp t2,
                         ptp_timestamp t3,
                         ptp_timestamp t4);

static void set_new_role(enum ptp_port_role_t new_role, int port_num) {

    unsigned t = get_local_time();

    if (new_role == PTP_SLAVE) {
        debug_printf("PTP Port %d Role: Slave\n", port_num);

        ptp_port_info[port_num].delay_info.valid = 0;
        g_ptp_adjust = 0;
        g_inv_ptp_adjust = 0;
        prev_adjust_valid = 0;
        g_ptp_adjust_valid = 0;
        last_pdelay_req_time[port_num] = t;
        sync_lock = 0;
        sync_count = 0;
    }

    if (new_role == PTP_MASTER) {
        debug_printf("PTP Port %d Role: Master\n", port_num);

        // Now we are the master so no rate matching is needed, but record the
        // last rate for the follow up TLV Our internal precision is 2^30, we
        // need to scale to (2^41 * 1/g_ptp_adjust) per the standard
        ptp_last_gm_freq_change = g_inv_ptp_adjust << 11;
        ptp_gm_timebase_ind++;
        g_ptp_adjust = 0;
        g_inv_ptp_adjust = 0;

        ptp_reference_local_ts = ptp_reference_local_ts;

        last_sync_time[port_num] = last_announce_time[port_num] = t;
    }

    ptp_port_info[port_num].role_state = new_role;

    if ((new_role == PTP_MASTER || new_role == PTP_UNCERTAIN)
#if (PTP_NUM_PORTS == 2)
        && (ptp_port_info[!port_num].role_state == PTP_MASTER)
#endif
    ) {
        create_my_announce_msg(&best_announce_msg);
    }
}

/* Assume very conservatively that the worst case is that
   the sync messages a .5sec apart. That is 5*10^9ns which can
   be stored in 29 bits. So we have 35 fractional bits to calculate
   with */
#define ADJUST_CALC_PREC 35

static int update_adjust(ptp_timestamp &master_ts, unsigned local_ts) {

    if (prev_adjust_valid) {
        signed long long adjust, inv_adjust, master_diff, local_diff;

        /* Calculated the difference between two sync message on
           the master port and the local port */
        master_diff = ptp_timestamp_diff(master_ts, prev_adjust_master_ts);
        local_diff = (signed)local_ts - (signed)prev_adjust_local_ts;

        /* The local timestamps are based on 100Mhz. So
           convert to nanoseconds */
        local_diff *= 10;

        /* Work at the new adjust value in 64 bits */
        adjust = master_diff - local_diff;
        inv_adjust = local_diff - master_diff;

        // Detect and ignore outliers
#if PTP_THROW_AWAY_SYNC_OUTLIERS
        if (master_diff > 150000000 || master_diff < 100000000) {
            prev_adjust_valid = 0;
            debug_printf("PTP threw away Sync outlier (master_diff %d)\n",
                         master_diff);
            return 1;
        }
#endif

        adjust <<= ADJUST_CALC_PREC;
        inv_adjust <<= ADJUST_CALC_PREC;

        if (master_diff == 0 || local_diff == 0) {
            prev_adjust_valid = 0;
            return 1;
        }

        adjust = adjust / master_diff;
        inv_adjust = inv_adjust / local_diff;

        /* Reduce it down to PTP_ADJUST_PREC */
        adjust >>= (ADJUST_CALC_PREC - PTP_ADJUST_PREC);
        inv_adjust >>= (ADJUST_CALC_PREC - PTP_ADJUST_PREC);

        /* Re-average the adjust with a given weighting.
           This method loses a few bits of precision */
        if (g_ptp_adjust_valid) {

            long long diff = adjust - (long long)g_ptp_adjust;

            if (diff < 0)
                diff = -diff;

            if (!sync_lock) {
                if (diff < PTP_SYNC_LOCK_ACCEPTABLE_VARIATION) {
                    sync_count++;
                    if (sync_count > PTP_SYNC_LOCK_STABILITY_COUNT) {
                        debug_printf("PTP sync locked\n");
                        sync_lock = 1;
                        sync_count = 0;
                    }
                } else
                    sync_count = 0;
            } else {
                if (diff > PTP_SYNC_LOCK_ACCEPTABLE_VARIATION) {
                    sync_count++;
                    if (sync_count > PTP_SYNC_LOCK_STABILITY_COUNT) {
                        debug_printf("PTP sync lock lost\n");
                        sync_lock = 0;
                        sync_count = 0;
                        prev_adjust_valid = 0;
                        return 1;
                    }
                } else
                    sync_count = 0;
            }

            adjust =
                (((long long)g_ptp_adjust) * (PTP_ADJUST_WEIGHT - 1) + adjust) /
                PTP_ADJUST_WEIGHT;

            g_ptp_adjust = (int)adjust;

            inv_adjust =
                (((long long)g_inv_ptp_adjust) * (PTP_ADJUST_WEIGHT - 1) +
                 inv_adjust) /
                PTP_ADJUST_WEIGHT;

            g_inv_ptp_adjust = (int)inv_adjust;
        } else {
            g_ptp_adjust = (int)adjust;
            g_inv_ptp_adjust = (int)inv_adjust;
            g_ptp_adjust_valid = 1;
        }
    }

    prev_adjust_local_ts = local_ts;
    prev_adjust_master_ts = master_ts;
    prev_adjust_valid = 1;

    return 0;
}

static void update_reference_timestamps(ptp_timestamp &master_egress_ts,
                                        unsigned local_ingress_ts,
                                        ptp_port_info_t &port_info) {
    ptp_timestamp master_ingress_ts;

    ptp_timestamp_offset64(master_ingress_ts, master_egress_ts,
                           port_info.delay_info.pdelay);

    /* Update the reference timestamps */
    ptp_reference_local_ts = local_ingress_ts;
    ptp_reference_ptp_ts = master_ingress_ts;
}

#define UPDATE_REFERENCE_TIMESTAMP_PERIOD (500000000) // 5 sec

static void periodic_update_reference_timestamps(unsigned int local_ts) {

    int local_diff = local_ts - ptp_reference_local_ts;

    if (local_diff > UPDATE_REFERENCE_TIMESTAMP_PERIOD) {
        long long ptp_diff = local_time_to_ptp_time(local_diff, g_ptp_adjust);

        ptp_reference_local_ts = local_ts;
        ptp_timestamp_offset64(ptp_reference_ptp_ts, ptp_reference_ptp_ts,
                               ptp_diff);
    }
}

// Unified delay calculation function for both PDelay and E2E mechanisms
static void update_delay(delay_mechanism_t mechanism,
                         int port_num,
                         ptp_timestamp t1,
                         ptp_timestamp t2,
                         ptp_timestamp t3,
                         ptp_timestamp t4) {
    long long delay;
    long long round_trip;

    /* Delay calculation: delay = ((t4-t1) - (t3-t2)) / 2
       For PDelay: t1=req_sent, t2=req_received, t3=resp_sent, t4=resp_received
       For E2E: t1=req_sent, t2=req_received, t3=resp_sent, t4=resp_received
       Same formula works for both mechanisms
    */

    // Both mechanisms use the same formula: delay = ((t4-t1) - (t3-t2)) / 2
    // All timestamps are now PTP timestamps for consistency
    long long t4_t1_diff = ptp_timestamp_diff(t4, t1);
    long long t3_t2_diff = ptp_timestamp_diff(t3, t2);
    round_trip = t4_t1_diff - t3_t2_diff;

    delay = round_trip / 2;

    if (delay < 0 || delay > MAX_REASONABLE_DELAY_NS) {
#if DEBUG_PRINT_PDELAY_CLAMP
        debug_printf("Clamp invalid delay %lld ns\n", delay);
#endif
        delay = 0;
    }

    // Update appropriate delay tracking based on mechanism
    if (mechanism == DELAY_MECHANISM_PDELAY && port_num >= 0 &&
        port_num < PTP_NUM_PORTS) {
        // Apply weighted average for PDelay (802.1AS requirement)
        if (ptp_port_info[port_num].delay_info.valid) {
            ptp_port_info[port_num].delay_info.pdelay =
                ((ptp_port_info[port_num].delay_info.pdelay *
                  (PTP_PATH_DELAY_WEIGHT - 1)) +
                 (int)delay) /
                PTP_PATH_DELAY_WEIGHT;
        } else {
            ptp_port_info[port_num].delay_info.pdelay = delay;
            ptp_port_info[port_num].delay_info.valid = 1;
        }
        ptp_port_info[port_num].delay_info.exchanges++;
    } else if (mechanism == DELAY_MECHANISM_E2E) {
        // For E2E, store in port 0's delay info for now (can be expanded later)
        if (ptp_port_info[0].delay_info.valid) {
            ptp_port_info[0].delay_info.pdelay =
                ((ptp_port_info[0].delay_info.pdelay *
                  (PTP_PATH_DELAY_WEIGHT - 1)) +
                 (int)delay) /
                PTP_PATH_DELAY_WEIGHT;
        } else {
            ptp_port_info[0].delay_info.pdelay = delay;
            ptp_port_info[0].delay_info.valid = 1;
        }
        ptp_port_info[0].delay_info.exchanges++;
    }
}

/* Returns:
      -1 - if clock is worse than me
      1  - if clock is better than me
      0  - if clocks are equal
*/
static int compare_clock_identity_to_me(const n64_t *clockIdentity) {
    for (int i = 0; i < 8; i++) {
        if (clockIdentity->data[i] > my_port_id.data[i]) {
            return -1;
        } else if (clockIdentity->data[i] < my_port_id.data[i]) {
            return 1;
        }
    }

    // Thje two clock identities are the same
    return 0;
}

static int compare_clock_identity(const n64_t *c1, const n64_t *c2) {
    for (int i = 0; i < 8; i++) {
        if (c1->data[i] > c2->data[i]) {
            return -1;
        } else if (c1->data[i] < c2->data[i]) {
            return 1;
        }
    }

    // The two clock identities are the same
    return 0;
}

static void
bmca_update_roles(const uint8_t *msg, size_t len, unsigned t, int port_num) {
    const ComMessageHdr *pComMesgHdr = (const ComMessageHdr *)msg;
    const AnnounceMessage *pAnnounceMesg =
        (const AnnounceMessage *)((const uint8_t *)pComMesgHdr +
                                  sizeof(ComMessageHdr));
    const AnnounceMessage *pBestAnnounceMsg =
        &best_announce_msg.announceMessage;
    int clock_identity_comp;
    int new_best = 0;

    clock_identity_comp =
        compare_clock_identity_to_me(&pAnnounceMesg->grandmasterIdentity);

    if (clock_identity_comp == 0) {
        /* If the message is about me then we win since our stepsRemoved is 0 */
    } else {
        /* Message is from a different clock. Let's work out if it is better or
           worse according to the BMCA */
        if (pAnnounceMesg->grandmasterPriority1 >
            pBestAnnounceMsg->grandmasterPriority1) {
            new_best = -1;
        } else if (pAnnounceMesg->grandmasterPriority1 <
                   pBestAnnounceMsg->grandmasterPriority1) {
            new_best = 1;
        } else if (pAnnounceMesg->clockClass > pBestAnnounceMsg->clockClass) {
            new_best = -1;
        } else if (pAnnounceMesg->clockClass < pBestAnnounceMsg->clockClass) {
            new_best = 1;
        } else if (pAnnounceMesg->clockAccuracy >
                   pBestAnnounceMsg->clockAccuracy) {
            new_best = -1;
        } else if (pAnnounceMesg->clockAccuracy <
                   pBestAnnounceMsg->clockAccuracy) {
            new_best = 1;
        } else if (ntoh16(pAnnounceMesg->clockOffsetScaledLogVariance) >
                   ntoh16(pBestAnnounceMsg->clockOffsetScaledLogVariance)) {
            new_best = -1;
        } else if (ntoh16(pAnnounceMesg->clockOffsetScaledLogVariance) <
                   ntoh16(pBestAnnounceMsg->clockOffsetScaledLogVariance)) {
            new_best = 1;
        } else if (pAnnounceMesg->grandmasterPriority2 >
                   pBestAnnounceMsg->grandmasterPriority2) {
            new_best = -1;
        } else if (pAnnounceMesg->grandmasterPriority2 <
                   pBestAnnounceMsg->grandmasterPriority2) {
            new_best = 1;
        } else {
            clock_identity_comp =
                compare_clock_identity(&pAnnounceMesg->grandmasterIdentity,
                                       &pBestAnnounceMsg->grandmasterIdentity);

            if (clock_identity_comp <= 0) {
                //
            } else {
                new_best = 1;
            }
        }
    }

    if (new_best > 0) {
        memset(&best_announce_msg, 0, sizeof(best_announce_msg));

        if (len > sizeof(best_announce_msg))
            len = sizeof(best_announce_msg);
        memcpy(&best_announce_msg, pAnnounceMesg, len);

        master_port_id = pComMesgHdr->sourcePortIdentity;

        {
#if DEBUG_PRINT_ANNOUNCE
            debug_printf("NEW BEST: %d\n", port_num);
#endif
            set_new_role(PTP_SLAVE, port_num);
            if (PTP_NUM_PORTS == 2) {
                set_new_role(PTP_MASTER, !port_num);
            }
            last_received_announce_time_valid[port_num] = 0;
            master_port_id = pComMesgHdr->sourcePortIdentity;
        }
    } else if (new_best < 0 &&
               ptp_port_info[port_num].role_state == PTP_SLAVE) {
        set_new_role(PTP_MASTER, port_num);
        last_received_announce_time_valid[port_num] = 0;
    }
}

static void timestamp_to_network(n80_t &msg, ptp_timestamp &ts) {
    char *sec0_p = (char *)&ts.seconds[0];
    char *sec1_p = (char *)&ts.seconds[1];
    char *nsec_p = (char *)&ts.nanoseconds;

    // Convert seconds to big-endian
    msg.data[0] = sec1_p[3];
    msg.data[1] = sec1_p[2];

    for (int i = 2; i < 6; i++)
        msg.data[i] = sec0_p[5 - i];

    // Now convert nanoseconds
    for (int i = 6; i < 10; i++)
        msg.data[i] = nsec_p[9 - i];
}

/*
extern void timestamp_to_network(n80_t &msg,
                                 ptp_timestamp &ts);
*/

static void network_to_ptp_timestamp(ptp_timestamp &ts, const n80_t &msg) {
    char *sec0_p = (char *)&ts.seconds[0];
    char *sec1_p = (char *)&ts.seconds[1];
    char *nsec_p = (char *)&ts.nanoseconds;

    sec1_p[3] = msg.data[0];
    sec1_p[2] = msg.data[1];
    sec1_p[1] = 0;
    sec1_p[0] = 0;

    for (int i = 2; i < 6; i++)
        sec0_p[5 - i] = msg.data[i];

    for (int i = 6; i < 10; i++)
        nsec_p[9 - i] = msg.data[i];
}

static int port_identity_equal(n64_t &a, n64_t &b) {
    for (int i = 0; i < 8; i++)
        if (a.data[i] != b.data[i])
            return 0;
    return 1;
}

static int source_port_identity_equal(n80_t &a, n80_t &b) {
    for (int i = 0; i < 10; i++)
        if (a.data[i] != b.data[i])
            return 0;
    return 1;
}

static int clock_id_equal(n64_t *a, n64_t *b) {
    for (int i = 0; i < 8; i++)
        if (a->data[i] != b->data[i])
            return 0;
    return 1;
}

static void ptp_tx_l2(client interface ethernet_tx_if i_eth,
                      unsigned int *buf,
                      int len,
                      int port_num) {
    len = len < 64 ? 64 : len;
    i_eth.send_packet((char *)buf, len, port_num);
}

static void ptp_tx_l3(client interface xtcp_if i_xtcp, uint8_t *buf, int len) {
    i_xtcp.send(g_ptp_l3_general_tx, (char *)buf, len);
}

static void ptp_tx_timed_l2(client interface ethernet_tx_if i_eth,
                            unsigned int buf[],
                            int len,
                            unsigned &ts,
                            int port_num) {
    len = len < 64 ? 64 : len;
    ts = i_eth.send_timed_packet((char *)buf, len, port_num);
    ts = ts - tile_timer_offset;
}

static void ptp_tx_timed_l3(client interface xtcp_if i_xtcp,
                            uint8_t *buf,
                            int len,
                            unsigned &ts) {
    uint32_t timestamp;

    i_xtcp.send_timed(g_ptp_l3_event_tx, (char *)buf, len, timestamp);
    ts = (unsigned)timestamp - tile_timer_offset;
}

static const xtcp_ipaddr_t ptp_mcast_group = PTP_1588_DEST_ADDR;

static uint8_t null_mac_addr[MACADDR_NUM_BYTES];
static uint8_t src_mac_addr[MACADDR_NUM_BYTES];
static uint8_t dest_mac_addr[MACADDR_NUM_BYTES] = PTP_DEFAULT_DEST_ADDR;

static void set_ptp_ethernet_hdr(uint8_t *buf) {
    ethernet_hdr_t *hdr = (ethernet_hdr_t *)buf;

    for (int i = 0; i < MACADDR_NUM_BYTES; i++) {
        hdr->src_addr[i] = src_mac_addr[i];
        hdr->dest_addr[i] = dest_mac_addr[i];
    }

    hdr->ethertype.data[0] = (PTP_ETHERTYPE >> 8);
    hdr->ethertype.data[1] = (PTP_ETHERTYPE & 0xff);
}

// Estimate of announce message processing time delay.
#define MESSAGE_PROCESS_TIME (3563)

static u16_t announce_seq_id[PTP_NUM_PORTS];

static void create_my_announce_msg(gPTPAnnounceMessage *pAnnounceMesg) {
    // setup the Announce message
    pAnnounceMesg->announceMessage.currentUtcOffset =
        hton16(PTP_CURRENT_UTC_OFFSET);
    pAnnounceMesg->announceMessage.grandmasterPriority1 = ptp_priority1;

    // grandMaster clock quality.
    pAnnounceMesg->announceMessage.clockClass = PTP_CLOCK_CLASS;
    pAnnounceMesg->announceMessage.clockAccuracy = PTP_CLOCK_ACCURACY;

    pAnnounceMesg->announceMessage.clockOffsetScaledLogVariance =
        hton16(PTP_OFFSET_SCALED_LOG_VARIANCE);

    // grandMaster priority
    pAnnounceMesg->announceMessage.grandmasterPriority2 = ptp_priority2;

    for (int i = 0; i < 8; i++)
        pAnnounceMesg->announceMessage.grandmasterIdentity.data[i] =
            my_port_id.data[i];

    pAnnounceMesg->announceMessage.stepsRemoved = hton16(0);
    pAnnounceMesg->announceMessage.timeSource = PTP_TIMESOURCE;

    pAnnounceMesg->tlvType = hton16(PTP_ANNOUNCE_TLV_TYPE);
    pAnnounceMesg->tlvLength = hton16(8);
    for (int i = 0; i < 8; i++)
        pAnnounceMesg->pathSequence[0].data[i] = my_port_id.data[i];
}

static void send_ptp_announce_msg(client interface ethernet_tx_if ?i_eth,
                                  client interface xtcp_if ?i_xtcp,
                                  int port_num) {
#define L3_ANNOUNCE_PACKET_SIZE                                                \
    (sizeof(ComMessageHdr) + sizeof(AnnounceMessage))
#define L2_ANNOUNCE_PACKET_SIZE                                                \
    (sizeof(ethernet_hdr_t) + L3_ANNOUNCE_PACKET_SIZE)

#define L2_GPTP_ANNOUNCE_PACKET_SIZE                                           \
    (sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr) +                          \
     sizeof(gPTPAnnounceMessage))

    unsigned int buf0[(L2_GPTP_ANNOUNCE_PACKET_SIZE + 3) / 4];
    uint8_t *buf = (uint8_t *)&buf0[0];
    ComMessageHdr *pComMesgHdr = (ComMessageHdr *)&buf[sizeof(ethernet_hdr_t)];
    AnnounceMessage *pAnnounceMesg =
        (AnnounceMessage *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];

    set_ptp_ethernet_hdr(buf);

    size_t message_length = L3_ANNOUNCE_PACKET_SIZE;

    // setup the common message header.
    memset(pComMesgHdr, 0, message_length);

    pComMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_ANNOUNCE_MESG;
    pComMesgHdr->versionPTP = PTP_VERSION_NUMBER;
    pComMesgHdr->flagField[1] =
        ((PTP_LEAP61 & 0x1)) | ((PTP_LEAP59 & 0x1) << 1) |
        ((PTP_CURRENT_UTC_OFFSET_VALID & 0x1) << 2) |
        ((PTP_TIMESCALE & 0x1) << 3) | ((PTP_TIME_TRACEABLE & 0x1) << 4) |
        ((PTP_FREQUENCY_TRACEABLE & 0x1) << 5);

    // portId assignment
    for (int i = 0; i < 8; i++)
        pComMesgHdr->sourcePortIdentity.data[i] = my_port_id.data[i];
    pComMesgHdr->sourcePortIdentity.data[9] = port_num + 1;

    // sequence id.
    announce_seq_id[port_num] += 1;
    pComMesgHdr->sequenceId = hton16(announce_seq_id[port_num]);
    pComMesgHdr->controlField = PTP_CTL_FIELD_OTHERS;
    pComMesgHdr->logMessageInterval = PTP_LOG_ANNOUNCE_INTERVAL;

    // create_my_announce_msg(pAnnounceMesg);
    // setup the Announce message
    pAnnounceMesg->currentUtcOffset = hton16(PTP_CURRENT_UTC_OFFSET);
    pAnnounceMesg->grandmasterPriority1 =
        best_announce_msg.announceMessage.grandmasterPriority1;

    // grandMaster clock quality.
    pAnnounceMesg->clockClass = best_announce_msg.announceMessage.clockClass;
    pAnnounceMesg->clockAccuracy =
        best_announce_msg.announceMessage.clockAccuracy;
    pAnnounceMesg->clockOffsetScaledLogVariance =
        best_announce_msg.announceMessage.clockOffsetScaledLogVariance;

    // grandMaster priority
    pAnnounceMesg->grandmasterPriority2 =
        best_announce_msg.announceMessage.grandmasterPriority2;

    steps_removed_from_gm =
        ntoh16(best_announce_msg.announceMessage.stepsRemoved);

    for (int i = 0; i < 8; i++)
        pAnnounceMesg->grandmasterIdentity.data[i] =
            best_announce_msg.announceMessage.grandmasterIdentity.data[i];

#if (PTP_NUM_PORTS == 2)
    if ((ptp_port_info[0].role_state == PTP_MASTER) ^
        (ptp_port_info[1].role_state == PTP_MASTER)) {
        // Only increment steps removed if we are not the grandmaster
        steps_removed_from_gm++;
    }
#endif

    pAnnounceMesg->stepsRemoved = hton16(steps_removed_from_gm);
    pAnnounceMesg->timeSource = PTP_TIMESOURCE;

    if (!isnull(i_eth)) {
        // Validate steps_removed_from_gm to prevent overflow
        if (steps_removed_from_gm >= PTP_MAXIMUM_PATH_TRACE_TLV) {
            debug_printf(
                "steps_removed_from_gm (%d) exceeds maximum, capping to %d\n",
                steps_removed_from_gm, PTP_MAXIMUM_PATH_TRACE_TLV - 1);
            steps_removed_from_gm = PTP_MAXIMUM_PATH_TRACE_TLV - 1;
        }

        gPTPAnnounceMessage *pgPtpAnnounceMessage =
            (gPTPAnnounceMessage *)pAnnounceMesg;

        pgPtpAnnounceMessage->tlvType = hton16(PTP_ANNOUNCE_TLV_TYPE);
        // Safe calculation: steps_removed_from_gm is now bounded
        pgPtpAnnounceMessage->tlvLength =
            hton16((steps_removed_from_gm + 1) * 8);
        memcpy(pgPtpAnnounceMessage->pathSequence,
               best_announce_msg.pathSequence, steps_removed_from_gm * 8);

        for (int i = 0; i < 8; i++)
            pgPtpAnnounceMessage->pathSequence[steps_removed_from_gm].data[i] =
                my_port_id.data[i];

        message_length += 4 + (steps_removed_from_gm + 1) * 8;
    }

    pComMesgHdr->messageLength = hton16(message_length);

    // send the message.
    if (!isnull(i_eth)) {
        ptp_tx_l2(i_eth, buf0, sizeof(ethernet_hdr_t) + message_length,
                  port_num);
    } else if (!isnull(i_xtcp)) {
        pComMesgHdr->transportSpecific_messageType &=
            ~(PTP_TRANSPORT_SPECIFIC_HDR);
        ptp_tx_l3(i_xtcp, (uint8_t *)pComMesgHdr, message_length);
    }

#if DEBUG_PRINT_ANNOUNCE
    debug_printf("TX Announce, Port %d\n", port_num);
#endif

    return;
}

static u16_t sync_seq_id = 0;

static void send_ptp_sync_msg(client interface ethernet_tx_if ?i_eth,
                              client interface xtcp_if ?i_xtcp,
                              int port_num) {
#define L3_SYNC_PACKET_SIZE (sizeof(ComMessageHdr) + sizeof(SyncMessage))
#define L2_SYNC_PACKET_SIZE (sizeof(ethernet_hdr_t) + L3_SYNC_PACKET_SIZE)

#define L3_FOLLOWUP_PACKET_SIZE                                                \
    (sizeof(ComMessageHdr) + sizeof(FollowUpMessage))
#define L2_FOLLOWUP_PACKET_SIZE                                                \
    (sizeof(ethernet_hdr_t) + L3_FOLLOWUP_PACKET_SIZE)

#define L2_GPTP_FOLLOWUP_PACKET_SIZE                                           \
    (sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr) +                          \
     sizeof(gPTPFollowUpMessage))

    unsigned int buf0[(L2_GPTP_FOLLOWUP_PACKET_SIZE + 3) / 4];
    uint8_t *buf = (uint8_t *)&buf0[0];
    ComMessageHdr *pComMesgHdr = (ComMessageHdr *)&buf[sizeof(ethernet_hdr_t)];
    SyncMessage *pSyncMesg =
        (SyncMessage *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];
    FollowUpMessage *pFollowUpMesg =
        (FollowUpMessage *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];
    unsigned local_egress_ts_l2 = 0;
    unsigned local_egress_ts_l3 = 0;
    ptp_timestamp ptp_egress_ts_l2;
    ptp_timestamp ptp_egress_ts_l3;

    set_ptp_ethernet_hdr(buf);

    memset(pComMesgHdr, 0, L3_FOLLOWUP_PACKET_SIZE);

    // 1. Send Sync message.
    pComMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_SYNC_MESG;
    pComMesgHdr->versionPTP = PTP_VERSION_NUMBER;
    pComMesgHdr->messageLength = hton16(L3_SYNC_PACKET_SIZE);
    pComMesgHdr->flagField[0] = 0x2; // set two steps flag
    pComMesgHdr->flagField[1] = (PTP_TIMESCALE & 0x1) << 3;

    if (!isnull(i_xtcp)) {
        // does originTimestamp need to be filled in as something for L3? not
        // sure
        local_to_ptp_ts(ptp_egress_ts_l2, 0);
        timestamp_to_network(pSyncMesg->originTimestamp, ptp_egress_ts_l2);
    }

    for (int i = 0; i < 8; i++)
        pComMesgHdr->correctionField.data[i] = 0;

    for (int i = 0; i < 8; i++)
        pComMesgHdr->sourcePortIdentity.data[i] = my_port_id.data[i];
    pComMesgHdr->sourcePortIdentity.data[9] = port_num + 1;

    sync_seq_id += 1;

    pComMesgHdr->sequenceId = hton16(sync_seq_id);
    pComMesgHdr->controlField = PTP_CTL_FIELD_SYNC;
    pComMesgHdr->logMessageInterval = PTP_LOG_SYNC_INTERVAL;

    // transmit the packet and record the egress time.
    if (!isnull(i_eth))
        ptp_tx_timed_l2(i_eth, buf0, L2_SYNC_PACKET_SIZE, local_egress_ts_l2,
                        port_num);
    else if (!isnull(i_xtcp))
        ptp_tx_timed_l3(i_xtcp, (uint8_t *)pComMesgHdr, L3_SYNC_PACKET_SIZE,
                        local_egress_ts_l3);

#if DEBUG_PRINT
    debug_printf("TX sync, Port %d\n", port_num);
#endif

    // Send Follow_Up message
    pComMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_FOLLOW_UP_MESG;
    pComMesgHdr->controlField = PTP_CTL_FIELD_FOLLOW_UP;
    pComMesgHdr->flagField[0] = 0; // clear two steps flag for follow up

    // populate the time in packet
    local_to_ptp_ts(ptp_egress_ts_l2, local_egress_ts_l2);
    timestamp_to_network(pFollowUpMesg->preciseOriginTimestamp,
                         ptp_egress_ts_l2);

    for (int i = 0; i < 8; i++)
        pComMesgHdr->correctionField.data[i] = 0;

    if (!isnull(i_eth)) {
        gPTPFollowUpMessage *pgPtpFollowUpMessage =
            (gPTPFollowUpMessage *)pFollowUpMesg;

        // Fill in follow up fields as per 802.1as section 11.4.4.2
        pgPtpFollowUpMessage->tlvType = hton16(0x3);
        pgPtpFollowUpMessage->lengthField = hton16(28);
        pgPtpFollowUpMessage->organizationId[0] = 0x00;
        pgPtpFollowUpMessage->organizationId[1] = 0x80;
        pgPtpFollowUpMessage->organizationId[2] = 0xc2;
        pgPtpFollowUpMessage->organizationSubType[0] = 0;
        pgPtpFollowUpMessage->organizationSubType[1] = 0;
        pgPtpFollowUpMessage->organizationSubType[2] = 1;

        pgPtpFollowUpMessage->scaledLastGmFreqChange =
            hton32(ptp_last_gm_freq_change);
        pgPtpFollowUpMessage->gmTimeBaseIndicator = hton16(ptp_gm_timebase_ind);

        pComMesgHdr->messageLength =
            hton16(sizeof(ComMessageHdr) + sizeof(gPTPFollowUpMessage));
        ptp_tx_l2(i_eth, buf0, L2_GPTP_FOLLOWUP_PACKET_SIZE, port_num);
    } else if (!isnull(i_xtcp)) {
        // ensure we use the correct L3 timestamp
        local_to_ptp_ts(ptp_egress_ts_l3, local_egress_ts_l3);
        timestamp_to_network(pFollowUpMesg->preciseOriginTimestamp,
                             ptp_egress_ts_l3);

        pComMesgHdr->transportSpecific_messageType &=
            ~(PTP_TRANSPORT_SPECIFIC_HDR);
        pComMesgHdr->flagField[0] =
            0x2; // reset two steps flag (TODO: necessary?)

        // note: we use L3_SYNC_PACKET_SIZE to avoid sending the Follow_Up TLVs.
        pComMesgHdr->messageLength = hton16(L3_FOLLOWUP_PACKET_SIZE);
        ptp_tx_l3(i_xtcp, (uint8_t *)pComMesgHdr, L3_FOLLOWUP_PACKET_SIZE);
    }

#if DEBUG_PRINT
    debug_printf("TX sync follow up, Port %d\n", port_num);
#endif

    return;
}

static u16_t pdelay_req_seq_id[PTP_NUM_PORTS];
static unsigned pdelay_request_sent[PTP_NUM_PORTS];
static unsigned pdelay_request_sent_ts[PTP_NUM_PORTS];

// E2E delay tracking (for IEEE 1588 end-to-end delay mechanism)
static u16_t e2e_delay_req_seq_id = 0;
static unsigned e2e_delay_request_sent = 0;
static unsigned e2e_delay_request_sent_ts = 0;
static unsigned e2e_last_delay_req_time = 0;

// DelayResp residence time tracking for improved correction field accuracy
static long long avg_residence_time_ns = 10000; // Initial estimate: 10µs
static int residence_time_samples = 0;
#define RESIDENCE_TIME_WEIGHT 8 // Weight for exponential moving average

static void send_ptp_pdelay_req_msg(client interface ethernet_tx_if i_eth,
                                    int port_num) {
#define L3_PDELAY_REQ_PACKET_SIZE                                              \
    (sizeof(ComMessageHdr) + sizeof(PdelayReqMessage))
#define L2_PDELAY_REQ_PACKET_SIZE                                              \
    (sizeof(ethernet_hdr_t) + L3_PDELAY_REQ_PACKET_SIZE)
    unsigned int buf0[(L2_PDELAY_REQ_PACKET_SIZE + 3) / 4];
    uint8_t *buf = (uint8_t *)&buf0[0];
    ComMessageHdr *pComMesgHdr = (ComMessageHdr *)&buf[sizeof(ethernet_hdr_t)];

    set_ptp_ethernet_hdr(buf);

    int message_length = L3_PDELAY_REQ_PACKET_SIZE;

    // clear the send data first.
    memset(pComMesgHdr, 0, message_length);

    // build up the packet as required.
    pComMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_PDELAY_REQ_MESG;
    pComMesgHdr->versionPTP = PTP_VERSION_NUMBER;
    pComMesgHdr->messageLength = hton16(message_length);
    pComMesgHdr->flagField[1] = (PTP_TIMESCALE & 0x1) << 3;

    // correction field, & flagField are zero.
    for (int i = 0; i < 8; i++)
        pComMesgHdr->correctionField.data[i] = 0;
    for (int i = 0; i < 8; i++)
        pComMesgHdr->sourcePortIdentity.data[i] = my_port_id.data[i];
    pComMesgHdr->sourcePortIdentity.data[9] = port_num + 1;

    // increment the sequence id.
    pdelay_req_seq_id[port_num] += 1;
    pComMesgHdr->sequenceId = hton16(pdelay_req_seq_id[port_num]);

    // control field for backward compatiability
    pComMesgHdr->controlField = PTP_CTL_FIELD_OTHERS;
    pComMesgHdr->logMessageInterval = PTP_LOG_MIN_PDELAY_REQ_INTERVAL;

    // sent out the data and record the time.
    if (!isnull(i_eth))
        ptp_tx_timed_l2(i_eth, buf0, L2_PDELAY_REQ_PACKET_SIZE,
                        pdelay_request_sent_ts[port_num], port_num);

    // TODO: if we support L3, clear PTP_TRANSPORT_SPECIFIC_HDR
    pdelay_request_sent[port_num] = 1;

#if DEBUG_PRINT
    debug_printf("TX Pdelay req, Port %d\n", port_num);
#endif

    return;
}

void local_to_epoch_ts(unsigned local_ts, ptp_timestamp *epoch_ts) {
    uint64_t sec;
    uint64_t nanosec;

    if (local_ts <= prev_pdelay_local_ts) // We overflowed 32 bits
    {
        pdelay_epoch_timer += ((UINT_MAX - prev_pdelay_local_ts) + local_ts);
    } else {
        pdelay_epoch_timer += (local_ts - prev_pdelay_local_ts);
    }

    nanosec = pdelay_epoch_timer * 10;

    sec = nanosec / NANOSECONDS_PER_SECOND;
    nanosec = nanosec % NANOSECONDS_PER_SECOND;

    epoch_ts->seconds[1] = (unsigned)(sec >> 32);
    epoch_ts->seconds[0] = (unsigned)sec;
    epoch_ts->nanoseconds = nanosec;

    prev_pdelay_local_ts = local_ts;
}

// TODO: implement PDelay for IEEE 1588

static void send_ptp_pdelay_resp_msg(client interface ethernet_tx_if i_eth,
                                     char *pdelay_req_msg,
                                     unsigned req_ingress_ts,
                                     int port_num) {
#define L3_PDELAY_RESP_PACKET_SIZE                                             \
    (sizeof(ComMessageHdr) + sizeof(PdelayRespMessage))
#define L2_PDELAY_RESP_PACKET_SIZE                                             \
    (sizeof(ethernet_hdr_t) + L3_PDELAY_RESP_PACKET_SIZE)
    unsigned int buf0[(L2_PDELAY_RESP_PACKET_SIZE + 3) / 4];
    uint8_t *buf = (uint8_t *)&buf0[0];
    // received packet pointers.
    ComMessageHdr *pRxMesgHdr = (ComMessageHdr *)pdelay_req_msg;
    // transmit packet pointers.
    ComMessageHdr *pTxMesgHdr = (ComMessageHdr *)&buf[sizeof(ethernet_hdr_t)];
    PdelayRespMessage *pTxRespHdr =
        (PdelayRespMessage
             *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];
    PdelayRespFollowUpMessage *pTxFollowUpHdr =
        (PdelayRespFollowUpMessage
             *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];

    ptp_timestamp epoch_req_ingress_ts;
    ptp_timestamp epoch_resp_ts;
    unsigned local_resp_ts;

    set_ptp_ethernet_hdr(buf);

    memset(pTxMesgHdr, 0, L3_PDELAY_RESP_PACKET_SIZE);

    pTxMesgHdr->versionPTP = PTP_VERSION_NUMBER;
    pTxMesgHdr->messageLength =
        hton16(sizeof(ComMessageHdr) + sizeof(PdelayRespMessage));
    pTxMesgHdr->flagField[0] = 0x2; // set two steps flag
    pTxMesgHdr->flagField[1] = (PTP_TIMESCALE & 0x1) << 3;

    for (int i = 0; i < 8; i++)
        pTxMesgHdr->sourcePortIdentity.data[i] = my_port_id.data[i];
    pTxMesgHdr->sourcePortIdentity.data[9] = port_num + 1;

    pTxMesgHdr->controlField = PTP_CTL_FIELD_OTHERS;
    pTxMesgHdr->logMessageInterval = 0x7F;

    pTxMesgHdr->sequenceId = pRxMesgHdr->sequenceId;

    memcpy(&pTxRespHdr->requestingPortIdentity, &pRxMesgHdr->sourcePortIdentity,
           sizeof(pTxRespHdr->requestingPortIdentity));
    pTxRespHdr->requestingPortId.data[0] =
        pRxMesgHdr->sourcePortIdentity.data[8];
    pTxRespHdr->requestingPortId.data[1] =
        pRxMesgHdr->sourcePortIdentity.data[9];

    pTxMesgHdr->domainNumber = pRxMesgHdr->domainNumber;

    pTxMesgHdr->correctionField = pRxMesgHdr->correctionField;

    /* Send the response message */
    pTxMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_PDELAY_RESP_MESG;

    local_to_epoch_ts(req_ingress_ts, &epoch_req_ingress_ts);
    timestamp_to_network(pTxRespHdr->requestReceiptTimestamp,
                         epoch_req_ingress_ts);

    if (!isnull(i_eth))
        ptp_tx_timed_l2(i_eth, buf0, L2_PDELAY_RESP_PACKET_SIZE, local_resp_ts,
                        port_num);
#if DEBUG_PRINT
    debug_printf("TX Pdelay resp, Port %d\n", port_num);
#endif

    /* Now send the follow up */
    local_to_epoch_ts(local_resp_ts, &epoch_resp_ts);

    pTxMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_PDELAY_RESP_FOLLOW_UP_MESG;

    pTxMesgHdr->flagField[0] = 0; // clear two steps flag

    timestamp_to_network(pTxFollowUpHdr->responseOriginTimestamp,
                         epoch_resp_ts);

    if (!isnull(i_eth))
        ptp_tx_l2(i_eth, buf0, L2_PDELAY_RESP_PACKET_SIZE, port_num);
        // TODO: if we support L3, clear PTP_TRANSPORT_SPECIFIC_HDR

#if DEBUG_PRINT
    debug_printf("TX Pdelay resp follow up, Port %d\n", port_num);
#endif

    return;
}

static void send_delay_req_msg(client interface ethernet_tx_if ?i_eth,
                               client interface xtcp_if ?i_xtcp,
                               int port_num) {
#define L3_DELAY_REQ_PACKET_SIZE                                               \
    (sizeof(ComMessageHdr) + sizeof(DelayReqMessage))
#define L2_DELAY_REQ_PACKET_SIZE                                               \
    (sizeof(ethernet_hdr_t) + L3_DELAY_REQ_PACKET_SIZE)
    unsigned int buf0[(L2_DELAY_REQ_PACKET_SIZE + 3) / 4];
    uint8_t *buf = (uint8_t *)&buf0[0];
    ComMessageHdr *pComMesgHdr = (ComMessageHdr *)&buf[sizeof(ethernet_hdr_t)];
    DelayReqMessage *pDelayReqMsg =
        (DelayReqMessage *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];

    set_ptp_ethernet_hdr(buf);

    int message_length = L3_DELAY_REQ_PACKET_SIZE;

    // clear the send data first
    memset(pComMesgHdr, 0, message_length);

    // build up the packet as required
    pComMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_DELAY_REQ_MESG;
    pComMesgHdr->versionPTP = PTP_VERSION_NUMBER;
    pComMesgHdr->messageLength = hton16(message_length);
    pComMesgHdr->flagField[1] = (PTP_TIMESCALE & 0x1) << 3;

    // correction field & flagField are zero
    for (int i = 0; i < 8; i++)
        pComMesgHdr->correctionField.data[i] = 0;

    for (int i = 0; i < 8; i++)
        pComMesgHdr->sourcePortIdentity.data[i] = my_port_id.data[i];
    pComMesgHdr->sourcePortIdentity.data[9] = port_num + 1;

    // increment the sequence id
    e2e_delay_req_seq_id += 1;
    pComMesgHdr->sequenceId = hton16(e2e_delay_req_seq_id);

    // control field for Delay_Req
    pComMesgHdr->controlField = PTP_CTL_FIELD_DELAY_REQ;
    pComMesgHdr->logMessageInterval =
        PTP_LOG_MIN_PDELAY_REQ_INTERVAL; // reuse same interval

    // Set origin timestamp to 0 (will be filled by hardware timestamping or
    // follow-up)
    for (int i = 0; i < 10; i++)
        pDelayReqMsg->originTimestamp.data[i] = 0;

    // send the message and record timestamp
    if (!isnull(i_eth)) {
        ptp_tx_timed_l2(i_eth, buf0, L2_DELAY_REQ_PACKET_SIZE,
                        e2e_delay_request_sent_ts, port_num);
    } else if (!isnull(i_xtcp)) {
        pComMesgHdr->transportSpecific_messageType &=
            ~(PTP_TRANSPORT_SPECIFIC_HDR);
        ptp_tx_timed_l3(i_xtcp, (uint8_t *)pComMesgHdr,
                        L3_DELAY_REQ_PACKET_SIZE, e2e_delay_request_sent_ts);
    }

    e2e_delay_request_sent = 1;

#if DEBUG_PRINT
    debug_printf("TX Delay Req\n");
#endif

    return;
}

static void send_delay_resp_msg(client interface ethernet_tx_if ?i_eth,
                                client interface xtcp_if ?i_xtcp,
                                char *delay_req_msg,
                                unsigned req_ingress_ts,
                                int port_num) {
#define L3_DELAY_RESP_PACKET_SIZE                                              \
    (sizeof(ComMessageHdr) + sizeof(DelayRespMessage))
#define L2_DELAY_RESP_PACKET_SIZE                                              \
    (sizeof(ethernet_hdr_t) + L3_DELAY_RESP_PACKET_SIZE)

    unsigned int buf0[(L2_DELAY_RESP_PACKET_SIZE + 3) / 4];
    uint8_t *buf = (uint8_t *)&buf0[0];
    // received packet pointers
    ComMessageHdr *pRxMesgHdr = (ComMessageHdr *)delay_req_msg;
    // transmit packet pointers
    ComMessageHdr *pTxMesgHdr = (ComMessageHdr *)&buf[sizeof(ethernet_hdr_t)];
    DelayRespMessage *pTxRespHdr =
        (DelayRespMessage
             *)&buf[sizeof(ethernet_hdr_t) + sizeof(ComMessageHdr)];

    ptp_timestamp receive_timestamp;
    unsigned resp_egress_ts = 0;

    set_ptp_ethernet_hdr(buf);
    memset(pTxMesgHdr, 0, L3_DELAY_RESP_PACKET_SIZE);

    // Build DelayResp message with precise timestamping
    pTxMesgHdr->transportSpecific_messageType =
        PTP_TRANSPORT_SPECIFIC_HDR | PTP_DELAY_RESP_MESG;
    pTxMesgHdr->versionPTP = PTP_VERSION_NUMBER;
    pTxMesgHdr->messageLength = hton16(L3_DELAY_RESP_PACKET_SIZE);
    pTxMesgHdr->flagField[1] = (PTP_TIMESCALE & 0x1) << 3;

    for (int i = 0; i < 8; i++)
        pTxMesgHdr->sourcePortIdentity.data[i] = my_port_id.data[i];
    pTxMesgHdr->sourcePortIdentity.data[9] = port_num + 1;

    pTxMesgHdr->controlField = PTP_CTL_FIELD_DELAY_RESP;
    pTxMesgHdr->logMessageInterval = 0x7F; // one-time message
    pTxMesgHdr->domainNumber = pRxMesgHdr->domainNumber;

    // Copy sequence ID and port identity from request
    pTxMesgHdr->sequenceId = pRxMesgHdr->sequenceId;
    memcpy(&pTxRespHdr->requestingPortIdentity, &pRxMesgHdr->sourcePortIdentity,
           sizeof(pTxRespHdr->requestingPortIdentity));
    pTxRespHdr->requestingPortId.data[0] =
        pRxMesgHdr->sourcePortIdentity.data[8];
    pTxRespHdr->requestingPortId.data[1] =
        pRxMesgHdr->sourcePortIdentity.data[9];

    // Set receive timestamp (t2 in E2E delay calculation)
    local_to_ptp_ts(receive_timestamp, req_ingress_ts);
    timestamp_to_network(pTxRespHdr->receiveTimestamp, receive_timestamp);

    // Use learned average residence time for correction field
    long long estimated_residence_ns = avg_residence_time_ns;
    uint64_t correction =
        (estimated_residence_ns << 16) / NANOSECONDS_PER_SECOND;

    // Set correction field using hton64()
    pTxMesgHdr->correctionField = hton64(correction);

    // Send DelayResp message with timed transmission to capture precise t3
    if (!isnull(i_eth)) {
        ptp_tx_timed_l2(i_eth, buf0, L2_DELAY_RESP_PACKET_SIZE, resp_egress_ts,
                        port_num);
    } else if (!isnull(i_xtcp)) {
        pTxMesgHdr->transportSpecific_messageType &=
            ~(PTP_TRANSPORT_SPECIFIC_HDR);
        ptp_tx_timed_l3(i_xtcp, (uint8_t *)pTxMesgHdr,
                        L3_DELAY_RESP_PACKET_SIZE, resp_egress_ts);
    }

    // Calculate actual residence time and update learned average
    long long actual_residence_ns =
        ((long long)resp_egress_ts - (long long)req_ingress_ts) * 10;

    // Update exponential moving average of residence time
    if (residence_time_samples < RESIDENCE_TIME_WEIGHT) {
        // Simple average for initial samples
        avg_residence_time_ns =
            ((avg_residence_time_ns * residence_time_samples) +
             actual_residence_ns) /
            (residence_time_samples + 1);
        residence_time_samples++;
    } else {
        // Exponential moving average: new_avg = old_avg + (new_sample -
        // old_avg) / weight
        avg_residence_time_ns = avg_residence_time_ns +
                                ((actual_residence_ns - avg_residence_time_ns) /
                                 RESIDENCE_TIME_WEIGHT);
    }

#if DEBUG_PRINT
    debug_printf("TX Delay Resp, t2=%u, t3=%u (residence: est=%lld, "
                 "actual=%lld, avg=%lld ns)\n",
                 req_ingress_ts, resp_egress_ts, estimated_residence_ns,
                 actual_residence_ns, avg_residence_time_ns);
#endif

    return;
}

static unsigned received_pdelay[PTP_NUM_PORTS];
static u16_t received_pdelay_id[PTP_NUM_PORTS];
static unsigned pdelay_resp_ingress_ts[PTP_NUM_PORTS];
static ptp_timestamp pdelay_request_receipt_ts[PTP_NUM_PORTS];

static int qualify_announce(const ComMessageHdr *alias header, int this_port) {
    const AnnounceMessage *announce_msg = (const AnnounceMessage *)(header + 1);
    uint16_t message_length = ntoh16(header->messageLength);

    for (int i = 0; i < 8; i++) {
        if (header->sourcePortIdentity.data[i] != my_port_id.data[i])
            break;
        if (i == 7)
            return 0;
    }

    if (ntoh16(announce_msg->stepsRemoved) >= 255)
        return 0;

    if (message_length == L3_ANNOUNCE_PACKET_SIZE) { // no TLVs
        return 1;
    }

    // Enhanced TLV bounds checking
    size_t min_tlv_size =
        L3_ANNOUNCE_PACKET_SIZE + sizeof(u16_t) + sizeof(u16_t);
    if (message_length < min_tlv_size) {
        debug_printf("message too short for TLV header: %d < %d\n",
                     message_length, min_tlv_size);
        return 0;
    }

    // Additional safety: ensure message_length is reasonable for TLV processing
    if (message_length >
        L3_ANNOUNCE_PACKET_SIZE + 4 + (PTP_MAXIMUM_PATH_TRACE_TLV * 8)) {
        debug_printf(
            "message length %d exceeds maximum expected announce size\n",
            message_length);
        return 0;
    }

    const gPTPAnnounceMessage *gptp_announce_msg =
        (const gPTPAnnounceMessage *)announce_msg;

    uint16_t tlv_type = ntoh16(gptp_announce_msg->tlvType);
    if (tlv_type != PTP_ANNOUNCE_TLV_TYPE) {
        debug_printf("unknown TLV type %d\n", tlv_type);
        return 0;
    }

    uint16_t tlv_length = ntoh16(gptp_announce_msg->tlvLength);
    size_t tlv_end_offset = sizeof(ComMessageHdr) + sizeof(AnnounceMessage) +
                            sizeof(u16_t) + sizeof(u16_t) + tlv_length;
    if (tlv_end_offset > message_length) {
        debug_printf("TLV offset %d past message length!\n", tlv_end_offset);
        return 0;
    }

    int tlv = tlv_length / 8;
    if (tlv) {
        if (tlv > PTP_MAXIMUM_PATH_TRACE_TLV) {
            tlv = PTP_MAXIMUM_PATH_TRACE_TLV;
        }
        for (int i = 0; i < tlv; i++) {
            if (!compare_clock_identity_to_me(
                    &gptp_announce_msg->pathSequence[i]))
                return 0;
        }
    }

    return 1;
}

static void set_ascapable(int eth_port) {
    if (!ptp_port_info[eth_port].asCapable) {
        ptp_port_info[eth_port].asCapable = 1;
        set_new_role(PTP_MASTER, eth_port);
#if DEBUG_PRINT_AS_CAPABLE
        debug_printf("asCapable = 1\n");
#endif
    }
}

static void reset_ascapable(int eth_port) {
    if (ptp_port_info[eth_port].asCapable) {
        ptp_port_info[eth_port].asCapable = 0;
        ptp_port_info[eth_port].delay_info.exchanges = 0;
        ptp_port_info[eth_port].delay_info.pdelay = 0;
        ptp_port_info[eth_port].delay_info.valid = 0;
        set_new_role(PTP_MASTER, eth_port);
#if DEBUG_PRINT_AS_CAPABLE
        debug_printf("asCapable = 0\n");
#endif
    }
}

static void pdelay_req_reset(int src_port) {
    if (ptp_port_info[src_port].delay_info.lost_responses <
        PTP_ALLOWED_LOST_RESPONSES) {
        ptp_port_info[src_port].delay_info.lost_responses++;
#if DEBUG_PRINT_AS_CAPABLE
        debug_printf("Lost responses: %d\n",
                     ptp_port_info[src_port].delay_info.lost_responses);
#endif
    } else {
        reset_ascapable(src_port);
    }
}

void ptp_recv(client interface ethernet_tx_if ?i_eth,
              client interface xtcp_if ?i_xtcp,
              uint8_t buf[len],
              unsigned local_ingress_ts,
              unsigned src_port,
              unsigned len) {

    /* Extract the ethernet header and ptp common message header */
    int is_l3 = isnull(i_eth) && !isnull(i_xtcp);
    ComMessageHdr *msg;

    if (is_l3) {
        msg = (ComMessageHdr *)buf;
    } else {
        struct ethernet_hdr_t *ethernet_hdr = (ethernet_hdr_t *)&buf[0];
        int has_qtag = ethernet_hdr->ethertype.data[1] == 0x18;
        int ethernet_pkt_size = has_qtag ? 18 : 14;

        if (len < ethernet_pkt_size)
            return;

        len -= ethernet_pkt_size;
        msg = (ComMessageHdr *)&buf[ethernet_pkt_size];
    }

    if (len < sizeof(ComMessageHdr)) {
        debug_printf("PTP message of %d bytes too small\n", len);
        return;
    }

    if (msg->versionPTP != PTP_VERSION_NUMBER)
        return; // ignore Dante PTPv1 packets

    uint16_t claimed_length = ntoh16(msg->messageLength);
    if (len < claimed_length)
        return;

    // Validate maximum messageLength to prevent oversized packets
    if (claimed_length > MAX_PTP_MESG_LENGTH) {
        debug_printf("PTP message length %d exceeds maximum %d\n",
                     claimed_length, MAX_PTP_MESG_LENGTH);
        return;
    }

    local_ingress_ts = local_ingress_ts - tile_timer_offset;

    if (GET_PTP_TRANSPORT_SPECIFIC(msg) != !is_l3)
        return;

    int asCapable = GET_PTP_TRANSPORT_SPECIFIC(msg) == 1 &&
                    ptp_port_info[src_port].asCapable;

    switch ((msg->transportSpecific_messageType & 0xf)) {
    case PTP_ANNOUNCE_MESG:
        if (len < L3_ANNOUNCE_PACKET_SIZE)
            return;

        if ((is_l3 || asCapable) && qualify_announce(msg, src_port)) {
#if DEBUG_PRINT_ANNOUNCE
            debug_printf("RX Announce, Port %d\n", src_port);
#endif
            bmca_update_roles((const uint8_t *)msg, len, local_ingress_ts,
                              src_port);

            AnnounceMessage *announce_msg = (AnnounceMessage *)(msg + 1);

            if (ptp_port_info[src_port].role_state == PTP_SLAVE &&
                source_port_identity_equal(msg->sourcePortIdentity,
                                           master_port_id) &&
                clock_id_equal(
                    &best_announce_msg.announceMessage.grandmasterIdentity,
                    &announce_msg->grandmasterIdentity)) {
                last_received_announce_time_valid[src_port] = 1;
                last_received_announce_time[src_port] = local_ingress_ts;
            }
        }
        break;
    case PTP_SYNC_MESG:
        if (len < L3_SYNC_PACKET_SIZE)
            return;

        if ((is_l3 || asCapable) && !received_sync &&
            ptp_port_info[src_port].role_state == PTP_SLAVE) {
            received_sync = 1;
            received_sync_id = ntoh16(msg->sequenceId);
            received_sync_ts = local_ingress_ts;
            last_received_sync_time[src_port] = local_ingress_ts;
            last_receive_sync_upstream_interval[src_port] =
                LOG_SEC_TO_TIMER_TICKS((signed char)(msg->logMessageInterval));
#if DEBUG_PRINT
            debug_printf("RX Sync, Port %d\n", src_port);
#endif
        }
        break;
    case PTP_FOLLOW_UP_MESG:
        if (len < L3_SYNC_PACKET_SIZE) // this is correct, don't expect TLVs as
                                       // we don't parse them
            return;

        if ((received_sync == 1) &&
            source_port_identity_equal(msg->sourcePortIdentity,
                                       master_port_id)) {

            if (received_sync_id == ntoh16(msg->sequenceId)) {
                const FollowUpMessage *follow_up_msg =
                    (const FollowUpMessage *)(msg + 1);
                int64_t correction = ntoh64(msg->correctionField);
                ptp_timestamp master_egress_ts;

                network_to_ptp_timestamp(master_egress_ts,
                                         follow_up_msg->preciseOriginTimestamp);
                ptp_timestamp_offset64(master_egress_ts, master_egress_ts,
                                       correction >> 16);

                if (update_adjust(master_egress_ts, received_sync_ts) == 0) {
                    update_reference_timestamps(master_egress_ts,
                                                received_sync_ts,
                                                ptp_port_info[src_port]);
                }
#if DEBUG_PRINT
                debug_printf("RX Follow Up, Port %d\n", src_port);
#endif
                received_sync = 0;
            }
        } else {
            received_sync = 0;
        }
        break;
    case PTP_PDELAY_REQ_MESG:
#if DEBUG_PRINT
        debug_printf("RX Pdelay req, Port %d\n", src_port);
#endif
        if (!isnull(i_eth))
            send_ptp_pdelay_resp_msg(i_eth, (char *)msg, local_ingress_ts,
                                     src_port);
        break;
    case PTP_PDELAY_RESP_MESG:
        if (len < L3_PDELAY_RESP_PACKET_SIZE)
            return;

        PdelayRespMessage *resp_msg = (PdelayRespMessage *)(msg + 1);

        if (!pdelay_request_sent[src_port] && received_pdelay[src_port] &&
            !source_port_identity_equal(
                msg->sourcePortIdentity,
                ptp_port_info[src_port].delay_info.rcvd_source_identity) &&
            pdelay_req_seq_id[src_port] == ntoh16(msg->sequenceId)) {

            if (!ptp_port_info[src_port].delay_info.multiple_resp_count ||
                (pdelay_req_seq_id[src_port] ==
                 ptp_port_info[src_port].delay_info.last_multiple_resp_seq_id +
                     1)) {
                // Count consecutive multiple pdelay responses for a single
                // pdelay request
                ptp_port_info[src_port].delay_info.multiple_resp_count++;
            } else {
                ptp_port_info[src_port].delay_info.multiple_resp_count = 0;
            }
            ptp_port_info[src_port].delay_info.last_multiple_resp_seq_id =
                pdelay_req_seq_id[src_port];
            pdelay_req_reset(src_port);
            break;
        }

        if (received_pdelay[src_port] &&
            pdelay_req_seq_id[src_port] == ntoh16(msg->sequenceId)) {
            // Count a lost follow up message
            received_pdelay[src_port] = 0;
            pdelay_req_reset(src_port);
        }

        if (pdelay_request_sent[src_port] &&
            pdelay_req_seq_id[src_port] == ntoh16(msg->sequenceId) &&
            port_identity_equal(resp_msg->requestingPortIdentity, my_port_id) &&
            src_port + 1 == ntoh16(resp_msg->requestingPortId)) {
            received_pdelay[src_port] = 1;
            received_pdelay_id[src_port] = ntoh16(msg->sequenceId);
            pdelay_resp_ingress_ts[src_port] = local_ingress_ts;
            network_to_ptp_timestamp(pdelay_request_receipt_ts[src_port],
                                     resp_msg->requestReceiptTimestamp);
#if DEBUG_PRINT
            debug_printf("RX Pdelay resp, Port %d\n", src_port);
#endif
            ptp_port_info[src_port].delay_info.rcvd_source_identity =
                msg->sourcePortIdentity;
        } else {
            pdelay_req_reset(src_port);
        }
        pdelay_request_sent[src_port] = 0;

        break;
    case PTP_PDELAY_RESP_FOLLOW_UP_MESG:
        if (received_pdelay[src_port]) {
            if (received_pdelay_id[src_port] == ntoh16(msg->sequenceId) &&
                source_port_identity_equal(
                    msg->sourcePortIdentity,
                    ptp_port_info[src_port].delay_info.rcvd_source_identity)) {
                ptp_timestamp pdelay_resp_egress_ts;
                PdelayRespFollowUpMessage *follow_up_msg =
                    (PdelayRespFollowUpMessage *)(msg + 1);

                network_to_ptp_timestamp(
                    pdelay_resp_egress_ts,
                    follow_up_msg->responseOriginTimestamp);

                // Convert local timestamps to PTP timestamps for unified delay
                // calculation
                ptp_timestamp t1, t4;
                local_to_ptp_ts(t1, pdelay_request_sent_ts[src_port]);
                local_to_ptp_ts(t4, pdelay_resp_ingress_ts[src_port]);

                update_delay(
                    DELAY_MECHANISM_PDELAY, src_port,
                    t1,                                  // t1: request sent
                    pdelay_request_receipt_ts[src_port], // t2: request received
                    pdelay_resp_egress_ts,               // t3: response sent
                    t4); // t4: response received

#if DEBUG_PRINT_AS_CAPABLE
                debug_printf("Average pdelay of %d ns\n",
                             ptp_port_info[src_port].delay_info.pdelay);
#endif

                if (ptp_port_info[src_port].delay_info.valid &&
                    ptp_port_info[src_port].delay_info.pdelay <=
                        PTP_NEIGHBOR_PROP_DELAY_THRESH_NS &&
                    ptp_port_info[src_port].delay_info.exchanges >= 2) {
                    set_ascapable(src_port);
                } else {
                    reset_ascapable(src_port);
                }
                ptp_port_info[src_port].delay_info.lost_responses = 0;
#if DEBUG_PRINT
                debug_printf("RX Pdelay resp follow up, Port %d\n", src_port);
#endif
            } else {
                pdelay_req_reset(src_port);
            }
        }
        received_pdelay[src_port] = 0;
        break;
    case PTP_DELAY_REQ_MESG:
        if (len < L3_DELAY_REQ_PACKET_SIZE)
            return;

        // E2E Delay Request - respond as master
        if (ptp_port_info[src_port].role_state == PTP_MASTER) {
#if DEBUG_PRINT
            debug_printf("RX Delay Req, Port %d\n", src_port);
#endif
            send_delay_resp_msg(i_eth, i_xtcp, (char *)msg, local_ingress_ts,
                                src_port);
        }
        break;
    case PTP_DELAY_RESP_MESG:
        if (len < L3_DELAY_RESP_PACKET_SIZE)
            return;

        // E2E Delay Response - process as slave
        if (e2e_delay_request_sent &&
            ptp_port_info[src_port].role_state == PTP_SLAVE) {
            DelayRespMessage *delay_resp_msg = (DelayRespMessage *)(msg + 1);

            // Verify this response matches our request
            if (ntoh16(msg->sequenceId) == e2e_delay_req_seq_id &&
                port_identity_equal(delay_resp_msg->requestingPortIdentity,
                                    my_port_id)) {

                ptp_timestamp t1, t2, t3, t4;

                // t1: Delay_Req sent time (stored when we sent the request)
                local_to_ptp_ts(t1, e2e_delay_request_sent_ts);

                // t2: Delay_Req received time (from DelayResp message)
                network_to_ptp_timestamp(t2, delay_resp_msg->receiveTimestamp);

                // t3: Delay_Resp sent time = t2 + correction field (per IEEE
                // 1588)
                t3 = t2;

                // Extract correction field using ntoh64() (in units of 2^-16
                // seconds)
                uint64_t correction = ntoh64(msg->correctionField);
                // Convert correction to nanoseconds: correction * 10^9 / 2^16
                long long correction_ns =
                    (correction * NANOSECONDS_PER_SECOND) >> 16;
                ptp_timestamp_offset(t3, correction_ns);

                // t4: Delay_Resp received time
                local_to_ptp_ts(t4, local_ingress_ts);

                // Calculate and update E2E delay using unified function
                update_delay(DELAY_MECHANISM_E2E, 0, t1, t2, t3, t4);

#if DEBUG_PRINT
                debug_printf("RX Delay Resp, E2E delay calculated %d\n",
                             ptp_port_info[0].delay_info.pdelay);
#endif
            }
            e2e_delay_request_sent = 0;
        }
        break;
    }
}

void ptp_reset(int port_num) {
    set_new_role(PTP_MASTER, port_num);
    last_received_announce_time_valid[port_num] = 0;
    ptp_port_info[port_num].delay_info.multiple_resp_count = 0;
    ptp_port_info[port_num].delay_info.pdelay = 0;
    ptp_port_info[port_num].delay_info.lost_responses = 0;
    periodic_counter[port_num] = 0;
    reset_ascapable(port_num);
}

static inline unsigned int get_tile_id_from_chanend(chanend c) {
    unsigned int ci;
    asm("shr %0, %1, 16" : "=r"(ci) : "r"(c));
    return ci;
}

void ptp_init(client interface ethernet_cfg_if i_eth_cfg,
              client interface ethernet_rx_if ?i_eth_rx,
              client interface xtcp_if ?i_xtcp,
              enum ptp_server_type stype,
              chanend c) {
    unsigned server_tile_id;
    unsigned other_tile_now;
    unsigned this_tile_now;

    i_eth_cfg.get_tile_id_and_timer_value(server_tile_id, other_tile_now);
    this_tile_now = get_local_time();

    if (server_tile_id != get_tile_id_from_chanend(c)) {
        tile_timer_offset =
            other_tile_now - this_tile_now -
            3; // 3 is an estimate of the channel + instruction latency
    } else {
        tile_timer_offset = 0;
    }

    if (stype == PTP_GRANDMASTER_CAPABLE) {
        ptp_priority1 = PTP_DEFAULT_GM_CAPABLE_PRIORITY1;
    } else {
        ptp_priority1 = PTP_DEFAULT_NON_GM_CAPABLE_PRIORITY1;
    }

    // await other task to set MAC address, so we don't advertise a NULL EUI64
    for (;;) {
        i_eth_cfg.get_macaddr(0, src_mac_addr);

        if (memcmp(src_mac_addr, null_mac_addr, MACADDR_NUM_BYTES) != 0)
            break;

        delay_milliseconds(1);
    }

    memcpy(&my_port_id.data[0], &src_mac_addr[0], 3);
    my_port_id.data[3] = 0xff;
    my_port_id.data[4] = 0xfe;
    memcpy(&my_port_id.data[5], &src_mac_addr[3], 3);

    debug_printf("Local %s port ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                 !isnull(i_eth_rx) ? "gPTP" : "PTPv2", my_port_id.data[0],
                 my_port_id.data[1], my_port_id.data[2], my_port_id.data[3],
                 my_port_id.data[4], my_port_id.data[5], my_port_id.data[6],
                 my_port_id.data[7]);

    if (!isnull(i_eth_rx)) {
        size_t eth_index = i_eth_rx.get_index();
        ethernet_macaddr_filter_t gptp_filter;
        gptp_filter.appdata = 0;
        memcpy(gptp_filter.addr, dest_mac_addr, 6);
        i_eth_cfg.add_macaddr_filter(eth_index, 0, gptp_filter);
        i_eth_cfg.add_ethertype_filter(eth_index, PTP_ETHERTYPE);
    } else if (!isnull(i_xtcp)) {
        xtcp_error_code_t err;

        i_xtcp.join_multicast_group(ptp_mcast_group);

        g_ptp_l3_event_rx = i_xtcp.socket(XTCP_PROTOCOL_UDP);
        err = i_xtcp.listen(g_ptp_l3_event_rx, PTP_1588_EVENT_PORT, any_addr);
        assert(err == XTCP_SUCCESS);

        g_ptp_l3_event_tx = i_xtcp.socket(XTCP_PROTOCOL_UDP);
        err = i_xtcp.connect(g_ptp_l3_event_tx, PTP_1588_EVENT_PORT,
                             ptp_mcast_group);
        assert(err == XTCP_SUCCESS);

        // Set DSCP for PTP event messages
        uint32_t dscp = DSCP_PTP_EVENT << 2;
        err = i_xtcp.setsockopt(g_ptp_l3_event_tx, XTCP_SOCKET_LEVEL_IP,
                                XTCP_IP_SOCKET_OPTION_TOS,
                                (const uint8_t*)&dscp, sizeof(dscp));
        assert(err == XTCP_SUCCESS);

        g_ptp_l3_general_rx = i_xtcp.socket(XTCP_PROTOCOL_UDP);
        err =
            i_xtcp.listen(g_ptp_l3_general_rx, PTP_1588_GENERAL_PORT, any_addr);
        assert(err == XTCP_SUCCESS);

        g_ptp_l3_general_tx = i_xtcp.socket(XTCP_PROTOCOL_UDP);
        err = i_xtcp.connect(g_ptp_l3_general_tx, PTP_1588_GENERAL_PORT,
                             ptp_mcast_group);
        assert(err == XTCP_SUCCESS);

        // Set DSCP for PTP general messages
        dscp = DSCP_PTP_GENERAL << 2;
        err = i_xtcp.setsockopt(g_ptp_l3_general_tx, XTCP_SOCKET_LEVEL_IP,
                                XTCP_IP_SOCKET_OPTION_TOS,
                                (const uint8_t*)&dscp, sizeof(dscp));
        assert(err == XTCP_SUCCESS);
    }

    for (int i = 0; i < PTP_NUM_PORTS; i++)
        ptp_reset(i);

    pdelay_epoch_timer = this_tile_now;
}

void ptp_periodic(client interface ethernet_tx_if ?i_eth,
                  client interface xtcp_if ?i_xtcp,
                  unsigned t) {
    for (int i = 0; i < PTP_NUM_PORTS; i++) {
        int role = ptp_port_info[i].role_state;
        int asCapable = ptp_port_info[i].asCapable;
        int is_l3_periodic = isnull(i_eth) && !isnull(i_xtcp);

        int recv_sync_timeout_interval =
            last_receive_sync_upstream_interval[i] *
            PTP_SYNC_RECEIPT_TIMEOUT_MULTIPLE;

        int sending_pdelay =
            (ptp_port_info[i].delay_info.multiple_resp_count < 3);

        if (!sending_pdelay) {
            periodic_counter[i]++;
            const int five_minutes_in_periodic =
                5 * 60 * (XS1_TIMER_HZ / PTP_PERIODIC_TIME);
            if (periodic_counter[i] >= five_minutes_in_periodic) {
                sending_pdelay = 1;
                ptp_port_info[i].delay_info.multiple_resp_count = 0;
                periodic_counter[i] = 0;
            }
        }

        // followUpReceiptTimeout:
        if ((received_sync == 1 && (ptp_port_info[i].role_state == PTP_SLAVE) &&
             timeafter(t, last_received_sync_time[i] +
                              last_receive_sync_upstream_interval[i]))) {
            received_sync = 0;
        }

        if ((last_received_announce_time_valid[i] &&
             timeafter(t,
                       last_received_announce_time[i] +
                           RECV_ANNOUNCE_TIMEOUT)) || // announceReceiptTimeout
                                                      // syncReceiptTimeout
            (received_sync && (ptp_port_info[i].role_state == PTP_SLAVE) &&
             timeafter(t, last_received_sync_time[i] +
                              recv_sync_timeout_interval))) {

            received_sync = 0;
            last_received_announce_time[i] = t;
            last_announce_time[i] = t - ANNOUNCE_PERIOD - 1;
            last_received_announce_time_valid[i] = 0;

            if (role == PTP_SLAVE) {
                set_new_role(PTP_UNCERTAIN, i);
            }
        }

        if ((is_l3_periodic || asCapable) &&
            (role == PTP_MASTER || role == PTP_UNCERTAIN) &&
            timeafter(t, last_announce_time[i] + ANNOUNCE_PERIOD)) {
            send_ptp_announce_msg(i_eth, i_xtcp, i);
            last_announce_time[i] = t;
        }

        if ((is_l3_periodic || asCapable) && role == PTP_MASTER &&
            timeafter(t, last_sync_time[i] + SYNC_PERIOD)) {
            send_ptp_sync_msg(i_eth, i_xtcp, i);
            last_sync_time[i] = t;
        }

        if (timeafter(t, last_pdelay_req_time[i] + PDELAY_REQ_PERIOD)) {
            if (pdelay_request_sent[i] && !received_pdelay[i]) {
                pdelay_req_reset(i);
            }
            if (sending_pdelay && !isnull(i_eth))
                send_ptp_pdelay_req_msg(i_eth, i);
            last_pdelay_req_time[i] = t;
        }
    }

    // E2E Delay Request timing (for IEEE 1588 over L3/UDP)
    if (ptp_port_info[0].role_state == PTP_SLAVE && !isnull(i_xtcp) &&
        timeafter(t, e2e_last_delay_req_time + DELAY_REQ_PERIOD)) {

        if (e2e_delay_request_sent) {
            // Previous request timed out - reset state
            e2e_delay_request_sent = 0;
        }

        send_delay_req_msg(i_eth, i_xtcp, 0);
        e2e_last_delay_req_time = t;
    }

    periodic_update_reference_timestamps(t);
}

void ptp_current_grandmaster(char grandmaster[8]) {
    memcpy(grandmaster,
           best_announce_msg.announceMessage.grandmasterIdentity.data, 8);
}

void ptp_get_path_sequence(uint16_t port_num,
                           uint16_t *count,
                           n64_t pathSequence[PTP_MAXIMUM_PATH_TRACE_TLV]) {
    int device_is_follower = !!compare_clock_identity_to_me(
        &best_announce_msg.announceMessage.grandmasterIdentity);

    *count = 0;

    if (port_num >= PTP_NUM_PORTS ||
        device_is_follower + steps_removed_from_gm + 1 >
            PTP_MAXIMUM_PATH_TRACE_TLV)
        return;

    if (device_is_follower)
        memcpy(&pathSequence[0].data,
               best_announce_msg.announceMessage.grandmasterIdentity.data, 8);

    memcpy(&pathSequence[device_is_follower].data,
           &best_announce_msg.pathSequence[0].data, steps_removed_from_gm * 8);
    memcpy(&pathSequence[device_is_follower + steps_removed_from_gm].data,
           my_port_id.data, 8);
    *count = device_is_follower + steps_removed_from_gm + 1;
}
