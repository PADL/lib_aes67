// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <print.h>
#include "media_clock_client.h"
#include "media_clock_internal.h"

void aes67_notify_buf_ctl_stream_info(chanend buf_ctl, uintptr_t fifo) {
    outuchar(buf_ctl, BUF_CTL_NOTIFY_STREAM_INFO);
    outuint(buf_ctl, fifo);
    outct(buf_ctl, XS1_CT_END);
}

void aes67_buf_ctl_ack(chanend buf_ctl) { outct(buf_ctl, XS1_CT_END); }

int aes67_get_buf_ctl_adjust(chanend buf_ctl) {
    int adjust;

    buf_ctl :> adjust;

    return adjust;
}

int aes67_get_buf_ctl_cmd(chanend buf_ctl) {
    int cmd;

    buf_ctl :> cmd;

    return cmd;
}

void aes67_send_buf_ctl_info(chanend buf_ctl,
                             int active,
                             uint32_t ptp_ts,
                             uint32_t local_ts,
                             uintptr_t rdptr,
                             uintptr_t wrptr,
                             timer tmr) {
    int thiscore_now;
    int tile_id = get_local_tile_id();

    slave {
        buf_ctl :> int;
        tmr :> thiscore_now;
        buf_ctl <: thiscore_now;
        buf_ctl <: active;

        // Skip ptp_ts and local_ts - not used by simplified buffer management
        (void) ptp_ts;
        (void) local_ts;

        buf_ctl <: rdptr;
        buf_ctl <: wrptr;
        buf_ctl <: tile_id;
    }
}
