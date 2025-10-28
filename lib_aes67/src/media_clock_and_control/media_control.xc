// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <string.h>
#include <stdlib.h>
#include <ethernet.h>

#include "aes67_internal.h"
#include "media_clock_internal.h"
#include "rtp_internal.h"
#include "ptp_internal.h"

// stream info is the parsed form of the SDP for use by the I/O tasks
// it should not contain any transient information such as sequence numbers
aes67_stream_info_t receiver_streams[NUM_AES67_RECEIVERS];
aes67_stream_info_t sender_streams[NUM_AES67_SENDERS];

static aes67_status_t
join_receiver_stream(client ethernet_cfg_if i_eth_cfg,
                     client xtcp_if i_xtcp,
                     const aes67_stream_info_t &stream_info,
                     uint32_t flags) {
    xtcp_ipaddr_t rtp_addr;
    ethernet_macaddr_filter_t macaddr_filter;

    memcpy(rtp_addr, stream_info.dest_addr, sizeof(xtcp_ipaddr_t));
    i_xtcp.join_multicast_group(rtp_addr);

    if (flags & AES67_FLAG_RTP_ETH_HP) {
        // Delete old filter and add HP filter for multicast MAC address
        ipv4_to_multicast_mac(rtp_addr, macaddr_filter.addr);
        macaddr_filter.appdata = 0;
        i_eth_cfg.del_macaddr_filter(0, 0, macaddr_filter);
        i_eth_cfg.add_macaddr_filter(0, 1, macaddr_filter);
    }

    return AES67_STATUS_OK;
}

static aes67_status_t
leave_receiver_stream(client ethernet_cfg_if i_eth_cfg,
                      client xtcp_if i_xtcp,
                      const aes67_stream_info_t &stream_info,
                      uint32_t flags) {
    xtcp_ipaddr_t rtp_addr;
    ethernet_macaddr_filter_t macaddr_filter;

    memcpy(rtp_addr, stream_info.dest_addr, sizeof(xtcp_ipaddr_t));
    i_xtcp.leave_multicast_group(rtp_addr);

    if (flags & AES67_FLAG_RTP_ETH_HP) {
        // Delete HP filter for multicast MAC address
        ipv4_to_multicast_mac(rtp_addr, macaddr_filter.addr);
        macaddr_filter.appdata = 0;
        i_eth_cfg.del_macaddr_filter(0, 1, macaddr_filter);
    }

    return AES67_STATUS_OK;
}

static const char *alias
control_command_to_string(aes67_media_control_command_t command) {
    switch (command) {
    case AES67_MEDIA_CONTROL_COMMAND_NOOP:
        return "NOOP";
    case AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE:
        return "SUBSCRIBE";
    case AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE:
        return "RESUBSCRIBE";
    case AES67_MEDIA_CONTROL_COMMAND_UNSUBSCRIBE:
        return "UNSUBSCRIBE";
    case AES67_MEDIA_CONTROL_COMMAND_GET_CLOCK_INFO:
        return "GET_CLOCK_INFO";
    case AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO:
        return "GET_TIME_SOURCE_INFO";
    case AES67_MEDIA_CONTROL_COMMAND_START_STREAMING:
        return "START_STREAMING";
    case AES67_MEDIA_CONTROL_COMMAND_STOP_STREAMING:
        return "STOP_STREAMING";
    case AES67_MEDIA_CONTROL_COMMAND_SET_SAMPLE_RATE:
        return "SET_SAMPLE_RATE";
    default:
        return "UNKNOWN";
    }
}

static void
log_subscription_control_command(aes67_media_control_command_t command,
                                 const aes67_stream_info_t &stream_info) {
    debug_printf(
        "%s: new state %08x ID %d dest %d.%d.%d.%d:%d\n",
        control_command_to_string(command), stream_info.state,
        stream_info.stream_id,
        stream_info.dest_addr[0],
        stream_info.dest_addr[1],
        stream_info.dest_addr[2],
        stream_info.dest_addr[3],
        stream_info.dest_port);
}

// copy the non-atomic elements of a stream_info
static void
copy_stream_info(aes67_stream_info_t &dest_stream_info,
                 const const aes67_stream_info_t &stream_info) {
    assert(dest_stream_info.state == AES67_STREAM_STATE_UPDATING);
    // copy in other fields (non-atomically)
    memcpy((uint8_t *)&dest_stream_info + sizeof(uint32_t),
           (const uint8_t *)&stream_info + sizeof(uint32_t),
           sizeof(stream_info) - sizeof(uint32_t));
}

static unsafe aes67_status_t
subscribe_or_resubscribe_stream(client ethernet_cfg_if ?i_eth_cfg,
                                client xtcp_if i_xtcp,
                                aes67_media_control_command_t command,
                                const aes67_stream_info_t &stream_info,
                                uint32_t flags) {
    const int32_t id = stream_info.stream_id;
    aes67_stream_info_t *unsafe dest_stream_info = aes67_get_receiver_stream(id);

    log_subscription_control_command(command, stream_info);

    // an existing stream can only be resubscribed to, or unsubscribed from
    if (dest_stream_info->state == AES67_STREAM_STATE_ENABLED &&
        command != AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE)
        return AES67_STATUS_ALREADY_SUBSCRIBED;
    else if (dest_stream_info->state == AES67_STREAM_STATE_DISABLED &&
        command == AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE)
        return AES67_STATUS_NOT_SUBSCRIBED;

    // atomically set the stream state to updating, pending update
    // (word writes on XMOS are atomic)
    dest_stream_info->state = AES67_STREAM_STATE_UPDATING;

    if (command == AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE)
        leave_receiver_stream(i_eth_cfg, i_xtcp, *dest_stream_info, flags);

    join_receiver_stream(i_eth_cfg, i_xtcp, stream_info, flags);

    copy_stream_info(*dest_stream_info, stream_info);

    COMPILER_BARRIER();
    dest_stream_info->state = AES67_STREAM_STATE_POTENTIAL;

    return AES67_STATUS_OK;
}

static unsafe aes67_status_t
unsubscribe_stream(client ethernet_cfg_if ?i_eth_cfg,
                   client xtcp_if i_xtcp,
                   const int32_t id,
                   uint32_t flags) {
    aes67_stream_info_t *unsafe stream_info = aes67_get_receiver_stream(id);

    if (stream_info->state == AES67_STREAM_STATE_DISABLED)
        return AES67_STATUS_NOT_SUBSCRIBED;

    leave_receiver_stream(i_eth_cfg, i_xtcp, *stream_info, flags);

    // atomic write, don't bother clearing other fields
    COMPILER_BARRIER();
    stream_info->state = AES67_STREAM_STATE_DISABLED;

    return AES67_STATUS_OK;
}

void aes67_media_control_init(void) {
    memset(receiver_streams, 0, sizeof(receiver_streams));
    memset(sender_streams, 0, sizeof(sender_streams));
}

#pragma select handler
void aes67_media_control(chanend media_control,
                         client ethernet_cfg_if i_eth_cfg,
                         client xtcp_if i_xtcp,
                         uint32_t flags) {
    uint8_t control_command;

    media_control :> control_command;

    switch (control_command) {
    case AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE:
        [[fallthrough]];
    case AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE:
        aes67_stream_info_t stream_info;

        media_control :> stream_info;
        unsafe {
            subscribe_or_resubscribe_stream(i_eth_cfg, i_xtcp, control_command, stream_info, flags);
        }
        break;
    case AES67_MEDIA_CONTROL_COMMAND_UNSUBSCRIBE:
        int32_t id;

        media_control :> id;
        unsafe {
            unsubscribe_stream(i_eth_cfg, i_xtcp, id, flags);
        }
        break;
    case AES67_MEDIA_CONTROL_COMMAND_GET_CLOCK_INFO:
        media_control <: ptp_media_clock.info;
        break;
    case AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO:
        aes67_time_source_info_t time_source_info;

        ptp_current_grandmaster(time_source_info.ptp_id);
        time_source_info.ptp_domain = 0;
        if (sync_lock)
            time_source_info.flags |= AES67_TIME_SOURCE_INFO_PTP_LOCKED_FLAG;
        time_source_info.reserved = 0;
        media_control <: time_source_info;
        break;
    case AES67_MEDIA_CONTROL_COMMAND_START_STREAMING:
        aes67_stream_info_t stream_info;

        media_control :> stream_info;
        sender_streams[stream_info.stream_id].state = AES67_STREAM_STATE_UPDATING;
        copy_stream_info(sender_streams[stream_info.stream_id], stream_info);
        COMPILER_BARRIER();
        sender_streams[stream_info.stream_id].state = AES67_STREAM_STATE_ENABLED;
        break;
    case AES67_MEDIA_CONTROL_COMMAND_STOP_STREAMING:
        int32_t id;

        media_control :> id;
        sender_streams[id].state = AES67_STREAM_STATE_DISABLED;
        break;
    case AES67_MEDIA_CONTROL_COMMAND_SET_SAMPLE_RATE:
        uint32_t rate;

        media_control :> rate;
        if (ptp_media_clock.info.rate != rate) {
            ptp_media_clock.info.rate = rate;
            sync_lock = 0;
            // TODO: do we need to do anything else to reset the media clock?
        }
        media_control <: AES67_STATUS_OK;
        break;
    default:
        break;
    }
}
