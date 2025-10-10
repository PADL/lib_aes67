// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <string.h>
#include <stdlib.h>

#include "aes67_internal.h"
#include "aes67_media_clock_internal.h"
#include "ptp_internal.h"
#include "nettypes.h"

// stream info is the parsed form of the SDP for use by the I/O tasks
// it should not contain any transient information such as sequence numbers
aes67_stream_info_t receiver_streams[NUM_AES67_RECEIVERS];
aes67_stream_info_t sender_streams[NUM_AES67_SENDERS];

void get_stream_rtp_address(xtcp_ipaddr_t rtp_addr,
                            const aes67_stream_info_t &stream_info) {
    uint32_t net_addr = stream_info.rtp_addr.addr;

    rtp_addr[0] = (net_addr >> 24) & 0xFF;
    rtp_addr[1] = (net_addr >> 16) & 0xFF;
    rtp_addr[2] = (net_addr >> 8) & 0xFF;
    rtp_addr[3] = net_addr & 0xFF;
}

static aes67_status_t
join_receiver_stream(client xtcp_if i_xtcp,
                     const aes67_stream_info_t &stream_info) {
    xtcp_ipaddr_t rtp_addr;

    get_stream_rtp_address(rtp_addr, stream_info);
    i_xtcp.join_multicast_group(rtp_addr);

    return AES67_STATUS_OK;
}

static aes67_status_t
leave_receiver_stream(client xtcp_if i_xtcp,
                      const aes67_stream_info_t &stream_info) {
    xtcp_ipaddr_t rtp_addr;

    get_stream_rtp_address(rtp_addr, stream_info);
    i_xtcp.leave_multicast_group(rtp_addr);

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
        stream_info.stream_id, (stream_info.rtp_addr.addr & 0xff000000) >> 24,
        (stream_info.rtp_addr.addr & 0x00ff0000) >> 16,
        (stream_info.rtp_addr.addr & 0x0000ff00) >> 8,
        (stream_info.rtp_addr.addr & 0x000000ff) >> 0, stream_info.rtp_port);
}

static void
reset_stream_info(aes67_stream_info_t &stream_info) {
    memset(&stream_info, 0, sizeof(stream_info));
}

static aes67_status_t
handle_subscription_control_command(client xtcp_if i_xtcp,
                                    aes67_media_control_command_t command,
                                    int32_t id,
                                    aes67_stream_info_t &?_stream_info) {
    aes67_stream_info_t &stream_info =
        isnull(_stream_info) ? receiver_streams[id] : _stream_info;

    log_subscription_control_command(command, stream_info);

    switch (command) {
    case AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE:
        if (stream_info.state == AES67_RECEIVER_STATE_ENABLED)
            return AES67_STATUS_ALREADY_SUBSCRIBED;
        stream_info.state = AES67_RECEIVER_STATE_POTENTIAL;
        [[fallthrough]];
    case AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE:
        if (stream_info.state == AES67_RECEIVER_STATE_DISABLED)
            return AES67_STATUS_NOT_SUBSCRIBED;
        if (command == AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE)
            leave_receiver_stream(i_xtcp, receiver_streams[id]);
        // check if GM ID changed
        int gm_changed = 0;

        if (memcmp(&receiver_streams[id].gm_id, &stream_info.gm_id,
                   sizeof(stream_info.gm_id)) != 0 ||
            receiver_streams[id].gm_port != stream_info.gm_port) {
            gm_changed = 1;
        }
        join_receiver_stream(i_xtcp, stream_info);
        memcpy(&receiver_streams[id], &stream_info, sizeof(stream_info));
        if (gm_changed)
            stream_info.state = AES67_RECEIVER_STATE_POTENTIAL;
        break;
    case AES67_MEDIA_CONTROL_COMMAND_UNSUBSCRIBE:
        if (stream_info.state == AES67_RECEIVER_STATE_DISABLED)
            return AES67_STATUS_NOT_SUBSCRIBED;
        leave_receiver_stream(i_xtcp, stream_info);
        reset_stream_info(receiver_streams[id]);
        stream_info.state = AES67_RECEIVER_STATE_DISABLED;
        break;
    }

    return AES67_STATUS_OK;
}

void aes67_media_control_init(void) {
    memset(receiver_streams, 0, sizeof(receiver_streams));
    memset(sender_streams, 0, sizeof(sender_streams));
}

#pragma select handler
void aes67_media_control(chanend media_control, client xtcp_if i_xtcp) {
    uint8_t control_command;

    media_control :> control_command;

    switch (control_command) {
    case AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE:
        [[fallthrough]];
    case AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE:
        aes67_stream_info_t stream_info;

        media_control :> stream_info;
        handle_subscription_control_command(
            i_xtcp, (aes67_media_control_command_t)control_command,
            stream_info.stream_id, stream_info);
        break;
    case AES67_MEDIA_CONTROL_COMMAND_UNSUBSCRIBE:
        int32_t id;

        media_control :> id;
        handle_subscription_control_command(
            i_xtcp, (aes67_media_control_command_t)control_command,
            id, null);
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
        assert(stream_info.state == AES67_SENDER_STATE_POTENTIAL);
        stream_info.state = AES67_SENDER_STATE_ENABLED;
        sender_streams[stream_info.stream_id] = stream_info;
        break;
    case AES67_MEDIA_CONTROL_COMMAND_STOP_STREAMING:
        int32_t id;

        media_control :> id;
        reset_stream_info(sender_streams[id]); // will set state to DISABLED
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
