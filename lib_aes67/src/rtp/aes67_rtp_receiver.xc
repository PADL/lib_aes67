// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <string.h>
#include <stdlib.h>

#include "aes67_internal.h"
#include "ptp_internal.h"
#include "nettypes.h"
#include "aes67_rtp.h"

aes67_receiver_t receivers[NUM_AES67_RECEIVERS];

static int32_t lookup_receiver_from_fd(int32_t fd) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        if (receivers[id].socket.fd == fd)
            return id;
    }

    return -1;
}

static aes67_status_t aes67_close_receiver(
    client xtcp_if i_xtcp, const aes67_stream_info_t &stream_info, int32_t id) {
    aes67_socket_t &receiver_socket = receivers[id].socket;

    i_xtcp.close(receiver_socket.fd);
    receiver_socket.fd = -1;

    return AES67_STATUS_OK;
}

static aes67_status_t aes67_open_receiver(
    client xtcp_if i_xtcp, const aes67_stream_info_t &stream_info, int32_t id) {
    xtcp_ipaddr_t rtp_addr;
    aes67_status_t err;

    aes67_socket_t &receiver_socket = receivers[id].socket;
    memset(&receiver_socket, 0, sizeof(receiver_socket));

    receiver_socket.fd = i_xtcp.socket(XTCP_PROTOCOL_UDP);
    if (receiver_socket.fd < 0)
        return AES67_STATUS_SOCKET_ERROR;

    get_stream_rtp_address(rtp_addr, stream_info);

    err = i_xtcp.listen(receiver_socket.fd, stream_info.rtp_port, rtp_addr);
    if (err) {
        aes67_close_receiver(i_xtcp, stream_info, id);
        return AES67_STATUS_SOCKET_ERROR;
    }

    return AES67_STATUS_OK;
}

static unsafe void aes67_poll_stream_info_changed(client xtcp_if i_xtcp) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        aes67_stream_info_t *unsafe stream_info = aes67_get_receiver_stream(id);
        aes67_socket_t &receiver_socket = receivers[id].socket;

        switch (stream_info->state) {
        case AES67_STREAM_STATE_DISABLED:
            if (receiver_socket.fd != -1)
                aes67_close_receiver(i_xtcp, *stream_info, id);
            unsafe {
                for (size_t ch = 0; ch < AES67_MAX_CHANNELS_PER_RECEIVER; ch++)
                    aes67_audio_fifo_disable(&receivers[id].fifos[ch]);
            }
            break;
        case AES67_STREAM_STATE_ENABLED:
            if (receiver_socket.fd == -1)
                aes67_open_receiver(i_xtcp, *stream_info, id);
            break;
        case AES67_STREAM_STATE_POTENTIAL:
            fail("potential state invalid for sender stream");
            break;
        case AES67_STREAM_STATE_UPDATING:
            [[fallthrough]];
        default:
            break;
        }
    }
}

static void aes67_audio_fifo_handle_buf_ctl(chanend buf_ctl,
                                            aes67_audio_fifo_t *fifo,
                                            int *buf_ctl_notified,
                                            timer tmr) {
    // FIXME: why is this cast necessary to avoid linking errors?
    unsafe {
        __aes67_audio_fifo_handle_buf_ctl((unsigned int)buf_ctl, fifo,
                                          buf_ctl_notified, (unsigned int)tmr);
    }
}

// depacketizer
void aes67_rtp_receiver(client xtcp_if i_xtcp, chanend buf_ctl) {
    aes67_rtp_packet_t packet = {0};
    timer t;
    unsigned time;

    unsafe {
        memset(&receivers, 0, sizeof(receivers));

        for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
            for (size_t ch = 0; ch < AES67_MAX_CHANNELS_PER_RECEIVER; ch++) {
                aes67_audio_fifo_t *fifo = &receivers[id].fifos[ch];

                // initialize FIFO and mark as ready to receive media control
                // commands
                aes67_audio_fifo_init(fifo);
                aes67_register_buf_fifo(id * NUM_AES67_RECEIVERS + ch,
                                        (uintptr_t)fifo);
            }
        }
    }

    t :> time;

    while (1) {
        select {
        case i_xtcp.event_ready():
            xtcp_event_type_t event;
            aes67_socket_t *socket;
            int32_t fd, id;

            event = i_xtcp.get_event(fd);
            if (event != XTCP_RECV_FROM_DATA ||
                ((id = lookup_receiver_from_fd(fd)) == -1))
                break;

            unsafe {
                aes67_receiver_t &receiver = receivers[id];

                if (aes67_rtp_recv(i_xtcp, &receivers[id].socket, &packet) ==
                    0) {
                    aes67_status_t status;

                    status =
                        aes67_process_rtp_packet(buf_ctl, id, receiver, packet);
#if DEBUG_RTP
                    if (status != AES67_STATUS_OK)
                        debug_printf("RTP packet error: %s\n",
                                     aes67_status_to_string(status));
#endif
                }
            }

            break;

            case buf_ctl :> int fifo_index:
                aes67_receiver_t &receiver =
                    receivers[fifo_index / AES67_MAX_CHANNELS_PER_RECEIVER];
                aes67_audio_fifo_t *fifo =
                    &receiver
                         .fifos[fifo_index % AES67_MAX_CHANNELS_PER_RECEIVER];

                aes67_audio_fifo_handle_buf_ctl(buf_ctl, fifo,
                                                &receiver.buf_ctl_notified, t);
                break;

            case t when timerafter(time) :> void:
                unsafe {
                    aes67_poll_stream_info_changed(i_xtcp);
                }
                time += XS1_TIMER_HZ;
                break;
            }
    }
}
