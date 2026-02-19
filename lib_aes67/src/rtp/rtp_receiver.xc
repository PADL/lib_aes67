// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <string.h>
#include <stdlib.h>

#include "aes67_internal.h"
#include "rtp_internal.h"
#include "ptp_internal.h"

aes67_receiver_t receivers[NUM_AES67_RECEIVERS];

static int32_t lookup_receiver_from_fd(int32_t fd) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        if (receivers[id].socket.fd == fd)
            return id;
    }

    return -1;
}

static aes67_status_t
aes67_close_receiver(client xtcp_if i_xtcp,
                     const aes67_stream_info_t &stream_info,
                     int32_t id,
                     const uint32_t flags) {
    aes67_socket_t &receiver_socket = receivers[id].socket;

    if (!(flags & AES67_FLAG_RTP_ETH_HP) && receiver_socket.fd != -1)
        i_xtcp.close(receiver_socket.fd);

    memset(&receiver_socket, 0, sizeof(receiver_socket));
    receiver_socket.fd = -1;

    return AES67_STATUS_OK;
}

static aes67_status_t
aes67_potential_receiver(client xtcp_if i_xtcp,
                         const aes67_stream_info_t &stream_info,
                         int32_t id,
                         const uint32_t flags) {
    aes67_socket_t &receiver_socket = receivers[id].socket;
    aes67_status_t err;

    assert(receiver_socket.fd == -1);

    // TODO: check if word length copies are atomic
    memcpy(receiver_socket.src_addr, stream_info.src_addr, sizeof(xtcp_ipaddr_t));
    memcpy(receiver_socket.dest_addr, stream_info.dest_addr, sizeof(xtcp_ipaddr_t));
    receiver_socket.dest_port = stream_info.dest_port;

    if (flags & AES67_FLAG_RTP_ETH_HP) {
        receiver_socket.fd = -2; // use some dummy value
    } else {
        receiver_socket.fd = i_xtcp.socket(XTCP_PROTOCOL_UDP);
        if (receiver_socket.fd < 0) {
            aes67_close_receiver(i_xtcp, stream_info, id, flags);
            return AES67_STATUS_SOCKET_ERROR;
        }

        err = i_xtcp.listen(receiver_socket.fd, receiver_socket.dest_port, receiver_socket.dest_addr);
        if (err) {
            aes67_close_receiver(i_xtcp, stream_info, id, flags);
            return AES67_STATUS_SOCKET_ERROR;
        }
    }

    return AES67_STATUS_OK;
}

static unsafe void aes67_poll_stream_info_changed(client xtcp_if i_xtcp, uint32_t flags) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        // note: we need to compile with -O0 to avoid getting a dual issue exception
        // here. as we're not writing to the stream info, make a temporary copy.
        const aes67_stream_info_t stream_info = *aes67_get_receiver_stream(id);
        aes67_socket_t &receiver_socket = receivers[id].socket;

        switch (stream_info.state) {
        case AES67_STREAM_STATE_DISABLED:
            if (receiver_socket.fd != -1) {
                COMPILER_BARRIER();
                aes67_close_receiver(i_xtcp, stream_info, id, flags);
            }
            unsafe {
                for (size_t ch = 0; ch < AES67_MAX_CHANNELS_PER_RECEIVER; ch++)
                    aes67_audio_fifo_disable(&receivers[id].fifos[ch]);
            }
            break;
        case AES67_STREAM_STATE_POTENTIAL:
            if (receiver_socket.fd == -1) {
                COMPILER_BARRIER();
                aes67_potential_receiver(i_xtcp, stream_info, id, flags);
            }
            break;
        default:
            break;
        }
    }
}

static void aes67_audio_fifo_handle_buf_ctl(chanend buf_ctl,
                                            aes67_audio_fifo_t *fifo,
                                            int *buf_ctl_notified) {
    unsafe {
        aes67_audio_fifo_handle_buf_ctl_unsafe((uint32_t)buf_ctl, fifo, buf_ctl_notified);
    }
}

// depacketizer
static unsafe void
aes67_rtp_receiver_unsafe(client xtcp_if i_xtcp,
                          chanend buf_ctl,
                          streaming chanend ?c_eth_rx_hp) {
    ethernet_packet_info_t packet_info;
    // RTP packet _struct_ is laid out as a 2 byte length/alignment field
    // followed by a RTP packet including Ethernet, IP and UDP headers.
    // the structure has 4 byte alignment so to avoid warnings must be
    // passed to C functions as a uint32_t array. (we cannot pass the
    // structure itself as XC does not support packed structs)
    union {
        uint8_t octets[AES67_RTP_PACKET_STRUCT_SIZE];
        uint32_t words[AES67_RTP_PACKET_STRUCT_SIZE_WORDS];
    } pbuf;
    uint32_t flags = 0;
    unsigned time;
    timer t;

    if (!isnull(c_eth_rx_hp))
        flags |= AES67_FLAG_RTP_ETH_HP; // indicate raw Ethernet frames to be used

    memset(&pbuf, 0, sizeof(pbuf));
    memset(&receivers, 0, sizeof(receivers));

    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        receivers[id].socket.fd = -1;

        for (size_t ch = 0; ch < AES67_MAX_CHANNELS_PER_RECEIVER; ch++) {
            aes67_audio_fifo_t *fifo = &receivers[id].fifos[ch];

            // initialize FIFO and mark as ready to receive media control
            // commands
            aes67_audio_fifo_init(fifo);
            aes67_register_buf_fifo(id * NUM_AES67_RECEIVERS + ch,
                                    (uintptr_t)fifo);
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

                aes67_receiver_t &receiver = receivers[id];

                if (aes67_rtp_recv_opaque(i_xtcp, &receivers[id].socket, pbuf.words) == 0) {
                    aes67_status_t status;

                    status = aes67_process_rtp_packet_opaque(buf_ctl, id, receiver, pbuf.words);
#if DEBUG_RTP
                    if (status != AES67_STATUS_OK)
                        debug_printf("RTP packet error: %s\n", aes67_status_to_string(status));
#endif
                }
                break;

                // note: offset of 2 is for alignment, ensures remainder of PDU is 4 byte aligned
                case !isnull(c_eth_rx_hp) => ethernet_receive_hp_packet(c_eth_rx_hp, &pbuf.octets[2], packet_info):
                    // Parse IP header to extract destination address, it may be junk but we'll parse properly later
                    uint32_t dest_ip_host = aes67_rtp_packet_get_dest_ip_opaque(pbuf.words);
                    int32_t id;

                    // Find receiver with matching destination address
                    for (id = 0; id < NUM_AES67_RECEIVERS; id++) {
                        assert(receivers[id].socket.fd != -1 || receivers[id].socket.dest_addr != 0);

                        if (receivers[id].socket.fd == -1)
                            continue;
                        else if (dest_ip_host == xtcp_ipaddr_to_host_uint32(receivers[id].socket.dest_addr))
                            break;
                    }

                    if (id == NUM_AES67_RECEIVERS)
                        break; // packet not for a stream that has been registered (yet)

                    aes67_receiver_t &receiver = receivers[id];
                    aes67_status_t status = aes67_rtp_parse_raw_opaque(receiver.socket, packet_info, pbuf.words);
                    if (status != AES67_STATUS_OK) {
#if DEBUG_RTP
                        debug_printf("RTP raw packet error: %s\n", aes67_status_to_string(status));
#endif
                        break;
                    }

                    status = aes67_process_rtp_packet_opaque(buf_ctl, id, receiver, pbuf.words);
#if DEBUG_RTP
                    if (status != AES67_STATUS_OK)
                        debug_printf("RTP raw packet error: %s\n", aes67_status_to_string(status));
#endif
                    break;

                case buf_ctl :> int fifo_index:
                    aes67_receiver_t &receiver =
                        receivers[fifo_index / AES67_MAX_CHANNELS_PER_RECEIVER];
                    aes67_audio_fifo_t *fifo =
                        &receiver.fifos[fifo_index % AES67_MAX_CHANNELS_PER_RECEIVER];

                    aes67_audio_fifo_handle_buf_ctl(buf_ctl, fifo, &receiver.buf_ctl_notified);
                    break;

                case t when timerafter(time) :> void:
                    aes67_poll_stream_info_changed(i_xtcp, flags);
                    time += XS1_TIMER_HZ;
                    break;
            }
    }
}

void
aes67_rtp_receiver(client xtcp_if i_xtcp,
                   chanend buf_ctl,
                   streaming chanend ?c_eth_rx_hp) {
    unsafe {
        aes67_rtp_receiver_unsafe(i_xtcp, buf_ctl, c_eth_rx_hp);
    }
}
