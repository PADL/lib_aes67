// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include "aes67_internal.h"
#include "ptp_internal.h"
#include "aes67_utils.h"
#include "sap.h"

#include <xassert.h>
#include <string.h>
#include <stdlib.h>

#define SAP_PERIODIC_TIME (XS1_TIMER_HZ * 10)
#define SAP_PORT 9875

static const xtcp_ipaddr_t any_addr;
static const xtcp_ipaddr_t sap_mcast_group = {239, 255, 255, 255};
static const n64_t zero_n64;
static uint32_t sample_rate = AES67_DEFAULT_SAMPLE_RATE;

/*
 * Receiver state: the receiver name contains the subscribed receiver name
 * (from the API), one per slot; the SDP contains the discovered SDP for
 * the receiver (if any).
 */
static aes67_session_name_t session_subscriptions[NUM_AES67_RECEIVERS];
static aes67_sdp_t sdp_subscriptions[NUM_AES67_RECEIVERS];

static void sdp_init_subscriptions(void) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++)
        aes67_sdp_set_defaults(sdp_subscriptions[id]);
}

/*
 * Sender state: the SDP we will advertise.
 */
static aes67_sdp_t sdp_advertisements[NUM_AES67_SENDERS];

static void sdp_init_advertisements(void) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_SENDERS; id++)
        aes67_sdp_set_defaults(sdp_advertisements[id]);
}

static aes67_status_t sdp_to_stream_info(aes67_stream_info_t &stream_info,
                                         const int32_t id,
                                         const aes67_sdp_t &sdp) {
    aes67_status_t status;

    stream_info.state = AES67_STREAM_STATE_POTENTIAL;

    switch (sdp.encoding) {
    case AES67_ENCODING_L16:
        [[fallthrough]];
    case AES67_ENCODING_L24:
        [[fallthrough]];
    case AES67_ENCODING_L32:
        break;
    default:
        return AES67_STATUS_UNKNOWN_RTP_ENCODING;
    }
    stream_info.encoding = sdp.encoding;

    stream_info.sample_size = sdp.sample_size / 8;
    stream_info.payload_type = sdp.payload_type;
    stream_info.channel_count = sdp.channel_count;
    stream_info.stream_id = id;
    stream_info.sample_rate = sdp.sample_rate;
    stream_info.packet_time_us =
        (uint32_t)(sdp.packet_duration * 1000.0); // convert ms to us

    // Parse destination IPv4 address
    status = aes67_sdp_get_ipv4_address(sdp, stream_info.dest_addr);
    if (status != AES67_STATUS_OK) {
        debug_printf("invalid SDP address %s\n", sdp.address);
        return status;
    }

    // Parse source IPv4 address (session origin)
    status = aes67_sdp_get_ipv4_session_origin(sdp, stream_info.src_addr);
    if (status != AES67_STATUS_OK) {
        debug_printf("invalid SDP session origin %s\n", sdp.session_origin);
        return status;
    }

    status = aes67_sdp_get_ipv4_port(sdp, stream_info.dest_port);
    if (status != AES67_STATUS_OK) {
        debug_printf("invalid SDP port %s\n", sdp.__port);
        return status;
    }

    status = aes67_sdp_get_ptp_gmid(sdp, stream_info.gm_id.data);
    if (status != AES67_STATUS_OK) {
        debug_printf("invalid SDP PTP GMID %s\n", sdp.ptp_gmid);
        return status;
    }

    stream_info.gm_port = (uint16_t)aes67_sdp_get_ptp_domain(sdp);
    stream_info.clock_offset = sdp.clock_offset;

    return AES67_STATUS_OK;
}

static int sdp_has_valid_session_name(const aes67_sdp_t &sdp) {
    return sdp.session_name[0] != '\0';
}

static void sdp_invalidate_payload(aes67_sdp_t &sdp) {
    if (sdp.payload_type < 0)
        sdp.payload_type--;
    else
        sdp.payload_type = -1;
}

static int sdp_is_subscribed(int32_t id) {
    if (!is_valid_receiver_id(id))
        return 0;

    return sdp_has_valid_session_name(sdp_subscriptions[id]);
}

static int sdp_is_advertising(int32_t id) {
    if (!is_valid_sender_id(id))
        return 0;

    return sdp_has_valid_session_name(sdp_advertisements[id]);
}

static void sdp_subscribe(int32_t id, const aes67_sdp_t &sdp) {
    memcpy(&sdp_subscriptions[id], &sdp, sizeof(sdp));
}

static void sdp_unsubscribe(int32_t id, const aes67_sdp_t &sdp) {
    memset(&sdp_subscriptions[id], 0, sizeof(sdp_subscriptions));
}

static int sdp_equal(const aes67_sdp_t &sdp1, const aes67_sdp_t &sdp2) {
    return memcmp(&sdp1, &sdp2, sizeof(sdp2));
}

#if AES67_FAST_CONNECT_ENABLED
static size_t sdp_get_fast_connect_page_count(uint32_t valid_flags, uint32_t &page_size) {
    page_size = fl_getPageSize();

    size_t validCount = popcount(valid_flags);
    size_t effectiveLength = sizeof(uint32_t) + sizeof(uint32_t) +
                             (validCount * sizeof(aes67_sdp_t));

    return (effectiveLength + page_size - 1) / page_size;
}

static void sdp_erase_fast_connect_info(void) {
    aes67_sdp_fast_connect_t fc;

    if (fl_readDataPage(0, (uint8_t *)&fc) != 0)
        return;

    if (fc.magic == AES67_FAST_CONNECT_MAGIC)
        fl_eraseDataSector(0);
}

static void sdp_start_fast_connect(void) {
#define DEFAULT_PAGE_SIZE (256)
    union {
        aes67_sdp_fast_connect_t fc;
        uint8_t buffer[sizeof(aes67_sdp_fast_connect_t) / DEFAULT_PAGE_SIZE + DEFAULT_PAGE_SIZE];
    } u;
    uint32_t page_size;

    if (fl_readDataPage(0, (uint8_t *)&u.fc) != 0 ||
        u.fc.magic != AES67_FAST_CONNECT_MAGIC)
        return;

    size_t page_count = sdp_get_fast_connect_page_count(u.fc.valid, page_size);

    if (page_size > DEFAULT_PAGE_SIZE)
        return;

    uint8_t *p = (uint8_t *)&u.fc + page_size;
    for (size_t i = 1; i < page_count; i++) {
        if (fl_readDataPage(i, p) != 0)
            return;
        p += page_size;
    }

    for (size_t id = 0, sdp_index = 0; id < NUM_AES67_RECEIVERS; id++) {
        if ((u.fc.valid & BIT(id)) == 0)
            continue;

        memcpy(&sdp_subscriptions[id], &u.fc.sdp[sdp_index], sizeof(aes67_sdp_t));
        memcpy(session_subscriptions[id], u.fc.sdp[sdp_index].session_name,
               sizeof(u.fc.sdp[sdp_index].session_name));

        sdp_index++;
    }
}

static void sdp_store_fast_connect_info(void) {
    aes67_sdp_fast_connect_t fc;
    uint32_t page_size;

    if (fl_readDataPage(0, (uint8_t *)&fc) != 0 ||
        fc.magic == AES67_FAST_CONNECT_MAGIC);
        fl_eraseDataSector(0);

    memset(&fc, 0, sizeof(fc));

    fc.magic = AES67_FAST_CONNECT_MAGIC;

    for (size_t id = 0, i = 0; id < NUM_AES67_RECEIVERS; id++) {
        if (!sdp_is_subscribed(id))
            continue;

        fc.valid |= BIT(id);
        memcpy(&fc.sdp[i++], &sdp_subscriptions[id], sizeof(aes67_sdp_t));
    }

    size_t page_count = sdp_get_fast_connect_page_count(fc.valid, page_size);
    uint8_t *p = (uint8_t *)&fc;

    for (size_t i = 0; i < page_count; i++) {
        fl_writeDataPage(i, p);
        p += page_size;
    }
}
#endif // AES67_FAST_CONNECT_ENABLED

static aes67_status_t sap_handle_message(client xtcp_if i_xtcp,
                                         chanend media_control,
                                         uint8_t buf[len],
                                         size_t len) {
    aes67_status_t status;
    aes67_sap_t sap;
    aes67_sdp_t sdp;
    int32_t id;

    unsafe {
        status = aes67_sap_parse(buf, len, sap);
        if (status != AES67_STATUS_OK) {
            debug_printf("failed to parse SAP %.*s\n", len, buf);
            return status;
        }

        status = aes67_sdp_parse_string(sap.sdp, sdp);
        if (status != AES67_STATUS_OK) {
            debug_printf("failed to parse SDP %s\n", sap.sdp);
            return status;
        }

#pragma unsafe arrays
        for (id = 0; id < NUM_AES67_RECEIVERS; id++) {
            if (strcmp(sdp.session_name, session_subscriptions[id]) == 0)
                break;
        }

        if (id == NUM_AES67_RECEIVERS) {
            debug_printf("ignoring session %s, not subscribed\n",
                         sdp.session_name);
            return AES67_STATUS_OK;
        }

        aes67_media_control_command_t command;
        aes67_stream_info_t stream_info;
        aes67_status_t status;

        status = sdp_to_stream_info(stream_info, id, sdp);
        if (status == AES67_STATUS_UNKNOWN_RTP_ENCODING) {
            const char *unsafe encoding = aes67_encoding_name(sdp.encoding);

            debug_printf("unknown RTP encoding %d/%s\n", sdp.encoding,
                         encoding != NULL ? encoding
                                          : (const char *unsafe) "<nil>");
        } else if (status != AES67_STATUS_OK) {
            debug_printf("failed to marshal stream info: %s\n",
                         aes67_status_to_string(status));
        }
        if (status != AES67_STATUS_OK)
            return status;

        if (sap.message_type == AES67_SAP_MESSAGE_DELETE) {
            debug_printf("unsubscribing from stream %s\n", sdp.session_name);
            sdp_unsubscribe(id, sdp);
            media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE;
            media_control <: id;
#if AES67_FAST_CONNECT_ENABLED
            sdp_store_fast_connect_info();
#endif
        } else if (!sdp_is_subscribed(id)) {
            debug_printf("subscribing to stream %s channel count %d sample "
                         "size %d sample rate %d encoding %s\n",
                         sdp.session_name, sdp.channel_count, sdp.sample_size,
                         sdp.sample_rate, aes67_encoding_name(sdp.encoding));
            sdp_subscribe(id, sdp);
            media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE;
            media_control <: stream_info;
#if AES67_FAST_CONNECT_ENABLED
            sdp_store_fast_connect_info();
#endif
        } else if (sdp_equal(sdp_subscriptions[id], sdp) == 0) {
            debug_printf("resubscribing to changed stream %s\n",
                         sdp.session_name);
            sdp_subscribe(id, sdp);
            media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE;
            media_control <: stream_info;
        }
    }

    return AES67_STATUS_OK;
}

static void sap_handle_event(client xtcp_if i_xtcp,
                             chanend media_control,
                             xtcp_event_type_t event,
                             int fd) {
    uint8_t buf[AES67_SAP_MAX_LEN];
    int32_t nrecv;
    xtcp_ipaddr_t ipaddr = {0};
    uint16_t port_number = 0;

    switch (event) {
    case XTCP_RECV_FROM_DATA:
        nrecv =
            i_xtcp.recvfrom(fd, buf, AES67_SAP_MAX_LEN, ipaddr, port_number);
        if (nrecv <= 0)
        break;

        sap_handle_message(i_xtcp, media_control, buf, nrecv);
        break;
    case XTCP_IFUP:
#if AES67_FAST_CONNECT_ENABLED
        sdp_start_fast_connect();
        break;
#endif
    default:
        break;
    }
}

static aes67_status_t
sap_advertise_sender(client xtcp_if i_xtcp,
                     int sap_tx_socket,
                     aes67_sdp_t &_sdp,
                     const aes67_time_source_info_t &time_source_info) {
    char sdp_string[AES67_SDP_MAX_LEN];
    xtcp_ipconfig_t ipconfig;
    aes67_sdp_t sdp = _sdp;
    aes67_status_t status;
    uint8_t message_type;
    aes67_socket_t sock;

    assert(sdp_has_valid_session_name(sdp));

    if (sdp.payload_type < 0)
        message_type = AES67_SAP_MESSAGE_DELETE;
    else
        message_type = AES67_SAP_MESSAGE_ANNOUNCE;

    ipconfig = i_xtcp.get_netif_ipconfig(0);

    sock.fd = sap_tx_socket;
    sock.joined_group = 1;
    memcpy(&sock.dest_addr, &sap_mcast_group, sizeof(sap_mcast_group));
    memcpy(&sock.src_addr, &ipconfig.ipaddr, sizeof(ipconfig.ipaddr));

    // fill in origin, ptp_gmid, and ptp_domain which may have changed from
    // when the advertisement was initially registered
    aes67_sdp_set_ipv4_session_origin(sdp, ipconfig.ipaddr);
    aes67_sdp_set_ptp_gmid(sdp, time_source_info.ptp_id);
    aes67_sdp_set_ptp_domain(sdp, time_source_info.ptp_domain);

    status = aes67_sdp_to_string(sdp, sdp_string, sizeof(sdp_string));
    if (status != AES67_STATUS_OK)
        return status;

    if (aes67_sap_send_sdp_string(i_xtcp, sock, sdp_string, message_type) < 0)
        return AES67_STATUS_SOCKET_ERROR;

    if (message_type == AES67_SAP_MESSAGE_DELETE) {
        // once we've sent one (?) delete message, withdraw the announcement
        aes67_sdp_set_defaults(sdp);
    }

    return AES67_STATUS_OK;
}

static void sap_advertise_senders(client xtcp_if i_xtcp,
                                  int sap_tx_socket,
                                  const aes67_time_source_info_t &time_source_info) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_SENDERS; id++) {
        // a non-empty session name indicates we should send an advertisement
        // payload_type == -1 indicates that we should withdarw the advertisement

        if (sdp_is_advertising(id))
            sap_advertise_sender(i_xtcp, sap_tx_socket, sdp_advertisements[id], time_source_info);
    }
}

static void aes67_periodic(client xtcp_if i_xtcp,
                           chanend media_control,
                           int sap_tx_socket,
                           unsigned sap_timeout) {
    aes67_time_source_info_t time_source_info;

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO;
    media_control :> time_source_info;

    sap_advertise_senders(i_xtcp, sap_tx_socket, time_source_info);
}

static aes67_status_t
start_streaming(chanend media_control,
                int32_t id,
                const char session_name[],
                size_t session_name_len,
                xtcp_ipaddr_t ip_addr,
                uint32_t sample_size,
                uint32_t channel_count,
                timer sap_timer) {
    aes67_stream_info_t stream_info;
    aes67_status_t status;
    aes67_sdp_t sdp;

    if (!is_valid_sender_id(id)) {
        return AES67_STATUS_INVALID_STREAM_ID;
    }

    if (sdp_is_advertising(id)) {
        return AES67_STREAM_ALREADY_ADVERTISING;
    }

    aes67_sdp_set_defaults(sdp);
    aes67_sdp_set_ipv4_address(sdp, ip_addr);
    aes67_sdp_set_port(sdp, AES67_DEFAULT_PORT_STR);

    uint32_t session_id;
    sap_timer :> session_id;
    aes67_sdp_set_session_id(sdp, session_id);

    // origin will be filled in at advertisement time
    memcpy(sdp.session_name, session_name, session_name_len + 1);
    // information we will leave unset (could contain "%d channels")
    aes67_sdp_set_payload_type(sdp, 96 + (session_id % 32));

    // aes67_sdp_set_encoding() will also set sample_size
    switch (sample_size) {
    case 16:
        aes67_sdp_set_encoding(sdp, AES67_ENCODING_L16);
        break;
    case 24:
        aes67_sdp_set_encoding(sdp, AES67_ENCODING_L24);
        break;
    case 32:
        aes67_sdp_set_encoding(sdp, AES67_ENCODING_L32);
        break;
    default:
        return AES67_STATUS_INVALID_SAMPLE_SIZE;
    }

    sdp.sample_rate = sample_rate;

    if (channel_count > AES67_MAX_CHANNELS_PER_SENDER)
        return AES67_STATUS_INVALID_CHANNEL_COUNT;
    sdp.channel_count = channel_count;
    sdp.packet_duration = 1.0; // TODO: make this configurable

    // ptp_gmid, ptp_domain will be filled in at advertisement time
    // clock offset is always zero for SMPTE 2110 interop
    sdp.clock_offset = 0;

    status = sdp_to_stream_info(stream_info, id, sdp);
    if (status != AES67_STATUS_OK)
        return status;

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_START_STREAMING;
    media_control <: stream_info;

    memcpy(&sdp_subscriptions[id], &sdp, sizeof(sdp));

    return AES67_STATUS_OK;
}

static aes67_status_t
stop_streaming(chanend media_control, int32_t id) {
    if (!is_valid_sender_id(id)) {
        return AES67_STATUS_INVALID_STREAM_ID;
    }

    if (sdp_is_advertising(id)) {
        return AES67_STREAM_NOT_ADVERTISED;
    }

    sdp_invalidate_payload(sdp_advertisements[id]);

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_STOP_STREAMING;
    media_control <: id;

    return AES67_STATUS_OK;
}

[[combinable]] void
aes67_manager(server interface aes67_interface i_aes67[num_aes67_clients],
              size_t num_aes67_clients,
              client xtcp_if i_xtcp,
              chanend media_control,
              fl_QSPIPorts &?qspi_ports) {
    timer sap_timer;
    int sap_timeout;
    int sap_rx_socket = -1;
    int sap_tx_socket = -1;
    xtcp_error_code_t err;
    uint32_t pending_events = 0;
    aes67_time_source_info_t last_time_source_info;
    aes67_media_clock_info_t last_media_clock_info;

#if AES67_FAST_CONNECT_ENABLED
    if (isnull(qspi_ports)) {
        fail("Fast connect enabled, but QSPI ports null");
    } else if (fl_connect(qspi_ports)) {
        fail("Could not connect to flash");
    }
#endif

    sdp_init_subscriptions();
    sdp_init_advertisements();

    sap_timer :> sap_timeout;

    i_xtcp.join_multicast_group(sap_mcast_group);

    sap_rx_socket = i_xtcp.socket(XTCP_PROTOCOL_UDP);
    err = i_xtcp.listen(sap_rx_socket, SAP_PORT, any_addr);
    assert(err == XTCP_SUCCESS);

    sap_tx_socket = i_xtcp.socket(XTCP_PROTOCOL_UDP);
    err = i_xtcp.connect(sap_tx_socket, SAP_PORT, sap_mcast_group);
    assert(err == XTCP_SUCCESS);

    while (1) {
        select {
        case i_xtcp.event_ready():
            xtcp_event_type_t event;
            int32_t fd, id;

            event = i_xtcp.get_event(fd);
            sap_handle_event(i_xtcp, media_control, event, fd);
            break;

        case i_aes67[size_t i].subscribe(int16_t id, const char session_name[])->aes67_status_t status:
            size_t len;

            if (!is_valid_receiver_id(id)) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }

#pragma unsafe arrays
            for (len = 0; session_name[len]; len++)
                ;
            memcpy(session_subscriptions[id], session_name, len + 1);
            debug_printf("subscribing SAP session %s into receiver slot %d\n",
                         session_subscriptions[id], id);
            status = AES67_STATUS_OK;
            break;

        case i_aes67[size_t i].unsubscribe(int16_t id, const char session_name[])->aes67_status_t status:
            if (!is_valid_receiver_id(id)) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }

            memset(session_subscriptions[id], 0, sizeof(session_subscriptions[id]));
            status = AES67_STATUS_OK;
            break;
        case i_aes67[size_t i].advertise(int16_t id, const char session_name[], uint8_t ip_addr[4], uint32_t sample_size, uint32_t channel_count)->aes67_status_t status:
            aes67_session_name_t _session_name;
            xtcp_ipaddr_t _ip_addr;
            size_t len;

#pragma unsafe arrays
            for (len = 0; session_name[len]; len++)
                ;
            if (len >= sizeof(_session_name)) {
                status = AES67_STATUS_OUT_OF_BUFFER_SPACE;
                break;
            }
            memcpy(_session_name, session_name, len + 1);
            memcpy(_ip_addr, ip_addr, sizeof(ip_addr));
            status = start_streaming(media_control, id, _session_name, len,
                                     _ip_addr, sample_size, channel_count, sap_timer);
            break;
        case i_aes67[size_t i].unadvertise(int16_t id)->aes67_status_t status:
            status = stop_streaming(media_control, id);
            break;
        case i_aes67[size_t i].get_time_source_info(aes67_time_source_info_t & info)->aes67_status_t status:
            media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO;
            media_control :> info;
            status = AES67_STATUS_OK;
            break;
        case i_aes67[size_t i].get_media_clock_info(aes67_media_clock_info_t & info)->aes67_status_t status:
            media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_CLOCK_INFO;
            media_control :> info;
            status = AES67_STATUS_OK;
            break;
        case i_aes67[size_t i].set_sample_rate(uint32_t rate)->aes67_status_t status:
            if ((rate % 48000) && (rate % 44100)) {
                status = AES67_STATUS_INVALID_SAMPLE_RATE;
                break;
            }

            uint8_t _status;

            sample_rate = rate;
            media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_SET_SAMPLE_RATE;
            media_control <: rate;
            media_control :> _status;
            status = (aes67_status_t)_status;
            break;
        case i_aes67[size_t i].get_event_info()->aes67_event_info_t event_info:
            if (pending_events & BIT(AES67_EVENT_TIME_SOURCE_INFO)) {
                event_info.event_type = AES67_EVENT_TIME_SOURCE_INFO;
                event_info.u.time_source_info = last_time_source_info;
                pending_events &= ~BIT(AES67_EVENT_TIME_SOURCE_INFO);
            } else if (pending_events & BIT(AES67_EVENT_MEDIA_CLOCK_INFO)) {
                event_info.event_type = AES67_EVENT_MEDIA_CLOCK_INFO;
                event_info.u.media_clock_info = last_media_clock_info;
                pending_events &= ~BIT(AES67_EVENT_MEDIA_CLOCK_INFO);
            } else {
                event_info.event_type = AES67_EVENT_NOOP;
            }
            break;
        case sap_timer when timerafter(sap_timeout) :> void:
            aes67_periodic(i_xtcp, media_control, sap_tx_socket, sap_timeout);
            sap_timeout += SAP_PERIODIC_TIME;
            break;
        }
    }
}
