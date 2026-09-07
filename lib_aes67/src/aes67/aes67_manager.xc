// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include <xassert.h>
#include <string.h>
#include <stdlib.h>

#include "aes67_internal.h"
#include "ptp_internal.h"
#include "aes67_utils.h"
#include "sap.h"

static const xtcp_ipaddr_t any_addr;
static const xtcp_ipaddr_t sap_mcast_group = {239, 255, 255, 255};
static const n64_t zero_n64;
static uint32_t sample_rate = AES67_DEFAULT_SAMPLE_RATE;

/*
 * Event state for hosts: the last values handed out, so only changes are
 * reported, plus a small queue of SAP announcements. The IO task is polled
 * once a second; the UART only carries what changed.
 */
#define AES67_STATUS_POLL_TIME (XS1_TIMER_HZ) // 1 second
#define AES67_SAP_ANNOUNCEMENT_QUEUE_LEN 4

static uint32_t pending_events = 0;
static uint32_t pending_receiver_events = 0;
static uint32_t pending_sender_events = 0;
static aes67_time_source_info_t last_time_source_info;
static aes67_media_clock_info_t last_media_clock_info;
static aes67_stream_endpoint_info_t last_receiver_info[NUM_AES67_RECEIVERS];
#if (NUM_AES67_SENDERS != 0)
static aes67_stream_endpoint_info_t last_sender_info[NUM_AES67_SENDERS];
#endif
static aes67_sap_announcement_t sap_announcement_queue[AES67_SAP_ANNOUNCEMENT_QUEUE_LEN];
static size_t sap_announcement_head = 0;
static size_t sap_announcement_count = 0;
static uint8_t network_link_up = 0;

static void sap_enqueue_announcement(aes67_sap_message_type_t message_type,
                                     const aes67_sdp_t &sdp);

#if AES67_FAST_CONNECT_ENABLED
static void sdp_store_fast_connect_info(void);
#endif

/*
 * Receiver state: the receiver name contains the subscribed receiver name
 * (from the API), one per slot; the SDP contains the discovered SDP for
 * the receiver (if any). Note: we don't currently cache all advertisements
 * pending a subscription, as that would require some care given limited
 * memory (e.g. a LRU cache). The disadvantage of this approach is the
 * subscription API must be called prior to receiving the SAP adverisement
 * (in practice this means there will be a delay). This is mitigated
 * somewhat by the fast connect support.
 */
static aes67_session_name_t session_subscriptions[NUM_AES67_RECEIVERS];
static aes67_sdp_t sdp_subscriptions[NUM_AES67_RECEIVERS];

static aes67_status_t _sap_handle_message(client xtcp_if i_xtcp,
                                          chanend media_control,
                                          int32_t id,
                                          aes67_sap_message_type_t message_type,
                                          const aes67_sdp_t &sdp,
                                          uint32_t flags,
                                          const uint32_t sap_timer_events);

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
                                         const uint8_t is_receiver,
                                         const aes67_sdp_t &sdp) {
    aes67_status_t status;
    const size_t max_channel_count =
        is_receiver ? AES67_MAX_CHANNELS_PER_RECEIVER : AES67_MAX_CHANNELS_PER_SENDER;

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
    stream_info.channel_count = sdp.channel_count > max_channel_count ? max_channel_count : sdp.channel_count;
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

#define SDP_SUBSCRIBE_FLAG_NEW 0x1
#define SDP_SUBSCRIBE_FLAG_STORE_FAST_CONNECT 0x2
#define SDP_SUBSCRIBE_FLAG_PERMANENT 0x4
#define SDP_SUBSCRIBE_FLAG_TIMED_OUT 0x8
#define SDP_SUBSCRIBE_FLAG_START_FAST_CONNECT 0x10

static void sdp_subscribe(int32_t id, const aes67_sdp_t &sdp, uint32_t flags, const uint32_t sap_timer_events) {
    assert(is_valid_receiver_id(id));

    memcpy(&sdp_subscriptions[id], &sdp, sizeof(sdp));

    sdp_subscriptions[id].flags = 0;
    sdp_subscriptions[id].timestamp = sap_timer_events;

    if (flags & AES67_SDP_FLAG_PERMANENT)
        sdp_subscriptions[id].flags |= AES67_SDP_FLAG_PERMANENT;

    if (flags & SDP_SUBSCRIBE_FLAG_NEW)
        memcpy(session_subscriptions[id], sdp.session_name,
               sizeof(sdp.session_name));
#if AES67_FAST_CONNECT_ENABLED
    if (flags & SDP_SUBSCRIBE_FLAG_STORE_FAST_CONNECT)
        sdp_store_fast_connect_info();
#endif
}

static void
sdp_unsubscribe(int32_t id, uint32_t flags) {
    assert(is_valid_receiver_id(id));

    // permanent entries should never expire
    if ((flags & SDP_SUBSCRIBE_FLAG_TIMED_OUT) &&
        (sdp_subscriptions[id].flags & AES67_SDP_FLAG_PERMANENT))
        return;

    memset(&sdp_subscriptions[id], 0, sizeof(sdp_subscriptions[id]));

    if (flags & SDP_SUBSCRIBE_FLAG_NEW)
        session_subscriptions[id][0] = '\0';
#if AES67_FAST_CONNECT_ENABLED
    if (flags & SDP_SUBSCRIBE_FLAG_STORE_FAST_CONNECT)
        sdp_store_fast_connect_info();
#endif
}

static int sdp_equal(const aes67_sdp_t &sdp1, const aes67_sdp_t &sdp2) {
    return (strcmp(sdp1.address, sdp2.address) == 0)
#ifdef __XC__
           && (strcmp(sdp1.__port, sdp2.__port) == 0)
#else
           && (strcmp(sdp1.port, sdp2.port) == 0)
#endif
           && (strcmp(sdp1.session_id, sdp2.session_id) == 0) &&
           (strcmp(sdp1.session_origin, sdp2.session_origin) == 0) &&
           (strcmp(sdp1.session_name, sdp2.session_name) == 0) &&
           (strcmp(sdp1.information, sdp2.information) == 0) &&
           (sdp1.payload_type == sdp2.payload_type) &&
           (sdp1.encoding == sdp2.encoding) &&
           (sdp1.sample_size == sdp2.sample_size) &&
           (sdp1.sample_rate == sdp2.sample_rate) &&
           (sdp1.channel_count == sdp2.channel_count) &&
           (sdp1.packet_duration == sdp2.packet_duration) &&
           (strcmp(sdp1.ptp_gmid, sdp2.ptp_gmid) == 0) &&
           (sdp1.ptp_domain == sdp2.ptp_domain) &&
           (sdp1.clock_offset == sdp2.clock_offset) &&
           (sdp1.framecount == sdp2.framecount) &&
           (sdp1.sync_time == sdp2.sync_time);
  // note: flags and timestamp are not compared
}

#if AES67_FAST_CONNECT_ENABLED
static size_t sdp_get_fast_connect_page_count(uint32_t valid_flags,
                                              uint32_t &page_size) {
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

static void sdp_start_fast_connect(client xtcp_if i_xtcp,
                                   chanend media_control,
                                   const uint32_t sap_timer_events) {
    union {
        aes67_sdp_fast_connect_t fc;
        uint8_t buffer[sizeof(aes67_sdp_fast_connect_t) / DEFAULT_PAGE_SIZE +
                       DEFAULT_PAGE_SIZE];
    } u;
    uint32_t page_size;

    if (fl_readDataPage(0, (uint8_t *)&u.fc) != 0 ||
        u.fc.magic != AES67_FAST_CONNECT_MAGIC)
        return;

    size_t page_count = sdp_get_fast_connect_page_count(u.fc.valid, page_size);

    if (page_size > DEFAULT_PAGE_SIZE)
        return;

    debug_printf("AES67 fast connect: reading magic %x valid %x page_count %d\n", u.fc.magic, u.fc.valid, page_count);

    uint8_t *p = (uint8_t *)&u.fc + page_size;
    for (size_t i = 1; i < page_count; i++) {
        if (fl_readDataPage(i, p) != 0)
            return;
        p += page_size;
    }

    for (size_t id = 0, sdp_index = 0; id < NUM_AES67_RECEIVERS; id++) {
        if ((u.fc.valid & BIT(id)) == 0)
            continue;

        _sap_handle_message(i_xtcp, media_control, id,
                            AES67_SAP_MESSAGE_ANNOUNCE,
                            u.fc.sdp[sdp_index],
                            SDP_SUBSCRIBE_FLAG_NEW | SDP_SUBSCRIBE_FLAG_START_FAST_CONNECT,
                            sap_timer_events);
        sdp_index++;
    }
}

static void sdp_store_fast_connect_info(void) {
    aes67_sdp_fast_connect_t fc;
    uint32_t page_size;

    if (fl_readDataPage(0, (uint8_t *)&fc) != 0 ||
        fc.magic == AES67_FAST_CONNECT_MAGIC)
        ;
    fl_eraseDataSector(0);

    memset(&fc, 0, sizeof(fc));

    fc.magic = AES67_FAST_CONNECT_MAGIC;

    for (size_t id = 0, i = 0; id < NUM_AES67_RECEIVERS; id++) {
        if (!sdp_is_subscribed(id))
            continue;

        fc.valid |= BIT(id);
        memcpy(&fc.sdp[i], &sdp_subscriptions[id], sizeof(aes67_sdp_t));
        fc.sdp[i].timestamp = 0; // the timestamp has no meaning across reboots
        i++;
    }

    size_t page_count = sdp_get_fast_connect_page_count(fc.valid, page_size);
    uint8_t *p = (uint8_t *)&fc;

    for (size_t i = 0; i < page_count; i++) {
        fl_writeDataPage(i, p);
        p += page_size;
    }

    debug_printf("AES67 fast connect: writing magic %x valid %x page_count %d\n", fc.magic, fc.valid, page_count);
}
#endif // AES67_FAST_CONNECT_ENABLED

static aes67_status_t _sap_handle_message(client xtcp_if i_xtcp,
                                          chanend media_control,
                                          int32_t id,
                                          aes67_sap_message_type_t message_type,
                                          const aes67_sdp_t &sdp,
                                          uint32_t flags,
                                          const uint32_t sap_timer_events) {
    aes67_stream_info_t stream_info;
    aes67_status_t status;

    if (!is_valid_receiver_id(id))
        return AES67_STATUS_INVALID_STREAM_ID;

#ifdef AES67_FAST_CONNECT_ENABLED
    flags |= SDP_SUBSCRIBE_FLAG_STORE_FAST_CONNECT;
#endif

    status = sdp_to_stream_info(stream_info, id, TRUE, sdp);
    if (status == AES67_STATUS_UNKNOWN_RTP_ENCODING) {
        const char *unsafe encoding = aes67_encoding_name(sdp.encoding);

        debug_printf("unknown RTP encoding %d/%s\n", sdp.encoding,
                     encoding != NULL ? encoding : (const char *)"<nil>");
    } else if (status != AES67_STATUS_OK) {
        debug_printf("failed to marshal stream info: %s\n",
                     aes67_status_to_string(status));
    }
    if (status != AES67_STATUS_OK)
        return status;

    if (message_type == AES67_SAP_MESSAGE_DELETE) {
        debug_printf("unsubscribing from stream %s\n", sdp.session_name);
        sdp_unsubscribe(id, flags);
        media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_UNSUBSCRIBE;
        media_control <: id;
    } else if (!sdp_is_subscribed(id)) {
        debug_printf("subscribing to stream %s channel count %d sample "
                     "size %d sample rate %d encoding %s offset %x:%x\n",
                     sdp.session_name, sdp.channel_count, sdp.sample_size,
                     sdp.sample_rate, aes67_encoding_name(sdp.encoding),
                     (uint32_t)((sdp.clock_offset >> 32) & 0xffffffff),
                     (uint32_t)((sdp.clock_offset >>  0) & 0xffffffff));
        sdp_subscribe(id, sdp, flags, sap_timer_events);
        media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_SUBSCRIBE;
        media_control <: stream_info;
    } else if (sdp_equal(sdp_subscriptions[id], sdp) == 0) {
        debug_printf("resubscribing to changed stream %s offset %x:%x\n",
                     sdp.session_name,
                     (uint32_t)((sdp.clock_offset >> 32) & 0xffffffff),
                     (uint32_t)((sdp.clock_offset >>  0) & 0xffffffff));
        sdp_subscribe(id, sdp, flags, sap_timer_events);
        media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_RESUBSCRIBE;
        media_control <: stream_info;
    }

    return AES67_STATUS_OK;
}

static aes67_status_t sap_handle_message(client xtcp_if i_xtcp,
                                         chanend media_control,
                                         uint8_t buf[len],
                                         size_t len,
                                         const uint32_t sap_timer_events) {
    aes67_status_t status;
    aes67_sap_t sap;
    aes67_sdp_t sdp;
    int32_t id;

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
    sap_enqueue_announcement(sap.message_type, sdp);

#pragma unsafe arrays
    for (id = 0; id < NUM_AES67_RECEIVERS; id++) {
        if (strcmp(sdp.session_name, session_subscriptions[id]) == 0)
            break;
    }

    if (id == NUM_AES67_RECEIVERS) {
        debug_printf("ignoring session %s, not subscribed\n", sdp.session_name);
        return AES67_STATUS_OK;
    }

    return _sap_handle_message(i_xtcp, media_control, id, sap.message_type, sdp,
                               0, sap_timer_events);
}

static void sap_handle_event(client xtcp_if i_xtcp,
                             chanend media_control,
                             xtcp_event_type_t event,
                             int fd,
                             const uint32_t sap_timer_events) {
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


        sap_handle_message(i_xtcp, media_control, buf, nrecv, sap_timer_events);
        break;
    case XTCP_IFUP:
        network_link_up = 1;
        pending_events |= BIT(AES67_EVENT_NETWORK_INFO);
#if AES67_FAST_CONNECT_ENABLED
        sdp_start_fast_connect(i_xtcp, media_control, sap_timer_events);
#endif
        break;
    case XTCP_IFDOWN:
        network_link_up = 0;
        pending_events |= BIT(AES67_EVENT_NETWORK_INFO);
        break;
    default:
        break;
    }
}

#if (NUM_AES67_SENDERS != 0)
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

static void
sap_advertise_senders(client xtcp_if i_xtcp,
                      int sap_tx_socket,
                      const aes67_time_source_info_t &time_source_info) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_SENDERS; id++) {
        // a non-empty session name indicates we should send an advertisement
        // payload_type == -1 indicates that we should withdarw the
        // advertisement

        if (sdp_is_advertising(id))
            sap_advertise_sender(i_xtcp, sap_tx_socket, sdp_advertisements[id],
                                 time_source_info);
    }
}
#endif // NUM_AES67_SENDERS != 0

static void aes67_periodic(client xtcp_if i_xtcp,
                           chanend media_control,
                           int sap_tx_socket,
                           unsigned sap_timeout,
                           const uint32_t sap_timer_events) {
#pragma unsafe arrays
    for (size_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        if (sap_timer_events > sdp_subscriptions[id].timestamp + AES67_SAP_TIMEOUT)
            sdp_unsubscribe(id, SDP_SUBSCRIBE_FLAG_TIMED_OUT);
    }

#if (NUM_AES67_SENDERS != 0)
    aes67_time_source_info_t time_source_info;

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO;
    media_control :> time_source_info;

    sap_advertise_senders(i_xtcp, sap_tx_socket, time_source_info);
#endif
}

static aes67_status_t start_streaming(chanend media_control,
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
    aes67_sdp_set_ipv4_port(sdp, AES67_RTP_PORT);

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

    status = sdp_to_stream_info(stream_info, id, FALSE, sdp);
    if (status != AES67_STATUS_OK)
        return status;

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_START_STREAMING;
    media_control <: stream_info;

    memcpy(&sdp_advertisements[id], &sdp, sizeof(sdp));

    return AES67_STATUS_OK;
}

static aes67_status_t stop_streaming(chanend media_control, int32_t id) {
    if (!is_valid_sender_id(id)) {
        return AES67_STATUS_INVALID_STREAM_ID;
    }

    if (!sdp_is_advertising(id)) {
        return AES67_STREAM_NOT_ADVERTISED;
    }

    sdp_invalidate_payload(sdp_advertisements[id]);

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_STOP_STREAMING;
    media_control <: id;

    return AES67_STATUS_OK;
}


/*
 * Host-facing status: endpoint info is the IO task's stream info plus the
 * session name the manager holds for the slot.
 */
static void stream_info_to_endpoint_info(aes67_stream_endpoint_info_t &info,
                                         const aes67_stream_info_t &stream_info,
                                         const char session_name[]) {
    size_t len;
    memset(&info, 0, sizeof(info));
    info.state = (uint8_t)stream_info.state;
    info.sample_size = stream_info.sample_size;
    info.channel_count = stream_info.channel_count;
    info.payload_type = stream_info.payload_type;
    info.sample_rate = stream_info.sample_rate;
    info.packet_time_us = stream_info.packet_time_us;
    memcpy(info.src_addr, stream_info.src_addr, sizeof(info.src_addr));
    memcpy(info.dest_addr, stream_info.dest_addr, sizeof(info.dest_addr));
    info.dest_port = stream_info.dest_port;
    info.clock_offset = stream_info.clock_offset;
    memcpy(info.gm_id, stream_info.gm_id.data, sizeof(info.gm_id));
#pragma unsafe arrays
    for (len = 0; session_name[len] && len < AES67_SESSION_NAME_MAX - 1; len++)
        info.session_name[len] = session_name[len];
    info.session_name[len] = '\0';
}

static void get_receiver_info_internal(chanend media_control,
                                       int32_t id,
                                       aes67_stream_endpoint_info_t &info) {
    aes67_stream_info_t stream_info;
    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_RECEIVER_STREAM_INFO;
    media_control <: id;
    media_control :> stream_info;
    stream_info_to_endpoint_info(info, stream_info, session_subscriptions[id]);
}

#if (NUM_AES67_SENDERS != 0)
static void get_sender_info_internal(chanend media_control,
                                     int32_t id,
                                     aes67_stream_endpoint_info_t &info) {
    aes67_stream_info_t stream_info;
    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_SENDER_STREAM_INFO;
    media_control <: id;
    media_control :> stream_info;
    stream_info_to_endpoint_info(info, stream_info, sdp_advertisements[id].session_name);
}
#endif

static void get_network_info_internal(client xtcp_if i_xtcp, aes67_network_info_t &info) {
    xtcp_ipconfig_t ipconfig = i_xtcp.get_netif_ipconfig(0);
    memset(&info, 0, sizeof(info));
    memcpy(info.ipaddr, ipconfig.ipaddr, sizeof(info.ipaddr));
    memcpy(info.netmask, ipconfig.netmask, sizeof(info.netmask));
    memcpy(info.gateway, ipconfig.gateway, sizeof(info.gateway));
    info.link_up = network_link_up;
}

/* Announcements are queued for the host; SAP repeats them, so a full queue drops. */
static void sap_enqueue_announcement(aes67_sap_message_type_t message_type,
                                     const aes67_sdp_t &sdp) {
    aes67_sap_announcement_t announcement;
    uint16_t dest_port = 0;
    size_t slot;
    if (sap_announcement_count == AES67_SAP_ANNOUNCEMENT_QUEUE_LEN)
        return;
    memset(&announcement, 0, sizeof(announcement));
    announcement.message_type = (uint8_t)message_type;
    announcement.channel_count = (uint8_t)sdp.channel_count;
    announcement.sample_size = (uint8_t)(sdp.sample_size / 8);
    announcement.payload_type = (uint8_t)sdp.payload_type;
    announcement.sample_rate = sdp.sample_rate;
    announcement.packet_time_us = (uint32_t)(sdp.packet_duration * 1000.0);
    aes67_sdp_get_ipv4_session_origin(sdp, announcement.origin_addr);
    aes67_sdp_get_ipv4_address(sdp, announcement.dest_addr);
    aes67_sdp_get_ipv4_port(sdp, dest_port);
    announcement.dest_port = dest_port;
    announcement.ptp_domain = (uint16_t)aes67_sdp_get_ptp_domain(sdp);
    announcement.clock_offset = sdp.clock_offset;
    aes67_sdp_get_ptp_gmid(sdp, announcement.gm_id);
    memcpy(announcement.session_name, sdp.session_name, sizeof(announcement.session_name));
    slot = (sap_announcement_head + sap_announcement_count) % AES67_SAP_ANNOUNCEMENT_QUEUE_LEN;
    memcpy(&sap_announcement_queue[slot], &announcement, sizeof(announcement));
    sap_announcement_count++;
    pending_events |= BIT(AES67_EVENT_SAP_ANNOUNCEMENT);
}

static int events_pending(void) {
    return pending_events != 0 || pending_receiver_events != 0 ||
           pending_sender_events != 0;
}

/* raised at the call sites so the interface array is not sliced */
#define NOTIFY_CLIENTS_IF_PENDING()                                            \
    do {                                                                       \
        if (events_pending())                                                  \
            for (size_t _i = 0; _i < num_aes67_clients; _i++)                  \
                i_aes67[_i].event_ready();                                     \
    } while (0)

/* interface array parameters cannot be handed to functions, so SDPs are copied in place */
#define COPY_SDP_STRING(sdp_struct, out, len, status)                          \
    do {                                                                       \
        char _local[AES67_SDP_MAX_LEN];                                        \
        size_t _n = 0;                                                         \
        status = aes67_sdp_to_string(sdp_struct, _local, sizeof(_local));      \
        if (status == AES67_STATUS_OK) {                                       \
            for (_n = 0; _n < len - 1 && _local[_n] != '\0'; _n++)             \
                out[_n] = _local[_n];                                          \
        }                                                                      \
        out[_n] = '\0';                                                        \
    } while (0)

/* Once a second: pick up clock, time source and stream state changes from the IO task. */
static void poll_status(chanend media_control) {
    aes67_media_clock_info_t media_clock_info;
    aes67_time_source_info_t time_source_info;
    aes67_stream_endpoint_info_t info;

    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_CLOCK_INFO;
    media_control :> media_clock_info;
    if (memcmp(&media_clock_info, &last_media_clock_info, sizeof(media_clock_info)) != 0) {
        memcpy(&last_media_clock_info, &media_clock_info, sizeof(media_clock_info));
        pending_events |= BIT(AES67_EVENT_MEDIA_CLOCK_INFO);
    }
    media_control <: (uint8_t)AES67_MEDIA_CONTROL_COMMAND_GET_TIME_SOURCE_INFO;
    media_control :> time_source_info;
    if (memcmp(&time_source_info, &last_time_source_info, sizeof(time_source_info)) != 0) {
        memcpy(&last_time_source_info, &time_source_info, sizeof(time_source_info));
        pending_events |= BIT(AES67_EVENT_TIME_SOURCE_INFO);
    }
#pragma unsafe arrays
    for (int32_t id = 0; id < NUM_AES67_RECEIVERS; id++) {
        get_receiver_info_internal(media_control, id, info);
        if (memcmp(&info, &last_receiver_info[id], sizeof(info)) != 0) {
            memcpy(&last_receiver_info[id], &info, sizeof(info));
            pending_receiver_events |= BIT(id);
        }
    }
#if (NUM_AES67_SENDERS != 0)
#pragma unsafe arrays
    for (int32_t id = 0; id < NUM_AES67_SENDERS; id++) {
        get_sender_info_internal(media_control, id, info);
        if (memcmp(&info, &last_sender_info[id], sizeof(info)) != 0) {
            memcpy(&last_sender_info[id], &info, sizeof(info));
            pending_sender_events |= BIT(id);
        }
    }
#endif
}

static int32_t first_set_bit(uint32_t bits) {
    for (int32_t i = 0; i < 32; i++)
        if (bits & BIT(i))
            return i;
    return -1;
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
    timer status_timer;
    int status_timeout;
    uint32_t sap_timer_events = 0;

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
    status_timer :> status_timeout;

    i_xtcp.join_multicast_group(sap_mcast_group);

    sap_rx_socket = i_xtcp.socket(XTCP_PROTOCOL_UDP);
    assert(sap_rx_socket >= 0);

    err = i_xtcp.listen(sap_rx_socket, AES67_SAP_PORT, any_addr);
    assert(err == XTCP_SUCCESS);

    sap_tx_socket = i_xtcp.socket(XTCP_PROTOCOL_UDP);
    assert(sap_tx_socket >= 0);

    err = i_xtcp.connect(sap_tx_socket, AES67_SAP_PORT, sap_mcast_group);
    assert(err == XTCP_SUCCESS);

    while (1) {
        select {
        case i_xtcp.event_ready():
            xtcp_event_type_t event;
            int32_t fd, id;

            event = i_xtcp.get_event(fd);
            sap_handle_event(i_xtcp, media_control, event, fd, sap_timer_events);
            NOTIFY_CLIENTS_IF_PENDING();
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
        case i_aes67[size_t i].handle_sap_message(aes67_sap_message_type_t message_type, int16_t id, const char sdp_string[])->aes67_status_t status:
            char sdp_string_copy[AES67_SDP_MAX_LEN];
            aes67_sdp_t sdp;
            size_t sdp_string_len;

            if (!is_valid_receiver_id(id)) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }

#pragma unsafe arrays
            for (sdp_string_len = 0; sdp_string[sdp_string_len]; sdp_string_len++)
                ;
            if (sdp_string_len >= AES67_SDP_MAX_LEN) {
                status = AES67_STATUS_OUT_OF_BUFFER_SPACE;
                break;
            }
            memcpy(sdp_string_copy, sdp_string, sdp_string_len + 1);

            status = aes67_sdp_parse_string(sdp_string_copy, sdp);
            if (status != AES67_STATUS_OK)
                break;

            status = _sap_handle_message(i_xtcp, media_control, id, message_type, sdp,
                                         SDP_SUBSCRIBE_FLAG_NEW | SDP_SUBSCRIBE_FLAG_PERMANENT, sap_timer_events);
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
        case i_aes67[size_t i].get_receiver_info(int16_t id, aes67_stream_endpoint_info_t &info)->aes67_status_t status:
            aes67_stream_endpoint_info_t local_info;
            if (!is_valid_receiver_id(id)) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }
            get_receiver_info_internal(media_control, id, local_info);
            info = local_info;
            status = AES67_STATUS_OK;
            break;
        case i_aes67[size_t i].get_sender_info(int16_t id, aes67_stream_endpoint_info_t &info)->aes67_status_t status:
#if (NUM_AES67_SENDERS != 0)
            if (!is_valid_sender_id(id)) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }
            aes67_stream_endpoint_info_t local_info;
            get_sender_info_internal(media_control, id, local_info);
            info = local_info;
            status = AES67_STATUS_OK;
#else
            status = AES67_STATUS_INVALID_STREAM_ID;
#endif
            break;
        case i_aes67[size_t i].get_receiver_sdp(int16_t id, char sdp[len], size_t len)->aes67_status_t status:
            if (!is_valid_receiver_id(id) || len == 0) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }
            if (sdp_is_subscribed(id)) {
                COPY_SDP_STRING(sdp_subscriptions[id], sdp, len, status);
            } else {
                sdp[0] = '\0';
                status = AES67_STATUS_OK;
            }
            break;
        case i_aes67[size_t i].get_sender_sdp(int16_t id, char sdp[len], size_t len)->aes67_status_t status:
#if (NUM_AES67_SENDERS != 0)
            if (!is_valid_sender_id(id) || len == 0) {
                status = AES67_STATUS_INVALID_STREAM_ID;
                break;
            }
            if (sdp_is_advertising(id)) {
                COPY_SDP_STRING(sdp_advertisements[id], sdp, len, status);
            } else {
                sdp[0] = '\0';
                status = AES67_STATUS_OK;
            }
#else
            status = AES67_STATUS_INVALID_STREAM_ID;
#endif
            break;
        case i_aes67[size_t i].get_network_info(aes67_network_info_t &info)->aes67_status_t status:
            aes67_network_info_t local_info;
            get_network_info_internal(i_xtcp, local_info);
            info = local_info;
            status = AES67_STATUS_OK;
            break;
        case i_aes67[size_t i].get_event_info()->aes67_event_info_t event_info:
            int32_t id;
            memset(&event_info, 0, sizeof(event_info));
            if (pending_events & BIT(AES67_EVENT_TIME_SOURCE_INFO)) {
                event_info.event_type = AES67_EVENT_TIME_SOURCE_INFO;
                event_info.u.time_source_info = last_time_source_info;
                pending_events &= ~BIT(AES67_EVENT_TIME_SOURCE_INFO);
            } else if (pending_events & BIT(AES67_EVENT_MEDIA_CLOCK_INFO)) {
                event_info.event_type = AES67_EVENT_MEDIA_CLOCK_INFO;
                event_info.u.media_clock_info = last_media_clock_info;
                pending_events &= ~BIT(AES67_EVENT_MEDIA_CLOCK_INFO);
            } else if (pending_events & BIT(AES67_EVENT_NETWORK_INFO)) {
                event_info.event_type = AES67_EVENT_NETWORK_INFO;
                get_network_info_internal(i_xtcp, event_info.u.network_info);
                pending_events &= ~BIT(AES67_EVENT_NETWORK_INFO);
            } else if ((id = first_set_bit(pending_receiver_events)) >= 0) {
                event_info.event_type = AES67_EVENT_RECEIVER_INFO;
                event_info.u.stream_endpoint_info.id = (int16_t)id;
                event_info.u.stream_endpoint_info.info = last_receiver_info[id];
                pending_receiver_events &= ~BIT(id);
#if (NUM_AES67_SENDERS != 0)
            } else if ((id = first_set_bit(pending_sender_events)) >= 0) {
                event_info.event_type = AES67_EVENT_SENDER_INFO;
                event_info.u.stream_endpoint_info.id = (int16_t)id;
                event_info.u.stream_endpoint_info.info = last_sender_info[id];
                pending_sender_events &= ~BIT(id);
#endif
            } else if (sap_announcement_count != 0) {
                event_info.event_type = AES67_EVENT_SAP_ANNOUNCEMENT;
                event_info.u.sap_announcement = sap_announcement_queue[sap_announcement_head];
                sap_announcement_head = (sap_announcement_head + 1) % AES67_SAP_ANNOUNCEMENT_QUEUE_LEN;
                sap_announcement_count--;
                if (sap_announcement_count == 0)
                    pending_events &= ~BIT(AES67_EVENT_SAP_ANNOUNCEMENT);
            } else {
                event_info.event_type = AES67_EVENT_NOOP;
            }
            // one event per call; re-arm the client while more are waiting
            NOTIFY_CLIENTS_IF_PENDING();
            break;
        case status_timer when timerafter(status_timeout) :> void:
            poll_status(media_control);
            NOTIFY_CLIENTS_IF_PENDING();
            status_timeout += AES67_STATUS_POLL_TIME;
            break;
        case sap_timer when timerafter(sap_timeout) :> void:
            aes67_periodic(i_xtcp, media_control, sap_tx_socket, sap_timeout, sap_timer_events);
            sap_timeout += AES67_SAP_PERIODIC_TIME;
            sap_timer_events++;
            break;
        }
    }
}
