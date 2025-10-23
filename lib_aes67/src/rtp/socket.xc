// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 PADL Software Pty Ltd

#include <stdlib.h> // atoi
#include <xassert.h> // assert
#include <string.h> // memcpy
#include <xtcp.h>

#include "rtp_protocol.h"

aes67_status_t
aes67_socket_recv(client xtcp_if xtcp,
                  const aes67_socket_t &sock,
                  uint8_t buffer[buffer_size],
                  size_t buffer_size,
                  size_t &received_len) {
    uint16_t port_number = 0;
    int nread;

    nread = xtcp.recvfrom(sock.fd, buffer, buffer_size, sock.src_addr, port_number);
    if (nread <= 0)
        return AES67_STATUS_SOCKET_ERROR;

    received_len = nread;

    return AES67_STATUS_OK;
}

aes67_status_t
aes67_socket_send(client xtcp_if xtcp,
                  const aes67_socket_t &sock,
                  const uint8_t buffer[len],
                  size_t len) {
    int nsent;

    nsent = xtcp.sendto(sock.fd, buffer, len, sock.dest_addr, sock.dest_port);
    if (nsent != (int)len)
        return AES67_STATUS_SOCKET_ERROR;

    return AES67_STATUS_OK;
}

void aes67_socket_close(client xtcp_if xtcp, aes67_socket_t &sock) {
    if (sock.joined_group) {
        xtcp.leave_multicast_group(sock.dest_addr);
        sock.joined_group = 0;
    }

    xtcp.close(sock.fd);
    sock.fd = -1;
}

static uint16_t calculate_ip_checksum(const void *unsafe header, size_t len) {
    uint32_t sum = 0;
    const uint16_t *data = (const uint16_t *)header;

    while (len > 1) {
        sum += ntohs(*data++);
        len -= 2;
    }

    if (len == 1)
        sum += *(const uint8_t *)data;

    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);

    return htons(~sum);
}

#define LL_IP4_MULTICAST_ADDR_0 0x01
#define LL_IP4_MULTICAST_ADDR_1 0x00
#define LL_IP4_MULTICAST_ADDR_2 0x5e

void ipv4_to_multicast_mac(xtcp_ipaddr_t ipv4_addr, uint8_t dest_mac[MACADDR_NUM_BYTES]) {
    dest_mac[0] = LL_IP4_MULTICAST_ADDR_0;
    dest_mac[1] = LL_IP4_MULTICAST_ADDR_1;
    dest_mac[2] = LL_IP4_MULTICAST_ADDR_2;
    dest_mac[3] = ipv4_addr[1] & 0x7F;
    dest_mac[4] = ipv4_addr[2];
    dest_mac[5] = ipv4_addr[3];
}

static aes67_status_t
validate_ethernet_header(aes67_rtp_packet_t &packet, const aes67_socket_t &sock) {
    uint8_t expected_dest_mac[MACADDR_NUM_BYTES];
    uint16_t ethertype_host;

    ipv4_to_multicast_mac(sock.dest_addr, expected_dest_mac);
    if (memcmp(packet.header.ip.eth.dest_addr, expected_dest_mac, MACADDR_NUM_BYTES) != 0)
        return AES67_STATUS_INVALID_ETH_DEST_MAC;

    ethertype_host = (packet.header.ip.eth.ethertype.data[0] << 8) | packet.header.ip.eth.ethertype.data[1];
    if (ethertype_host != ETH_TYPE_IP)
        return AES67_STATUS_INVALID_ETH_TYPE;

    return AES67_STATUS_OK;
}

static aes67_status_t
validate_ip_header(aes67_rtp_packet_t &packet,
                   const aes67_socket_t &sock,
                   uint32_t total_packet_len,
                   uint8_t &ip_header_length) {
    if ((packet.header.ip.version_ihl >> 4) != IP_VERSION_4)
        return AES67_STATUS_INVALID_IP_VERSION;

    // Get and validate IP header length (IHL field)
    uint8_t ihl = packet.header.ip.version_ihl & 0x0F;
    if (ihl != 5)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    ip_header_length = ihl * 4;

    if (packet.header.ip.protocol != IP_PROTO_UDP)
        return AES67_STATUS_INVALID_IP_PROTOCOL;

    // Validate IP checksum (before converting to host byte order)
    uint16_t received_checksum = packet.header.ip.checksum;
    uint16_t calculated_checksum;

    packet.header.ip.checksum = 0;  // Set to 0 for calculation

    unsafe {
        calculated_checksum = calculate_ip_checksum(&packet.header.ip.version_ihl, ip_header_length);
    }

    if (received_checksum != calculated_checksum)
        return AES67_STATUS_INVALID_IP_CHECKSUM;

    packet.header.ip.checksum = received_checksum;  // Restore for byte order conversion

    // Convert IP addresses to host byte order and validate
    packet.header.ip.src_ip = ntohl(packet.header.ip.src_ip);
    packet.header.ip.dest_ip = ntohl(packet.header.ip.dest_ip);

    // we allow expected_src_ip to be zero, in case we only want to validate
    // the destination IP address
    uint32_t expected_src_ip = xtcp_ipaddr_to_host_uint32(sock.src_addr);
    uint32_t expected_dest_ip = xtcp_ipaddr_to_host_uint32(sock.dest_addr);

    if (expected_src_ip && packet.header.ip.src_ip != expected_src_ip)
        return AES67_STATUS_INVALID_IP_SRC_ADDR;
    else if (packet.header.ip.dest_ip != expected_dest_ip)
        return AES67_STATUS_INVALID_IP_DEST_ADDR;

    // Convert other IP fields to host byte order for validation
    uint16_t ip_total_length = ntohs(packet.header.ip.total_length);
    uint16_t flags_fragment = ntohs(packet.header.ip.flags_fragment);

    if (ip_total_length < ip_header_length)
        return AES67_STATUS_BAD_PACKET_LENGTH;
    else if (ip_total_length + ETH_HEADER_LENGTH > total_packet_len)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Validate MF (More Fragments) flag is zero - no fragmentation allowed
    if (flags_fragment & 0x2000)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Validate fragment offset is zero - no fragmentation allowed
    if (flags_fragment & 0x1FFF)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Now safely store converted values
    packet.header.ip.total_length = ip_total_length;
    packet.header.ip.identification = ntohs(packet.header.ip.identification);
    packet.header.ip.flags_fragment = flags_fragment;
    packet.header.ip.checksum = ntohs(packet.header.ip.checksum);

    return AES67_STATUS_OK;
}

static aes67_status_t
validate_udp_header(aes67_rtp_packet_t &packet, const aes67_socket_t &sock) {
    // Convert UDP header to host byte order for validation
    uint16_t udp_src_port = ntohs(packet.header.src_port);
    uint16_t udp_dest_port = ntohs(packet.header.dest_port);
    uint16_t udp_length = ntohs(packet.header.length);
    uint16_t udp_checksum = ntohs(packet.header.checksum);

    // Validate UDP length (RFC 768: minimum 8 bytes for header)
    if (udp_length < UDP_HEADER_LENGTH)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Validate UDP length matches IP payload length
    uint16_t ip_payload_length = packet.header.ip.total_length - IP_HEADER_LENGTH;
    if (udp_length != ip_payload_length)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Validate UDP ports
    if (sock.src_port && udp_src_port != sock.src_port)
        return AES67_STATUS_INVALID_UDP_SRC_PORT;
    else if (udp_dest_port != sock.dest_port)
        return AES67_STATUS_INVALID_UDP_DEST_PORT;

    // Store converted values
    packet.header.src_port = udp_src_port;
    packet.header.dest_port = udp_dest_port;
    packet.header.length = udp_length;
    packet.header.checksum = udp_checksum;

    return AES67_STATUS_OK;
}

aes67_status_t
aes67_raw_send_rtp(const uint8_t src_mac_addr[MACADDR_NUM_BYTES],
                   streaming chanend c_eth_tx_hp,
                   const aes67_socket_t &sock,
                   aes67_rtp_packet_t &packet) {
    uint32_t rtp_packet_len = aes67_rtp_packet_length_rtp(packet);

    // Format Ethernet header
    ipv4_to_multicast_mac(sock.dest_addr, packet.header.ip.eth.dest_addr);
    memcpy(packet.header.ip.eth.src_addr, src_mac_addr, MACADDR_NUM_BYTES);
    packet.header.ip.eth.ethertype.data[0] = (ETH_TYPE_IP >> 8);
    packet.header.ip.eth.ethertype.data[1] = (ETH_TYPE_IP & 0xff);

    // Format IP header
    packet.header.ip.version_ihl = (IP_VERSION_4 << 4) | 5; // IPv4 with 20-byte header
    packet.header.ip.tos = DSCP_RTP << 2;
    packet.header.ip.total_length = htons(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + rtp_packet_len);
    packet.header.ip.identification = 0;
    packet.header.ip.flags_fragment = htons(0x4000); // Don't fragment flag
    packet.header.ip.ttl = 1;
    packet.header.ip.protocol = IP_PROTO_UDP;
    packet.header.ip.checksum = 0; // Set to 0 for checksum calculation

    // Set IP addresses
    packet.header.ip.src_ip = xtcp_ipaddr_to_network_uint32(sock.src_addr);
    packet.header.ip.dest_ip = xtcp_ipaddr_to_network_uint32(sock.dest_addr);

    // Calculate IP header checksum
    unsafe {
        packet.header.ip.checksum = calculate_ip_checksum(
            &packet.header.ip.version_ihl, IP_HEADER_LENGTH);
    }

    // Format UDP header
    packet.header.src_port = htons(sock.src_port);
    packet.header.dest_port = htons(sock.dest_port);
    packet.header.length = htons(UDP_HEADER_LENGTH + rtp_packet_len);
    packet.header.checksum = 0; // UDP checksum is optional for IPv4

    uint32_t total_packet_len = aes67_rtp_packet_length_raw(packet);

    unsafe {
        ethernet_send_hp_packet(c_eth_tx_hp, (uint8_t *)aes67_rtp_packet_start_raw(packet),
                                total_packet_len, ETHERNET_ALL_INTERFACES);
    }

    return AES67_STATUS_OK;
}

aes67_status_t
aes67_raw_recv_rtp(streaming unsafe chanend c_eth_rx_hp,
                   const aes67_socket_t &sock,
                   const ethernet_packet_info_t &packet_info,
                   aes67_rtp_packet_t &packet) {
    uint8_t ip_header_length;
    aes67_status_t status;

    if (packet_info.len < ETH_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH ||
        packet_info.len > ETH_MTU)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    status = validate_ethernet_header(packet, sock);
    if (status != AES67_STATUS_OK)
        return status;

    status = validate_ip_header(packet, sock, packet_info.len, ip_header_length);
    if (status != AES67_STATUS_OK)
        return status;

    status = validate_udp_header(packet, sock);
    if (status != AES67_STATUS_OK)
        return status;

    // validate UDP header does not extend past the end of the Ethernet packet
    // the UDP header length (packet.header.length) includes the size of the
    // UDP header as well as the UDP payload (i.e. not the Ethernet or IP headers)

    if (packet.header.length > packet_info.len - ETH_HEADER_LENGTH - IP_HEADER_LENGTH)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    packet.rtp_length = packet.header.length - UDP_HEADER_LENGTH;

    unsafe {
        status = aes67_rtp_parse(&packet);
        if (status != AES67_STATUS_OK)
            return status;
    }

    return AES67_STATUS_OK;
}
