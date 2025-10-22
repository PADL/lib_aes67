// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include "aes67_internal.h" // for HAVE_XXX

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>

#include <unistd.h>
#include <sys/types.h>

#if AES67_XMOS
#include <xtcp.h>
#else
#include <sys/time.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#endif

#include "rtp_protocol.h"
#include "aes67_utils.h"

enum { DO_BIND_SOCKET, DONT_BIND_SOCKET };

#if !AES67_XMOS

static int _create_socket(aes67_socket_t *sock,
                          int flags,
                          const char *address,
                          const char *port) {
    struct addrinfo hints, *res, *cur;
    int error = -1;
    int retval = -1;

    // Setup hints for getaddrinfo
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    error = getaddrinfo(address, port, &hints, &res);
    if (error || res == NULL) {
        aes67_warn("getaddrinfo failed: %s", gai_strerror(error));
        return error;
    }

    // Check each of the results
    cur = res;
    while (cur) {
        sock->fd = socket(cur->ai_family, cur->ai_socktype, cur->ai_protocol);

        if (sock->fd >= 0) {
            int one = 1;
            // These socket options help re-binding to a socket
            // after a previous process was killed

#ifdef SO_REUSEADDR
            if (setsockopt(sock->fd, SOL_SOCKET, SO_REUSEADDR, &one,
                           sizeof(one))) {
                perror("SO_REUSEADDR failed");
            }
#endif

#ifdef SO_REUSEPORT
            if (setsockopt(sock->fd, SOL_SOCKET, SO_REUSEPORT, &one,
                           sizeof(one))) {
                perror("SO_REUSEPORT failed");
            }
#endif

            if (flags == DONT_BIND_SOCKET ||
                bind(sock->fd, cur->ai_addr, cur->ai_addrlen) == 0) {
                // Success!
                memcpy(&sock->dest_addr, cur->ai_addr, cur->ai_addrlen);
                retval = 0;
                break;
            }

            close(sock->fd);
        }
        cur = cur->ai_next;
    }

    freeaddrinfo(res);

    return retval;
}


static int _sockaddr_len(sa_family_t family) {
    switch (family) {
    case AF_INET:
        return sizeof(struct sockaddr_in);
    case AF_INET6:
        return sizeof(struct sockaddr_in6);
    default:
        return -1;
    }
}

static int _get_interface_address(const char *ifname,
                                  sa_family_t family,
                                  struct sockaddr_storage *addr) {
    struct ifaddrs *addrs, *cur;
    int retval = -1;
    int foundAddress = FALSE;

    if (ifname == NULL || strlen(ifname) < 1) {
        aes67_error("No interface given");
        return -1;
    }

    // Get a linked list of all the interfaces
    retval = getifaddrs(&addrs);
    if (retval < 0) {
        return retval;
    }

    // Iterate through each of the interfaces
    for (cur = addrs; cur; cur = cur->ifa_next) {
        if (cur->ifa_addr && cur->ifa_addr->sa_family == family) {
            // FIXME: ignore link-local and temporary IPv6 addresses?
            if (strcmp(cur->ifa_name, ifname) == 0) {
                size_t addr_len = _sockaddr_len(cur->ifa_addr->sa_family);
                if (addr_len > 0) {
                    memcpy(addr, cur->ifa_addr, addr_len);
                    foundAddress = TRUE;
                }
                break;
            }
        }
    }

    freeifaddrs(addrs);

    return foundAddress ? 0 : -1;
}

static int _choose_best_interface(sa_family_t family, char *ifname) {
    struct ifaddrs *addrs, *cur;
    int retval = -1;

    // Get a linked list of all the interfaces
    retval = getifaddrs(&addrs);
    if (retval < 0) {
        return retval;
    }

    // Iterate through each of the interfaces
    for (cur = addrs; cur; cur = cur->ifa_next) {
        // Check if address is for right family
        if (cur->ifa_addr == NULL || cur->ifa_addr->sa_family != family)
            continue;

        // Ignore loopback and point-to-point network interfaces
        if ((cur->ifa_flags & IFF_LOOPBACK) ||
            (cur->ifa_flags & IFF_POINTOPOINT))
            continue;

        // Ignore interfaces that arn't up and running
        if (!(cur->ifa_flags & IFF_RUNNING))
            continue;

        // FIXME: find a way to avoid Wifi interfaces
        // FIXME: Could we prefer network interfaces that support IFCAP_AV?

        // Found one!
        retval = 0;
        strncpy(ifname, cur->ifa_name, IFNAMSIZ - 1);
        break;
    }

    freeifaddrs(addrs);

    return retval;
}

static void
format_sockaddr(struct sockaddr_storage *ss, char *dst, socklen_t size) {
    switch (ss->ss_family) {
    case AF_INET:
        inet_ntop(AF_INET, &(((struct sockaddr_in *)ss)->sin_addr), dst, size);
        break;

    case AF_INET6:
        inet_ntop(AF_INET6, &(((struct sockaddr_in6 *)ss)->sin6_addr), dst,
                  size);
        break;

    default:
        aes67_warn("Unknown socket address family: %d", ss->ss_family);
        strncpy(dst, "UNKNOWN", size);
        break;
    }
}

static int _lookup_interface(aes67_socket_t *sock, const char *ifname) {
    char chosen_ifname[IFNAMSIZ];
    char ipaddr[INET6_ADDRSTRLEN];
    int retval = -1;

    sock->if_index = 0;

    // Choose an interface, if none given
    if (ifname == NULL || strlen(ifname) == 0) {
        retval =
            _choose_best_interface(sock->dest_addr.ss_family, chosen_ifname);
        if (retval)
            return retval;
        ifname = chosen_ifname;
    }

    aes67_debug("Looking up interface: %s", ifname);

    // Check the interface exists, and store the interface index
    sock->if_index = if_nametoindex(ifname);
    if (sock->if_index == 0) {
        if (errno == ENXIO) {
            aes67_error("Network interface not found: %s", ifname);
            return -1;
        } else {
            aes67_error("Error looking up interface: %s", strerror(errno));
        }
    }

    // Get the address for the interface
    retval = _get_interface_address(ifname, sock->dest_addr.ss_family,
                                    &sock->src_addr);
    if (retval) {
        aes67_warn("Failed to get address of network interface");
    }

    format_sockaddr(&sock->src_addr, ipaddr, sizeof(ipaddr));
    aes67_info("Using network interface: %s (%s)", ifname, ipaddr);

    return retval;
}

static void _set_imr(aes67_socket_t *sock) {
    switch (sock->dest_addr.ss_family) {
    case AF_INET:
        memset(&sock->imr, 0, sizeof(sock->imr));
        memcpy(&sock->imr.imr_multiaddr,
               &((struct sockaddr_in *)&sock->dest_addr)->sin_addr,
               sizeof(struct in_addr));

        memcpy(&sock->imr.imr_interface,
               &((struct sockaddr_in *)&sock->src_addr)->sin_addr,
               sizeof(struct in_addr));
        break;

    case AF_INET6:
        memset(&sock->imr6, 0, sizeof(sock->imr6));
        memcpy(&sock->imr6.ipv6mr_multiaddr,
               &((struct sockaddr_in6 *)&sock->dest_addr)->sin6_addr,
               sizeof(struct in6_addr));

        sock->imr6.ipv6mr_interface = sock->if_index;

        break;

    default:
        aes67_error("Unknown socket address family: %d",
                    sock->dest_addr.ss_family);
        break;
    }
}

static int _join_group(aes67_socket_t *sock) {
    int retval = -1;

    _set_imr(sock);

    switch (sock->dest_addr.ss_family) {
    case AF_INET:
        retval = setsockopt(sock->fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &sock->imr,
                            sizeof(sock->imr));
        if (retval < 0)
            aes67_warn("IP_ADD_MEMBERSHIP failed: %s", strerror(errno));
        break;

    case AF_INET6:
        retval = setsockopt(sock->fd, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP,
                            &sock->imr6, sizeof(sock->imr6));
        if (retval < 0)
            aes67_warn("IPV6_ADD_MEMBERSHIP failed: %s", strerror(errno));
        break;
    }

    if (retval == 0)
        sock->joined_group = TRUE;

    return retval;
}

static int _leave_group(aes67_socket_t *sock) {
    int retval = -1;

    _set_imr(sock);

    switch (sock->dest_addr.ss_family) {
    case AF_INET:
        retval = setsockopt(sock->fd, IPPROTO_IP, IP_DROP_MEMBERSHIP,
                            &(sock->imr), sizeof(sock->imr));
        if (retval < 0)
            aes67_warn("IP_DROP_MEMBERSHIP failed: %s", strerror(errno));
        break;

    case AF_INET6:
        retval = setsockopt(sock->fd, IPPROTO_IPV6, IPV6_DROP_MEMBERSHIP,
                            &(sock->imr6), sizeof(sock->imr6));
        if (retval < 0)
            aes67_warn("IPV6_DROP_MEMBERSHIP failed: %s", strerror(errno));
        break;
    }

    return retval;
}

static int _set_multicast_interface(aes67_socket_t *sock) {
    int retval = -1;

    _set_imr(sock);

    switch (sock->dest_addr.ss_family) {
    case AF_INET:
        retval = setsockopt(sock->fd, IPPROTO_IP, IP_MULTICAST_IF,
                            &sock->imr.imr_interface,
                            sizeof(sock->imr.imr_interface));
        if (retval < 0)
            aes67_warn("IP_MULTICAST_IF failed: %s", strerror(errno));
        break;

    case AF_INET6:
        retval = setsockopt(sock->fd, IPPROTO_IPV6, IPV6_MULTICAST_IF,
                            &sock->imr6.ipv6mr_interface,
                            sizeof(sock->imr6.ipv6mr_interface));
        if (retval < 0)
            aes67_warn("IPV6_MULTICAST_IF failed: %s", strerror(errno));
        break;
    }

    return retval;
}

static int _set_multicast_hops(aes67_socket_t *sock, int hops) {
    int retval = -1;

    switch (sock->dest_addr.ss_family) {
    case AF_INET:
        retval = setsockopt(sock->fd, IPPROTO_IP, IP_MULTICAST_TTL, &hops,
                            sizeof(hops));
        if (retval < 0)
            aes67_warn("IP_MULTICAST_TTL failed: %s", strerror(errno));
        break;

    case AF_INET6:
        retval = setsockopt(sock->fd, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &hops,
                            sizeof(hops));
        if (retval < 0)
            aes67_warn("IPV6_MULTICAST_HOPS failed: %s", strerror(errno));
        break;
    }

    return retval;
}

static int _set_multicast_loopback(aes67_socket_t *sock, char loop) {
    int retval = -1;

    switch (sock->dest_addr.ss_family) {
    case AF_INET:
        retval = setsockopt(sock->fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop,
                            sizeof(loop));
        if (retval < 0)
            aes67_warn("IP_MULTICAST_LOOP failed: %s", strerror(errno));
        break;

    case AF_INET6:
        retval = setsockopt(sock->fd, IPPROTO_IPV6, IPV6_MULTICAST_LOOP, &loop,
                            sizeof(loop));
        if (retval < 0)
            aes67_warn("IPV6_MULTICAST_LOOP failed: %s", strerror(errno));
        break;
    }

    return retval;
}

aes67_status_t aes67_socket_open_recv(aes67_socket_t *sock,
                                      const char *address,
                                      const char *port,
                                      const char *ifname) {
    int is_multicast;

    // Initialise
    memset(sock, 0, sizeof(aes67_socket_t));

    aes67_info("Opening socket: %s/%s", address, port);
    if (_create_socket(sock, DO_BIND_SOCKET, address, port)) {
        aes67_error("Failed to open socket for receiving.");
        return AES67_STATUS_SOCKET_ERROR;
    }

    // Work out what interface to receive packets on
    _lookup_interface(sock, ifname);

    // Join multicast group ?
    is_multicast = _is_multicast(&sock->dest_addr);
    if (is_multicast == 1) {
        aes67_debug("Joining multicast group");
        if (_join_group(sock)) {
            aes67_socket_close(sock);
            return AES67_STATUS_SOCKET_ERROR;
        }

    } else if (is_multicast != 0) {
        aes67_warn("Error checking if address is multicast");
    }

    return AE67_STATUS_OK;
}

aes67_status_t aes67_socket_open_send(aes67_socket_t *sock,
                                      const char *address,
                                      const char *port,
                                      const char *ifname) {
    int is_multicast;

    // Initialise
    memset(sock, 0, sizeof(aes67_socket_t));

    aes67_info("Opening transmit socket: %s/%s", address, port);
    if (_create_socket(sock, DONT_BIND_SOCKET, address, port)) {
        aes67_error("Failed to open socket for sending.");
        return AES67_STATUS_SOCKET_ERROR;
    }

    // Work out what interface to send packets on
    _lookup_interface(sock, ifname);

    // Set multicast socket options?
    is_multicast = _is_multicast(&sock->dest_addr);
    if (is_multicast == 1) {
        _set_multicast_interface(sock);
        _set_multicast_hops(sock, 255);
        _set_multicast_loopback(sock, TRUE);
    } else if (is_multicast != 0) {
        aes67_warn("Error checking if address is multicast");
    }

    return AES67_STATUS_OK;
}


void aes67_socket_close(aes67_socket_t *sock) {
    // Drop Multicast membership
    if (sock->joined_group) {
        _leave_group(sock);
        sock->joined_group = 0;
    }

    // Close the sockets
    if (sock->fd >= 0) {
        close(sock->fd);
        sock->fd = -1;
    }
}

aes67_status_t aes67_socket_recv(const aes67_socket_t *sock, uint8_t *buffer, size_t buffer_size, size_t *received_len) {
    fd_set readfds;
    struct timeval timeout;
    int packet_len, retval;

    timeout.tv_sec = 60;
    timeout.tv_usec = 0;

    // Watch socket to see when it has input.
    FD_ZERO(&readfds);
    FD_SET(sock->fd, &readfds);
    retval = select(FD_SETSIZE, &readfds, NULL, NULL, &timeout);

    // Check return value
    if (retval == -1) {
        perror("select()");
        return AES67_STATUS_SOCKET_ERROR;

    } else if (retval == 0) {
        aes67_warn("Timed out waiting for packet after %ld seconds",
                   timeout.tv_sec);
        return AES67_STATUS_SOCKET_ERROR;
    }

    // Packet is waiting - read it in
    packet_len = recv(sock->fd, buffer, buffer_size, 0);

    if (packet_len > 0) {
        *received_len = packet_len;
        return AES67_STATUS_OK;
    }

    return AES67_STATUS_SOCKET_ERROR;
}

aes67_status_t aes67_socket_send(const aes67_socket_t *sock, const uint8_t *buffer, size_t len) {
    ssize_t bytes_sent = send(sock->fd, buffer, len, 0);

    if (bytes_sent == (ssize_t)len) {
        return AES67_STATUS_OK;
    } else {
        return AES67_STATUS_SOCKET_ERROR;
    }
}

#endif // !AES67_XMOS
