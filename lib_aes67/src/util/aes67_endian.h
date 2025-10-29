// Copyright (c) 2014-2017, XMOS Ltd, All rights reserved

#pragma once

#include <inttypes.h>
#include <ethernet.h>
#include <xtcp.h>

/* Host data types - little endian */
typedef uint8_t u8_t;

typedef uint16_t u16_t;

typedef uint32_t u32_t;

typedef int64_t u64_t;

typedef struct {
    uint8_t data[10];
} u80_t;

typedef struct {
    uint32_t data[3];
} u96_t;

/* Network data types - big endian  */
typedef uint8_t n8_t;

typedef struct {
    uint8_t data[2];
} n16_t;

typedef struct {
    uint8_t data[4];
} n32_t;

typedef struct {
    uint8_t data[8];
} n64_t;

typedef struct {
    uint8_t data[10];
} n80_t;

typedef struct {
    uint8_t data[12];
} n96_t;

#ifdef __XC__
[[always_inline]]
#endif
static inline n16_t
hton16(u16_t x) {
    n16_t ret;

    ret.data[0] = x >> 8;
    ret.data[1] = (x & 0xff);

    return ret;
}

#ifdef __XC__
[[always_inline]]
#endif
static inline u16_t
ntoh16(n16_t x) {
    return ((x.data[0] << 8) | x.data[1]);
}

#ifdef __XC__
[[always_inline]]
#endif
static inline u32_t
ntoh32(n32_t x) {
    return ((x.data[0] << 24) | x.data[1] << 16 | x.data[2] << 8 | x.data[3]);
}

#ifdef __XC__
[[always_inline]]
#endif
static inline u64_t
ntoh64(n64_t x) {
    int64_t ret = 0;

    for (int i = 0; i < 8; i++)
        ret = (ret << 8) + x.data[i];

    return ret;
}

#ifdef __XC__
[[always_inline]]
#endif
static inline n64_t
hton64(u64_t x) {
    n64_t ret;

    for (int i = 0; i < 8; i++)
        ret.data[i] = (x >> (56 - i * 8)) & 0xff;

    return ret;
}

#ifdef __XC__
[[always_inline]]
#endif
static inline n32_t
hton32(u32_t x) {
    n32_t ret;

    ret.data[0] = ((x >> 24) & 0xff);
    ret.data[1] = ((x >> 16) & 0xff);
    ret.data[2] = ((x >> 8) & 0xff);
    ret.data[3] = ((x >> 0) & 0xff);

    return ret;
}

#ifdef __XC__
[[always_inline]]
#endif
static inline n80_t
hton80(u80_t x) {
    n80_t ret;

    for (int i = 0; i < 10; i++)
        ret.data[i] = x.data[9 - i];

    return ret;
}

/* Ethernet headers */
typedef struct ethernet_hdr_t {
    uint8_t dest_addr[MACADDR_NUM_BYTES];
    uint8_t src_addr[MACADDR_NUM_BYTES];
    n16_t ethertype;
} ethernet_hdr_t;

typedef struct tagged_ethernet_hdr_t {
    uint8_t dest_addr[MACADDR_NUM_BYTES];
    uint8_t src_addr[MACADDR_NUM_BYTES];
    n32_t qtag;
    n16_t ethertype;
} tagged_ethernet_hdr_t;

/* Standard network byte order conversion functions */
#if defined(__XC__) || defined(BUILDING_AES67_ENDIAN_C)
uint16_t htons(uint16_t hostshort);
uint16_t ntohs(uint16_t netshort);
uint32_t htonl(uint32_t hostlong);
uint32_t ntohl(uint32_t netlong);
#else
static inline uint16_t htons(uint16_t hostshort) {
#if WORDS_BIGENDIAN
    return hostshort;
#else
    return __builtin_bswap16(hostshort);
#endif
}
static inline uint16_t ntohs(uint16_t netshort) {
#if WORDS_BIGENDIAN
    return netshort;
#else
    return __builtin_bswap16(netshort);
#endif
}
static inline uint32_t htonl(uint32_t hostlong) {
#if WORDS_BIGENDIAN
    return hostlong;
#else
    return __builtin_bswap32(hostlong);
#endif
}
static inline uint32_t ntohl(uint32_t netlong) {
#if WORDS_BIGENDIAN
    return netlong;
#else
    return __builtin_bswap32(netlong);
#endif
}
#endif

/* IP headers, from lwIP */
typedef struct ip4_addr {
    // NB: needs to remain unsigned long to link with lwIP
    unsigned long addr;
} ip4_addr_t;

static inline ip4_addr_t ntoh_ip4addr(ip4_addr_t addr) {
    ip4_addr_t result;

    result.addr = ntohl(addr.addr);

    return result;
}

static inline ip4_addr_t hton_ip4addr(ip4_addr_t addr) {
    ip4_addr_t result;

    result.addr = htonl(addr.addr);

    return result;
}

static inline uint32_t xtcp_ipaddr_to_host_uint32(xtcp_ipaddr_t addr) {
    return (addr[0] << 24) | (addr[1] << 16) | (addr[2] << 8) | addr[3];
}

static inline uint32_t xtcp_ipaddr_to_network_uint32(xtcp_ipaddr_t addr) {
    return htonl(xtcp_ipaddr_to_host_uint32(addr));
}

// xtcp_ipaddr_t is an octet array in big-endian form, i.e. as we would read it
static inline void host_uint32_to_xtcp_ipaddr(xtcp_ipaddr_t addr, uint32_t ip) {
    addr[0] = (ip >> 24) & 0xFF;
    addr[1] = (ip >> 16) & 0xFF;
    addr[2] = (ip >> 8) & 0xFF;
    addr[3] = ip & 0xFF;
}

static inline void network_uint32_to_xtcp_ipaddr(xtcp_ipaddr_t addr, uint32_t ip) {
    host_uint32_to_xtcp_ipaddr(addr, ntohl(ip));
}
