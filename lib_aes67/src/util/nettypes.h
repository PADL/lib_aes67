// Copyright (c) 2014-2017, XMOS Ltd, All rights reserved

#pragma once

#include <inttypes.h>
#include <xtcp.h>

/* Useful types for network packet processing */

/* Host data types - little endian */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef unsigned int u32_t;
typedef long long u64_t;
typedef struct {
    uint8_t data[10];
} u80_t;
typedef struct {
    unsigned int data[3];
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
    long long ret = 0;
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
    for (int i = 0; i < 8; i++) {
        ret.data[i] = (x >> (56 - i * 8)) & 0xff;
    }
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
    uint8_t dest_addr[6];
    uint8_t src_addr[6];
    n16_t ethertype;
} ethernet_hdr_t;

typedef struct tagged_ethernet_hdr_t {
    uint8_t dest_addr[6];
    uint8_t src_addr[6];
    n32_t qtag;
    n16_t ethertype;
} tagged_ethernet_hdr_t;

/* IP headers */
typedef struct ip4_addr {
    unsigned long addr;
} ip4_addr_t;

static inline ip4_addr_t bswap_ip4addr(ip4_addr_t addr) {
    ip4_addr_t result;
    uint32_t net_addr = addr.addr;

    result.addr =
        ((net_addr & 0x000000FF) << 24) | ((net_addr & 0x0000FF00) << 8) |
        ((net_addr & 0x00FF0000) >> 8) | ((net_addr & 0xFF000000) >> 24);

    return result;
}

static inline ip4_addr_t ntoh_ip4addr(ip4_addr_t addr) {
    return bswap_ip4addr(addr);
}

static inline ip4_addr_t hton_ip4addr(ip4_addr_t addr) {
    return bswap_ip4addr(addr);
}
