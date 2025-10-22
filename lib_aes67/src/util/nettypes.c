// Copyright (c) 2014-2017, XMOS Ltd, All rights reserved

#define BUILDING_NETTYPES 1
#include "nettypes.h"

extern inline n16_t hton16(u16_t x);
extern inline u16_t ntoh16(n16_t x);
extern inline n32_t hton32(u32_t x);
extern inline n64_t hton64(u64_t x);
extern inline n80_t hton80(u80_t x);

extern inline u16_t ntoh16(n16_t x);
extern inline u32_t ntoh32(n32_t x);
extern inline u64_t ntoh64(n64_t x);

uint16_t htons(uint16_t hostshort) {
    return __builtin_bswap16(hostshort);
}

uint16_t ntohs(uint16_t netshort) {
    return __builtin_bswap16(netshort);
}

uint32_t htonl(uint32_t hostlong) {
    return __builtin_bswap32(hostlong);
}

uint32_t ntohl(uint32_t netlong) {
    return __builtin_bswap32(netlong);
}
