// Copyright 2016-2021 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#pragma once

#include <inttypes.h>

typedef uintptr_t xc_ptr;

// Note that this function is marked as const to avoid the XC
// parallel usage checks, this is only really going to work if this
// is the *only* way the array a is accessed (and everything else uses
// the xc_ptr)
static inline xc_ptr long_long_array_to_xc_ptr(const uint64_t a[]) {
    xc_ptr x;

    asm("mov %0, %1" : "=r"(x) : "r"(a));

    return x;
}

static inline xc_ptr long_array_to_xc_ptr(const uint32_t a[]) {
    xc_ptr x;

    asm("mov %0, %1" : "=r"(x) : "r"(a));

    return x;
}

static inline xc_ptr short_array_to_xc_ptr(const uint16_t a[]) {
    xc_ptr x;

    asm("mov %0, %1" : "=r"(x) : "r"(a));

    return x;
}

static inline xc_ptr byte_array_to_xc_ptr(const uint8_t a[]) {
    xc_ptr x;

    asm("mov %0, %1" : "=r"(x) : "r"(a));

    return x;
}

#define write_byte_via_xc_ptr_indexed(p, i, x)                                 \
    asm volatile("st8 %0, %1[%2]" ::"r"(x), "r"(p), "r"(i))
#define write_short_via_xc_ptr_indexed(p, i, x)                                \
    asm volatile("st16 %0, %1[%2]" ::"r"(x), "r"(p), "r"(i))
#define write_long_via_xc_ptr_indexed(p, i, x)                                 \
    asm volatile("stw %0, %1[%2]" ::"r"(x), "r"(p), "r"(i))
#define write_long_long_via_xc_ptr_indexed(p, i, hi, lo)                       \
    asm volatile("std %0, %1, %2[%3]" ::"r"(hi), "r"(lo), "r"(p), "r"(i))

#define write_byte_via_xc_ptr(p, x) write_byte_via_xc_ptr_indexed(p, 0, x)
#define write_short_via_xc_ptr(p, x) write_short_via_xc_ptr_indexed(p, 0, x)
#define write_long_via_xc_ptr(p, x)                                            \
    asm volatile("stw %0, %1[0]" ::"r"(x), "r"(p))
#define write_long_long_via_xc_ptr(p, x)                                       \
    asm volatile("std %0, %1, %2[0]" ::"r"(x), "r"(p))

#define read_byte_via_xc_ptr_indexed(x, p, i)                                  \
    asm("ld8u %0, %1[%2]" : "=r"(x) : "r"(p), "r"(i));
#define read_short_via_xc_ptr_indexed(x, p, i)                                 \
    asm("ld16s %0, %1[%2]" : "=r"(x) : "r"(p), "r"(i));
#define read_long_via_xc_ptr_indexed(x, p, i)                                  \
    asm("ldw %0, %1[%2]" : "=r"(x) : "r"(p), "r"(i));
#define read_long_long_via_xc_ptr_indexed(lo, hi, p, i)                        \
    asm("ldd %0, %1, %2[%3]" : "=r"(hi), "=r"(lo) : "r"(p), "r"(i));

#define read_byte_via_xc_ptr(x, p) read_byte_via_xc_ptr_indexed(x, p, 0)
#define read_short_via_xc_ptr(x, p) read_short_via_xc_ptr_indexed(x, p, 0)
#define read_long_via_xc_ptr(x, p) asm("ldw %0, %1[0]" : "=r"(x) : "r"(p));
