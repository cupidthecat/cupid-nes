/* SHA-1 image identity. SPDX-License-Identifier: GPL-3.0-or-later */
#include "sha1.h"
#include <stdint.h>
#include <string.h>

static uint32_t rotate(uint32_t value, unsigned bits) {
    return (value << bits) | (value >> (32 - bits));
}

static void block(uint32_t hash[5], const uint8_t bytes[64]) {
    uint32_t words[80];
    for (unsigned i = 0; i < 16; ++i)
        words[i] = (uint32_t)bytes[i*4] << 24 | (uint32_t)bytes[i*4+1] << 16
                 | (uint32_t)bytes[i*4+2] << 8 | bytes[i*4+3];
    for (unsigned i = 16; i < 80; ++i)
        words[i] = rotate(words[i-3] ^ words[i-8] ^ words[i-14] ^ words[i-16], 1);
    uint32_t a = hash[0], b = hash[1], c = hash[2], d = hash[3], e = hash[4];
    for (unsigned i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20) { f = (b & c) | (~b & d); k = 0x5a827999; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ed9eba1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8f1bbcdc; }
        else { f = b ^ c ^ d; k = 0xca62c1d6; }
        uint32_t next = rotate(a, 5) + f + e + k + words[i];
        e = d; d = c; c = rotate(b, 30); b = a; a = next;
    }
    hash[0] += a; hash[1] += b; hash[2] += c; hash[3] += d; hash[4] += e;
}

void nes_sha1(const void *data, size_t size, char hex[41]) {
    uint32_t hash[5] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0};
    const uint8_t *bytes = data;
    uint64_t bits = (uint64_t)size * 8;
    while (size >= 64) { block(hash, bytes); bytes += 64; size -= 64; }
    uint8_t tail[128] = {0};
    if (size) memcpy(tail, bytes, size);
    tail[size] = 0x80;
    unsigned length = size >= 56 ? 128 : 64;
    for (unsigned i = 0; i < 8; ++i) tail[length - 1 - i] = (uint8_t)(bits >> (i*8));
    block(hash, tail);
    if (length == 128) block(hash, tail + 64);
    const char digits[] = "0123456789abcdef";
    for (unsigned i = 0; i < 40; ++i) hex[i] = digits[(hash[i/8] >> (28 - (i%8)*4)) & 15];
    hex[40] = 0;
}
