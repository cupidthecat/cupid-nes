/* Movie ROM identity. SPDX-License-Identifier: GPL-3.0-or-later */
#include "md5.h"
#include <string.h>

static uint32_t rotate(uint32_t value, unsigned bits) {
    return (value << bits) | (value >> (32 - bits));
}

static void transform(uint32_t state[4], const uint8_t block[64]) {
    static const uint32_t constants[64] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
        0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
        0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
        0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
        0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
        0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
    static const unsigned shifts[4][4] = {{7, 12, 17, 22}, {5, 9, 14, 20}, {4, 11, 16, 23}, {6, 10, 15, 21}};
    uint32_t words[16];
    for (unsigned i = 0; i < 16; ++i) {
        words[i] = (uint32_t)block[i * 4] | (uint32_t)block[i * 4 + 1] << 8 | (uint32_t)block[i * 4 + 2] << 16 |
                   (uint32_t)block[i * 4 + 3] << 24;
    }

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    for (unsigned i = 0; i < 64; ++i) {
        uint32_t function;
        unsigned index;
        if (i < 16) {
            function = (b & c) | (~b & d);
            index = i;
        } else if (i < 32) {
            function = (d & b) | (~d & c);
            index = (5 * i + 1) & 15;
        } else if (i < 48) {
            function = b ^ c ^ d;
            index = (3 * i + 5) & 15;
        } else {
            function = c ^ (b | ~d);
            index = (7 * i) & 15;
        }

        uint32_t next = b + rotate(a + function + constants[i] + words[index], shifts[i / 16][i & 3]);
        a = d;
        d = c;
        c = b;
        b = next;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
}

void nes_md5_init(NesMd5 *hash) {
    if (hash) {
        *hash = (NesMd5){.state = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476}};
    }
}

bool nes_md5_update(NesMd5 *hash, const void *data, size_t size) {
    if (!hash || (!data && size) || hash->used >= sizeof(hash->block)) {
        return false;
    }

    const uint8_t *bytes = data;
    hash->bytes += size;
    while (size) {
        size_t count = sizeof(hash->block) - hash->used;
        if (count > size) {
            count = size;
        }

        memcpy(hash->block + hash->used, bytes, count);
        hash->used += count;
        bytes += count;
        size -= count;
        if (hash->used == sizeof(hash->block)) {
            transform(hash->state, hash->block);
            hash->used = 0;
        }
    }

    return true;
}

bool nes_md5_final(const NesMd5 *hash, uint8_t digest[16]) {
    if (!hash || !digest || hash->used >= sizeof(hash->block)) {
        return false;
    }

    NesMd5 copy = *hash;
    uint8_t tail[128] = {0};
    uint64_t bits = copy.bytes * 8;
    size_t count = copy.used < 56 ? 64 - copy.used : 128 - copy.used;
    tail[0] = 0x80;
    for (unsigned i = 0; i < 8; ++i) {
        tail[count - 8 + i] = (uint8_t)(bits >> (8 * i));
    }

    if (!nes_md5_update(&copy, tail, count)) {
        return false;
    }

    for (unsigned i = 0; i < 16; ++i) {
        digest[i] = (uint8_t)(copy.state[i / 4] >> ((i & 3) * 8));
    }

    return true;
}
