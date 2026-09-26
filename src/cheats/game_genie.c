/*
 * game_genie.c - Shared NES Game Genie encoder and decoder
 *
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cheats.h"

#include <string.h>

static const char alphabet[] = "APZLGITYEOXUKSVN";

static const uint8_t address_bits[] = {14, 13, 12, 19, 22, 21, 20, 7, 10, 9, 8, 15, 18, 17, 16};

static const uint8_t value_bits[2][8] = {{3, 6, 5, 4, 23, 2, 1, 0}, {3, 6, 5, 4, 31, 2, 1, 0}};

static const uint8_t compare_bits[] = {27, 30, 29, 28, 23, 26, 25, 24};

static uint32_t decode_bits(uint32_t raw, const uint8_t *bits, size_t count) {
    uint32_t value = 0;
    for (size_t i = 0; i < count; ++i) {
        value = (value << 1) | ((raw >> bits[i]) & 1u);
    }

    return value;
}

static uint32_t encode_bits(uint32_t value, const uint8_t *bits, size_t count) {
    uint32_t raw = 0;
    for (size_t i = 0; i < count; ++i) {
        raw |= ((value >> (count - i - 1)) & 1u) << bits[i];
    }

    return raw;
}

CheatResult cheats_game_genie_decode(const char *text, CheatRecord *out) {
    if (!text || !out) {
        return CHEAT_INVALID_ARGUMENT;
    }

    CheatRecord decoded = {0};
    uint32_t raw = 0;
    size_t length = 0;
    for (; length < 8 && text[length]; ++length) {
        unsigned char c = (unsigned char)text[length];
        if (c >= 'a' && c <= 'z') {
            c -= 'a' - 'A';
        }

        const char *letter = strchr(alphabet, c);
        if (!letter) {
            return CHEAT_INVALID_CODE;
        }

        raw |= (uint32_t)(letter - alphabet) << (length * 4);
        decoded.code[length] = (char)c;
    }

    if ((length != 6 && length != 8) || text[length]) {
        return CHEAT_INVALID_CODE;
    }

    decoded.format = CHEAT_FORMAT_GAME_GENIE;
    decoded.enabled = true;
    decoded.has_compare = length == 8;
    decoded.address = (uint16_t)(0x8000u | decode_bits(raw, address_bits, 15));
    decoded.value = (uint8_t)decode_bits(raw, value_bits[decoded.has_compare], 8);
    decoded.compare = decoded.has_compare ? (uint8_t)decode_bits(raw, compare_bits, 8) : 0;
    *out = decoded;
    return CHEAT_OK;
}

CheatResult cheats_game_genie_encode(uint32_t address, uint32_t value, int compare, char *out, size_t capacity) {
    if (!out) {
        return CHEAT_INVALID_ARGUMENT;
    }

    if (address < 0x8000 || address > 0xFFFF || value > 0xFF || compare < -1 || compare > 0xFF) {
        return CHEAT_OUT_OF_RANGE;
    }

    bool has_compare = compare >= 0;
    size_t length = has_compare ? 8 : 6;
    if (capacity <= length) {
        return CHEAT_OUT_OF_RANGE;
    }

    uint32_t raw = encode_bits(address & 0x7FFF, address_bits, 15) | encode_bits(value, value_bits[has_compare], 8);
    if (has_compare) {
        /* The third letter's high bit distinguishes canonical eight-letter
         * codes. Decoding also accepts historical aliases with this bit clear. */
        raw |= (1u << 11) | encode_bits((uint32_t)compare, compare_bits, 8);
    }

    char encoded[9] = {0};
    for (size_t i = 0; i < length; ++i) {
        encoded[i] = alphabet[(raw >> (i * 4)) & 15u];
    }

    memcpy(out, encoded, length + 1);
    return CHEAT_OK;
}
