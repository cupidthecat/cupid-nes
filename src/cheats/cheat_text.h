/*
 * cheat_text.h - UTF-8 validation for cheat files and catalogs
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_CHEAT_TEXT_H
#define CUPID_CHEAT_TEXT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static bool cheat_text_valid_utf8(const uint8_t *data, size_t size) {
    for (size_t i = 0; i < size;) {
        uint8_t c = data[i++];
        if (!c) {
            return false;
        }

        if (c < 0x80) {
            continue;
        }

        unsigned extra;
        uint32_t code;
        if ((c & 0xE0) == 0xC0) {
            extra = 1;
            code = c & 0x1F;
            if (code < 2) {
                return false;
            }
        } else if ((c & 0xF0) == 0xE0) {
            extra = 2;
            code = c & 0x0F;
        } else if ((c & 0xF8) == 0xF0) {
            extra = 3;
            code = c & 0x07;
        } else {
            return false;
        }

        if (i + extra > size) {
            return false;
        }

        for (unsigned j = 0; j < extra; ++j) {
            uint8_t tail = data[i++];
            if ((tail & 0xC0) != 0x80) {
                return false;
            }

            code = (code << 6) | (tail & 0x3F);
        }

        if ((extra == 2 && code < 0x800) || (extra == 3 && code < 0x10000) || code > 0x10FFFF ||
            (code >= 0xD800 && code <= 0xDFFF)) {
            return false;
        }
    }

    return true;
}

#endif
