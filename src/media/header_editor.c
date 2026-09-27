/*
 * header_editor.c - Cartridge header editing
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "header_editor.h"
#include "../rom/board.h"
#include "../system/vs_system.h"
#include "../ui/output_guard.h"
#include "../util/file_io.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

struct HeaderEditor {
    uint8_t *bytes;
    size_t size;
    char *path;
};

const char *const header_editor_labels[HEADER_FIELD_COUNT] = {"Format (1=iNES, 2=NES 2.0)",
                                                              "PRG ROM bytes",
                                                              "CHR ROM bytes",
                                                              "Mapper",
                                                              "Submapper",
                                                              "Mirroring (0=horizontal, 1=vertical, 2=four-screen)",
                                                              "Battery (0/1)",
                                                              "Trainer (0/1)",
                                                              "PRG RAM bytes",
                                                              "PRG NVRAM bytes",
                                                              "CHR RAM bytes",
                                                              "CHR NVRAM bytes",
                                                              "Console (0=NES, 1=VS, 2=PlayChoice, 3=extended)",
                                                              "Timing (0=NTSC, 1=PAL, 2=multi, 3=Dendy)",
                                                              "VS PPU code",
                                                              "VS hardware code",
                                                              "Extended console code",
                                                              "Miscellaneous ROM count",
                                                              "Default input / expansion device code",
                                                              "iNES byte 8 RAM units (board default applies)"};

static bool fail(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

HeaderEditor *header_editor_create(void) {
    return calloc(1, sizeof(HeaderEditor));
}

void header_editor_destroy(HeaderEditor *e) {
    if (e) {
        free(e->bytes);
        free(e->path);
        free(e);
    }
}

static bool decode_size(uint8_t low, unsigned high, size_t unit, uint64_t *out) {
    if (high != 15) {
        *out = ((uint64_t)high * 256 + low) * unit;
        return true;
    }
    unsigned exp = low >> 2;
    uint64_t mul = (low & 3u) * 2u + 1;
    if (exp >= sizeof(size_t) * CHAR_BIT || mul > (SIZE_MAX >> exp)) {
        return false;
    }
    *out = mul << exp;
    return true;
}

bool header_editor_open(HeaderEditor *e, const char *path, char *error, size_t size) {
    uint8_t *bytes = NULL;
    size_t count = 0;
    if (!e || !path || !*path) {
        return fail(error, size, "Choose an image file");
    }
    NesFileResult result = nes_file_read_all(path, 512u * 1024u * 1024u, &bytes, &count);
    if (result != NES_FILE_OK) {
        return fail(error, size, nes_file_result_message(result));
    }
    if (count < 16 || memcmp(bytes, "NES\032", 4)) {
        free(bytes);
        return fail(error, size, "Expected a complete 16-byte iNES or NES 2.0 header");
    }
    char *copy = malloc(strlen(path) + 1);
    if (!copy) {
        free(bytes);
        return fail(error, size, "Out of memory");
    }
    strcpy(copy, path);
    free(e->bytes);
    free(e->path);
    e->bytes = bytes;
    e->size = count;
    e->path = copy;
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool header_editor_metadata(const HeaderEditor *e, HeaderEditorMetadata *m) {
    if (!e || !e->bytes || !m) {
        return false;
    }
    const uint8_t *h = e->bytes;
    memset(m, 0, sizeof(*m));
    uint64_t *v = m->value;
    bool nes2 = (h[7] & 12) == 8;
    v[HEADER_FORMAT] = nes2 ? 2 : 1;
    v[HEADER_PRG] = (h[4] ? h[4] : 256u) * 16384u;
    v[HEADER_CHR] = h[5] * 8192u;
    if (nes2) {
        if (!decode_size(h[4], h[9] & 15, 16384, &v[HEADER_PRG])) {
            v[HEADER_PRG] = UINT64_MAX;
        }
        if (!decode_size(h[5], h[9] >> 4, 8192, &v[HEADER_CHR])) {
            v[HEADER_CHR] = UINT64_MAX;
        }
        v[HEADER_SUBMAPPER] = h[8] >> 4;
        for (unsigned i = 0; i < 4; ++i) {
            unsigned shift = (h[10 + i / 2] >> ((i % 2) * 4)) & 15;
            v[HEADER_PRG_RAM + i] = shift ? UINT64_C(64) << shift : 0;
        }
        v[HEADER_TIMING] = h[12] & 3;
        v[HEADER_MISC_ROMS] = h[14] & 3;
        v[HEADER_INPUT] = h[15] & 63;
        if ((h[7] & 3) == 1) {
            v[HEADER_VS_PPU] = h[13] & 15;
            v[HEADER_VS_HARDWARE] = h[13] >> 4;
        }
        if ((h[7] & 3) == 3) {
            v[HEADER_EXTENDED_CONSOLE] = h[13] & 15;
            if ((h[13] & 15) == 1) {
                v[HEADER_VS_HARDWARE] = h[13] >> 4;
            }
        }
    } else {
        v[HEADER_TIMING] = h[9] & 1;
        v[HEADER_LEGACY_RAM] = h[8];
    }
    iNESHeader header;
    memcpy(&header, h, 16);
    v[HEADER_MAPPER] = (unsigned)rom_mapper_number(&header);
    v[HEADER_MIRROR] = (h[6] & 8) ? 2 : h[6] & 1;
    v[HEADER_BATTERY] = (h[6] >> 1) & 1;
    v[HEADER_TRAINER] = (h[6] >> 2) & 1;
    v[HEADER_CONSOLE] = h[7] & 3;
    return true;
}

static bool encode_size(uint64_t bytes, unsigned unit, uint8_t *low, unsigned *high) {
    if (bytes % unit == 0 && bytes / unit < 3840) {
        *low = (uint8_t)(bytes / unit);
        *high = (unsigned)(bytes / unit) >> 8;
        return true;
    }
    for (unsigned exp = 0; exp < 64; ++exp) {
        for (unsigned m = 0; m < 4; ++m) {
            uint64_t mul = m * 2u + 1;
            if (mul <= (UINT64_MAX >> exp) && (mul << exp) == bytes) {
                *low = (uint8_t)(exp * 4 + m);
                *high = 15;
                return true;
            }
        }
    }
    return false;
}

bool header_editor_encode(const HeaderEditorMetadata *m, iNESHeader *out, char *error, size_t size) {
    if (!m || !out) {
        return fail(error, size, "Missing header draft");
    }
    const uint64_t *v = m->value;
    bool n = v[HEADER_FORMAT] == 2;
    if (v[HEADER_FORMAT] != 1 && !n) {
        return fail(error, size, "Format must be 1 or 2");
    }
    const uint64_t limits[HEADER_FIELD_COUNT] = {
        2, SIZE_MAX, SIZE_MAX, n ? 4095 : 255, 15, 2, 1, 1, 2097152, 2097152, 2097152, 2097152, 3, 3, 15, 15, 15,
        3, 63,       255};
    for (unsigned i = 0; i < HEADER_FIELD_COUNT; ++i) {
        if (v[i] > limits[i]) {
            return fail(error, size, "Header field is out of range");
        }
    }
    if (!v[HEADER_PRG]) {
        return fail(error, size, "PRG ROM cannot be empty");
    }
    uint8_t h[16] = {'N', 'E', 'S', 26};
    if (n) {
        unsigned p, c;
        if (!encode_size(v[HEADER_PRG], 16384, &h[4], &p) || !encode_size(v[HEADER_CHR], 8192, &h[5], &c)) {
            return fail(error, size, "ROM size has no NES 2.0 encoding");
        }
        h[9] = (uint8_t)(p | (c << 4));
        h[8] = (uint8_t)((v[HEADER_SUBMAPPER] << 4) | (v[HEADER_MAPPER] >> 8));
        for (unsigned i = 0; i < 4; ++i) {
            uint64_t bytes = v[HEADER_PRG_RAM + i];
            unsigned shift = 0;
            if (bytes) {
                for (shift = 1; shift < 16 && (UINT64_C(64) << shift) != bytes; ++shift) {
                }
            }
            if (shift == 16) {
                return fail(error, size, "RAM must be zero or 64 shifted by 1..15 bytes");
            }
            h[10 + i / 2] |= (uint8_t)(shift << ((i % 2) * 4));
        }
        if ((v[HEADER_PRG_NVRAM] || v[HEADER_CHR_NVRAM]) && !v[HEADER_BATTERY]) {
            return fail(error, size, "NVRAM requires the battery flag");
        }
        if (v[HEADER_LEGACY_RAM]) {
            return fail(error, size, "Clear legacy RAM units before selecting NES 2.0");
        }
        h[12] = (uint8_t)v[HEADER_TIMING];
        h[14] = (uint8_t)v[HEADER_MISC_ROMS];
        h[15] = (uint8_t)v[HEADER_INPUT];
        if (v[HEADER_CONSOLE] == 1) {
            h[13] = (uint8_t)(v[HEADER_VS_PPU] | (v[HEADER_VS_HARDWARE] << 4));
        } else if (v[HEADER_VS_PPU] ||
                   (v[HEADER_VS_HARDWARE] && !(v[HEADER_CONSOLE] == 3 && v[HEADER_EXTENDED_CONSOLE] == 1))) {
            return fail(error, size, "VS fields require the VS console");
        }
        if (v[HEADER_CONSOLE] == 3) {
            h[13] = (uint8_t)(v[HEADER_EXTENDED_CONSOLE] | (v[HEADER_VS_HARDWARE] << 4));
        } else if (v[HEADER_EXTENDED_CONSOLE]) {
            return fail(error, size, "Extended subtype requires extended console");
        }
    } else {
        if (v[HEADER_PRG] % 16384 || v[HEADER_PRG] > 4194304 || v[HEADER_CHR] % 8192 || v[HEADER_CHR] > 2088960) {
            return fail(error, size, "iNES requires 1..256 PRG and 0..255 CHR banks");
        }
        if (v[HEADER_SUBMAPPER] || v[HEADER_TIMING] > 1 || v[HEADER_CONSOLE] > 2) {
            return fail(error, size, "Selected metadata requires NES 2.0");
        }
        for (unsigned i = HEADER_PRG_RAM; i <= HEADER_CHR_NVRAM; ++i) {
            if (v[i]) {
                return fail(error, size, "Explicit RAM sizes require NES 2.0");
            }
        }
        for (unsigned i = HEADER_VS_PPU; i <= HEADER_INPUT; ++i) {
            if (v[i]) {
                return fail(error, size, "Selected metadata requires NES 2.0");
            }
        }
        h[4] = (uint8_t)(v[HEADER_PRG] / 16384);
        h[5] = (uint8_t)(v[HEADER_CHR] / 8192);
        h[8] = (uint8_t)v[HEADER_LEGACY_RAM];
        h[9] = (uint8_t)v[HEADER_TIMING];
    }
    h[6] = (uint8_t)((v[HEADER_MAPPER] << 4) | (v[HEADER_MIRROR] == 2 ? 8 : v[HEADER_MIRROR]) |
                     (v[HEADER_BATTERY] << 1) | (v[HEADER_TRAINER] << 2));
    h[7] = (uint8_t)((v[HEADER_MAPPER] & 240) | (n ? 8 : 0) | v[HEADER_CONSOLE]);
    memcpy(out, h, 16);
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool header_editor_validate(const HeaderEditor *e, const HeaderEditorMetadata *m, iNESHeader *h, char *error,
                            size_t size) {
    if (!e || !e->bytes) {
        return fail(error, size, "Choose an image first");
    }
    if (!header_editor_encode(m, h, error, size)) {
        return false;
    }
    const uint64_t *v = m->value;
    size_t offset = 16 + (v[HEADER_TRAINER] ? 512 : 0);
    if (offset > e->size || v[HEADER_PRG] > e->size - offset ||
        v[HEADER_CHR] > e->size - offset - (size_t)v[HEADER_PRG]) {
        return fail(error, size, "Truncated trainer, PRG ROM or CHR ROM payload");
    }
    if (v[HEADER_TRAINER] != ((e->bytes[6] >> 2) & 1)) {
        return fail(error, size, "Changing trainer presence would reinterpret preserved payload bytes");
    }
    if (v[HEADER_CONSOLE] == 3 && v[HEADER_EXTENDED_CONSOLE] > 2 && v[HEADER_EXTENDED_CONSOLE] != 4 &&
        v[HEADER_EXTENDED_CONSOLE] != 12) {
        return fail(error, size, "Unsupported extended console type");
    }
    RomRamSizes ram;
    if (rom_ram_sizes(h, &ram) != 0 || (!v[HEADER_CHR] && !ram.chr_ram && !ram.chr_nvram && !board_handles_header(h))) {
        return fail(error, size, "Cartridge declares no CHR ROM or CHR RAM");
    }
    VsRomConfig config;
    NesRegion region = v[HEADER_TIMING] == 1   ? NES_REGION_PAL
                       : v[HEADER_TIMING] == 3 ? NES_REGION_DENDY
                                               : NES_REGION_NTSC;
    if (!vs_decode_header(h, rom_mapper_number(h), (size_t)v[HEADER_PRG], (size_t)v[HEADER_CHR], region, &config, error,
                          size)) {
        return false;
    }
    if (v[HEADER_MISC_ROMS] && e->size - offset - (size_t)v[HEADER_PRG] - (size_t)v[HEADER_CHR] == 0) {
        return fail(error, size, "Miscellaneous ROM count requires trailing payload");
    }
    return true;
}

bool header_editor_save_copy(const HeaderEditor *e, const HeaderEditorMetadata *m, const char *path,
                             const struct FrontendExecutionRuntime *execution, char *error, size_t size) {
    iNESHeader h;
    if (!header_editor_validate(e, m, &h, error, size)) {
        return false;
    }
    const char *protected_path = e->path;
    if (!frontend_output_path_allowed(path, execution, &protected_path, 1, error, size)) {
        return false;
    }
    NesFileTransaction *tx = NULL;
    NesFileResult result = nes_file_transaction_begin(path, e->size, &tx);
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(tx, &h, 16);
    }
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(tx, e->bytes + 16, e->size - 16);
    }
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_commit(&tx);
    }
    nes_file_transaction_abort(&tx);
    if (result != NES_FILE_OK) {
        return fail(error, size, nes_file_result_message(result));
    }
    return true;
}
