/*
 * fm2_write.c - bounded FM2/FM3 serializer
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "fm2_internal.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t *data;
    size_t capacity;
    size_t offset;
    bool measure_only;
} NesFm2Writer;

enum { FM3_PROJECT_HEADER_SIZE = 12 + NES_FM3_PROJECT_MODULE_COUNT * 4 };

static NesFm2Result writer_reserve(NesFm2Writer *writer, size_t bytes, NesFm2Diagnostic *diagnostic) {
    size_t end;

    if (!nes_fm2__checked_add(writer->offset, bytes, &end)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0, "serialized movie size overflows size_t");
    }
    if (!writer->measure_only && end > writer->capacity) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0,
                             "serialized movie exceeds destination capacity");
    }
    return NES_FM2_OK;
}

static NesFm2Result writer_bytes(NesFm2Writer *writer, const void *data, size_t size, NesFm2Diagnostic *diagnostic) {
    NesFm2Result result = writer_reserve(writer, size, diagnostic);

    if (result != NES_FM2_OK) {
        return result;
    }
    if (!writer->measure_only && size != 0) {
        memcpy(writer->data + writer->offset, data, size);
    }
    writer->offset += size;
    return NES_FM2_OK;
}

static NesFm2Result writer_text(NesFm2Writer *writer, const char *text, NesFm2Diagnostic *diagnostic) {
    return writer_bytes(writer, text, strlen(text), diagnostic);
}

static NesFm2Result writer_number_line(NesFm2Writer *writer, const char *key, uint32_t value,
                                       NesFm2Diagnostic *diagnostic) {
    char line[64];
    int length = snprintf(line, sizeof(line), "%s %" PRIu32 "\n", key, value);

    if (length < 0 || (size_t)length >= sizeof(line)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0, "numeric FM2 header line is too long");
    }
    return writer_bytes(writer, line, (size_t)length, diagnostic);
}

static NesFm2Result writer_signed_line(NesFm2Writer *writer, const char *key, int32_t value,
                                       NesFm2Diagnostic *diagnostic) {
    char line[64];
    int length = snprintf(line, sizeof(line), "%s %" PRId32 "\n", key, value);

    if (length < 0 || (size_t)length >= sizeof(line)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0, "signed FM2 header line is too long");
    }
    return writer_bytes(writer, line, (size_t)length, diagnostic);
}

static NesFm2Result writer_field_line(NesFm2Writer *writer, const char *key, const char *value,
                                      NesFm2Diagnostic *diagnostic) {
    NesFm2Result result;

    result = writer_text(writer, key, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_bytes(writer, " ", 1, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_text(writer, value ? value : "", diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    return writer_bytes(writer, "\n", 1, diagnostic);
}

static bool line_value_valid(const char *value) {
    if (!value) {
        return false;
    }
    for (const unsigned char *at = (const unsigned char *)value; *at; ++at) {
        if (*at == '\r' || *at == '\n') {
            return false;
        }
    }
    return true;
}

static bool extension_key_valid(const char *key) {
    if (!key || !*key) {
        return false;
    }
    for (const unsigned char *at = (const unsigned char *)key; *at; ++at) {
        if (*at == ' ' || *at == '\t' || *at == '\r' || *at == '\n') {
            return false;
        }
    }
    return true;
}

static bool port_valid(NesFm2Port port) {
    return port >= NES_FM2_PORT_NONE && port <= NES_FM2_PORT_ZAPPER;
}

static NesFm2Result validate_string_list(const NesFm2StringList *list, const char *name, NesFm2Diagnostic *diagnostic) {
    if (list->count != 0 && !list->items) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "%s list has no item array", name);
    }
    for (size_t i = 0; i < list->count; ++i) {
        if (!line_value_valid(list->items[i])) {
            return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, i, "%s contains a null string or newline",
                                 name);
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result validate_blob(const NesFm2Blob *blob, const char *name, NesFm2Diagnostic *diagnostic) {
    if (blob->size != 0 && !blob->data) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "%s has a nonzero size and no data", name);
    }
    return NES_FM2_OK;
}

static NesFm2Result validate_movie(const NesFm2Movie *movie, NesFm2Container container, NesFm2Diagnostic *diagnostic) {
    NesFm2Result result;

    if (container < NES_FM2_TEXT || container > NES_FM3_PROJECT) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "unknown FM2 output container");
    }
    if (movie->version != 3) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0, "only FM2 version 3 can be serialized");
    }
    if (!line_value_valid(movie->rom_filename ? movie->rom_filename : "")) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "ROM filename contains a newline");
    }
    for (size_t i = 0; i < 3; ++i) {
        if (!port_valid(movie->ports[i])) {
            return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0, "movie contains an unsupported input device");
        }
    }
    if (movie->ports[2] != NES_FM2_PORT_NONE) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0, "FC expansion input devices are unsupported");
    }
    if (movie->frame_count != 0 && !movie->frames) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0,
                             "movie has a nonzero frame count and no frame array");
    }
    result = validate_string_list(&movie->comments, "comment", diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = validate_string_list(&movie->subtitles, "subtitle", diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = validate_blob(&movie->savestate, "savestate", diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = validate_blob(&movie->saveram, "saveram", diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    if (movie->extension_count != 0 && !movie->extensions) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0,
                             "movie has extension fields and no extension array");
    }
    for (size_t i = 0; i < movie->extension_count; ++i) {
        const NesFm2HeaderField *field = &movie->extensions[i];

        if (!extension_key_valid(field->key) || !line_value_valid(field->value)) {
            return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, i,
                                 "extension header field is not representable as FM2 text");
        }
    }

    if (container != NES_FM3_PROJECT) {
        return NES_FM2_OK;
    }
    if (!movie->project_present) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "FM3 export requires project module data");
    }
    if (!movie->project_timeline_valid) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0,
                             "FM3 project modules are stale after timeline edits");
    }
    if (movie->project_version != 3) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0, "only FM3 project version 3 can be serialized");
    }
    if ((movie->project_saved_modules & ~0x3fu) != 0) {
        return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, 0, "FM3 saved-module mask contains unknown bits");
    }
    if (movie->frame_count > UINT32_MAX) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, 0, movie->frame_count,
                             "FM3 length exceeds its 32-bit header field");
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        if (movie->project_modules[i].size != 0 && !movie->project_modules[i].data) {
            return nes_fm2__fail(diagnostic, NES_FM2_INVALID_ARGUMENT, 0, i,
                                 "FM3 project module has a nonzero size and no data");
        }
    }
    return NES_FM2_OK;
}

static size_t base64_encoded_size(size_t size, bool *ok) {
    size_t groups;
    size_t encoded;

    *ok = nes_fm2__checked_add(size, 2, &groups);
    if (!*ok) {
        return 0;
    }
    groups /= 3;
    *ok = nes_fm2__checked_mul(groups, 4, &encoded);
    return *ok ? encoded : 0;
}

static NesFm2Result writer_base64(NesFm2Writer *writer, const uint8_t *data, size_t size,
                                  NesFm2Diagnostic *diagnostic) {
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    bool ok;
    size_t encoded = base64_encoded_size(size, &ok);
    NesFm2Result result;

    if (!ok) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0, "base64 output size overflows size_t");
    }
    if (writer->measure_only) {
        result = writer_reserve(writer, encoded, diagnostic);
        if (result == NES_FM2_OK) {
            writer->offset += encoded;
        }
        return result;
    }

    for (size_t at = 0; at < size; at += 3) {
        size_t remain = size - at;
        uint32_t bits = (uint32_t)data[at] << 16;
        char out[4];

        if (remain > 1) {
            bits |= (uint32_t)data[at + 1] << 8;
        }
        if (remain > 2) {
            bits |= data[at + 2];
        }
        out[0] = alphabet[(bits >> 18) & 0x3f];
        out[1] = alphabet[(bits >> 12) & 0x3f];
        out[2] = remain > 1 ? alphabet[(bits >> 6) & 0x3f] : '=';
        out[3] = remain > 2 ? alphabet[bits & 0x3f] : '=';
        result = writer_bytes(writer, out, sizeof(out), diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result writer_blob_line(NesFm2Writer *writer, const char *key, const uint8_t *data, size_t size,
                                     NesFm2Diagnostic *diagnostic) {
    NesFm2Result result;

    result = writer_text(writer, key, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_text(writer, " base64:", diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_base64(writer, data, size, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    return writer_bytes(writer, "\n", 1, diagnostic);
}

static NesFm2Result writer_guid_line(NesFm2Writer *writer, const uint8_t guid[16], NesFm2Diagnostic *diagnostic) {
    static const char hex[] = "0123456789ABCDEF";
    char value[37];
    size_t at = 0;

    for (size_t i = 0; i < 16; ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10) {
            value[at++] = '-';
        }
        value[at++] = hex[guid[i] >> 4];
        value[at++] = hex[guid[i] & 0x0f];
    }
    value[at] = '\0';
    return writer_field_line(writer, "guid", value, diagnostic);
}

static NesFm2Result writer_common_header(NesFm2Writer *writer, const NesFm2Movie *movie, NesFm2Container container,
                                         NesFm2Diagnostic *diagnostic) {
    NesFm2Result result;

#define WRITE_RESULT(call)                                                                                             \
    do {                                                                                                               \
        result = (call);                                                                                               \
        if (result != NES_FM2_OK) {                                                                                    \
            return result;                                                                                             \
        }                                                                                                              \
    } while (0)

    WRITE_RESULT(writer_number_line(writer, "version", movie->version, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "emuVersion", movie->emu_version, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "rerecordCount", movie->rerecord_count, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "palFlag", movie->pal ? 1u : 0u, diagnostic));
    WRITE_RESULT(writer_field_line(writer, "romFilename", movie->rom_filename ? movie->rom_filename : "", diagnostic));
    WRITE_RESULT(writer_blob_line(writer, "romChecksum", movie->rom_md5, NES_FM2_MD5_SIZE, diagnostic));
    WRITE_RESULT(writer_guid_line(writer, movie->guid, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "fourscore", movie->fourscore ? 1u : 0u, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "microphone", movie->microphone ? 1u : 0u, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "port0", (uint32_t)movie->ports[0], diagnostic));
    WRITE_RESULT(writer_number_line(writer, "port1", (uint32_t)movie->ports[1], diagnostic));
    WRITE_RESULT(writer_number_line(writer, "port2", (uint32_t)movie->ports[2], diagnostic));
    WRITE_RESULT(writer_number_line(writer, "FDS", movie->fds ? 1u : 0u, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "NewPPU", movie->new_ppu ? 1u : 0u, diagnostic));
    WRITE_RESULT(writer_number_line(writer, "RAMInitOption", movie->ram_init_option, diagnostic));
    WRITE_RESULT(writer_signed_line(writer, "RAMInitSeed", (int32_t)movie->ram_init_seed, diagnostic));

    for (size_t i = 0; i < movie->comments.count; ++i) {
        WRITE_RESULT(writer_field_line(writer, "comment", movie->comments.items[i], diagnostic));
    }
    for (size_t i = 0; i < movie->subtitles.count; ++i) {
        WRITE_RESULT(writer_field_line(writer, "subtitle", movie->subtitles.items[i], diagnostic));
    }
    for (size_t i = 0; i < movie->extension_count; ++i) {
        WRITE_RESULT(writer_field_line(writer, movie->extensions[i].key, movie->extensions[i].value, diagnostic));
    }

    if (container != NES_FM2_TEXT) {
        WRITE_RESULT(writer_text(writer, "binary 1\n", diagnostic));
    }
    if (movie->savestate.size != 0) {
        WRITE_RESULT(writer_blob_line(writer, "savestate", movie->savestate.data, movie->savestate.size, diagnostic));
    }
    if (movie->saveram.size != 0) {
        WRITE_RESULT(writer_blob_line(writer, "saveram", movie->saveram.data, movie->saveram.size, diagnostic));
    }
    if (container == NES_FM3_PROJECT) {
        WRITE_RESULT(writer_number_line(writer, "length", (uint32_t)movie->frame_count, diagnostic));
    }

#undef WRITE_RESULT
    return NES_FM2_OK;
}

static NesFm2Result writer_text_pad(NesFm2Writer *writer, uint8_t pad, NesFm2Diagnostic *diagnostic) {
    static const char labels[] = "RLDUTSBA";
    char text[8];

    for (size_t i = 0; i < sizeof(text); ++i) {
        uint8_t mask = (uint8_t)(1u << (7u - (unsigned)i));
        text[i] = (pad & mask) ? labels[i] : '.';
    }
    return writer_bytes(writer, text, sizeof(text), diagnostic);
}

static NesFm2Result writer_text_zapper(NesFm2Writer *writer, const NesFm2Zapper *zapper, NesFm2Diagnostic *diagnostic) {
    char text[96];
    int length = snprintf(text, sizeof(text), "%03u %03u %u %u %" PRIu64, (unsigned)zapper->x, (unsigned)zapper->y,
                          (unsigned)zapper->button, (unsigned)zapper->bogo, zapper->zaphit);

    if (length < 0 || (size_t)length >= sizeof(text)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0, "zapper input text is too long");
    }
    return writer_bytes(writer, text, (size_t)length, diagnostic);
}

static NesFm2Result writer_text_frame(NesFm2Writer *writer, const NesFm2Movie *movie, const NesFm2Frame *frame,
                                      size_t frame_index, NesFm2Diagnostic *diagnostic) {
    char command[16];
    int length = snprintf(command, sizeof(command), "|%u|", (unsigned)frame->commands);
    NesFm2Result result;

    if (length < 0 || (size_t)length >= sizeof(command)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, frame_index, "movie command text is too long");
    }
    result = writer_bytes(writer, command, (size_t)length, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }

    if (movie->fourscore) {
        for (size_t pad = 0; pad < 4; ++pad) {
            result = writer_text_pad(writer, frame->pads[pad], diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
            result = writer_bytes(writer, "|", 1, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
        }
    } else {
        for (size_t port = 0; port < 2; ++port) {
            if (movie->ports[port] == NES_FM2_PORT_GAMEPAD) {
                result = writer_text_pad(writer, frame->pads[port], diagnostic);
            } else if (movie->ports[port] == NES_FM2_PORT_ZAPPER) {
                result = writer_text_zapper(writer, &frame->zappers[port], diagnostic);
            } else {
                result = NES_FM2_OK;
            }
            if (result != NES_FM2_OK) {
                return result;
            }
            result = writer_bytes(writer, "|", 1, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
        }
    }
    return writer_bytes(writer, "|\n", 2, diagnostic);
}

static NesFm2Result writer_text_frames(NesFm2Writer *writer, const NesFm2Movie *movie, NesFm2Diagnostic *diagnostic) {
    for (size_t i = 0; i < movie->frame_count; ++i) {
        NesFm2Result result = writer_text_frame(writer, movie, &movie->frames[i], i, diagnostic);

        if (result != NES_FM2_OK) {
            return result;
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result writer_u64le(NesFm2Writer *writer, uint64_t value, NesFm2Diagnostic *diagnostic) {
    uint8_t bytes[8];

    for (unsigned int i = 0; i < 8; ++i) {
        bytes[i] = (uint8_t)(value >> (i * 8));
    }
    return writer_bytes(writer, bytes, sizeof(bytes), diagnostic);
}

static NesFm2Result writer_binary_frame(NesFm2Writer *writer, const NesFm2Movie *movie, const NesFm2Frame *frame,
                                        NesFm2Diagnostic *diagnostic) {
    NesFm2Result result = writer_bytes(writer, &frame->commands, 1, diagnostic);

    if (result != NES_FM2_OK) {
        return result;
    }
    if (movie->fourscore) {
        return writer_bytes(writer, frame->pads, 4, diagnostic);
    }
    for (size_t port = 0; port < 2; ++port) {
        if (movie->ports[port] == NES_FM2_PORT_GAMEPAD) {
            result = writer_bytes(writer, &frame->pads[port], 1, diagnostic);
        } else if (movie->ports[port] == NES_FM2_PORT_ZAPPER) {
            const NesFm2Zapper *zapper = &frame->zappers[port];
            uint8_t prefix[4] = {zapper->x, zapper->y, zapper->button, zapper->bogo};

            result = writer_bytes(writer, prefix, sizeof(prefix), diagnostic);
            if (result == NES_FM2_OK) {
                result = writer_u64le(writer, zapper->zaphit, diagnostic);
            }
        }
        if (result != NES_FM2_OK) {
            return result;
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result writer_binary_frames(NesFm2Writer *writer, const NesFm2Movie *movie, NesFm2Diagnostic *diagnostic) {
    NesFm2Result result = writer_bytes(writer, "|", 1, diagnostic);

    if (result != NES_FM2_OK) {
        return result;
    }
    for (size_t i = 0; i < movie->frame_count; ++i) {
        result = writer_binary_frame(writer, movie, &movie->frames[i], diagnostic);
        if (result != NES_FM2_OK) {
            if (diagnostic) {
                diagnostic->frame_index = i;
            }
            return result;
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result writer_u32le(NesFm2Writer *writer, uint32_t value, NesFm2Diagnostic *diagnostic) {
    uint8_t bytes[4];

    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value >> 16);
    bytes[3] = (uint8_t)(value >> 24);
    return writer_bytes(writer, bytes, sizeof(bytes), diagnostic);
}

static NesFm2Result writer_project(NesFm2Writer *writer, const NesFm2Movie *movie, NesFm2Diagnostic *diagnostic) {
    uint32_t offsets[NES_FM3_PROJECT_MODULE_COUNT];
    size_t next;
    NesFm2Result result;

    if (!nes_fm2__checked_add(writer->offset, FM3_PROJECT_HEADER_SIZE, &next) || next > UINT32_MAX) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0,
                             "FM3 project header offset exceeds 32-bit format limits");
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        offsets[i] = (uint32_t)next;
        if (!nes_fm2__checked_add(next, movie->project_modules[i].size, &next)) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, i,
                                 "FM3 project module offsets overflow size_t");
        }
        if (i + 1 < NES_FM3_PROJECT_MODULE_COUNT && next > UINT32_MAX) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, i,
                                 "FM3 project module offset exceeds 32-bit format limits");
        }
    }
    if (next > UINT32_MAX) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, writer->offset, 0,
                             "FM3 project size exceeds 32-bit format limits");
    }

    result = writer_u32le(writer, movie->project_version, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_u32le(writer, movie->project_saved_modules, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_u32le(writer, NES_FM3_PROJECT_MODULE_COUNT, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        result = writer_u32le(writer, offsets[i], diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        result = writer_bytes(writer, movie->project_modules[i].data, movie->project_modules[i].size, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result emit_movie(NesFm2Writer *writer, const NesFm2Movie *movie, NesFm2Container container,
                               NesFm2Diagnostic *diagnostic) {
    NesFm2Result result = validate_movie(movie, container, diagnostic);

    if (result != NES_FM2_OK) {
        return result;
    }
    result = writer_common_header(writer, movie, container, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    if (container == NES_FM2_TEXT) {
        return writer_text_frames(writer, movie, diagnostic);
    }
    result = writer_binary_frames(writer, movie, diagnostic);
    if (result != NES_FM2_OK || container == NES_FM2_BINARY) {
        return result;
    }
    return writer_project(writer, movie, diagnostic);
}

NesFm2Result nes_fm2__measure_movie(const NesFm2Movie *movie, NesFm2Container container, size_t *size,
                                    NesFm2Diagnostic *diagnostic) {
    NesFm2Writer writer;
    NesFm2Result result;

    writer.data = NULL;
    writer.capacity = 0;
    writer.offset = 0;
    writer.measure_only = true;
    result = emit_movie(&writer, movie, container, diagnostic);
    if (result == NES_FM2_OK) {
        *size = writer.offset;
    }
    return result;
}

NesFm2Result nes_fm2__write_movie(const NesFm2Movie *movie, NesFm2Container container, uint8_t *destination,
                                  size_t capacity, size_t *written, NesFm2Diagnostic *diagnostic) {
    NesFm2Writer writer;
    NesFm2Result result;

    writer.data = destination;
    writer.capacity = capacity;
    writer.offset = 0;
    writer.measure_only = false;
    result = emit_movie(&writer, movie, container, diagnostic);
    if (result == NES_FM2_OK) {
        *written = writer.offset;
    }
    return result;
}
