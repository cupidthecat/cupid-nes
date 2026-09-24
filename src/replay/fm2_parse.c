/*
 * fm2_parse.c - bounded FM2/FM3 parser
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "fm2_internal.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    bool binary;
    bool have_version;
    bool first_field;
    size_t declared_frames;
    size_t frame_capacity;
} ParseState;

static bool reader_has(const NesFm2Reader *reader, size_t bytes) {
    return reader->offset <= reader->size && bytes <= reader->size - reader->offset;
}

static uint32_t read_u32le(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint64_t read_u64le(const uint8_t *data) {
    uint64_t value = 0;

    for (unsigned int i = 0; i < 8; ++i) {
        value |= (uint64_t)data[i] << (i * 8);
    }
    return value;
}

static NesFm2Result account_text(NesFm2Reader *reader, size_t bytes, NesFm2Diagnostic *diagnostic) {
    size_t total;

    if (!nes_fm2__checked_add(reader->text_bytes, bytes, &total) || total > reader->limits.max_text_bytes) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, reader->offset, 0, "movie text exceeds max_text_bytes");
    }
    reader->text_bytes = total;
    return NES_FM2_OK;
}

static bool parse_u32(const uint8_t *data, size_t size, uint32_t *value) {
    uint64_t parsed = 0;

    if (!data || !value || size == 0) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        if (data[i] < '0' || data[i] > '9') {
            return false;
        }
        parsed = parsed * 10u + (uint64_t)(data[i] - '0');
        if (parsed > UINT32_MAX) {
            return false;
        }
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool parse_u64(const uint8_t *data, size_t size, uint64_t *value) {
    uint64_t parsed = 0;

    if (!data || !value || size == 0) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        unsigned int digit;

        if (data[i] < '0' || data[i] > '9') {
            return false;
        }
        digit = (unsigned int)(data[i] - '0');
        if (parsed > (UINT64_MAX - digit) / 10u) {
            return false;
        }
        parsed = parsed * 10u + digit;
    }
    *value = parsed;
    return true;
}

static bool parse_i32_bits(const uint8_t *data, size_t size, uint32_t *value) {
    bool negative = false;
    uint64_t magnitude = 0;
    size_t at = 0;

    if (!data || !value || size == 0) {
        return false;
    }
    if (data[0] == '-') {
        negative = true;
        at = 1;
        if (at == size) {
            return false;
        }
    } else if (data[0] == '+') {
        at = 1;
        if (at == size) {
            return false;
        }
    }
    for (; at < size; ++at) {
        if (data[at] < '0' || data[at] > '9') {
            return false;
        }
        magnitude = magnitude * 10u + (uint64_t)(data[at] - '0');
        if ((!negative && magnitude > INT32_MAX) || (negative && magnitude > (uint64_t)INT32_MAX + 1u)) {
            return false;
        }
    }
    if (negative) {
        int64_t signed_value = -(int64_t)magnitude;
        *value = (uint32_t)(int32_t)signed_value;
    } else {
        *value = (uint32_t)(int32_t)magnitude;
    }
    return true;
}

static int hex_value(uint8_t ch) {
    if (ch >= '0' && ch <= '9') {
        return (int)(ch - '0');
    }
    if (ch >= 'a' && ch <= 'f') {
        return 10 + (int)(ch - 'a');
    }
    if (ch >= 'A' && ch <= 'F') {
        return 10 + (int)(ch - 'A');
    }
    return -1;
}

static NesFm2Result decode_hex(const uint8_t *data, size_t size, size_t limit, uint8_t **out_data, size_t *out_size,
                               size_t offset, NesFm2Diagnostic *diagnostic) {
    size_t prefix = 0;
    size_t bytes;
    uint8_t *decoded;

    if (size >= 2 && data[0] == '0' && (data[1] == 'x' || data[1] == 'X')) {
        prefix = 2;
    }
    if ((size - prefix) % 2 != 0) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "hex field has an odd number of digits");
    }
    bytes = (size - prefix) / 2;
    if (bytes > limit) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, offset, 0, "decoded field exceeds configured blob limit");
    }
    decoded = bytes ? malloc(bytes) : NULL;
    if (bytes && !decoded) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to allocate decoded hex field");
    }
    for (size_t i = 0; i < bytes; ++i) {
        int hi = hex_value(data[prefix + i * 2]);
        int lo = hex_value(data[prefix + i * 2 + 1]);

        if (hi < 0 || lo < 0) {
            free(decoded);
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + prefix + i * 2, 0,
                                 "hex field contains a non-hex digit");
        }
        decoded[i] = (uint8_t)((hi << 4) | lo);
    }
    *out_data = decoded;
    *out_size = bytes;
    return NES_FM2_OK;
}

static int base64_value(uint8_t ch) {
    if (ch >= 'A' && ch <= 'Z') {
        return (int)(ch - 'A');
    }
    if (ch >= 'a' && ch <= 'z') {
        return 26 + (int)(ch - 'a');
    }
    if (ch >= '0' && ch <= '9') {
        return 52 + (int)(ch - '0');
    }
    if (ch == '+') {
        return 62;
    }
    if (ch == '/') {
        return 63;
    }
    return -1;
}

static NesFm2Result decode_base64(const uint8_t *data, size_t size, size_t limit, uint8_t **out_data, size_t *out_size,
                                  size_t offset, NesFm2Diagnostic *diagnostic) {
    size_t prefix = 0;
    size_t padding = 0;
    size_t bytes;
    uint8_t *decoded;
    size_t out_at = 0;

    if (size >= 7 && memcmp(data, "base64:", 7) == 0) {
        prefix = 7;
    }
    data += prefix;
    size -= prefix;
    if (size == 0 || size % 4 != 0) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "base64 field length is invalid");
    }
    if (size >= 1 && data[size - 1] == '=') {
        ++padding;
    }
    if (size >= 2 && data[size - 2] == '=') {
        ++padding;
    }
    bytes = (size / 4) * 3 - padding;
    if (bytes > limit) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, offset, 0, "decoded field exceeds configured blob limit");
    }
    decoded = bytes ? malloc(bytes) : NULL;
    if (bytes && !decoded) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to allocate decoded base64 field");
    }
    for (size_t i = 0; i < size; i += 4) {
        uint32_t bits = 0;
        unsigned int valid = 3;

        for (unsigned int j = 0; j < 4; ++j) {
            int value;
            uint8_t ch = data[i + j];

            if (ch == '=') {
                if (i + 4 != size || j < 2) {
                    free(decoded);
                    return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + prefix + i + j, 0,
                                         "base64 padding is misplaced");
                }
                value = 0;
                valid = j - 1;
                for (unsigned int k = j; k < 4; ++k) {
                    if (data[i + k] != '=') {
                        free(decoded);
                        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + prefix + i + k, 0,
                                             "base64 data follows padding");
                    }
                }
            } else {
                value = base64_value(ch);
                if (value < 0) {
                    free(decoded);
                    return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + prefix + i + j, 0,
                                         "base64 field contains an invalid character");
                }
            }
            bits = (bits << 6) | (uint32_t)value;
        }
        if (valid >= 1 && out_at < bytes) {
            decoded[out_at++] = (uint8_t)(bits >> 16);
        }
        if (valid >= 2 && out_at < bytes) {
            decoded[out_at++] = (uint8_t)(bits >> 8);
        }
        if (valid >= 3 && out_at < bytes) {
            decoded[out_at++] = (uint8_t)bits;
        }
    }
    if (out_at != bytes) {
        free(decoded);
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "base64 decoded length is inconsistent");
    }
    *out_data = decoded;
    *out_size = bytes;
    return NES_FM2_OK;
}

static NesFm2Result decode_bytes(const uint8_t *data, size_t size, size_t limit, uint8_t **out_data, size_t *out_size,
                                 size_t offset, NesFm2Diagnostic *diagnostic) {
    if (size >= 7 && memcmp(data, "base64:", 7) == 0) {
        return decode_base64(data, size, limit, out_data, out_size, offset, diagnostic);
    }
    return decode_hex(data, size, limit, out_data, out_size, offset, diagnostic);
}

static NesFm2Result append_string(NesFm2StringList *list, const uint8_t *value, size_t value_size, size_t limit,
                                  size_t offset, NesFm2Diagnostic *diagnostic) {
    char **items;
    char *copy;

    if (list->count >= limit || list->count == SIZE_MAX / sizeof(*items)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, offset, 0, "movie contains too many repeated header fields");
    }
    copy = nes_fm2__dup_slice(value, value_size);
    if (!copy) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to allocate header string");
    }
    items = realloc(list->items, (list->count + 1) * sizeof(*items));
    if (!items) {
        free(copy);
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to grow header string list");
    }
    list->items = items;
    list->items[list->count++] = copy;
    return NES_FM2_OK;
}

static NesFm2Result append_extension(NesFm2Movie *movie, const uint8_t *key, size_t key_size, const uint8_t *value,
                                     size_t value_size, const NesFm2Limits *limits, size_t offset,
                                     NesFm2Diagnostic *diagnostic) {
    NesFm2HeaderField *fields;
    char *key_copy;
    char *value_copy;

    if (movie->extension_count >= limits->max_extensions || movie->extension_count == SIZE_MAX / sizeof(*fields)) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, offset, 0, "movie contains too many extension header fields");
    }
    key_copy = nes_fm2__dup_slice(key, key_size);
    value_copy = nes_fm2__dup_slice(value, value_size);
    if (!key_copy || !value_copy) {
        free(key_copy);
        free(value_copy);
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to allocate extension header field");
    }
    fields = realloc(movie->extensions, (movie->extension_count + 1) * sizeof(*fields));
    if (!fields) {
        free(key_copy);
        free(value_copy);
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to grow extension header list");
    }
    movie->extensions = fields;
    fields[movie->extension_count].key = key_copy;
    fields[movie->extension_count].value = value_copy;
    ++movie->extension_count;
    return NES_FM2_OK;
}

static bool key_equals(const uint8_t *key, size_t key_size, const char *expected) {
    size_t length = strlen(expected);

    return key_size == length && memcmp(key, expected, length) == 0;
}

static NesFm2Result install_bool(bool *target, const uint8_t *value, size_t value_size, size_t offset,
                                 NesFm2Diagnostic *diagnostic) {
    uint32_t parsed;

    if (!parse_u32(value, value_size, &parsed) || parsed > 1) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "boolean header field must be 0 or 1");
    }
    *target = parsed != 0;
    return NES_FM2_OK;
}

static NesFm2Result install_port(NesFm2Port *target, const uint8_t *value, size_t value_size, size_t offset,
                                 NesFm2Diagnostic *diagnostic) {
    uint32_t parsed;

    if (!parse_u32(value, value_size, &parsed) || parsed > NES_FM2_PORT_ZAPPER) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, offset, 0, "unsupported FM2 input port type");
    }
    *target = (NesFm2Port)parsed;
    return NES_FM2_OK;
}

static NesFm2Result install_guid(NesFm2Movie *movie, const uint8_t *value, size_t value_size, size_t offset,
                                 NesFm2Diagnostic *diagnostic) {
    static const size_t hyphens[] = {8, 13, 18, 23};
    size_t byte_at = 0;

    if (value_size != 36) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0,
                             "GUID must contain 32 hex digits and four hyphens");
    }
    for (size_t i = 0; i < sizeof(hyphens) / sizeof(hyphens[0]); ++i) {
        if (value[hyphens[i]] != '-') {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + hyphens[i], 0, "GUID hyphen is missing");
        }
    }
    for (size_t i = 0; i < value_size;) {
        int hi;
        int lo;

        if (value[i] == '-') {
            ++i;
            continue;
        }
        if (i + 1 >= value_size || byte_at >= NES_FM2_GUID_SIZE) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + i, 0, "GUID has an invalid hex layout");
        }
        hi = hex_value(value[i]);
        lo = hex_value(value[i + 1]);
        if (hi < 0 || lo < 0) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + i, 0, "GUID contains a non-hex digit");
        }
        movie->guid[byte_at++] = (uint8_t)((hi << 4) | lo);
        i += 2;
    }
    if (byte_at != NES_FM2_GUID_SIZE) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "GUID decoded to the wrong size");
    }
    return NES_FM2_OK;
}

static NesFm2Result install_checksum(NesFm2Movie *movie, const uint8_t *value, size_t value_size, size_t offset,
                                     NesFm2Diagnostic *diagnostic) {
    uint8_t *decoded = NULL;
    size_t decoded_size = 0;
    NesFm2Result result =
        decode_bytes(value, value_size, NES_FM2_MD5_SIZE, &decoded, &decoded_size, offset, diagnostic);

    if (result != NES_FM2_OK) {
        return result;
    }
    if (decoded_size != NES_FM2_MD5_SIZE) {
        free(decoded);
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "ROM checksum must decode to 16 bytes");
    }
    memcpy(movie->rom_md5, decoded, NES_FM2_MD5_SIZE);
    free(decoded);
    return NES_FM2_OK;
}

static NesFm2Result install_blob(NesFm2Blob *blob, const uint8_t *value, size_t value_size, const NesFm2Limits *limits,
                                 size_t offset, NesFm2Diagnostic *diagnostic) {
    uint8_t *decoded = NULL;
    size_t decoded_size = 0;
    NesFm2Result result =
        decode_bytes(value, value_size, limits->max_blob_bytes, &decoded, &decoded_size, offset, diagnostic);

    if (result != NES_FM2_OK) {
        return result;
    }
    free(blob->data);
    blob->data = decoded;
    blob->size = decoded_size;
    return NES_FM2_OK;
}

static NesFm2Result install_header_field(NesFm2Movie *movie, ParseState *state, const uint8_t *key, size_t key_size,
                                         const uint8_t *value, size_t value_size, const NesFm2Limits *limits,
                                         size_t offset, NesFm2Diagnostic *diagnostic) {
    uint32_t parsed;
    NesFm2Result result;

    if (state->first_field && !key_equals(key, key_size, "version")) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "first FM2 header field must be version");
    }
    state->first_field = false;

    if (key_equals(key, key_size, "version")) {
        if (!parse_u32(value, value_size, &parsed)) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "FM2 version is not a valid integer");
        }
        if (parsed != 3) {
            return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, offset, 0, "only FM2 version 3 is supported");
        }
        movie->version = parsed;
        state->have_version = true;
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "emuVersion")) {
        if (!parse_u32(value, value_size, &movie->emu_version)) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "emuVersion is not a valid uint32");
        }
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "rerecordCount")) {
        if (!parse_u32(value, value_size, &movie->rerecord_count)) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "rerecordCount is not a valid uint32");
        }
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "palFlag")) {
        return install_bool(&movie->pal, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "romFilename")) {
        char *copy = nes_fm2__dup_slice(value, value_size);

        if (!copy) {
            return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to allocate ROM filename");
        }
        free(movie->rom_filename);
        movie->rom_filename = copy;
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "romChecksum")) {
        return install_checksum(movie, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "guid")) {
        return install_guid(movie, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "fourscore")) {
        return install_bool(&movie->fourscore, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "microphone")) {
        return install_bool(&movie->microphone, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "port0")) {
        return install_port(&movie->ports[0], value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "port1")) {
        return install_port(&movie->ports[1], value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "port2")) {
        result = install_port(&movie->ports[2], value, value_size, offset, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
        if (movie->ports[2] != NES_FM2_PORT_NONE) {
            return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, offset, 0,
                                 "FC expansion input devices are unsupported");
        }
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "FDS")) {
        return install_bool(&movie->fds, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "NewPPU")) {
        return install_bool(&movie->new_ppu, value, value_size, offset, diagnostic);
    }
    if (key_equals(key, key_size, "RAMInitOption")) {
        if (!parse_u32(value, value_size, &movie->ram_init_option)) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "RAMInitOption is not a valid uint32");
        }
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "RAMInitSeed")) {
        if (!parse_i32_bits(value, value_size, &movie->ram_init_seed)) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "RAMInitSeed is not a valid int32");
        }
        return NES_FM2_OK;
    }
    if (key_equals(key, key_size, "comment")) {
        return append_string(&movie->comments, value, value_size, limits->max_comments, offset, diagnostic);
    }
    if (key_equals(key, key_size, "subtitle")) {
        return append_string(&movie->subtitles, value, value_size, limits->max_subtitles, offset, diagnostic);
    }
    if (key_equals(key, key_size, "binary")) {
        bool binary = false;

        result = install_bool(&binary, value, value_size, offset, diagnostic);
        if (result == NES_FM2_OK) {
            state->binary = binary;
        }
        return result;
    }
    if (key_equals(key, key_size, "savestate")) {
        return install_blob(&movie->savestate, value, value_size, limits, offset, diagnostic);
    }
    if (key_equals(key, key_size, "saveram")) {
        return install_blob(&movie->saveram, value, value_size, limits, offset, diagnostic);
    }
    if (key_equals(key, key_size, "length")) {
        if (!parse_u32(value, value_size, &parsed)) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, 0, "length is not a valid uint32");
        }
        if ((size_t)parsed > limits->max_frames) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, offset, parsed, "declared movie length exceeds max_frames");
        }
        state->declared_frames = (size_t)parsed;
        movie->explicit_length = true;
        return NES_FM2_OK;
    }

    return append_extension(movie, key, key_size, value, value_size, limits, offset, diagnostic);
}

static NesFm2Result parse_header(NesFm2Reader *reader, NesFm2Movie *movie, ParseState *state,
                                 NesFm2Diagnostic *diagnostic) {
    while (reader->offset < reader->size) {
        size_t line_start = reader->offset;
        size_t line_end;
        size_t content_end;
        size_t split;
        NesFm2Result result;

        if (reader->data[reader->offset] == '|') {
            if (!state->have_version) {
                return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, reader->offset, 0, "FM2 header is missing version 3");
            }
            return NES_FM2_OK;
        }
        line_end = line_start;
        while (line_end < reader->size && reader->data[line_end] != '\n' && reader->data[line_end] != '\r') {
            if (reader->data[line_end] == 0) {
                return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, line_end, 0, "NUL byte appears in FM2 text header");
            }
            ++line_end;
        }
        content_end = line_end;
        if (line_end < reader->size) {
            if (reader->data[line_end] == '\r' && line_end + 1 < reader->size && reader->data[line_end + 1] == '\n') {
                reader->offset = line_end + 2;
            } else {
                reader->offset = line_end + 1;
            }
        } else {
            reader->offset = line_end;
        }
        result = account_text(reader, reader->offset - line_start, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
        if (content_end == line_start) {
            continue;
        }
        split = line_start;
        while (split < content_end && reader->data[split] != ' ' && reader->data[split] != '\t') {
            ++split;
        }
        if (split == line_start) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, line_start, 0, "FM2 header key is empty");
        }
        size_t value_start = split;
        while (value_start < content_end && (reader->data[value_start] == ' ' || reader->data[value_start] == '\t')) {
            ++value_start;
        }
        result = install_header_field(movie, state, reader->data + line_start, split - line_start,
                                      reader->data + value_start, content_end - value_start, &reader->limits,
                                      line_start, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
    }
    if (!state->have_version) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->offset, 0,
                             "FM2 header ended before version 3 was found");
    }
    return NES_FM2_OK;
}

static NesFm2Result ensure_frame_capacity(NesFm2Movie *movie, ParseState *state, size_t needed, size_t offset,
                                          NesFm2Diagnostic *diagnostic) {
    size_t capacity;
    size_t bytes;
    NesFm2Frame *frames;

    if (needed > state->frame_capacity) {
        capacity = state->frame_capacity ? state->frame_capacity : 256;
        while (capacity < needed) {
            if (capacity > SIZE_MAX / 2) {
                capacity = needed;
                break;
            }
            capacity *= 2;
        }
        if (capacity < needed || !nes_fm2__checked_mul(capacity, sizeof(*movie->frames), &bytes)) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, offset, needed, "frame array size overflows size_t");
        }
        frames = realloc(movie->frames, bytes);
        if (!frames) {
            return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, needed, "unable to grow frame array");
        }
        movie->frames = frames;
        state->frame_capacity = capacity;
    }
    return NES_FM2_OK;
}

static size_t binary_record_size(const NesFm2Movie *movie) {
    size_t size = 1;

    if (movie->fourscore) {
        return size + 4;
    }
    for (size_t port = 0; port < 2; ++port) {
        if (movie->ports[port] == NES_FM2_PORT_GAMEPAD) {
            size += 1;
        } else if (movie->ports[port] == NES_FM2_PORT_ZAPPER) {
            size += 12;
        }
    }
    return size;
}

static void parse_binary_frame(const uint8_t *data, const NesFm2Movie *movie, NesFm2Frame *frame) {
    size_t at = 0;

    memset(frame, 0, sizeof(*frame));
    frame->commands = data[at++];
    if (movie->fourscore) {
        memcpy(frame->pads, data + at, 4);
        return;
    }
    for (size_t port = 0; port < 2; ++port) {
        if (movie->ports[port] == NES_FM2_PORT_GAMEPAD) {
            frame->pads[port] = data[at++];
        } else if (movie->ports[port] == NES_FM2_PORT_ZAPPER) {
            frame->zappers[port].x = data[at++];
            frame->zappers[port].y = data[at++];
            frame->zappers[port].button = data[at++];
            frame->zappers[port].bogo = data[at++];
            frame->zappers[port].zaphit = read_u64le(data + at);
            at += 8;
        }
    }
}

static NesFm2Result parse_binary_frames(NesFm2Reader *reader, NesFm2Movie *movie, ParseState *state,
                                        NesFm2Diagnostic *diagnostic) {
    size_t record_size = binary_record_size(movie);
    size_t count;
    size_t bytes;
    NesFm2Result result;

    if (!reader_has(reader, 1) || reader->data[reader->offset] != '|') {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->offset, 0,
                             "binary FM2 input log is missing its leading pipe");
    }
    ++reader->offset;
    if (movie->explicit_length) {
        count = state->declared_frames;
        if (!nes_fm2__checked_mul(count, record_size, &bytes)) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, reader->offset, count,
                                 "binary movie record data overflows size_t");
        }
        if (!reader_has(reader, bytes)) {
            return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->size, count,
                                 "binary movie ends before its declared frame count");
        }
    } else {
        bytes = reader->size - reader->offset;
        if (bytes % record_size != 0) {
            return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->size, 0,
                                 "binary movie ends inside a frame record");
        }
        count = bytes / record_size;
        if (count > reader->limits.max_frames) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, reader->offset, count, "binary movie exceeds max_frames");
        }
    }
    if (count != 0) {
        size_t allocation;

        if (!nes_fm2__checked_mul(count, sizeof(*movie->frames), &allocation)) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, reader->offset, count, "frame array size overflows size_t");
        }
        movie->frames = malloc(allocation);
        if (!movie->frames) {
            return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, reader->offset, count,
                                 "unable to allocate binary frame array");
        }
        state->frame_capacity = count;
    }
    for (size_t i = 0; i < count; ++i) {
        parse_binary_frame(reader->data + reader->offset, movie, &movie->frames[i]);
        reader->offset += record_size;
    }
    movie->frame_count = count;
    result = NES_FM2_OK;
    return result;
}

static NesFm2Result parse_gamepad_field(const uint8_t *data, size_t size, uint8_t *pad, size_t offset,
                                        size_t frame_index, NesFm2Diagnostic *diagnostic) {
    uint8_t value = 0;

    if (size != 8) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, frame_index,
                             "gamepad input field must contain exactly 8 characters");
    }
    for (size_t i = 0; i < 8; ++i) {
        value <<= 1;
        if (data[i] != '.' && data[i] != ' ') {
            value |= 1;
        }
    }
    *pad = value;
    return NES_FM2_OK;
}

static NesFm2Result parse_zapper_field(const uint8_t *data, size_t size, NesFm2Zapper *zapper, size_t offset,
                                       size_t frame_index, NesFm2Diagnostic *diagnostic) {
    uint64_t values[5] = {0};
    size_t at = 0;

    for (size_t field = 0; field < 5; ++field) {
        size_t start;

        while (at < size && (data[at] == ' ' || data[at] == '\t')) {
            ++at;
        }
        start = at;
        while (at < size && data[at] >= '0' && data[at] <= '9') {
            ++at;
        }
        if (start == at || !parse_u64(data + start, at - start, &values[field])) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + start, frame_index,
                                 "zapper field contains an invalid integer");
        }
        if (field != 4) {
            if (at >= size || (data[at] != ' ' && data[at] != '\t')) {
                return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + at, frame_index,
                                     "zapper values must be whitespace separated");
            }
        }
    }
    while (at < size && (data[at] == ' ' || data[at] == '\t')) {
        ++at;
    }
    if (at != size || values[0] > UINT8_MAX || values[1] > UINT8_MAX || values[2] > UINT8_MAX ||
        values[3] > UINT8_MAX) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset + at, frame_index,
                             "zapper field is outside its representable range");
    }
    zapper->x = (uint8_t)values[0];
    zapper->y = (uint8_t)values[1];
    zapper->button = (uint8_t)values[2];
    zapper->bogo = (uint8_t)values[3];
    zapper->zaphit = values[4];
    return NES_FM2_OK;
}

static NesFm2Result parse_text_device(const uint8_t *data, size_t size, NesFm2Port port, uint8_t *pad,
                                      NesFm2Zapper *zapper, size_t offset, size_t frame_index,
                                      NesFm2Diagnostic *diagnostic) {
    if (port == NES_FM2_PORT_NONE) {
        if (size != 0) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, offset, frame_index,
                                 "input is present for a disconnected port");
        }
        return NES_FM2_OK;
    }
    if (port == NES_FM2_PORT_GAMEPAD) {
        return parse_gamepad_field(data, size, pad, offset, frame_index, diagnostic);
    }
    if (port == NES_FM2_PORT_ZAPPER) {
        return parse_zapper_field(data, size, zapper, offset, frame_index, diagnostic);
    }
    return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, offset, frame_index,
                         "unsupported input device in text movie");
}

static NesFm2Result take_pipe_field(NesFm2Reader *reader, const uint8_t **data, size_t *size, size_t frame_index,
                                    NesFm2Diagnostic *diagnostic) {
    size_t start = reader->offset;

    while (reader->offset < reader->size && reader->data[reader->offset] != '|' &&
           reader->data[reader->offset] != '\n' && reader->data[reader->offset] != '\r') {
        ++reader->offset;
    }
    if (reader->offset >= reader->size || reader->data[reader->offset] != '|') {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->offset, frame_index,
                             "text movie frame ends before its next pipe delimiter");
    }
    *data = reader->data + start;
    *size = reader->offset - start;
    ++reader->offset;
    return NES_FM2_OK;
}

static NesFm2Result parse_text_frame(NesFm2Reader *reader, const NesFm2Movie *movie, NesFm2Frame *frame,
                                     size_t frame_index, NesFm2Diagnostic *diagnostic) {
    const uint8_t *field;
    size_t field_size;
    uint32_t commands;
    NesFm2Result result;

    memset(frame, 0, sizeof(*frame));
    if (!reader_has(reader, 1) || reader->data[reader->offset] != '|') {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, reader->offset, frame_index,
                             "text movie frame does not begin with a pipe");
    }
    ++reader->offset;
    result = take_pipe_field(reader, &field, &field_size, frame_index, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    if (!parse_u32(field, field_size, &commands) || commands > UINT8_MAX) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, (size_t)(field - reader->data), frame_index,
                             "movie command field must be an 8-bit decimal integer");
    }
    frame->commands = (uint8_t)commands;

    if (movie->fourscore) {
        for (size_t pad = 0; pad < 4; ++pad) {
            result = take_pipe_field(reader, &field, &field_size, frame_index, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
            result = parse_gamepad_field(field, field_size, &frame->pads[pad], (size_t)(field - reader->data),
                                         frame_index, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
        }
    } else {
        for (size_t port = 0; port < 2; ++port) {
            result = take_pipe_field(reader, &field, &field_size, frame_index, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
            result = parse_text_device(field, field_size, movie->ports[port], &frame->pads[port], &frame->zappers[port],
                                       (size_t)(field - reader->data), frame_index, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
        }
    }
    result = take_pipe_field(reader, &field, &field_size, frame_index, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    if (field_size != 0) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, (size_t)(field - reader->data), frame_index,
                             "FC expansion input data is unsupported");
    }

    if (reader->offset < reader->size) {
        if (reader->data[reader->offset] == '\r') {
            ++reader->offset;
            if (reader->offset < reader->size && reader->data[reader->offset] == '\n') {
                ++reader->offset;
            }
        } else if (reader->data[reader->offset] == '\n') {
            ++reader->offset;
        } else if (!movie->explicit_length) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, reader->offset, frame_index,
                                 "text movie frame has trailing bytes before newline");
        }
    }
    return NES_FM2_OK;
}

static NesFm2Result parse_text_frames(NesFm2Reader *reader, NesFm2Movie *movie, ParseState *state,
                                      NesFm2Diagnostic *diagnostic) {
    size_t count = 0;

    while (reader->offset < reader->size && (!movie->explicit_length || count < state->declared_frames)) {
        size_t frame_start = reader->offset;
        NesFm2Result result;

        if (reader->data[reader->offset] == '\r' || reader->data[reader->offset] == '\n') {
            size_t newline_start = reader->offset++;

            if (reader->data[newline_start] == '\r' && reader->offset < reader->size &&
                reader->data[reader->offset] == '\n') {
                ++reader->offset;
            }
            result = account_text(reader, reader->offset - newline_start, diagnostic);
            if (result != NES_FM2_OK) {
                return result;
            }
            continue;
        }
        if (reader->data[reader->offset] != '|') {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, reader->offset, count,
                                 "text input log contains a non-frame line");
        }
        if (count >= reader->limits.max_frames) {
            return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, reader->offset, count, "text movie exceeds max_frames");
        }
        result = ensure_frame_capacity(movie, state, count + 1, reader->offset, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
        result = parse_text_frame(reader, movie, &movie->frames[count], count, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
        result = account_text(reader, reader->offset - frame_start, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
        ++count;
        movie->frame_count = count;
    }
    if (movie->explicit_length && count != state->declared_frames) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->offset, count,
                             "text movie ends before its declared frame count");
    }
    return NES_FM2_OK;
}

static NesFm2Result copy_project_module(NesFm2Movie *movie, size_t index, const uint8_t *data, size_t size,
                                        size_t offset, NesFm2Diagnostic *diagnostic) {
    if (size == 0) {
        return NES_FM2_OK;
    }
    movie->project_modules[index].data = malloc(size);
    if (!movie->project_modules[index].data) {
        return nes_fm2__fail(diagnostic, NES_FM2_OUT_OF_MEMORY, offset, 0, "unable to allocate FM3 project module");
    }
    memcpy(movie->project_modules[index].data, data, size);
    movie->project_modules[index].size = size;
    return NES_FM2_OK;
}

static NesFm2Result parse_project(NesFm2Reader *reader, NesFm2Movie *movie, NesFm2Diagnostic *diagnostic) {
    enum { PROJECT_HEADER_SIZE = 12 + NES_FM3_PROJECT_MODULE_COUNT * 4 };

    size_t start = reader->offset;
    uint32_t offsets[NES_FM3_PROJECT_MODULE_COUNT];
    uint32_t pointer_count;
    NesFm2Result result;

    if (reader->size - start > reader->limits.max_project_bytes) {
        return nes_fm2__fail(diagnostic, NES_FM2_LIMIT, start, 0, "FM3 project data exceeds max_project_bytes");
    }
    if (!reader_has(reader, PROJECT_HEADER_SIZE)) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->size, 0, "FM3 project header is truncated");
    }
    movie->project_version = read_u32le(reader->data + start);
    movie->project_saved_modules = read_u32le(reader->data + start + 4);
    pointer_count = read_u32le(reader->data + start + 8);
    if (movie->project_version != 3) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, start, 0, "only FM3 project version 3 is supported");
    }
    if (pointer_count != NES_FM3_PROJECT_MODULE_COUNT) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, start + 8, 0, "FM3 project pointer count is not 6");
    }
    if ((movie->project_saved_modules & ~0x3fu) != 0) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, start + 4, 0,
                             "FM3 saved-module mask contains unknown bits");
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        offsets[i] = read_u32le(reader->data + start + 12 + i * 4);
    }
    if ((size_t)offsets[0] != start + PROJECT_HEADER_SIZE) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, start + 12, 0,
                             "FM3 first module offset does not follow its pointer table");
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        size_t begin = offsets[i];
        size_t end = i + 1 < NES_FM3_PROJECT_MODULE_COUNT ? offsets[i + 1] : reader->size;

        if (begin > reader->size || end > reader->size || end < begin || begin < start) {
            return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, start + 12 + i * 4, 0,
                                 "FM3 project module offset is outside the file");
        }
        result = copy_project_module(movie, i, reader->data + begin, end - begin, begin, diagnostic);
        if (result != NES_FM2_OK) {
            return result;
        }
    }
    movie->project_present = true;
    movie->project_timeline_valid = true;
    movie->original_container = NES_FM3_PROJECT;
    reader->offset = reader->size;
    return NES_FM2_OK;
}

NesFm2Result nes_fm2__parse_movie(NesFm2Reader *reader, NesFm2Movie *movie, NesFm2Diagnostic *diagnostic) {
    ParseState state;
    NesFm2Result result;

    memset(&state, 0, sizeof(state));
    state.first_field = true;
    if (reader->size < 9) {
        return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->size, 0,
                             "movie is too short to contain an FM2 version header");
    }
    if (memcmp(reader->data, "version 3", 9) != 0) {
        return nes_fm2__fail(diagnostic, NES_FM2_UNSUPPORTED, 0, 0, "movie does not begin with FM2 version 3");
    }
    result = parse_header(reader, movie, &state, diagnostic);
    if (result != NES_FM2_OK) {
        return result;
    }
    if (reader->offset == reader->size) {
        if (movie->explicit_length && state.declared_frames != 0) {
            return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->size, 0,
                                 "movie ends before its declared input log");
        }
        movie->original_container = NES_FM2_TEXT;
        return movie->explicit_length ? parse_project(reader, movie, diagnostic) : NES_FM2_OK;
    }

    if (state.binary) {
        movie->original_container = NES_FM2_BINARY;
        result = parse_binary_frames(reader, movie, &state, diagnostic);
    } else {
        movie->original_container = NES_FM2_TEXT;
        result = parse_text_frames(reader, movie, &state, diagnostic);
    }
    if (result != NES_FM2_OK) {
        return result;
    }
    if (movie->explicit_length) {
        if (reader->offset >= reader->size) {
            return nes_fm2__fail(diagnostic, NES_FM2_TRUNCATED, reader->offset, movie->frame_count,
                                 "FM3 file is missing project data");
        }
        return parse_project(reader, movie, diagnostic);
    }
    if (reader->offset != reader->size) {
        return nes_fm2__fail(diagnostic, NES_FM2_CORRUPT, reader->offset, movie->frame_count,
                             "trailing data follows FM2 input log");
    }
    return NES_FM2_OK;
}
