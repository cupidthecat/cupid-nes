/* capture_riff.c - RIFF hierarchy, AVI headers and index validation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#define _FILE_OFFSET_BITS 64
#endif
#include "capture_riff.h"
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define seek64 _fseeki64
#define tell64 _ftelli64
#else
#define seek64 fseeko
#define tell64 ftello
#endif

typedef struct {
    FILE *file;
    NesRiffReport report;
    uint64_t movi;
    bool io_error;
    bool main_header;
    uint64_t video_chunks;
    uint32_t stream_headers;
} Reader;

static uint32_t u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static unsigned u16(const uint8_t *p) {
    return p[0] | (unsigned)p[1] << 8;
}

static bool read_at(Reader *r, uint64_t offset, void *data, size_t bytes) {
    if (offset > r->report.file_size || bytes > r->report.file_size - offset) {
        return false;
    }
    if (offset > INT64_MAX || seek64(r->file, (int64_t)offset, SEEK_SET) || fread(data, 1, bytes, r->file) != bytes) {
        r->io_error = true;
        return false;
    }
    return true;
}

static void tag(char out[5], const uint8_t *bytes) {
    for (unsigned i = 0; i < 4; ++i) {
        out[i] = bytes[i] >= 32 && bytes[i] <= 126 ? (char)bytes[i] : '.';
    }
    out[4] = 0;
}

static void warn(Reader *r, NesRiffNode *node, const char *message) {
    node->warning = true;
    ++r->report.warnings;
    size_t length = strlen(node->detail);
    snprintf(node->detail + length, sizeof(node->detail) - length, "%s%s", length ? "; " : "", message);
}

static bool index_target(Reader *r, uint64_t offset, const uint8_t *entry) {
    uint8_t header[8];
    return read_at(r, offset, header, 8) && !memcmp(header, entry, 4) && u32(header + 4) == u32(entry + 12) &&
           (uint64_t)u32(header + 4) <= r->report.file_size - offset - 8u;
}

static void metadata(Reader *r, NesRiffNode *node, uint64_t payload, uint32_t size, unsigned stream_type) {
    uint8_t b[64] = {0};
    size_t bytes = size < sizeof(b) ? size : sizeof(b);
    if (!read_at(r, payload, b, bytes)) {
        return;
    }
    if (!strcmp(node->tag, "avih")) {
        if (size < 56) {
            warn(r, node, "Short AVI header");
            return;
        }
        r->main_header = true;
        r->report.video_frames = u32(b + 16);
        r->report.streams = u32(b + 24);
        r->report.width = u32(b + 32);
        r->report.height = u32(b + 36);
        snprintf(node->detail, sizeof(node->detail), "%ux%u, %u frames, %u streams, %u us/frame", r->report.width,
                 r->report.height, r->report.video_frames, r->report.streams, u32(b));
        if (!u32(b) || !r->report.streams) {
            warn(r, node, "Invalid timing or stream count");
        }
    } else if (!strcmp(node->tag, "strh")) {
        ++r->stream_headers;
        if (size < 56) {
            warn(r, node, "Short stream header");
            return;
        }
        char type[5], codec[5];
        tag(type, b);
        tag(codec, b + 4);
        snprintf(node->detail, sizeof(node->detail), "%s codec=%s rate=%u/%u length=%u sample=%u", type, codec,
                 u32(b + 24), u32(b + 20), u32(b + 32), u32(b + 44));
        if (!u32(b + 20) || !u32(b + 24)) {
            warn(r, node, "Zero stream rate or scale");
        }
    } else if (!strcmp(node->tag, "strf")) {
        if (stream_type == 1 && size >= 40) {
            char codec[5];
            tag(codec, b + 16);
            snprintf(node->detail, sizeof(node->detail), "%dx%d, %u-bit, compression=%s (0x%08X), image=%u",
                     (int32_t)u32(b + 4), (int32_t)u32(b + 8), u16(b + 14), codec, u32(b + 16), u32(b + 20));
        } else if (stream_type == 2 && size >= 16) {
            snprintf(node->detail, sizeof(node->detail), "Audio format=%u, %u channels, %u Hz, %u-bit, block=%u",
                     u16(b), u16(b + 2), u32(b + 4), u16(b + 14), u16(b + 12));
        } else {
            warn(r, node, "Unknown or short stream format");
        }
    } else if (!strcmp(node->tag, "idx1")) {
        snprintf(node->detail, sizeof(node->detail), "%u index entries", size / 16u);
        if (size % 16u) {
            warn(r, node, "Partial index entry");
        }
        for (uint32_t i = 0; i < size / 16u; ++i) {
            if (r->report.count == NES_RIFF_NODE_LIMIT) {
                warn(r, node, "Index inspection limit reached");
                break;
            }
            uint8_t entry[16];
            if (!read_at(r, payload + (uint64_t)i * 16u, entry, 16)) {
                break;
            }
            NesRiffNode *child = &r->report.nodes[r->report.count++];
            child->offset = payload + (uint64_t)i * 16u;
            child->size = u32(entry + 12);
            child->depth = node->depth + 1;
            tag(child->tag, entry);
            uint32_t relative = u32(entry + 8);
            snprintf(child->detail, sizeof(child->detail), "Index #%u: offset=%u, flags=0x%X", i, relative,
                     u32(entry + 4));
            if (!index_target(r, r->movi + relative, entry) && !index_target(r, relative, entry)) {
                warn(r, child, "Index target does not match chunk");
            }
        }
    }
}

static void walk(Reader *r, uint64_t begin, uint64_t end, unsigned depth) {
    unsigned stream_type = 0;
    while (begin < end) {
        if (r->report.count == NES_RIFF_NODE_LIMIT) {
            warn(r, &r->report.nodes[r->report.count - 1], "Chunk inspection limit reached");
            return;
        }
        NesRiffNode *node = &r->report.nodes[r->report.count++];
        node->offset = begin;
        node->depth = depth;
        uint8_t header[12];
        if (end - begin < 8 || !read_at(r, begin, header, 8)) {
            strcpy(node->tag, "????");
            warn(r, node, "Truncated chunk header");
            return;
        }
        tag(node->tag, header);
        node->size = u32(header + 4);
        uint64_t payload = begin + 8;
        uint64_t stop = payload + node->size;
        bool truncated = stop > end;
        if (truncated) {
            stop = end;
            warn(r, node, "Chunk extends beyond enclosing container");
        }
        if (!strcmp(node->tag, "LIST") || !strcmp(node->tag, "RIFF")) {
            if (stop - payload < 4 || !read_at(r, payload, header + 8, 4)) {
                warn(r, node, "Missing container type");
            } else {
                tag(node->kind, header + 8);
                if (!strcmp(node->kind, "movi")) {
                    r->movi = payload;
                }
                if (depth == NES_RIFF_DEPTH_LIMIT) {
                    warn(r, node, "Nesting limit reached");
                } else {
                    walk(r, payload + 4, stop, depth + 1);
                }
            }
        } else {
            if (node->tag[0] >= '0' && node->tag[0] <= '9' && node->tag[1] >= '0' && node->tag[1] <= '9' &&
                node->tag[2] == 'd' && (node->tag[3] == 'b' || node->tag[3] == 'c')) {
                ++r->video_chunks;
            }
            if (!strcmp(node->tag, "strh") && stop - payload >= 4 && read_at(r, payload, header, 4)) {
                stream_type = !memcmp(header, "vids", 4) ? 1u : !memcmp(header, "auds", 4) ? 2u : 0u;
            }
            metadata(r, node, payload, (uint32_t)(stop - payload), stream_type);
        }
        if (truncated) {
            return;
        }
        if ((node->size & 1u) && stop == end) {
            warn(r, node, "Missing odd-byte padding");
            return;
        }
        begin = stop + (node->size & 1u);
    }
}

void nes_capture_riff_free(NesRiffReport *report) {
    if (!report) {
        return;
    }
    free(report->nodes);
    memset(report, 0, sizeof(*report));
}

NesFileResult nes_capture_riff_inspect(const char *path, NesRiffReport *report) {
    if (!path || !report) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    Reader r = {0};
    r.file = nes_file_open(path, "rb");
    if (!r.file) {
        return errno == ENOENT ? NES_FILE_NOT_FOUND : NES_FILE_IO_ERROR;
    }
    if (seek64(r.file, 0, SEEK_END)) {
        fclose(r.file);
        return NES_FILE_IO_ERROR;
    }
    int64_t length = (int64_t)tell64(r.file);
    if (length < 0) {
        fclose(r.file);
        return NES_FILE_IO_ERROR;
    }
    r.report.file_size = (uint64_t)length;
    r.report.nodes = calloc(NES_RIFF_NODE_LIMIT, sizeof(*r.report.nodes));
    if (!r.report.nodes) {
        fclose(r.file);
        return NES_FILE_OUT_OF_MEMORY;
    }
    walk(&r, 0, r.report.file_size, 0);
    if (!r.report.count) {
        r.report.count = 1;
        warn(&r, r.report.nodes, "Empty file");
    } else if (strcmp(r.report.nodes[0].tag, "RIFF") || strcmp(r.report.nodes[0].kind, "AVI ")) {
        warn(&r, r.report.nodes, "Not a RIFF AVI file");
    }
    if (r.main_header && r.video_chunks != r.report.video_frames) {
        warn(&r, r.report.nodes, "Declared frame count differs from inspected video chunks");
    }
    if (r.main_header && r.stream_headers != r.report.streams) {
        warn(&r, r.report.nodes, "Declared stream count differs from inspected stream headers");
    }
    fclose(r.file);
    if (r.io_error) {
        nes_capture_riff_free(&r.report);
        return NES_FILE_IO_ERROR;
    }
    nes_capture_riff_free(report);
    *report = r.report;
    return NES_FILE_OK;
}
