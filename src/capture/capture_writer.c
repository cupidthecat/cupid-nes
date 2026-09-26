/*
 * capture_writer.c - Portable PCM/AVI containers with atomic finalization
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "capture_writer.h"
#include "capture_codec.h"
#include <stdlib.h>
#include <string.h>

enum {
    WAVE_HEADER_SIZE = 44,
    AVI_HEADER_SIZE = 324,
    AVI_INDEX_ENTRY_SIZE = 16,
    AVI_INDEX_MAX_ENTRIES = 1000000,
    PCM_BLOCK_FRAMES = 2048
};

typedef struct {
    uint8_t bytes[AVI_INDEX_ENTRY_SIZE];
} CaptureIndexEntry;

struct NesCaptureStream {
    NesFileTransaction *transaction;
    NesFileResult error;
    uint64_t byte_limit;
    uint64_t audio_frames;
    uint32_t video_frames;
    uint32_t fps_numerator;
    uint32_t fps_denominator;
    unsigned sample_rate;
    unsigned width;
    unsigned height;
    uint32_t row_bytes;
    uint32_t frame_bytes;
    uint8_t *row;
    CaptureIndexEntry *index;
    size_t index_count;
    size_t index_capacity;
    bool video;
    NesCaptureCodec codec;
    unsigned compression_level;
};

static void put16(uint8_t *bytes, uint16_t value) {
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *bytes, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        bytes[i] = (uint8_t)(value >> (i * 8u));
    }
}

static bool valid_frame(const NesCaptureFrame *frame) {
    return frame && frame->pixels && frame->width && frame->height && frame->width <= NES_CAPTURE_MAX_DIMENSION &&
           frame->height <= NES_CAPTURE_MAX_DIMENSION && frame->stride >= frame->width &&
           frame->stride <= SIZE_MAX / frame->height / sizeof(uint32_t);
}

static NesFileResult write_bytes(NesCaptureStream *stream, const void *bytes, size_t size) {
    if (stream->error != NES_FILE_OK) {
        return stream->error;
    }
    NesFileResult result = nes_file_transaction_write(stream->transaction, bytes, size);
    if (result != NES_FILE_OK) {
        stream->error = result;
    }
    return result;
}

static NesFileResult write_pcm(NesCaptureStream *stream, const int16_t *samples, size_t frames) {
    uint8_t bytes[PCM_BLOCK_FRAMES * 4];
    while (frames) {
        size_t count = frames > PCM_BLOCK_FRAMES ? PCM_BLOCK_FRAMES : frames;
        for (size_t i = 0; i < count * 2u; ++i) {
            put16(bytes + i * 2u, (uint16_t)samples[i]);
        }
        NesFileResult result = write_bytes(stream, bytes, count * 4u);
        if (result != NES_FILE_OK) {
            return result;
        }
        samples += count * 2u;
        frames -= count;
    }
    return NES_FILE_OK;
}

static void wave_header(const NesCaptureStream *stream, uint8_t header[WAVE_HEADER_SIZE]) {
    memset(header, 0, WAVE_HEADER_SIZE);
    uint32_t audio_bytes = (uint32_t)(stream->audio_frames * 4u);
    memcpy(header, "RIFF", 4);
    put32(header + 4, audio_bytes + 36u);
    memcpy(header + 8, "WAVEfmt ", 8);
    put32(header + 16, 16);
    put16(header + 20, 1);
    put16(header + 22, 2);
    put32(header + 24, stream->sample_rate);
    put32(header + 28, stream->sample_rate * 4u);
    put16(header + 32, 4);
    put16(header + 34, 16);
    memcpy(header + 36, "data", 4);
    put32(header + 40, audio_bytes);
}

static void avi_header(const NesCaptureStream *stream, uint8_t header[AVI_HEADER_SIZE], uint32_t movi_bytes,
                       uint32_t file_bytes) {
    memset(header, 0, AVI_HEADER_SIZE);
    memcpy(header, "RIFF", 4);
    put32(header + 4, file_bytes - 8u);
    memcpy(header + 8, "AVI LIST", 8);
    put32(header + 16, 292);
    memcpy(header + 20, "hdrlavih", 8);
    put32(header + 28, 56);
    uint64_t microseconds = (uint64_t)stream->fps_denominator * 1000000u;
    put32(header + 32, (uint32_t)((microseconds + stream->fps_numerator / 2u) / stream->fps_numerator));
    put32(header + 44, 0x110); // Indexed, interleaved streams.
    put32(header + 48, stream->video_frames);
    put32(header + 56, 2);
    put32(header + 60, stream->frame_bytes);
    put32(header + 64, stream->width);
    put32(header + 68, stream->height);

    memcpy(header + 88, "LIST", 4);
    put32(header + 92, 116);
    memcpy(header + 96, "strlstrh", 8);
    put32(header + 104, 56);
    memcpy(header + 108, "vidsDIB ", 8);
    put32(header + 128, stream->fps_denominator);
    put32(header + 132, stream->fps_numerator);
    put32(header + 140, stream->video_frames);
    put32(header + 144, stream->frame_bytes);
    put32(header + 148, UINT32_MAX);
    put16(header + 160, (uint16_t)stream->width);
    put16(header + 162, (uint16_t)stream->height);
    memcpy(header + 164, "strf", 4);
    put32(header + 168, 40);
    put32(header + 172, 40);
    put32(header + 176, stream->width);
    put32(header + 180, stream->height);
    put16(header + 184, 1);
    put16(header + 186, stream->codec == NES_CAPTURE_CODEC_ZMBV ? 32 : 24);
    if (stream->codec == NES_CAPTURE_CODEC_ZMBV) {
        memcpy(header + 112, "ZMBV", 4);
        memcpy(header + 188, "ZMBV", 4);
    }
    put32(header + 192, stream->frame_bytes);

    memcpy(header + 212, "LIST", 4);
    put32(header + 216, 92);
    memcpy(header + 220, "strlstrh", 8);
    put32(header + 228, 56);
    memcpy(header + 232, "auds", 4);
    put32(header + 252, 4);
    put32(header + 256, stream->sample_rate * 4u);
    put32(header + 264, (uint32_t)stream->audio_frames);
    put32(header + 268, PCM_BLOCK_FRAMES * 4u);
    put32(header + 272, UINT32_MAX);
    put32(header + 276, 4);
    memcpy(header + 288, "strf", 4);
    put32(header + 292, 16);
    put16(header + 296, 1);
    put16(header + 298, 2);
    put32(header + 300, stream->sample_rate);
    put32(header + 304, stream->sample_rate * 4u);
    put16(header + 308, 4);
    put16(header + 310, 16);
    memcpy(header + 312, "LIST", 4);
    put32(header + 316, movi_bytes);
    memcpy(header + 320, "movi", 4);
}

static NesFileResult open_stream(const char *path, unsigned sample_rate, uint64_t byte_limit, bool video,
                                 NesCaptureStream **out) {
    if (out) {
        *out = NULL;
    }
    if (!out || sample_rate < 8000 || sample_rate > 192000 || byte_limit > UINT32_MAX ||
        byte_limit < (video ? AVI_HEADER_SIZE + 8u : WAVE_HEADER_SIZE)) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesCaptureStream *stream = calloc(1, sizeof(*stream));
    if (!stream) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    stream->sample_rate = sample_rate;
    stream->byte_limit = byte_limit;
    stream->video = video;
    NesFileResult result = nes_file_transaction_begin(path, byte_limit, &stream->transaction);
    if (result != NES_FILE_OK) {
        free(stream);
        return result;
    }
    *out = stream;
    return NES_FILE_OK;
}

NesFileResult nes_capture_wav_open(const char *path, unsigned sample_rate, uint64_t byte_limit,
                                   NesCaptureStream **out) {
    NesFileResult result = open_stream(path, sample_rate, byte_limit, false, out);
    if (result != NES_FILE_OK) {
        return result;
    }
    uint8_t header[WAVE_HEADER_SIZE];
    wave_header(*out, header);
    result = write_bytes(*out, header, sizeof(header));
    if (result != NES_FILE_OK) {
        nes_capture_abort(out);
    }
    return result;
}

NesFileResult nes_capture_avi_open(const char *path, unsigned width, unsigned height, unsigned sample_rate,
                                   uint32_t fps_numerator, uint32_t fps_denominator, uint64_t byte_limit,
                                   NesCaptureStream **out) {
    return nes_capture_avi_open_codec(path, width, height, sample_rate, fps_numerator, fps_denominator, byte_limit,
                                      NES_CAPTURE_CODEC_RAW, 6, out);
}

NesFileResult nes_capture_avi_open_codec(const char *path, unsigned width, unsigned height, unsigned sample_rate,
                                         uint32_t fps_numerator, uint32_t fps_denominator, uint64_t byte_limit,
                                         NesCaptureCodec codec, unsigned compression_level, NesCaptureStream **out) {
    if (out) {
        *out = NULL;
    }
    if ((codec != NES_CAPTURE_CODEC_RAW && codec != NES_CAPTURE_CODEC_ZMBV) || compression_level > 9 || !width ||
        !height || width > NES_CAPTURE_MAX_DIMENSION || height > NES_CAPTURE_MAX_DIMENSION || !fps_numerator ||
        !fps_denominator || (double)fps_numerator / fps_denominator < 1.0 ||
        (double)fps_numerator / fps_denominator > 1000.0) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesFileResult result = open_stream(path, sample_rate, byte_limit, true, out);
    if (result != NES_FILE_OK) {
        return result;
    }
    NesCaptureStream *stream = *out;
    stream->codec = codec;
    stream->compression_level = compression_level;
    stream->width = width;
    stream->height = height;
    stream->fps_numerator = fps_numerator;
    stream->fps_denominator = fps_denominator;
    stream->row_bytes = (width * 3u + 3u) & ~3u;
    stream->frame_bytes = codec == NES_CAPTURE_CODEC_ZMBV ? width * height * 4u : stream->row_bytes * height;
    stream->row = malloc(stream->row_bytes);
    if (!stream->row) {
        nes_capture_abort(out);
        return NES_FILE_OUT_OF_MEMORY;
    }
    uint8_t header[AVI_HEADER_SIZE];
    avi_header(stream, header, 4, AVI_HEADER_SIZE + 8u);
    result = write_bytes(stream, header, sizeof(header));
    if (result != NES_FILE_OK) {
        nes_capture_abort(out);
    }
    return result;
}

static NesFileResult reserve_index(NesCaptureStream *stream, size_t count) {
    if (count > AVI_INDEX_MAX_ENTRIES - stream->index_count) {
        return NES_FILE_TOO_LARGE;
    }
    size_t required = stream->index_count + count;
    if (required <= stream->index_capacity) {
        return NES_FILE_OK;
    }
    size_t capacity = stream->index_capacity ? stream->index_capacity * 2u : 256u;
    if (capacity < required) {
        capacity = required;
    }
    if (capacity > AVI_INDEX_MAX_ENTRIES) {
        capacity = AVI_INDEX_MAX_ENTRIES;
    }
    CaptureIndexEntry *grown = realloc(stream->index, capacity * sizeof(*grown));
    if (!grown) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    stream->index = grown;
    stream->index_capacity = capacity;
    return NES_FILE_OK;
}

static void add_index(NesCaptureStream *stream, const char tag[4], uint32_t position, uint32_t size, uint32_t flags) {
    uint8_t *bytes = stream->index[stream->index_count++].bytes;
    memcpy(bytes, tag, 4);
    put32(bytes + 4, flags);
    put32(bytes + 8, position - (AVI_HEADER_SIZE - 4u));
    put32(bytes + 12, size);
}

NesFileResult nes_capture_write_audio(NesCaptureStream *stream, const int16_t *samples, size_t frames) {
    if (!stream || stream->video || (!samples && frames)) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (stream->error != NES_FILE_OK) {
        return stream->error;
    }
    uint64_t remaining = (stream->byte_limit - WAVE_HEADER_SIZE) / 4u;
    if ((uint64_t)frames > remaining - stream->audio_frames) {
        return NES_FILE_TOO_LARGE;
    }
    NesFileResult result = write_pcm(stream, samples, frames);
    if (result == NES_FILE_OK) {
        stream->audio_frames += frames;
    }
    return result;
}

NesFileResult nes_capture_write_frame(NesCaptureStream *stream, const NesCaptureFrame *frame, const int16_t *samples,
                                      size_t audio_frames) {
    if (!stream || !stream->video || !valid_frame(frame) || frame->width != stream->width ||
        frame->height != stream->height || (!samples && audio_frames)) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (stream->error != NES_FILE_OK) {
        return stream->error;
    }
    if (audio_frames > UINT32_MAX / 4u || stream->video_frames == UINT32_MAX ||
        (uint64_t)audio_frames > UINT32_MAX - stream->audio_frames) {
        return NES_FILE_TOO_LARGE;
    }
    uint32_t audio_bytes = (uint32_t)(audio_frames * 4u);
    size_t entries = audio_frames ? 2u : 1u;
    uint8_t *encoded = NULL;
    size_t encoded_size = stream->frame_bytes;
    if (stream->codec == NES_CAPTURE_CODEC_ZMBV) {
        NesFileResult encoded_result = nes_capture_zmbv(frame, stream->compression_level, &encoded, &encoded_size);
        if (encoded_result != NES_FILE_OK) {
            return encoded_result;
        }
    }
    uint64_t final_size = nes_file_transaction_size(stream->transaction) + 8u + encoded_size + (encoded_size & 1u) +
                          (audio_frames ? 8u + audio_bytes : 0u) + 8u +
                          (stream->index_count + entries) * AVI_INDEX_ENTRY_SIZE;
    if (final_size > stream->byte_limit) {
        free(encoded);
        return NES_FILE_TOO_LARGE;
    }
    NesFileResult result = reserve_index(stream, entries);
    if (result != NES_FILE_OK) {
        free(encoded);
        return result;
    }

    uint8_t chunk_header[8];
    uint32_t position = (uint32_t)nes_file_transaction_position(stream->transaction);
    const char *video_tag = encoded ? "00dc" : "00db";
    memcpy(chunk_header, video_tag, 4);
    put32(chunk_header + 4, (uint32_t)encoded_size);
    result = write_bytes(stream, chunk_header, sizeof(chunk_header));
    if (encoded && result == NES_FILE_OK) {
        result = write_bytes(stream, encoded, encoded_size);
    }
    for (unsigned y = frame->height; !encoded && result == NES_FILE_OK && y-- > 0;) {
        const uint32_t *pixels = frame->pixels + (size_t)y * frame->stride;
        memset(stream->row, 0, stream->row_bytes);
        for (unsigned x = 0; x < frame->width; ++x) {
            stream->row[x * 3u] = (uint8_t)pixels[x];
            stream->row[x * 3u + 1u] = (uint8_t)(pixels[x] >> 8);
            stream->row[x * 3u + 2u] = (uint8_t)(pixels[x] >> 16);
        }
        result = write_bytes(stream, stream->row, stream->row_bytes);
    }
    free(encoded);
    if (result == NES_FILE_OK && (encoded_size & 1u)) {
        result = write_bytes(stream, "", 1);
    }
    if (result != NES_FILE_OK) {
        return result;
    }
    add_index(stream, video_tag, position, (uint32_t)encoded_size, 0x10);
    if (audio_frames) {
        position = (uint32_t)nes_file_transaction_position(stream->transaction);
        memcpy(chunk_header, "01wb", 4);
        put32(chunk_header + 4, audio_bytes);
        result = write_bytes(stream, chunk_header, sizeof(chunk_header));
        if (result == NES_FILE_OK) {
            result = write_pcm(stream, samples, audio_frames);
        }
        if (result != NES_FILE_OK) {
            return result;
        }
        add_index(stream, "01wb", position, audio_bytes, 0);
    }
    ++stream->video_frames;
    stream->audio_frames += audio_frames;
    return NES_FILE_OK;
}

void nes_capture_abort(NesCaptureStream **stream_ptr) {
    if (!stream_ptr || !*stream_ptr) {
        return;
    }
    NesCaptureStream *stream = *stream_ptr;
    *stream_ptr = NULL;
    nes_file_transaction_abort(&stream->transaction);
    free(stream->index);
    free(stream->row);
    free(stream);
}

NesFileResult nes_capture_close(NesCaptureStream **stream_ptr) {
    if (!stream_ptr || !*stream_ptr) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesCaptureStream *stream = *stream_ptr;
    NesFileResult result = stream->error;
    if (result == NES_FILE_OK && stream->video) {
        uint32_t movi_bytes = (uint32_t)nes_file_transaction_size(stream->transaction) - (AVI_HEADER_SIZE - 4u);
        uint8_t index_header[8];
        memcpy(index_header, "idx1", 4);
        put32(index_header + 4, (uint32_t)(stream->index_count * AVI_INDEX_ENTRY_SIZE));
        result = write_bytes(stream, index_header, sizeof(index_header));
        if (result == NES_FILE_OK) {
            result = write_bytes(stream, stream->index, stream->index_count * sizeof(*stream->index));
        }
        uint8_t header[AVI_HEADER_SIZE];
        avi_header(stream, header, movi_bytes, (uint32_t)nes_file_transaction_size(stream->transaction));
        if (result == NES_FILE_OK) {
            result = nes_file_transaction_seek(stream->transaction, 0);
        }
        if (result == NES_FILE_OK) {
            result = write_bytes(stream, header, sizeof(header));
        }
    } else if (result == NES_FILE_OK) {
        uint8_t header[WAVE_HEADER_SIZE];
        wave_header(stream, header);
        result = nes_file_transaction_seek(stream->transaction, 0);
        if (result == NES_FILE_OK) {
            result = write_bytes(stream, header, sizeof(header));
        }
    }
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_commit(&stream->transaction);
    }
    nes_capture_abort(stream_ptr);
    return result;
}

uint64_t nes_capture_audio_frames(const NesCaptureStream *stream) {
    return stream ? stream->audio_frames : 0;
}

uint32_t nes_capture_video_frames(const NesCaptureStream *stream) {
    return stream ? stream->video_frames : 0;
}

uint64_t nes_capture_file_size(const NesCaptureStream *stream) {
    if (!stream) {
        return 0;
    }
    return nes_file_transaction_size(stream->transaction) +
           (stream->video ? 8u + stream->index_count * AVI_INDEX_ENTRY_SIZE : 0u);
}
