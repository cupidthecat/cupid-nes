/*
 * capture_container_accuracy.c - Encoded pixels, PCM, AVI indexes and file failures
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../capture/capture_writer.h"
#include "../third_party/miniz/miniz.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        failed = 1; \
        goto cleanup; \
    } \
} while (0)

static uint32_t little32(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t big32(const uint8_t *data) {
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16)
        | ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

/* Independently inspect the PNG chunk structure, CRCs, deflate data and RGBA
 * rows. The screenshot writer deliberately uses PNG filter zero. */
int capture_test_png_matches(const char *path, const NesCaptureFrame *frame) {
    int failed = 0;
    uint8_t *file = NULL, *compressed = NULL, *decoded = NULL;
    size_t size = 0, compressed_size = 0;
    bool header = false, end = false;
    CHECK(frame && frame->pixels && frame->width && frame->height);
    CHECK(nes_file_read_all(path, 80u * 1024u * 1024u, &file, &size) == NES_FILE_OK);
    CHECK(size >= 57 && !memcmp(file, "\x89PNG\r\n\x1A\n", 8));
    compressed = malloc(size);
    CHECK(compressed);
    size_t offset = 8;
    while (offset < size) {
        CHECK(size - offset >= 12);
        uint32_t length = big32(file + offset);
        CHECK(length <= size - offset - 12);
        const uint8_t *tag = file + offset + 4;
        const uint8_t *data = tag + 4;
        CHECK((uint32_t)mz_crc32(0, tag, (size_t)length + 4u) == big32(data + length));
        if (!memcmp(tag, "IHDR", 4)) {
            CHECK(!header && offset == 8 && length == 13);
            CHECK(big32(data) == frame->width && big32(data + 4) == frame->height);
            CHECK(data[8] == 8 && data[9] == 6 && !data[10] && !data[11] && !data[12]);
            header = true;
        } else if (!memcmp(tag, "IDAT", 4)) {
            CHECK(header && !end && length <= size - compressed_size);
            memcpy(compressed + compressed_size, data, length);
            compressed_size += length;
        } else if (!memcmp(tag, "IEND", 4)) {
            CHECK(header && !end && length == 0 && offset + 12 == size);
            end = true;
        }
        offset += (size_t)length + 12;
    }
    CHECK(header && end && compressed_size);
    size_t row_bytes = (size_t)frame->width * 4u + 1u;
    mz_ulong decoded_size = (mz_ulong)(row_bytes * frame->height);
    decoded = malloc(decoded_size);
    CHECK(decoded);
    CHECK(mz_uncompress(decoded, &decoded_size, compressed, (mz_ulong)compressed_size) == MZ_OK);
    CHECK(decoded_size == row_bytes * frame->height);
    for (unsigned y = 0; y < frame->height; ++y) {
        const uint8_t *row = decoded + y * row_bytes;
        CHECK(row[0] == 0);
        for (unsigned x = 0; x < frame->width; ++x) {
            uint32_t pixel = frame->pixels[y * frame->stride + x];
            CHECK(row[1 + x * 4u] == (uint8_t)(pixel >> 16));
            CHECK(row[2 + x * 4u] == (uint8_t)(pixel >> 8));
            CHECK(row[3 + x * 4u] == (uint8_t)pixel);
            CHECK(row[4 + x * 4u] == (uint8_t)(pixel >> 24));
        }
    }
cleanup:
    free(decoded);
    free(compressed);
    free(file);
    return failed;
}

static int test_transaction(const char *directory) {
    int failed = 0;
    char path[256];
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileTransaction *transaction = NULL;
    snprintf(path, sizeof(path), "%s/\xC3\xA9-\xE7\x8C\xAB-\xF0\x9F\x92\xBE.bin", directory);
    CHECK(nes_file_write_atomic(path, "original", 8) == NES_FILE_OK);
    CHECK(nes_file_transaction_begin(path, 12, &transaction) == NES_FILE_OK);
    CHECK(nes_file_transaction_write(transaction, "headpayload!", 12) == NES_FILE_OK);
    CHECK(nes_file_transaction_position(transaction) == 12 && nes_file_transaction_size(transaction) == 12);
    CHECK(nes_file_read_all(path, 16, &data, &size) == NES_FILE_OK);
    CHECK(size == 8 && !memcmp(data, "original", 8));
    free(data); data = NULL;
    CHECK(nes_file_transaction_seek(transaction, 0) == NES_FILE_OK);
    CHECK(nes_file_transaction_write(transaction, "HEAD", 4) == NES_FILE_OK);
    CHECK(nes_file_transaction_size(transaction) == 12 && nes_file_transaction_position(transaction) == 4);
    CHECK(nes_file_transaction_commit(&transaction) == NES_FILE_OK && !transaction);
    CHECK(nes_file_read_all(path, 16, &data, &size) == NES_FILE_OK);
    CHECK(size == 12 && !memcmp(data, "HEADpayload!", 12));
    free(data); data = NULL;

    CHECK(nes_file_transaction_begin(path, 4, &transaction) == NES_FILE_OK);
    CHECK(nes_file_transaction_write(transaction, "discard", 7) == NES_FILE_TOO_LARGE);
    CHECK(nes_file_transaction_commit(&transaction) == NES_FILE_TOO_LARGE && !transaction);
    CHECK(nes_file_transaction_begin(path, 16, &transaction) == NES_FILE_OK);
    CHECK(nes_file_transaction_write(transaction, "discard", 7) == NES_FILE_OK);
    nes_file_transaction_abort(&transaction);
    CHECK(!transaction);
    CHECK(nes_file_transaction_begin(path, 16, &transaction) == NES_FILE_OK);
    CHECK(nes_file_transaction_seek(transaction, 1) == NES_FILE_INVALID_ARGUMENT);
    CHECK(nes_file_transaction_commit(&transaction) == NES_FILE_INVALID_ARGUMENT && !transaction);
    CHECK(nes_file_read_all(path, 16, &data, &size) == NES_FILE_OK);
    CHECK(size == 12 && !memcmp(data, "HEADpayload!", 12));
    free(data); data = NULL;
    CHECK(nes_file_transaction_begin(path, 0, &transaction) == NES_FILE_OK);
    CHECK(nes_file_transaction_write(transaction, NULL, 0) == NES_FILE_OK);
    CHECK(nes_file_transaction_commit(&transaction) == NES_FILE_OK);
    CHECK(nes_file_read_all(path, 0, &data, &size) == NES_FILE_OK && size == 0);
cleanup:
    nes_file_transaction_abort(&transaction);
    free(data);
    (void)nes_file_remove(path);
    return failed;
}

static int test_png(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/pixels.png", directory);
    const uint32_t pixels[] = {
        0xFF112233, 0x80446688, 0xDEADBEEF,
        0x00778899, 0xFFFFFFFF, 0xDEADBEEF
    };
    NesCaptureFrame frame = {pixels, 2, 2, 3};
    CHECK(nes_capture_png(path, &frame) == NES_FILE_OK);
    CHECK(capture_test_png_matches(path, &frame) == 0);
    frame.stride = 1;
    CHECK(nes_capture_png(path, &frame) == NES_FILE_INVALID_ARGUMENT);
    frame.stride = 3;
    frame.height = NES_CAPTURE_MAX_DIMENSION + 1u;
    CHECK(nes_capture_png(path, &frame) == NES_FILE_INVALID_ARGUMENT);
    frame.height = 2;
    CHECK(capture_test_png_matches(path, &frame) == 0);
cleanup:
    (void)nes_file_remove(path);
    return failed;
}

static int test_wave(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/audio.wav", directory);
    NesCaptureStream *stream = NULL;
    uint8_t *data = NULL;
    size_t size = 0;
    const int16_t samples[] = {32767, -32768, 0x1234, -2};
    CHECK(nes_file_write_atomic(path, "kept", 4) == NES_FILE_OK);
    CHECK(nes_capture_wav_open(path, 48000, 52, &stream) == NES_FILE_OK);
    CHECK(nes_capture_write_audio(stream, samples, 1) == NES_FILE_OK);
    CHECK(nes_capture_write_audio(stream, samples + 2, 1) == NES_FILE_OK);
    CHECK(nes_capture_audio_frames(stream) == 2 && nes_capture_file_size(stream) == 52);
    CHECK(nes_capture_write_audio(stream, samples, 1) == NES_FILE_TOO_LARGE);
    CHECK(nes_file_read_all(path, 64, &data, &size) == NES_FILE_OK && size == 4 && !memcmp(data, "kept", 4));
    free(data); data = NULL;
    CHECK(nes_capture_close(&stream) == NES_FILE_OK && !stream);
    CHECK(nes_file_read_all(path, 64, &data, &size) == NES_FILE_OK && size == 52);
    CHECK(!memcmp(data, "RIFF", 4) && little32(data + 4) == size - 8 && !memcmp(data + 8, "WAVEfmt ", 8));
    CHECK(little32(data + 16) == 16 && data[20] == 1 && data[22] == 2);
    CHECK(little32(data + 24) == 48000 && little32(data + 28) == 192000);
    CHECK(data[32] == 4 && data[34] == 16 && !memcmp(data + 36, "data", 4));
    CHECK(little32(data + 40) == 8 && !memcmp(data + 44, "\xFF\x7F\0\x80\x34\x12\xFE\xFF", 8));
    free(data); data = NULL;
    CHECK(nes_capture_wav_open(path, 7999, 1024, &stream) == NES_FILE_INVALID_ARGUMENT && !stream);
    CHECK(nes_capture_wav_open(path, 44100, 43, &stream) == NES_FILE_INVALID_ARGUMENT && !stream);
    CHECK(nes_capture_wav_open(directory, 44100, 1024, &stream) == NES_FILE_OK);
    CHECK(nes_capture_write_audio(stream, samples, 2) == NES_FILE_OK);
    CHECK(nes_capture_close(&stream) == NES_FILE_IO_ERROR && !stream);
    CHECK(nes_file_read_all(path, 64, &data, &size) == NES_FILE_OK && size == 52);
cleanup:
    nes_capture_abort(&stream);
    free(data);
    (void)nes_file_remove(path);
    return failed;
}

static int test_avi(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/video.avi", directory);
    NesCaptureStream *stream = NULL;
    uint8_t *data = NULL;
    size_t size = 0;
    const uint32_t pixels[] = {0xFF112233,0xFF445566,0xFF778899, 0xFFAABBCC,0xFFDDEEFF,0xFF010203};
    NesCaptureFrame frame = {pixels, 3, 2, 3};
    const int16_t samples[] = {0x1234,-2,32767,-32768};
    /* Each frame is 24 DIB bytes plus two stereo samples, with two index entries. */
    const uint64_t exact_limit = 324 + 2 * (8 + 24 + 8 + 8) + 8 + 4 * 16;
    CHECK(nes_capture_avi_open(path, 3, 2, 44100, 60000, 1001, exact_limit, &stream) == NES_FILE_OK);
    CHECK(nes_capture_write_frame(stream, &frame, samples, 2) == NES_FILE_OK);
    CHECK(nes_capture_write_frame(stream, &frame, samples, 2) == NES_FILE_OK);
    CHECK(nes_capture_write_frame(stream, &frame, samples, 2) == NES_FILE_TOO_LARGE);
    CHECK(nes_capture_audio_frames(stream) == 4 && nes_capture_video_frames(stream) == 2);
    CHECK(nes_capture_file_size(stream) == exact_limit);
    CHECK(nes_capture_close(&stream) == NES_FILE_OK);
    CHECK(nes_file_read_all(path, 1024, &data, &size) == NES_FILE_OK && size == exact_limit);
    CHECK(!memcmp(data, "RIFF", 4) && little32(data + 4) == size - 8 && !memcmp(data + 8, "AVI ", 4));
    CHECK(little32(data + 48) == 2 && little32(data + 56) == 2);
    CHECK(little32(data + 64) == 3 && little32(data + 68) == 2);
    CHECK(!memcmp(data + 108, "vidsDIB ", 8));
    CHECK(little32(data + 128) == 1001 && little32(data + 132) == 60000 && little32(data + 140) == 2);
    CHECK(little32(data + 176) == 3 && little32(data + 180) == 2 && data[186] == 24);
    CHECK(!memcmp(data + 232, "auds", 4) && little32(data + 264) == 4 && little32(data + 276) == 4);
    CHECK(!memcmp(data + 320, "movi", 4) && little32(data + 316) == 100);
    CHECK(!memcmp(data + 324, "00db", 4) && little32(data + 328) == 24);
    CHECK(!memcmp(data + 332, "\xCC\xBB\xAA\xFF\xEE\xDD\x03\x02\x01\0\0\0", 12));
    CHECK(!memcmp(data + 344, "\x33\x22\x11\x66\x55\x44\x99\x88\x77\0\0\0", 12));
    CHECK(!memcmp(data + 356, "01wb", 4) && little32(data + 360) == 8);
    CHECK(!memcmp(data + 364, "\x34\x12\xFE\xFF\xFF\x7F\0\x80", 8));
    CHECK(!memcmp(data + 420, "idx1", 4) && little32(data + 424) == 64);
    for (unsigned i = 0; i < 4; ++i) {
        const uint8_t *entry = data + 428 + i * 16u;
        uint32_t position = 320 + little32(entry + 8);
        CHECK(position + 8 < 420 && !memcmp(entry, data + position, 4));
        CHECK(little32(entry + 12) == little32(data + position + 4));
        CHECK(little32(entry + 4) == (i % 2 ? 0u : 0x10u));
    }
    free(data); data = NULL;
    CHECK(nes_capture_avi_open(path, 3, 2, 44100, 60, 1, 1024, &stream) == NES_FILE_OK);
    frame.width = 2;
    CHECK(nes_capture_write_frame(stream, &frame, samples, 2) == NES_FILE_INVALID_ARGUMENT);
    CHECK(nes_capture_close(&stream) == NES_FILE_OK);
    CHECK(nes_file_read_all(path, 1024, &data, &size) == NES_FILE_OK && size == 332);
    CHECK(little32(data + 48) == 0 && !memcmp(data + 324, "idx1", 4) && little32(data + 328) == 0);
cleanup:
    nes_capture_abort(&stream);
    free(data);
    (void)nes_file_remove(path);
    return failed;
}

int test_capture_container_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    snprintf(directory, sizeof(directory), "build/capture-containers-%u", (unsigned)_getpid());
    if (_mkdir(directory) != 0) return 1;
#else
    snprintf(directory, sizeof(directory), "build/capture-containers-%u", (unsigned)getpid());
    if (mkdir(directory, 0700) != 0) return 1;
#endif
    int failures = test_transaction(directory) + test_png(directory) + test_wave(directory) + test_avi(directory);
#ifdef _WIN32
    if (_rmdir(directory) != 0) ++failures;
#else
    if (rmdir(directory) != 0) ++failures;
#endif
    printf("Capture containers: 4 groups, %d failures\n", failures);
    return failures;
}
