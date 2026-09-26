/* capture_extensions_accuracy.c - Decode capture output and exercise recovery paths
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../capture/capture_writer.h"
#include "../capture/capture_gif.h"
#include "../capture/capture_riff.h"
#include "../capture/capture_overlay.h"
#include "../capture/movie_preferences.h"
#include "../third_party/miniz/miniz.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#define mkdir_test(path) _mkdir(path)
#define rmdir_test(path) _rmdir(path)
#define process_id _getpid
#else
#include <sys/stat.h>
#include <unistd.h>
#define mkdir_test(path) mkdir(path, 0700)
#define rmdir_test(path) rmdir(path)
#define process_id getpid
#endif

#define REQUIRE(x)                                                                                                     \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x);                                                    \
            failed = 1;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static unsigned u16(const uint8_t *p) {
    return p[0] | (unsigned)p[1] << 8;
}

static uint32_t u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void p32(uint8_t *p, uint32_t n) {
    for (unsigned i = 0; i < 4; ++i) {
        p[i] = (uint8_t)(n >> (8u * i));
    }
}

static int test_avi(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/lossless.avi", directory);
    uint32_t pixels[17 * 11];
    for (unsigned i = 0; i < 17 * 11; ++i) {
        pixels[i] = 0xFF000000u | i * 17113u;
    }
    NesCaptureFrame frame = {pixels, 17, 11, 17};
    NesCaptureStream *stream = NULL;
    NesRiffReport report = {0};
    uint8_t *bytes = NULL;
    size_t size = 0;
    int16_t audio[] = {-32768, 32767, 0, 12, -45, 234};
    REQUIRE(nes_capture_avi_open_codec(path, 17, 11, 44100, 60000, 1001, 200000, NES_CAPTURE_CODEC_ZMBV, 10, &stream) ==
                NES_FILE_INVALID_ARGUMENT &&
            !stream);
    REQUIRE(nes_capture_avi_open_codec(path, 17, 11, 44100, 60000, 1001, 200000, NES_CAPTURE_CODEC_ZMBV, 6, &stream) ==
            NES_FILE_OK);
    uint32_t first_pixel = pixels[0];
    for (unsigned i = 0; i < 120; ++i) {
        pixels[0] = first_pixel ^ i;
        REQUIRE(nes_capture_write_frame(stream, &frame, audio, 3) == NES_FILE_OK);
    }
    pixels[0] = first_pixel;
    REQUIRE(nes_capture_video_frames(stream) == 120 && nes_capture_audio_frames(stream) == 360);
    REQUIRE(nes_capture_close(&stream) == NES_FILE_OK && !stream);
    REQUIRE(nes_file_read_all(path, 200000, &bytes, &size) == NES_FILE_OK);
    REQUIRE(!memcmp(bytes + 112, "ZMBV", 4) && !memcmp(bytes + 188, "ZMBV", 4));
    REQUIRE(u16(bytes + 186) == 32 && u32(bytes + 140) == 120 && u32(bytes + 264) == 360);
    size_t offset = 324;
    for (unsigned i = 0; i < 120; ++i) {
        REQUIRE(size - offset >= 15 && !memcmp(bytes + offset, "00dc", 4));
        uint32_t encoded = u32(bytes + offset + 4);
        REQUIRE(encoded >= 7 && encoded <= size - offset - 8);
        REQUIRE(!memcmp(bytes + offset + 8, "\1\0\1\1\10\20\20", 7));
        uint8_t raw[17 * 11 * 4];
        mz_ulong count = sizeof(raw);
        REQUIRE(mz_uncompress(raw, &count, bytes + offset + 15, encoded - 7) == MZ_OK && count == sizeof(raw));
        for (unsigned p = 0; p < 17 * 11; ++p) {
            REQUIRE(u32(raw + p * 4u) == ((pixels[p] ^ (p ? 0u : i)) & 0xFFFFFF));
        }
        offset += 8 + encoded + (encoded & 1u);
        REQUIRE(size - offset >= 20 && !memcmp(bytes + offset, "01wb", 4) && u32(bytes + offset + 4) == 12);
        for (unsigned a = 0; a < 6; ++a) {
            REQUIRE(u16(bytes + offset + 8 + a * 2) == (uint16_t)audio[a]);
        }
        offset += 20;
    }
    REQUIRE(!memcmp(bytes + offset, "idx1", 4));
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK);
    REQUIRE(!report.warnings && report.video_frames == 120 && report.width == 17 && report.height == 11);
    uint32_t original_offset = u32(bytes + offset + 16);
    p32(bytes + offset + 16, UINT32_MAX);
    REQUIRE(nes_file_write_atomic(path, bytes, size) == NES_FILE_OK);
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK && report.warnings);
    p32(bytes + offset + 16, original_offset);
    p32(bytes + 48, 121);
    REQUIRE(nes_file_write_atomic(path, bytes, size) == NES_FILE_OK);
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK && report.warnings);
    p32(bytes + 48, 120);
    REQUIRE(nes_file_write_atomic(path, bytes, size) == NES_FILE_OK);
    free(bytes);
    bytes = NULL;
    REQUIRE(nes_capture_avi_open_codec(path, 17, 11, 44100, 50, 1, 340, NES_CAPTURE_CODEC_ZMBV, 0, &stream) ==
            NES_FILE_OK);
    REQUIRE(nes_capture_write_frame(stream, &frame, audio, 3) == NES_FILE_TOO_LARGE);
    nes_capture_abort(&stream);
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK && report.video_frames == 120);
cleanup:
    nes_capture_abort(&stream);
    nes_capture_riff_free(&report);
    free(bytes);
    if (!getenv("CUPID_CAPTURE_KEEP")) {
        (void)nes_file_remove(path);
    }
    return failed;
}

/* Independent GIF LZW reader: dictionary entries are reconstructed from codes,
 * including the code==next special case and clear codes at full dictionaries. */
static bool lzw_decode(const uint8_t *input, size_t bytes, uint8_t *out, size_t pixels) {
    uint16_t prefix[4096];
    uint8_t suffix[4096], stack[4096];
    unsigned width = 9, next = 258, old = 4096, first = 0;
    size_t bit = 0, written = 0;
    while (bit + width <= bytes * 8u) {
        unsigned value = 0;
        for (unsigned i = 0; i < width; ++i) {
            value |= ((input[(bit + i) / 8] >> ((bit + i) & 7)) & 1u) << i;
        }
        bit += width;
        if (value == 256) {
            width = 9;
            next = 258;
            old = 4096;
            continue;
        }
        if (value == 257) {
            return written == pixels;
        }
        if (value > next || value >= 4096) {
            return false;
        }
        unsigned original = value, depth = 0;
        if (value == next) {
            if (old == 4096) {
                return false;
            }
            stack[depth++] = (uint8_t)first;
            value = old;
        }
        while (value >= 256) {
            if (value >= next || depth == 4096) {
                return false;
            }
            stack[depth++] = suffix[value];
            value = prefix[value];
        }
        first = value;
        if (depth == 4096) {
            return false;
        }
        stack[depth++] = (uint8_t)first;
        if (depth > pixels - written) {
            return false;
        }
        while (depth) {
            out[written++] = stack[--depth];
        }
        if (old != 4096 && next < 4096) {
            prefix[next] = (uint16_t)old;
            suffix[next++] = (uint8_t)first;
            if (next == (1u << width) && width < 12) {
                ++width;
            }
        }
        old = original;
    }
    return false;
}

static bool decode_gif(const uint8_t *file, size_t size, const NesCaptureFrame *frame, unsigned scale,
                       unsigned expected_frames, uint32_t numerator, uint32_t denominator) {
    if (size < 801 || memcmp(file, "GIF89a", 6) || file[size - 1] != ';' || u16(file + 6) != frame->width * scale ||
        u16(file + 8) != frame->height * scale) {
        return false;
    }
    size_t pixels = (size_t)frame->width * frame->height * scale * scale;
    uint8_t *compressed = malloc(size), *decoded = malloc(pixels);
    if (!compressed || !decoded) {
        free(compressed);
        free(decoded);
        return false;
    }
    bool ok = true;
    size_t offset = 800;
    unsigned frames = 0;
    uint64_t delay = 0;
    while (ok && offset < size - 1) {
        if (size - offset < 19 || memcmp(file + offset, "\x21\xF9\4\4", 4) || file[offset + 8] != 0x2C ||
            file[offset + 18] != 8) {
            ok = false;
            break;
        }
        delay += u16(file + offset + 4);
        offset += 19;
        size_t count = 0;
        while (offset < size && file[offset]) {
            unsigned n = file[offset++];
            if (n > size - offset) {
                ok = false;
                break;
            }
            memcpy(compressed + count, file + offset, n);
            count += n;
            offset += n;
        }
        if (!ok || offset >= size) {
            ok = false;
            break;
        }
        ++offset;
        if (!lzw_decode(compressed, count, decoded, pixels)) {
            ok = false;
            break;
        }
        for (size_t p = 0; p < pixels; ++p) {
            uint32_t color =
                frame
                    ->pixels[(p / (frame->width * scale) / scale) * frame->stride + p % (frame->width * scale) / scale];
            if (p / (frame->width * scale) < scale && p % (frame->width * scale) < scale) {
                color ^= frames * 0x234567u;
            }
            unsigned index = ((color >> 16) & 0xE0) | ((color >> 11) & 0x1C) | ((color >> 6) & 3);
            if (decoded[p] != index) {
                ok = false;
                break;
            }
        }
        ++frames;
    }
    ok = ok && frames == expected_frames && delay == (uint64_t)frames * denominator * 100u / numerator;
    free(compressed);
    free(decoded);
    return ok;
}

static int test_gif(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/animated.gif", directory);
    NesCaptureGif *gif = NULL;
    uint8_t *data = NULL;
    size_t size = 0;
    uint32_t pixels[128 * 97], rng = 12345;
    for (unsigned i = 0; i < 128 * 97; ++i) {
        rng = rng * 1664525u + 1013904223u;
        pixels[i] = rng | 0xFF000000;
    }
    NesCaptureFrame frame = {pixels, 128, 97, 128};
    REQUIRE(nes_capture_gif_open(path, 128, 97, 0, 60, 1, 4000000, &gif) == NES_FILE_INVALID_ARGUMENT);
    REQUIRE(nes_capture_gif_open("no-such-capture-dir/test.gif", 128, 97, 1, 60, 1, 4000000, &gif) != NES_FILE_OK &&
            !gif);
    for (unsigned region = 0; region < 3; ++region) {
        uint32_t numerator = region == 0 ? 60000 : 50, denominator = region == 0 ? 1001 : 1;
        unsigned scale = region == 0 ? 1 : region + 1;
        REQUIRE(nes_capture_gif_open(path, 128, 97, scale, numerator, denominator, 4000000, &gif) == NES_FILE_OK);
        uint32_t first_pixel = pixels[0];
        for (unsigned i = 0; i < 5; ++i) {
            pixels[0] = first_pixel ^ (i * 0x234567u);
            REQUIRE(nes_capture_gif_frame(gif, &frame) == NES_FILE_OK);
        }
        pixels[0] = first_pixel;
        uint64_t paused_size = nes_capture_gif_size(gif);
        REQUIRE(nes_capture_gif_size(gif) == paused_size);
        REQUIRE(nes_capture_gif_close(&gif) == NES_FILE_OK && !gif);
        REQUIRE(nes_file_read_all(path, 4000000, &data, &size) == NES_FILE_OK);
        REQUIRE(decode_gif(data, size, &frame, scale, 5, numerator, denominator));
        free(data);
        data = NULL;
    }
    REQUIRE(nes_capture_gif_open(path, 128, 97, 1, 50, 1, 802, &gif) == NES_FILE_OK);
    REQUIRE(nes_capture_gif_frame(gif, &frame) == NES_FILE_TOO_LARGE);
    nes_capture_gif_abort(&gif);
    REQUIRE(nes_file_read_all(path, 4000000, &data, &size) == NES_FILE_OK);
    REQUIRE(decode_gif(data, size, &frame, 3, 5, 50, 1));
cleanup:
    nes_capture_gif_abort(&gif);
    free(data);
    if (!getenv("CUPID_CAPTURE_KEEP")) {
        (void)nes_file_remove(path);
    }
    return failed;
}

static int test_riff_damage(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/damage.avi", directory);
    NesRiffReport report = {0};
    uint8_t file[12 + 20 * 12] = {0};
    memcpy(file, "RIFF", 4);
    p32(file + 4, sizeof(file) - 8);
    memcpy(file + 8, "AVI ", 4);
    for (unsigned i = 0; i < 20; ++i) {
        size_t offset = 12 + i * 12;
        memcpy(file + offset, "LIST", 4);
        p32(file + offset + 4, (uint32_t)(sizeof(file) - offset - 8));
        memcpy(file + offset + 8, "nest", 4);
    }
    REQUIRE(nes_file_write_atomic(path, file, sizeof(file)) == NES_FILE_OK);
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK && report.warnings &&
            report.count <= NES_RIFF_DEPTH_LIMIT + 1);
    p32(file + 4, UINT32_MAX);
    REQUIRE(nes_file_write_atomic(path, file, 13) == NES_FILE_OK);
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK && report.warnings >= 2);
    NesRiffNode *before = report.nodes;
    REQUIRE(nes_capture_riff_inspect("missing-riff-file.avi", &report) == NES_FILE_NOT_FOUND && report.nodes == before);
    REQUIRE(nes_file_write_atomic(path, "", 0) == NES_FILE_OK);
    REQUIRE(nes_capture_riff_inspect(path, &report) == NES_FILE_OK && report.warnings == 1);
cleanup:
    nes_capture_riff_free(&report);
    (void)nes_file_remove(path);
    return failed;
}

static bool file_text(const char *path, const char *expected) {
    uint8_t *data = NULL;
    size_t size = 0;
    bool ok = nes_file_read_all(path, 1024, &data, &size) == NES_FILE_OK && size == strlen(expected) &&
              !memcmp(data, expected, size);
    free(data);
    return ok;
}

static int test_backup(const char *directory) {
    int failed = 0;
    char path[256], backup[NES_FILE_PATH_LIMIT], custom[256];
    snprintf(path, sizeof(path), "%s/movie.fm2", directory);
    snprintf(custom, sizeof(custom), "%s/recovery", directory);
    FILE *locked = NULL;
    NesMovieBackupOptions options;
    nes_movie_backup_defaults(&options);
    options.retained = 2;
    REQUIRE(nes_movie_backup_write(path, "one", 3, &options) == NES_FILE_OK);
    REQUIRE(nes_movie_backup_write(path, "two", 3, &options) == NES_FILE_OK);
    REQUIRE(nes_movie_backup_write(path, "three", 5, &options) == NES_FILE_OK);
    REQUIRE(nes_movie_backup_path(path, &options, 1, backup, sizeof(backup)) == NES_FILE_OK &&
            file_text(backup, "two"));
    REQUIRE(nes_movie_backup_path(path, &options, 2, backup, sizeof(backup)) == NES_FILE_OK &&
            file_text(backup, "one"));
    REQUIRE(file_text(path, "three"));
#ifdef _WIN32
    /* An open read handle denies replacement while recovery copies are saved. */
    locked = nes_file_open(path, "rb");
    REQUIRE(locked);
    REQUIRE(nes_movie_backup_write(path, "locked", 6, &options) == NES_FILE_IO_ERROR);
    REQUIRE(file_text(path, "three"));
    fclose(locked);
    locked = NULL;
    REQUIRE(nes_movie_backup_path(path, &options, 1, backup, sizeof(backup)) == NES_FILE_OK &&
            file_text(backup, "three"));
#endif
    /* A bad recovery directory prevents replacement of the active file. */
    strcpy(options.directory, custom);
    REQUIRE(nes_movie_backup_write(path, "bad", 3, &options) != NES_FILE_OK && file_text(path, "three"));
    REQUIRE(mkdir_test(custom) == 0);
    REQUIRE(nes_movie_backup_write(path, "four", 4, &options) == NES_FILE_OK);
    REQUIRE(nes_movie_backup_path(path, &options, 1, backup, sizeof(backup)) == NES_FILE_OK &&
            file_text(backup, "three"));
    REQUIRE(nes_movie_backup_write(path, "three", 5, &options) == NES_FILE_OK && file_text(path, "three"));
    options.retained = 1;
    REQUIRE(nes_movie_backup_write(path, "five", 4, &options) == NES_FILE_OK);
    REQUIRE(nes_movie_backup_path(path, &options, 2, backup, sizeof(backup)) == NES_FILE_OK);
    uint8_t *data = NULL;
    size_t size = 0;
    REQUIRE(nes_file_read_all(backup, 100, &data, &size) == NES_FILE_NOT_FOUND);
    options.enabled = false;
    REQUIRE(nes_movie_backup_write(path, "six", 3, &options) == NES_FILE_OK && file_text(path, "six"));
    REQUIRE(nes_movie_backup_path(path, &options, 1, backup, sizeof(backup)) == NES_FILE_OK &&
            file_text(backup, "three"));
cleanup:
    if (locked) {
        fclose(locked);
    }
    for (unsigned custom_path = 0; custom_path < 2; ++custom_path) {
        snprintf(options.directory, sizeof(options.directory), "%s", custom_path ? custom : "");
        for (unsigned i = 1; i <= 5; ++i) {
            if (nes_movie_backup_path(path, &options, i, backup, sizeof(backup)) == NES_FILE_OK) {
                (void)nes_file_remove(backup);
            }
        }
    }
    (void)nes_file_remove(path);
    (void)rmdir_test(custom);
    return failed;
}

static int test_overlays(void) {
    int failed = 0;
    NesMovieSubtitles *track = NULL;
    NesCaptureOverlay overlay = {0};
    uint32_t *pixels = malloc(512 * 480 * 4u), *saved = malloc(512 * 480 * 4u);
    REQUIRE(pixels && saved);
    for (unsigned i = 0; i < 512 * 480; ++i) {
        pixels[i] = 0xFF123456;
    }
    char *records[] = {"20 Later", "0 First", "20 Latest", "50 "};
    NesFm2StringList list = {records, 4};
    REQUIRE(nes_movie_subtitles_load(&track, &list) == NES_FILE_OK);
    REQUIRE(!strcmp(nes_movie_subtitle_at(track, 0, 180), "First"));
    REQUIRE(!strcmp(nes_movie_subtitle_at(track, 19, 180), "First"));
    REQUIRE(!strcmp(nes_movie_subtitle_at(track, 20, 180), "Latest"));
    REQUIRE(!strcmp(nes_movie_subtitle_at(track, 49, 180), "Latest"));
    REQUIRE(!*nes_movie_subtitle_at(track, 50, 180));
    REQUIRE(!strcmp(nes_movie_subtitle_at(track, 5, 180), "First"));
    REQUIRE(!*nes_movie_subtitle_at(track, 19, 10));
    char *bad[] = {"18446744073709551616 invalid"};
    list = (NesFm2StringList){bad, 1};
    REQUIRE(nes_movie_subtitles_load(&track, &list) != NES_FILE_OK);
    REQUIRE(!strcmp(nes_movie_subtitle_at(track, 0, 180), "First"));
    char *invalid_utf8[] = {"0 \xF0\x80"};
    list = (NesFm2StringList){invalid_utf8, 1};
    REQUIRE(nes_movie_subtitles_load(&track, &list) == NES_FILE_INVALID_ARGUMENT);
    NesCaptureOverlayState state = {.pads = {1, 2, 4, 8, 16, 32},
                                    .active_players = 63,
                                    .frame = 20,
                                    .movie_active = true,
                                    .subtitle = "Scheduled subtitle",
                                    .status = "TAS Playback"};
    NesCaptureFrame source = {pixels, 512, 480, 512}, output;
    REQUIRE(nes_capture_overlay_compose(&overlay, &source, &state, 0, NES_OVERLAY_TOP_LEFT, &output) == NES_FILE_OK);
    REQUIRE(output.pixels == pixels && !overlay.pixels);
    for (unsigned bit = 1; bit <= 8; bit <<= 1) {
        REQUIRE(nes_capture_overlay_compose(&overlay, &source, &state, bit, NES_OVERLAY_TOP_LEFT, &output) ==
                NES_FILE_OK);
        REQUIRE(memcmp(output.pixels, pixels, 512 * 480 * 4u));
        memcpy(saved, output.pixels, 512 * 480 * 4u);
        REQUIRE(nes_capture_overlay_compose(&overlay, &source, &state, bit, NES_OVERLAY_TOP_LEFT, &output) ==
                NES_FILE_OK);
        REQUIRE(!memcmp(saved, output.pixels, 512 * 480 * 4u));
    }
    for (unsigned position = 0; position < 4; ++position) {
        REQUIRE(nes_capture_overlay_compose(&overlay, &source, &state, 15, (NesOverlayPosition)position, &output) ==
                NES_FILE_OK);
    }
    for (unsigned i = 0; i < 512 * 480; ++i) {
        REQUIRE(pixels[i] == 0xFF123456);
    }
    state.movie_active = false;
    REQUIRE(nes_capture_overlay_compose(&overlay, &source, &state, 2, NES_OVERLAY_TOP_LEFT, &output) == NES_FILE_OK);
    REQUIRE(!memcmp(output.pixels, pixels, 512 * 480 * 4u));
cleanup:
    free(pixels);
    free(saved);
    nes_capture_overlay_destroy(&overlay);
    nes_movie_subtitles_free(&track);
    return failed;
}

static int test_preferences(void) {
    int failed = 0;
    NesMoviePreferences before, after;
    nes_movie_preferences_defaults(&before);
    nes_movie_preferences_defaults(&after);
    REQUIRE(before.read_only && before.subtitles && before.backup.enabled && before.capture_overlays == 0);
    REQUIRE(nes_movie_preferences_set(&before, "movie.end", "1"));
    REQUIRE(nes_movie_preferences_set(&before, "movie.read_only", "0"));
    REQUIRE(nes_movie_preferences_set(&before, "movie.record_insert", "1"));
    REQUIRE(nes_movie_preferences_set(&before, "movie.record_power_on", "1"));
    REQUIRE(nes_movie_preferences_set(&before, "movie.capture_overlays", "15"));
    REQUIRE(nes_movie_preferences_set(&before, "movie.backup_directory", "movies/recovery"));
    for (unsigned i = 0; i < NES_MOVIE_PREFERENCE_COUNT; ++i) {
        char value[1024], checked[1024];
        const char *key = nes_movie_preferences_key(i);
        REQUIRE(nes_movie_preferences_get(&before, key, value, sizeof(value)));
        REQUIRE(nes_movie_preferences_set(&after, key, value));
        REQUIRE(nes_movie_preferences_get(&after, key, checked, sizeof(checked)) && !strcmp(value, checked));
    }
    REQUIRE(!nes_movie_preferences_set(&after, "movie.backup_count", "0"));
    REQUIRE(!nes_movie_preferences_set(&after, "movie.read_only", "2"));
    REQUIRE(!nes_movie_preferences_set(&after, "movie.capture_overlays", "16"));
    REQUIRE(!nes_movie_preferences_set(&after, "movie.backup_directory", "injected\nsetting=1"));
    REQUIRE(!nes_movie_preferences_set(&after, "unknown", "1"));
    REQUIRE(after.backup.retained == before.backup.retained &&
            !strcmp(after.backup.directory, before.backup.directory));
cleanup:
    return failed;
}

int test_capture_extensions_accuracy(void) {
    char directory[128];
    snprintf(directory, sizeof(directory), "capture-extensions-%ld", (long)process_id());
    if (mkdir_test(directory)) {
        return 1;
    }
    int failed = test_avi(directory) + test_gif(directory) + test_riff_damage(directory) + test_backup(directory) +
                 test_overlays() + test_preferences();
    if (!getenv("CUPID_CAPTURE_KEEP")) {
        (void)rmdir_test(directory);
    } else {
        printf("Capture fixtures: %s\n", directory);
    }
    printf("Capture extensions: %s\n", failed ? "FAIL" : "PASS");
    return failed;
}

#ifdef CUPID_CAPTURE_TEST_MAIN
int main(void) {
    return test_capture_extensions_accuracy();
}
#endif
