/*
 * family_basic.c - Family BASIC keyboard matrix and cassette signal
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "family_basic.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

static uint8_t matrix[9];
static unsigned scan_row;
static bool scan_half;
static bool enabled;
static struct {
    uint8_t *data;
    size_t capacity;
    size_t samples;
    uint64_t cycle;
    FamilyBasicTapeMode mode;
    bool failed;
} tape;

void family_basic_reset(void) {
    memset(matrix, 0, sizeof(matrix));
    scan_row = 0;
    scan_half = false;
    enabled = false;
    tape.mode = FB_TAPE_STOPPED;
}

void family_basic_shutdown(void) {
    free(tape.data);
    memset(&tape, 0, sizeof(tape));
    family_basic_reset();
}

bool family_basic_set_key(FamilyBasicKey key, bool pressed) {
    if ((unsigned)key >= FB_KEY_COUNT) return false;
    unsigned row = (unsigned)key / 8;
    uint8_t bit = (uint8_t)(1u << ((unsigned)key & 7u));
    if (pressed) matrix[row] |= bit;
    else matrix[row] &= (uint8_t)~bit;
    return true;
}

static bool reserve_samples(size_t count) {
    if (count > SIZE_MAX - 7) return false;
    size_t bytes = (count + 7) / 8;
    if (bytes <= tape.capacity) return true;
    size_t capacity = tape.capacity ? tape.capacity : 4096;
    while (capacity < bytes) {
        if (capacity > SIZE_MAX / 2) { capacity = bytes; break; }
        capacity *= 2;
    }
    uint8_t *grown = realloc(tape.data, capacity);
    if (!grown) return false;
    tape.data = grown;
    tape.capacity = capacity;
    return true;
}

static bool append_samples(uint64_t count, bool high) {
    if (count > SIZE_MAX - tape.samples || !reserve_samples(tape.samples + (size_t)count))
        return false;
    size_t end = tape.samples + (size_t)count;
    while (tape.samples < end) {
        unsigned bit = (unsigned)(tape.samples & 7u);
        if (!bit && end - tape.samples >= 8) {
            size_t bytes = (end - tape.samples) / 8;
            memset(tape.data + tape.samples / 8, high ? 0xFF : 0, bytes);
            tape.samples += bytes * 8;
        } else {
            if (!bit) tape.data[tape.samples / 8] = 0;
            if (high) tape.data[tape.samples / 8] |= (uint8_t)(1u << bit);
            else tape.data[tape.samples / 8] &= (uint8_t)~(1u << bit);
            ++tape.samples;
        }
    }
    return true;
}

void family_basic_write(uint8_t value, uint64_t cpu_cycles) {
    bool previous_half = scan_half;
    scan_half = (value & 2u) != 0;
    if (previous_half && !scan_half) scan_row = (scan_row + 1) % 10;
    if (value & 1u) scan_row = 0;
    enabled = (value & 4u) != 0;

    if (tape.mode != FB_TAPE_RECORDING) return;
    if (cpu_cycles < tape.cycle) tape.cycle = cpu_cycles;
    uint64_t elapsed = cpu_cycles - tape.cycle;
    if (elapsed > FAMILY_BASIC_TAPE_SAMPLE_CYCLES) {
        uint64_t count = (elapsed - 1) / FAMILY_BASIC_TAPE_SAMPLE_CYCLES;
        if (!append_samples(count, (value & 1u) != 0)) {
            tape.failed = true;
            tape.mode = FB_TAPE_STOPPED;
            return;
        }
        tape.cycle += count * FAMILY_BASIC_TAPE_SAMPLE_CYCLES;
    }
}

uint8_t family_basic_read(unsigned port, uint64_t cpu_cycles) {
    if (port == 1) {
        if (!enabled) return 0;
        unsigned keys = scan_row < 9 ? matrix[scan_row] : 0;
        if (scan_half) keys >>= 4;
        return (uint8_t)((~keys << 1) & 0x1Eu);
    }
    if (port != 0 || tape.mode != FB_TAPE_PLAYING) return 0;
    if (cpu_cycles < tape.cycle) tape.cycle = cpu_cycles;
    uint64_t sample = (cpu_cycles - tape.cycle) / FAMILY_BASIC_TAPE_SAMPLE_CYCLES;
    if (sample >= tape.samples) {
        tape.mode = FB_TAPE_STOPPED;
        return 0;
    }
    unsigned bit = (tape.data[(size_t)sample / 8] >> (sample & 7u)) & 1u;
    return enabled ? (uint8_t)(bit << 1) : 0;
}

static void replace_tape(uint8_t *data, size_t size) {
    free(tape.data);
    tape.data = data;
    tape.capacity = size;
    tape.samples = size * 8;
    tape.mode = FB_TAPE_STOPPED;
    tape.failed = false;
}

bool family_basic_tape_load(const uint8_t *data, size_t size) {
    if ((size && !data) || size > SIZE_MAX / 8) return false;
    uint8_t *copy = size ? malloc(size) : NULL;
    if (size && !copy) return false;
    if (size) memcpy(copy, data, size);
    replace_tape(copy, size);
    return true;
}

bool family_basic_tape_load_file(const char *path) {
    if (!path || !*path) return false;
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return false; }
    long length = ftell(file);
    if (length < 0 || (uint64_t)length > SIZE_MAX / 8 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return false;
    }
    size_t size = (size_t)length;
    uint8_t *data = size ? malloc(size) : NULL;
    if (size && !data) { fclose(file); return false; }
    bool read_ok = !size || fread(data, 1, size, file) == size;
    bool close_ok = fclose(file) == 0;
    if (!read_ok || !close_ok) { free(data); return false; }
    replace_tape(data, size);
    return true;
}

bool family_basic_tape_save_file(const char *path) {
    if (!path || !*path || tape.mode != FB_TAPE_STOPPED) return false;
    static const char suffix[] = ".cupid-tape.tmp";
    size_t length = strlen(path);
    if (length > SIZE_MAX - sizeof(suffix)) return false;
    char *temporary = malloc(length + sizeof(suffix));
    if (!temporary) return false;
    memcpy(temporary, path, length);
    memcpy(temporary + length, suffix, sizeof(suffix));
    FILE *file = fopen(temporary, "wbx");
    if (!file) { free(temporary); return false; }
    // The raw tape format contains complete bytes; an incomplete final byte is omitted.
    size_t size = tape.samples / 8;
    bool written = !size || fwrite(tape.data, 1, size, file) == size;
    if (fclose(file) != 0) written = false;
    if (written) {
#ifdef _WIN32
        written = MoveFileExA(temporary, path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        written = rename(temporary, path) == 0;
#endif
    }
    if (!written) remove(temporary);
    free(temporary);
    return written;
}

bool family_basic_tape_play(uint64_t cpu_cycles) {
    if (!tape.samples) return false;
    tape.mode = FB_TAPE_PLAYING;
    tape.cycle = cpu_cycles;
    return true;
}

void family_basic_tape_record(uint64_t cpu_cycles) {
    tape.samples = 0;
    tape.cycle = cpu_cycles;
    tape.mode = FB_TAPE_RECORDING;
    tape.failed = false;
}

void family_basic_tape_stop(void) { tape.mode = FB_TAPE_STOPPED; }
FamilyBasicTapeMode family_basic_tape_mode(void) { return tape.mode; }
bool family_basic_tape_failed(void) { return tape.failed; }
