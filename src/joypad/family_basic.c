/*
 * family_basic.c - Family BASIC keyboard matrix and cassette signal
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "family_basic.h"
#include "../replay/input_event.h"
#include "../system/execution_policy.h"
#include "../util/file_io.h"
#include "../state/state_alloc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

static uint8_t matrix[9];
static uint8_t host_matrix[2][9];
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
    memset(host_matrix, 0, sizeof(host_matrix));
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
    NesInputEvent event = {
        .type = NES_INPUT_EVENT_FAMILY_BASIC_KEY,
        .a = (int32_t)key,
        .b = pressed
    };
    if (!nes_input_event_submit(&event)) return false;
    unsigned row = (unsigned)key / 8;
    uint8_t bit = (uint8_t)(1u << ((unsigned)key & 7u));
    if (pressed) matrix[row] |= bit;
    else matrix[row] &= (uint8_t)~bit;
    return true;
}

bool family_basic_key_pressed(FamilyBasicKey key) {
    return (unsigned)key < FB_KEY_COUNT && (matrix[(unsigned)key / 8] & (1u << ((unsigned)key & 7))) != 0;
}

bool family_basic_set_host_key(FamilyBasicKey key, bool pressed, bool virtual_key) {
    if ((unsigned)key >= FB_KEY_COUNT) return false;
    unsigned row = (unsigned)key / 8, lane = virtual_key ? 1 : 0;
    uint8_t bit = (uint8_t)(1u << ((unsigned)key & 7));
    uint8_t next = pressed ? host_matrix[lane][row] | bit : host_matrix[lane][row] & (uint8_t)~bit;
    bool combined = ((next | host_matrix[1 - lane][row]) & bit) != 0;
    if (!family_basic_set_key(key, combined)) return false;
    host_matrix[lane][row] = next;
    return true;
}

void family_basic_release_host_keys(void) {
    for (unsigned key = 0; key < FB_KEY_COUNT; ++key) {
        unsigned row = key / 8, bit = 1u << (key & 7);
        if ((host_matrix[0][row] | host_matrix[1][row]) & bit) {
            (void)family_basic_set_key((FamilyBasicKey)key, false);
        }
    }
    memset(host_matrix, 0, sizeof(host_matrix));
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
    if (nes_execution_policy() & (NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_MOVIE_PLAYBACK
                                | NES_EXECUTION_NETPLAY)) return false;
    if ((size && !data) || size > SIZE_MAX / 8) return false;
    uint8_t *copy = size ? malloc(size) : NULL;
    if (size && !copy) return false;
    if (size) memcpy(copy, data, size);
    replace_tape(copy, size);
    return true;
}

bool family_basic_tape_load_file(const char *path) {
    if (nes_execution_policy() & (NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_MOVIE_PLAYBACK
                                | NES_EXECUTION_NETPLAY)) return false;
    if (!path || !*path) return false;
    size_t size = 0;
    uint8_t *data = NULL;
    if (nes_file_read_all(path, 256u * 1024u * 1024u, &data, &size) != NES_FILE_OK) return false;
    replace_tape(data, size);
    return true;
}

bool family_basic_tape_save_file(const char *path) {
    if (!path || !*path || tape.mode != FB_TAPE_STOPPED || !nes_execution_allows_persistence()) return false;
    // The raw tape format contains complete bytes; an incomplete final byte is omitted.
    size_t size = tape.samples / 8;
    return nes_file_write_atomic(path, tape.data, size) == NES_FILE_OK;
}

bool family_basic_tape_play(uint64_t cpu_cycles) {
    if (!tape.samples) return false;
    NesInputEvent event = {.type = NES_INPUT_EVENT_FAMILY_BASIC_TAPE_PLAY};
    if (!nes_input_event_submit(&event)) return false;
    tape.mode = FB_TAPE_PLAYING;
    tape.cycle = cpu_cycles;
    return true;
}

void family_basic_tape_record(uint64_t cpu_cycles) {
    NesInputEvent event = {.type = NES_INPUT_EVENT_FAMILY_BASIC_TAPE_RECORD};
    if (!nes_input_event_submit(&event)) return;
    tape.samples = 0;
    tape.cycle = cpu_cycles;
    tape.mode = FB_TAPE_RECORDING;
    tape.failed = false;
}

void family_basic_tape_stop(void) {
    NesInputEvent event = {.type = NES_INPUT_EVENT_FAMILY_BASIC_TAPE_STOP};
    if (!nes_input_event_submit(&event)) return;
    tape.mode = FB_TAPE_STOPPED;
}
FamilyBasicTapeMode family_basic_tape_mode(void) { return tape.mode; }
bool family_basic_tape_failed(void) { return tape.failed; }

static bool family_basic_state_decode(NesStateReader *reader, uint8_t saved_matrix[9],
                                      unsigned *saved_row, bool *saved_half,
                                      bool *saved_enabled, size_t *saved_samples,
                                      uint64_t *saved_cycle, FamilyBasicTapeMode *saved_mode,
                                      bool *saved_failed, const uint8_t **saved_data,
                                      size_t *saved_bytes) {
    uint32_t row, mode;
    uint64_t samples;
    if (!reader || !saved_matrix || !saved_row || !saved_half || !saved_enabled
        || !saved_samples || !saved_cycle || !saved_mode || !saved_failed
        || !saved_data || !saved_bytes
        || !nes_state_read_bytes(reader, saved_matrix, 9)
        || !nes_state_read_u32(reader, &row)
        || !nes_state_read_bool(reader, saved_half)
        || !nes_state_read_bool(reader, saved_enabled)
        || !nes_state_read_u64(reader, &samples)
        || !nes_state_read_u64(reader, saved_cycle)
        || !nes_state_read_u32(reader, &mode)
        || !nes_state_read_bool(reader, saved_failed)
        || row >= 9 || mode > FB_TAPE_RECORDING || samples > SIZE_MAX) return false;
    size_t count = (size_t)samples;
    if (count > SIZE_MAX - 7) return false;
    size_t bytes = (count + 7) / 8;
    if (nes_state_reader_remaining(reader) != bytes) return false;
    *saved_row = row;
    *saved_samples = count;
    *saved_mode = (FamilyBasicTapeMode)mode;
    *saved_data = reader->data + reader->offset;
    *saved_bytes = bytes;
    reader->offset += bytes;
    return true;
}

bool family_basic_state_capture(NesStateWriter *writer) {
    if (!writer || tape.samples > SIZE_MAX - 7) return false;
    size_t bytes = (tape.samples + 7) / 8;
    return nes_state_write_bytes(writer, matrix, sizeof(matrix))
        && nes_state_write_u32(writer, scan_row)
        && nes_state_write_bool(writer, scan_half)
        && nes_state_write_bool(writer, enabled)
        && nes_state_write_u64(writer, tape.samples)
        && nes_state_write_u64(writer, tape.cycle)
        && nes_state_write_u32(writer, (uint32_t)tape.mode)
        && nes_state_write_bool(writer, tape.failed)
        && nes_state_write_bytes(writer, tape.data, bytes);
}

bool family_basic_state_validate(NesStateReader *reader) {
    uint8_t saved_matrix[9];
    unsigned row;
    bool half, active, failed;
    size_t samples, bytes;
    uint64_t cycle;
    FamilyBasicTapeMode mode;
    const uint8_t *data;
    return family_basic_state_decode(reader, saved_matrix, &row, &half, &active,
                                     &samples, &cycle, &mode, &failed, &data, &bytes);
}

struct FamilyBasicStateRestore {
    uint8_t matrix[9];
    unsigned row;
    bool half;
    bool active;
    size_t samples;
    uint64_t cycle;
    FamilyBasicTapeMode mode;
    bool failed;
    uint8_t *data;
    size_t bytes;
};

NesStateResult family_basic_state_prepare(NesStateReader *reader, FamilyBasicStateRestore **out_restore) {
    if (!reader || !out_restore) return NES_STATE_ERROR_ARGUMENT;
    *out_restore = NULL;
    FamilyBasicStateRestore decoded = {0};
    const uint8_t *data = NULL;
    if (!family_basic_state_decode(reader, decoded.matrix, &decoded.row, &decoded.half,
                                   &decoded.active, &decoded.samples, &decoded.cycle,
                                   &decoded.mode, &decoded.failed, &data, &decoded.bytes))
        return NES_STATE_ERROR_CORRUPT;
    FamilyBasicStateRestore *restore = nes_state_alloc(sizeof(*restore));
    if (!restore) return NES_STATE_ERROR_OUT_OF_MEMORY;
    *restore = decoded;
    if (decoded.bytes) {
        restore->data = nes_state_alloc(decoded.bytes);
        if (!restore->data) {
            nes_state_dealloc(restore);
            return NES_STATE_ERROR_OUT_OF_MEMORY;
        }
        memcpy(restore->data, data, decoded.bytes);
    }
    *out_restore = restore;
    return NES_STATE_OK;
}

void family_basic_state_apply_prepared(FamilyBasicStateRestore *restore) {
    if (!restore) return;
    memcpy(matrix, restore->matrix, sizeof(matrix));
    scan_row = restore->row;
    scan_half = restore->half;
    enabled = restore->active;
    free(tape.data);
    tape.data = restore->data;
    tape.capacity = restore->bytes;
    tape.samples = restore->samples;
    tape.cycle = restore->cycle;
    tape.mode = restore->mode;
    tape.failed = restore->failed;
    restore->data = NULL;
    restore->bytes = 0;
}

void family_basic_state_restore_free(FamilyBasicStateRestore *restore) {
    if (!restore) return;
    nes_state_dealloc(restore->data);
    nes_state_dealloc(restore);
}

bool family_basic_state_apply(NesStateReader *reader) {
    FamilyBasicStateRestore *restore = NULL;
    if (family_basic_state_prepare(reader, &restore) != NES_STATE_OK) return false;
    family_basic_state_apply_prepared(restore);
    family_basic_state_restore_free(restore);
    return true;
}
