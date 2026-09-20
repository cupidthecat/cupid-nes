/*
 * state_io.c - Bounded little-endian save-state codec
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 */
#include "state_io.h"

#include <stdlib.h>
#include <string.h>

static bool state_add_size(size_t left, size_t right, size_t *out) {
    if (right > SIZE_MAX - left) return false;
    *out = left + right;
    return true;
}

void nes_state_writer_init(NesStateWriter *writer, size_t limit) {
    if (!writer) return;
    memset(writer, 0, sizeof(*writer));
    writer->limit = limit && limit <= NES_STATE_MAX_SIZE ? limit : NES_STATE_MAX_SIZE;
}

void nes_state_writer_destroy(NesStateWriter *writer) {
    if (!writer) return;
    free(writer->data);
    memset(writer, 0, sizeof(*writer));
}

uint8_t *nes_state_writer_release(NesStateWriter *writer, size_t *size) {
    if (!writer || writer->failed) return NULL;
    uint8_t *data = writer->data;
    if (size) *size = writer->size;
    writer->data = NULL;
    writer->size = writer->capacity = 0;
    return data;
}

bool nes_state_writer_reserve(NesStateWriter *writer, size_t amount) {
    if (!writer || writer->failed) return false;
    size_t required;
    if (!state_add_size(writer->size, amount, &required) || required > writer->limit) {
        writer->failed = true;
        return false;
    }
    if (required <= writer->capacity) return true;
    size_t capacity = writer->capacity ? writer->capacity : 4096;
    while (capacity < required) {
        size_t next = capacity <= writer->limit / 2 ? capacity * 2 : writer->limit;
        if (next <= capacity) break;
        capacity = next;
    }
    if (capacity < required) {
        writer->failed = true;
        return false;
    }
    uint8_t *data = (uint8_t *)realloc(writer->data, capacity);
    if (!data) {
        writer->failed = true;
        return false;
    }
    writer->data = data;
    writer->capacity = capacity;
    return true;
}

bool nes_state_write_bytes(NesStateWriter *writer, const void *data, size_t size) {
    if ((!data && size) || !nes_state_writer_reserve(writer, size)) return false;
    if (size) memcpy(writer->data + writer->size, data, size);
    writer->size += size;
    return true;
}

bool nes_state_write_u8(NesStateWriter *writer, uint8_t value) {
    return nes_state_write_bytes(writer, &value, 1);
}

bool nes_state_write_u16(NesStateWriter *writer, uint16_t value) {
    uint8_t bytes[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
    return nes_state_write_bytes(writer, bytes, sizeof(bytes));
}

bool nes_state_write_u32(NesStateWriter *writer, uint32_t value) {
    uint8_t bytes[4];
    for (unsigned i = 0; i < 4; ++i) bytes[i] = (uint8_t)(value >> (i * 8));
    return nes_state_write_bytes(writer, bytes, sizeof(bytes));
}

bool nes_state_write_u64(NesStateWriter *writer, uint64_t value) {
    uint8_t bytes[8];
    for (unsigned i = 0; i < 8; ++i) bytes[i] = (uint8_t)(value >> (i * 8));
    return nes_state_write_bytes(writer, bytes, sizeof(bytes));
}

bool nes_state_write_f32(NesStateWriter *writer, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return nes_state_write_u32(writer, bits);
}

bool nes_state_write_f64(NesStateWriter *writer, double value) {
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return nes_state_write_u64(writer, bits);
}

void nes_state_reader_init(NesStateReader *reader, const void *data, size_t size) {
    if (!reader) return;
    reader->data = (const uint8_t *)data;
    reader->size = data ? size : 0;
    reader->offset = 0;
    reader->failed = !data && size != 0;
}

size_t nes_state_reader_remaining(const NesStateReader *reader) {
    return reader && !reader->failed && reader->offset <= reader->size
         ? reader->size - reader->offset : 0;
}

bool nes_state_read_bytes(NesStateReader *reader, void *out, size_t size) {
    if (!reader || reader->failed || (!out && size) || size > nes_state_reader_remaining(reader)) {
        if (reader) reader->failed = true;
        return false;
    }
    if (size) memcpy(out, reader->data + reader->offset, size);
    reader->offset += size;
    return true;
}

bool nes_state_read_u8(NesStateReader *reader, uint8_t *value) {
    return nes_state_read_bytes(reader, value, 1);
}

bool nes_state_read_u16(NesStateReader *reader, uint16_t *value) {
    uint8_t bytes[2];
    if (!value || !nes_state_read_bytes(reader, bytes, sizeof(bytes))) return false;
    *value = (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8));
    return true;
}

bool nes_state_read_u32(NesStateReader *reader, uint32_t *value) {
    uint8_t bytes[4];
    if (!value || !nes_state_read_bytes(reader, bytes, sizeof(bytes))) return false;
    *value = 0;
    for (unsigned i = 0; i < 4; ++i) *value |= (uint32_t)bytes[i] << (i * 8);
    return true;
}

bool nes_state_read_u64(NesStateReader *reader, uint64_t *value) {
    uint8_t bytes[8];
    if (!value || !nes_state_read_bytes(reader, bytes, sizeof(bytes))) return false;
    *value = 0;
    for (unsigned i = 0; i < 8; ++i) *value |= (uint64_t)bytes[i] << (i * 8);
    return true;
}

bool nes_state_read_f32(NesStateReader *reader, float *value) {
    uint32_t bits;
    if (!value || !nes_state_read_u32(reader, &bits)) return false;
    memcpy(value, &bits, sizeof(bits));
    return true;
}

bool nes_state_read_f64(NesStateReader *reader, double *value) {
    uint64_t bits;
    if (!value || !nes_state_read_u64(reader, &bits)) return false;
    memcpy(value, &bits, sizeof(bits));
    return true;
}
