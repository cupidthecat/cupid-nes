/*
 * state_io.h - Bounded little-endian save-state codec
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 */
#ifndef CUPID_NES_STATE_IO_H
#define CUPID_NES_STATE_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { NES_STATE_MAX_SIZE = 128 * 1024 * 1024 };

typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
    size_t limit;
    bool failed;
} NesStateWriter;

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t offset;
    bool failed;
} NesStateReader;

void nes_state_writer_init(NesStateWriter *writer, size_t limit);
void nes_state_writer_destroy(NesStateWriter *writer);
uint8_t *nes_state_writer_release(NesStateWriter *writer, size_t *size);
bool nes_state_writer_reserve(NesStateWriter *writer, size_t amount);
bool nes_state_write_bytes(NesStateWriter *writer, const void *data, size_t size);
bool nes_state_write_u8(NesStateWriter *writer, uint8_t value);
bool nes_state_write_u16(NesStateWriter *writer, uint16_t value);
bool nes_state_write_u32(NesStateWriter *writer, uint32_t value);
bool nes_state_write_u64(NesStateWriter *writer, uint64_t value);
bool nes_state_write_bool(NesStateWriter *writer, bool value);
bool nes_state_write_f32(NesStateWriter *writer, float value);
bool nes_state_write_f64(NesStateWriter *writer, double value);

void nes_state_reader_init(NesStateReader *reader, const void *data, size_t size);
size_t nes_state_reader_remaining(const NesStateReader *reader);
bool nes_state_reader_slice(NesStateReader *reader, size_t size, NesStateReader *slice);
bool nes_state_read_bytes(NesStateReader *reader, void *out, size_t size);
bool nes_state_read_u8(NesStateReader *reader, uint8_t *value);
bool nes_state_read_u16(NesStateReader *reader, uint16_t *value);
bool nes_state_read_u32(NesStateReader *reader, uint32_t *value);
bool nes_state_read_u64(NesStateReader *reader, uint64_t *value);
bool nes_state_read_bool(NesStateReader *reader, bool *value);
bool nes_state_read_f32(NesStateReader *reader, float *value);
bool nes_state_read_f64(NesStateReader *reader, double *value);

#ifdef __cplusplus
}
#endif

#endif
