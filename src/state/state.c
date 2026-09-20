/*
 * state.c - Versioned emulator save-state orchestration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "state.h"
#include "state_io.h"

#include "../apu/apu.h"
#include "../apu/epsm.h"
#include "../cpu/cpu.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../joypad/special_peripherals.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../rom/mapper_state.h"
#include "../rom/fds.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../util/file_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    NES_STATE_FORMAT_VERSION = 1,
    NES_STATE_MAGIC_SIZE = 8,
    NES_STATE_FIXED_HEADER_SIZE = NES_STATE_MAGIC_SIZE + 12
};

enum {
    STATE_COMPONENT_TIME,
    STATE_COMPONENT_HARDWARE,
    STATE_COMPONENT_CPU,
    STATE_COMPONENT_PPU,
    STATE_COMPONENT_APU,
    STATE_COMPONENT_MAPPER,
    STATE_COMPONENT_EPSM,
    STATE_COMPONENT_INPUT,
    STATE_COMPONENT_FAMILY_BASIC,
    STATE_COMPONENT_SPECIAL,
    STATE_COMPONENT_FDS,
    STATE_COMPONENT_VS,
    NES_STATE_COMPONENT_COUNT
};

static const uint8_t state_magic[NES_STATE_MAGIC_SIZE] = {'C','U','P','S','T','A','T','E'};

typedef bool (*StateCaptureFn)(NesStateWriter *writer);
typedef bool (*StateReaderFn)(NesStateReader *reader);

typedef struct {
    uint32_t tag;
    StateCaptureFn capture;
    StateReaderFn validate;
    StateReaderFn apply;
} StateComponent;

typedef struct {
    const uint8_t *data;
    size_t size;
    bool found;
} StateChunk;

typedef struct {
    RomMetadataSource source;
    iNESHeader header;
    uint32_t file_crc;
    uint32_t prg_crc;
    uint32_t prg_chr_crc;
    uint64_t prg_bytes;
    uint64_t chr_bytes;
} StateIdentity;

#define STATE_TAG(a,b,c,d) ((uint32_t)(uint8_t)(a) | ((uint32_t)(uint8_t)(b) << 8) \
                          | ((uint32_t)(uint8_t)(c) << 16) | ((uint32_t)(uint8_t)(d) << 24))

static const StateComponent components[NES_STATE_COMPONENT_COUNT] = {
    {STATE_TAG('T','I','M','E'), timing_state_capture, timing_state_validate, timing_state_apply},
    {STATE_TAG('H','W','P','R'), hardware_state_capture, hardware_state_validate, hardware_state_apply},
    {STATE_TAG('C','P','U',' '), cpu_state_capture, cpu_state_validate, cpu_state_apply},
    {STATE_TAG('P','P','U',' '), ppu_state_capture, ppu_state_validate, ppu_state_apply},
    {STATE_TAG('A','P','U',' '), apu_state_capture, apu_state_validate, apu_state_apply},
    {STATE_TAG('M','A','P','R'), mapper_state_capture, NULL, NULL},
    {STATE_TAG('E','P','S','M'), epsm_state_capture, epsm_state_validate, epsm_state_apply},
    {STATE_TAG('I','N','P','T'), joypad_state_capture, joypad_state_validate, joypad_state_apply},
    {STATE_TAG('F','B','A','S'), family_basic_state_capture, family_basic_state_validate, family_basic_state_apply},
    {STATE_TAG('S','P','E','R'), special_peripherals_state_capture, special_peripherals_state_validate, special_peripherals_state_apply},
    {STATE_TAG('F','D','S',' '), fds_state_capture, fds_state_validate, fds_state_apply},
    {STATE_TAG('V','S','Y','S'), vs_state_capture, vs_state_validate, vs_state_apply}
};

typedef struct {
    MapperStateRestore *mapper;
    EpsmStateRestore *epsm;
    FamilyBasicStateRestore *family_basic;
} StatePreparedRestore;

static uint32_t state_crc32(const uint8_t *data, size_t size) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static StateIdentity current_identity(void) {
    StateIdentity identity;
    memset(&identity, 0, sizeof(identity));
    identity.source = rom_metadata_source();
    identity.header = ines_header;
    identity.file_crc = rom_file_crc32();
    identity.prg_crc = rom_prg_crc32();
    identity.prg_chr_crc = rom_prg_chr_crc32();
    identity.prg_bytes = prg_size;
    identity.chr_bytes = chr_size;
    return identity;
}

static bool state_identity_matches(const StateIdentity *saved) {
    StateIdentity active = current_identity();
    return saved && saved->source == active.source
        && memcmp(&saved->header, &active.header, sizeof(saved->header)) == 0
        && saved->file_crc == active.file_crc
        && saved->prg_crc == active.prg_crc
        && saved->prg_chr_crc == active.prg_chr_crc
        && saved->prg_bytes == active.prg_bytes
        && saved->chr_bytes == active.chr_bytes;
}

static bool state_write_identity(NesStateWriter *writer, const StateIdentity *identity) {
    return nes_state_write_u8(writer, (uint8_t)identity->source)
        && nes_state_write_bytes(writer, &identity->header, sizeof(identity->header))
        && nes_state_write_u32(writer, identity->file_crc)
        && nes_state_write_u32(writer, identity->prg_crc)
        && nes_state_write_u32(writer, identity->prg_chr_crc)
        && nes_state_write_u64(writer, identity->prg_bytes)
        && nes_state_write_u64(writer, identity->chr_bytes);
}

static bool state_read_identity(NesStateReader *reader, StateIdentity *identity) {
    uint8_t source;
    memset(identity, 0, sizeof(*identity));
    if (!nes_state_read_u8(reader, &source)
        || source > ROM_METADATA_NSF
        || !nes_state_read_bytes(reader, &identity->header, sizeof(identity->header))
        || !nes_state_read_u32(reader, &identity->file_crc)
        || !nes_state_read_u32(reader, &identity->prg_crc)
        || !nes_state_read_u32(reader, &identity->prg_chr_crc)
        || !nes_state_read_u64(reader, &identity->prg_bytes)
        || !nes_state_read_u64(reader, &identity->chr_bytes)) return false;
    identity->source = (RomMetadataSource)source;
    return true;
}

static NesStateResult state_append_component(NesStateWriter *payload,
                                             const StateComponent *component) {
    NesStateWriter nested;
    nes_state_writer_init(&nested, NES_STATE_MAX_SIZE);
    if (!component->capture(&nested)) {
        NesStateResult result = nested.failed ? NES_STATE_ERROR_OUT_OF_MEMORY
                                              : NES_STATE_ERROR_UNSUPPORTED;
        nes_state_writer_destroy(&nested);
        return result;
    }
    if (nested.size > UINT32_MAX
        || !nes_state_write_u32(payload, component->tag)
        || !nes_state_write_u32(payload, (uint32_t)nested.size)
        || !nes_state_write_bytes(payload, nested.data, nested.size)) {
        nes_state_writer_destroy(&nested);
        return NES_STATE_ERROR_OUT_OF_MEMORY;
    }
    nes_state_writer_destroy(&nested);
    return NES_STATE_OK;
}

static int state_component_index(uint32_t tag) {
    for (unsigned i = 0; i < NES_STATE_COMPONENT_COUNT; ++i)
        if (components[i].tag == tag) return (int)i;
    return -1;
}

static NesStateResult state_parse_payload(const uint8_t *data, size_t size,
                                          StateChunk chunks[NES_STATE_COMPONENT_COUNT]) {
    NesStateReader reader;
    StateIdentity identity;
    memset(chunks, 0, sizeof(StateChunk) * NES_STATE_COMPONENT_COUNT);
    nes_state_reader_init(&reader, data, size);
    if (!state_read_identity(&reader, &identity)) return NES_STATE_ERROR_CORRUPT;
    if (!state_identity_matches(&identity)) return NES_STATE_ERROR_INCOMPATIBLE;

    uint32_t count;
    if (!nes_state_read_u32(&reader, &count) || count < NES_STATE_COMPONENT_COUNT)
        return NES_STATE_ERROR_CORRUPT;
    for (uint32_t chunk = 0; chunk < count; ++chunk) {
        uint32_t tag, length;
        NesStateReader nested;
        if (!nes_state_read_u32(&reader, &tag) || !nes_state_read_u32(&reader, &length)
            || !nes_state_reader_slice(&reader, length, &nested)) return NES_STATE_ERROR_CORRUPT;
        int index = state_component_index(tag);
        if (index < 0) continue;
        if (chunks[index].found) return NES_STATE_ERROR_CORRUPT;
        chunks[index].data = nested.data;
        chunks[index].size = nested.size;
        chunks[index].found = true;
    }
    if (nes_state_reader_remaining(&reader) != 0) return NES_STATE_ERROR_CORRUPT;
    for (unsigned i = 0; i < NES_STATE_COMPONENT_COUNT; ++i)
        if (!chunks[i].found) return NES_STATE_ERROR_CORRUPT;
    return NES_STATE_OK;
}

static void state_prepared_restore_free(StatePreparedRestore *prepared) {
    if (!prepared) return;
    mapper_state_restore_free(prepared->mapper);
    epsm_state_restore_free(prepared->epsm);
    family_basic_state_restore_free(prepared->family_basic);
    memset(prepared, 0, sizeof(*prepared));
}

static NesStateResult state_prepare_chunks(const StateChunk chunks[NES_STATE_COMPONENT_COUNT],
                                           StatePreparedRestore *prepared) {
    if (!prepared) return NES_STATE_ERROR_ARGUMENT;
    memset(prepared, 0, sizeof(*prepared));
    for (unsigned i = 0; i < NES_STATE_COMPONENT_COUNT; ++i) {
        NesStateReader reader;
        nes_state_reader_init(&reader, chunks[i].data, chunks[i].size);
        NesStateResult result = NES_STATE_OK;
        if (i == STATE_COMPONENT_MAPPER) {
            result = mapper_state_prepare(&reader, &prepared->mapper);
        } else if (i == STATE_COMPONENT_EPSM) {
            result = epsm_state_prepare(&reader, &prepared->epsm);
        } else if (i == STATE_COMPONENT_FAMILY_BASIC) {
            result = family_basic_state_prepare(&reader, &prepared->family_basic);
        } else if (!components[i].validate || !components[i].validate(&reader)) {
            result = NES_STATE_ERROR_CORRUPT;
        }
        if (result == NES_STATE_OK && nes_state_reader_remaining(&reader) != 0)
            result = NES_STATE_ERROR_CORRUPT;
        if (result != NES_STATE_OK) {
            state_prepared_restore_free(prepared);
            return result;
        }
    }
    return NES_STATE_OK;
}

static bool state_apply_chunks(const StateChunk chunks[NES_STATE_COMPONENT_COUNT],
                               StatePreparedRestore *prepared) {
    for (unsigned i = 0; i < NES_STATE_COMPONENT_COUNT; ++i) {
        if (i == STATE_COMPONENT_MAPPER) {
            mapper_state_apply_prepared(prepared->mapper);
            continue;
        }
        if (i == STATE_COMPONENT_EPSM) {
            epsm_state_apply_prepared(prepared->epsm);
            continue;
        }
        if (i == STATE_COMPONENT_FAMILY_BASIC) {
            family_basic_state_apply_prepared(prepared->family_basic);
            continue;
        }
        NesStateReader reader;
        nes_state_reader_init(&reader, chunks[i].data, chunks[i].size);
        if (!components[i].apply || !components[i].apply(&reader)
            || nes_state_reader_remaining(&reader) != 0)
            return false;
    }
    return true;
}

NesStateResult nes_state_capture(NesStateBlob *out) {
    if (!out) return NES_STATE_ERROR_ARGUMENT;
    out->data = NULL;
    out->size = 0;
    if (rom_metadata_source() == ROM_METADATA_NONE || !cart) return NES_STATE_ERROR_NO_IMAGE;

    NesStateWriter payload;
    nes_state_writer_init(&payload, NES_STATE_MAX_SIZE);
    StateIdentity identity = current_identity();
    if (!state_write_identity(&payload, &identity)
        || !nes_state_write_u32(&payload, NES_STATE_COMPONENT_COUNT)) {
        nes_state_writer_destroy(&payload);
        return NES_STATE_ERROR_OUT_OF_MEMORY;
    }
    for (unsigned i = 0; i < NES_STATE_COMPONENT_COUNT; ++i) {
        NesStateResult result = state_append_component(&payload, &components[i]);
        if (result != NES_STATE_OK) {
            nes_state_writer_destroy(&payload);
            return result;
        }
    }
    if (payload.size > UINT32_MAX) {
        nes_state_writer_destroy(&payload);
        return NES_STATE_ERROR_OUT_OF_MEMORY;
    }

    NesStateWriter file;
    nes_state_writer_init(&file, NES_STATE_MAX_SIZE);
    uint32_t crc = state_crc32(payload.data, payload.size);
    bool ok = nes_state_write_bytes(&file, state_magic, sizeof(state_magic))
        && nes_state_write_u32(&file, NES_STATE_FORMAT_VERSION)
        && nes_state_write_u32(&file, (uint32_t)payload.size)
        && nes_state_write_u32(&file, crc)
        && nes_state_write_bytes(&file, payload.data, payload.size);
    nes_state_writer_destroy(&payload);
    if (!ok) {
        nes_state_writer_destroy(&file);
        return NES_STATE_ERROR_OUT_OF_MEMORY;
    }
    out->data = nes_state_writer_release(&file, &out->size);
    nes_state_writer_destroy(&file);
    return out->data ? NES_STATE_OK : NES_STATE_ERROR_OUT_OF_MEMORY;
}

NesStateResult nes_state_restore(const void *data, size_t size) {
    if ((!data && size) || !data) return NES_STATE_ERROR_ARGUMENT;
    if (rom_metadata_source() == ROM_METADATA_NONE || !cart) return NES_STATE_ERROR_NO_IMAGE;
    if (size < NES_STATE_FIXED_HEADER_SIZE || size > NES_STATE_MAX_SIZE)
        return NES_STATE_ERROR_FORMAT;

    NesStateReader reader;
    uint8_t magic[NES_STATE_MAGIC_SIZE];
    uint32_t version, payload_size, expected_crc;
    nes_state_reader_init(&reader, data, size);
    if (!nes_state_read_bytes(&reader, magic, sizeof(magic))
        || memcmp(magic, state_magic, sizeof(magic)) != 0) return NES_STATE_ERROR_FORMAT;
    if (!nes_state_read_u32(&reader, &version)) return NES_STATE_ERROR_FORMAT;
    if (version != NES_STATE_FORMAT_VERSION) return NES_STATE_ERROR_VERSION;
    if (!nes_state_read_u32(&reader, &payload_size)
        || !nes_state_read_u32(&reader, &expected_crc)
        || payload_size != nes_state_reader_remaining(&reader)) return NES_STATE_ERROR_CORRUPT;
    const uint8_t *payload = reader.data + reader.offset;
    if (state_crc32(payload, payload_size) != expected_crc) return NES_STATE_ERROR_CORRUPT;

    StateChunk chunks[NES_STATE_COMPONENT_COUNT];
    NesStateResult result = state_parse_payload(payload, payload_size, chunks);
    if (result != NES_STATE_OK) return result;
    StatePreparedRestore prepared;
    result = state_prepare_chunks(chunks, &prepared);
    if (result != NES_STATE_OK) return result;
    bool applied = state_apply_chunks(chunks, &prepared);
    state_prepared_restore_free(&prepared);
    return applied ? NES_STATE_OK : NES_STATE_ERROR_CORRUPT;
}

void nes_state_blob_free(NesStateBlob *blob) {
    if (!blob) return;
    free(blob->data);
    blob->data = NULL;
    blob->size = 0;
}

static NesStateResult state_file_result(NesFileResult result) {
    switch (result) {
        case NES_FILE_OK: return NES_STATE_OK;
        case NES_FILE_INVALID_ARGUMENT: return NES_STATE_ERROR_ARGUMENT;
        case NES_FILE_OUT_OF_MEMORY: return NES_STATE_ERROR_OUT_OF_MEMORY;
        case NES_FILE_NOT_FOUND:
        case NES_FILE_TOO_LARGE:
        case NES_FILE_IO_ERROR:
        default: return NES_STATE_ERROR_IO;
    }
}

NesStateResult nes_state_save_file(const char *path) {
    if (!path || !*path) return NES_STATE_ERROR_ARGUMENT;
    NesStateBlob blob;
    NesStateResult result = nes_state_capture(&blob);
    if (result != NES_STATE_OK) return result;
    NesFileResult file = nes_file_write_atomic(path, blob.data, blob.size);
    nes_state_blob_free(&blob);
    return state_file_result(file);
}

NesStateResult nes_state_load_file(const char *path) {
    if (!path || !*path) return NES_STATE_ERROR_ARGUMENT;
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult file = nes_file_read_all(path, NES_STATE_MAX_SIZE, &data, &size);
    if (file != NES_FILE_OK) return state_file_result(file);
    NesStateResult result = nes_state_restore(data, size);
    free(data);
    return result;
}

static char *state_slot_path(const char *directory, unsigned slot) {
    if (!directory || !*directory || slot >= NES_STATE_SLOT_COUNT) return NULL;
    size_t length = strlen(directory);
    if (length > NES_FILE_PATH_LIMIT - 32) return NULL;
    bool separator = directory[length - 1] == '/' || directory[length - 1] == '\\';
    size_t capacity = length + 32;
    char *path = malloc(capacity);
    if (!path) return NULL;
    snprintf(path, capacity, "%s%sslot-%u.cstate", directory, separator ? "" : "/", slot);
    return path;
}

NesStateResult nes_state_save_slot(const char *directory, unsigned slot) {
    if (slot >= NES_STATE_SLOT_COUNT) return NES_STATE_ERROR_ARGUMENT;
    char *path = state_slot_path(directory, slot);
    if (!path) return directory && *directory ? NES_STATE_ERROR_OUT_OF_MEMORY
                                               : NES_STATE_ERROR_ARGUMENT;
    NesStateResult result = nes_state_save_file(path);
    free(path);
    return result;
}

NesStateResult nes_state_load_slot(const char *directory, unsigned slot) {
    if (slot >= NES_STATE_SLOT_COUNT) return NES_STATE_ERROR_ARGUMENT;
    char *path = state_slot_path(directory, slot);
    if (!path) return directory && *directory ? NES_STATE_ERROR_OUT_OF_MEMORY
                                               : NES_STATE_ERROR_ARGUMENT;
    NesStateResult result = nes_state_load_file(path);
    free(path);
    return result;
}

const char *nes_state_result_string(NesStateResult result) {
    switch (result) {
        case NES_STATE_OK: return "State operation completed";
        case NES_STATE_ERROR_ARGUMENT: return "Invalid state argument";
        case NES_STATE_ERROR_NO_IMAGE: return "No emulated image is active";
        case NES_STATE_ERROR_OUT_OF_MEMORY: return "Not enough memory for state data";
        case NES_STATE_ERROR_IO: return "State file I/O failed";
        case NES_STATE_ERROR_FORMAT: return "Unrecognized state file";
        case NES_STATE_ERROR_VERSION: return "Unsupported state version";
        case NES_STATE_ERROR_INCOMPATIBLE: return "State belongs to a different image";
        case NES_STATE_ERROR_CORRUPT: return "State data is corrupt or incomplete";
        case NES_STATE_ERROR_UNSUPPORTED: return "Active hardware cannot be serialized";
        default: return "Unknown state error";
    }
}
