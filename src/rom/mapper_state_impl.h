/*
 * mapper_state_impl.h - Native mapper save-state implementation
 *
 * Author: @frankischilling
 *
 * Included by mapper.c after all mapper implementations so this codec can keep
 * mapper internals private while still providing transactional state restores.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef CUPID_MAPPER_STATE_IMPL_H
#define CUPID_MAPPER_STATE_IMPL_H

#include "mapper_state.h"
#include "boards/board_state.h"
#include "../state/state_alloc.h"

enum {
    MAPPER_STATE_NATIVE = 0,
    MAPPER_STATE_BOARD = 1,
    MAPPER_STATE_FDS = 2,
    MAPPER_STATE_NSF = 3
};

struct MapperStateRestore {
    uint8_t *data;
    size_t size;
    BoardStateRestore *board;
};

static unsigned mapper_state_kind(void) {
    if (active_board) return MAPPER_STATE_BOARD;
    if (cart == &mapper_fds) return MAPPER_STATE_FDS;
    if (cart == &mapper_nsf) return MAPPER_STATE_NSF;
    return MAPPER_STATE_NATIVE;
}

static bool mapper_state_write_raw(NesStateWriter *writer, const void *data, size_t size) {
    return size <= UINT32_MAX
        && nes_state_write_u32(writer, (uint32_t)size)
        && nes_state_write_bytes(writer, data, size);
}

static bool mapper_state_read_raw(NesStateReader *reader, void *data, size_t size, bool apply) {
    uint32_t encoded_size;
    if (!nes_state_read_u32(reader, &encoded_size) || encoded_size != size
        || encoded_size > nes_state_reader_remaining(reader)) return false;
    if (apply) return nes_state_read_bytes(reader, data, size);
    reader->offset += size;
    return true;
}

static bool mapper_state_write_ram(NesStateWriter *writer, const RamBlock *ram) {
    return nes_state_write_u64(writer, (uint64_t)ram->size)
        && nes_state_write_bytes(writer, ram->data, ram->size);
}

static bool mapper_state_read_ram(NesStateReader *reader, RamBlock *ram, bool apply) {
    uint64_t size;
    if (!nes_state_read_u64(reader, &size) || size != ram->size
        || size > nes_state_reader_remaining(reader)) return false;
    if (apply) return nes_state_read_bytes(reader, ram->data, ram->size);
    reader->offset += ram->size;
    return true;
}

static bool mapper_state_write_optional_memory(NesStateWriter *writer, bool present,
                                                const uint8_t *data, size_t size) {
    return nes_state_write_bool(writer, present)
        && (!present || (nes_state_write_u64(writer, (uint64_t)size)
            && nes_state_write_bytes(writer, data, size)));
}

static bool mapper_state_read_optional_memory(NesStateReader *reader, bool expected,
                                               uint8_t *data, size_t size, bool apply) {
    bool present;
    uint64_t encoded_size;
    if (!nes_state_read_bool(reader, &present) || present != expected) return false;
    if (!present) return true;
    if (!nes_state_read_u64(reader, &encoded_size) || encoded_size != size
        || encoded_size > nes_state_reader_remaining(reader)) return false;
    if (apply) return nes_state_read_bytes(reader, data, size);
    reader->offset += size;
    return true;
}

static bool mapper_state_write_config(NesStateWriter *writer) {
    return nes_state_write_u8(writer, (uint8_t)mapper_state_kind())
        && nes_state_write_u16(writer, C.mapper_no)
        && nes_state_write_u8(writer, C.submapper)
        && nes_state_write_bool(writer, C.chr_is_ram)
        && nes_state_write_u64(writer, (uint64_t)C.prg_sz)
        && nes_state_write_u64(writer, (uint64_t)C.chr_sz)
        && nes_state_write_bool(writer, C.mmc1a)
        && nes_state_write_bool(writer, C.bus_conflicts)
        && nes_state_write_bool(writer, C.nes2)
        && nes_state_write_u64(writer, (uint64_t)C.ram.prg_ram)
        && nes_state_write_u64(writer, (uint64_t)C.ram.prg_nvram)
        && nes_state_write_u64(writer, (uint64_t)C.ram.chr_ram)
        && nes_state_write_u64(writer, (uint64_t)C.ram.chr_nvram)
        && nes_state_write_u8(writer, (uint8_t)C.mirr_base)
        && nes_state_write_bool(writer, mmc3_revision_a_profile)
        && nes_state_write_u32(writer, cart_dip_value);
}

static bool mapper_state_read_config(NesStateReader *reader) {
    uint8_t kind, submapper, mirroring;
    uint16_t mapper;
    uint64_t prg_size, chr_size, prg_ram, prg_nvram, chr_ram, chr_nvram;
    uint32_t dip;
    bool chr_is_ram, mmc1a, bus_conflicts, nes2, revision_a;
    return nes_state_read_u8(reader, &kind) && kind == mapper_state_kind()
        && nes_state_read_u16(reader, &mapper) && mapper == C.mapper_no
        && nes_state_read_u8(reader, &submapper) && submapper == C.submapper
        && nes_state_read_bool(reader, &chr_is_ram) && chr_is_ram == C.chr_is_ram
        && nes_state_read_u64(reader, &prg_size) && prg_size == C.prg_sz
        && nes_state_read_u64(reader, &chr_size) && chr_size == C.chr_sz
        && nes_state_read_bool(reader, &mmc1a) && mmc1a == C.mmc1a
        && nes_state_read_bool(reader, &bus_conflicts) && bus_conflicts == C.bus_conflicts
        && nes_state_read_bool(reader, &nes2) && nes2 == C.nes2
        && nes_state_read_u64(reader, &prg_ram) && prg_ram == C.ram.prg_ram
        && nes_state_read_u64(reader, &prg_nvram) && prg_nvram == C.ram.prg_nvram
        && nes_state_read_u64(reader, &chr_ram) && chr_ram == C.ram.chr_ram
        && nes_state_read_u64(reader, &chr_nvram) && chr_nvram == C.ram.chr_nvram
        && nes_state_read_u8(reader, &mirroring) && mirroring == (uint8_t)C.mirr_base
        && mirroring <= MIRROR_FOUR
        && nes_state_read_bool(reader, &revision_a) && revision_a == mmc3_revision_a_profile
        && nes_state_read_u32(reader, &dip) && dip == cart_dip_value;
}

static bool mapper_state_write_common(NesStateWriter *writer) {
    unsigned kind = mapper_state_kind();
    bool mapper_memory = kind == MAPPER_STATE_NATIVE || kind == MAPPER_STATE_NSF;
    bool mutable_prg = mapper_memory
        && (cart == &mapper_unrom512 || cart == &mapper_m111 || cart == &mapper_nsf);
    bool mutable_chr = mapper_memory && C.chr_is_ram;
    if (!nes_state_write_bool(writer, mapper_irq_line)
        || !nes_state_write_u8(writer, cart_cpu_bus_input)
        || !nes_state_write_u8(writer, (uint8_t)cart_ppu_fetch_source)
        || !nes_state_write_bool(writer, cart_cpu_cycle_is_write)
        || !mapper_state_write_ram(writer, &prg_work_ram)
        || !mapper_state_write_ram(writer, &prg_save_ram)
        || !mapper_state_write_ram(writer, &chr_work_ram)
        || !mapper_state_write_ram(writer, &chr_save_ram)
        || !mapper_state_write_optional_memory(writer, mutable_prg, C.prg, C.prg_sz)
        || !mapper_state_write_optional_memory(writer, mutable_chr, C.chr, C.chr_sz)
        || !nes_state_write_bool(writer, prg_ram_dirty)
        || !nes_state_write_bool(writer, chr_ram_dirty)
        || !nes_state_write_bool(writer, mmc5_exram_dirty)
        || !nes_state_write_bool(writer, namco163_audio_dirty)
        || !nes_state_write_bool(writer, flash_dirty)) return false;
    bool mmc5_ram = cart == &mapper_mmc5
        || (cart == &mapper_nsf && (nsf_player.metadata.sound_chips & NSF_SOUND_MMC5));
    return mapper_state_write_optional_memory(writer, mmc5_ram, mmc5_exram, sizeof(mmc5_exram));
}

static bool mapper_state_read_common(NesStateReader *reader, bool apply) {
    bool irq, write_cycle, saved_prg_dirty, saved_chr_dirty, saved_exram_dirty;
    bool saved_namco_dirty, saved_flash_dirty;
    uint8_t bus, fetch;
    unsigned kind = mapper_state_kind();
    bool mapper_memory = kind == MAPPER_STATE_NATIVE || kind == MAPPER_STATE_NSF;
    bool mutable_prg = mapper_memory
        && (cart == &mapper_unrom512 || cart == &mapper_m111 || cart == &mapper_nsf);
    bool mutable_chr = mapper_memory && C.chr_is_ram;
    if (!nes_state_read_bool(reader, &irq)
        || !nes_state_read_u8(reader, &bus)
        || !nes_state_read_u8(reader, &fetch) || fetch > CART_PPU_FETCH_SPRITE
        || !nes_state_read_bool(reader, &write_cycle)
        || !mapper_state_read_ram(reader, &prg_work_ram, apply)
        || !mapper_state_read_ram(reader, &prg_save_ram, apply)
        || !mapper_state_read_ram(reader, &chr_work_ram, apply)
        || !mapper_state_read_ram(reader, &chr_save_ram, apply)
        || !mapper_state_read_optional_memory(reader, mutable_prg, C.prg, C.prg_sz, apply)
        || !mapper_state_read_optional_memory(reader, mutable_chr, C.chr, C.chr_sz, apply)
        || !nes_state_read_bool(reader, &saved_prg_dirty)
        || !nes_state_read_bool(reader, &saved_chr_dirty)
        || !nes_state_read_bool(reader, &saved_exram_dirty)
        || !nes_state_read_bool(reader, &saved_namco_dirty)
        || !nes_state_read_bool(reader, &saved_flash_dirty)) return false;
    bool mmc5_ram = cart == &mapper_mmc5
        || (cart == &mapper_nsf && (nsf_player.metadata.sound_chips & NSF_SOUND_MMC5));
    if (!mapper_state_read_optional_memory(reader, mmc5_ram, mmc5_exram, sizeof(mmc5_exram), apply))
        return false;
    if (apply) {
        mapper_irq_line = irq;
        cart_cpu_bus_input = bus;
        cart_ppu_fetch_source = (CartPpuFetchSource)fetch;
        cart_cpu_cycle_is_write = write_cycle;
        prg_ram_dirty = prg_ram_dirty || saved_prg_dirty;
        chr_ram_dirty = chr_ram_dirty || saved_chr_dirty;
        mmc5_exram_dirty = mmc5_exram_dirty || saved_exram_dirty;
        namco163_audio_dirty = namco163_audio_dirty || saved_namco_dirty;
        flash_dirty = flash_dirty || saved_flash_dirty;
    }
    return true;
}

#define MAPPER_WRITE_STATE(value) mapper_state_write_raw(writer, &(value), sizeof(value))
#define MAPPER_READ_STATE(value) mapper_state_read_raw(reader, &(value), sizeof(value), apply)

static bool mapper_state_write_vrc7(NesStateWriter *writer) {
    size_t regs_size = (size_t)((uint8_t *)&vrc7.fm - (uint8_t *)&vrc7);
    NesStateWriter nested;
    if (!mapper_state_write_raw(writer, &vrc7, regs_size)) return false;
    nes_state_writer_init(&nested, NES_STATE_MAX_SIZE);
    bool ok = vrc7_fm_state_capture(&nested, &vrc7.fm)
        && mapper_state_write_raw(writer, nested.data, nested.size);
    nes_state_writer_destroy(&nested);
    return ok;
}

static bool mapper_state_read_vrc7(NesStateReader *reader, bool apply) {
    size_t regs_size = (size_t)((uint8_t *)&vrc7.fm - (uint8_t *)&vrc7);
    uint32_t nested_size;
    if (!mapper_state_read_raw(reader, &vrc7, regs_size, apply)
        || !nes_state_read_u32(reader, &nested_size)
        || nested_size > nes_state_reader_remaining(reader)) return false;
    NesStateReader nested;
    if (!nes_state_reader_slice(reader, nested_size, &nested)) return false;
    return apply ? vrc7_fm_state_apply(&vrc7.fm, &nested)
                 : vrc7_fm_state_validate(&vrc7.fm, &nested);
}

static bool mapper_state_write_nsf(NesStateWriter *writer) {
    uint8_t chips = nsf_player.metadata.sound_chips;
    if (!nes_state_write_bytes(writer, nsf_player.banks, sizeof(nsf_player.banks))) return false;
    for (unsigned i = 0; i < 2; ++i)
        if (!nes_state_write_bool(writer, nsf_player.lower_program[i])) return false;
    if (!nes_state_write_u8(writer, nsf_player.song)
        || !nes_state_write_u32(writer, nsf_player.play_counter)
        || !nes_state_write_u64(writer, nsf_player.track_start_cycle)
        || !nes_state_write_bytes(writer, nsf_player.mmc5_multiplier,
                                  sizeof(nsf_player.mmc5_multiplier))) return false;
    if ((chips & NSF_SOUND_MMC5) && !MAPPER_WRITE_STATE(mmc5)) return false;
    if ((chips & NSF_SOUND_VRC6) && !MAPPER_WRITE_STATE(vrc6)) return false;
    if (chips & NSF_SOUND_VRC7) {
        NesStateWriter nested;
        nes_state_writer_init(&nested, NES_STATE_MAX_SIZE);
        bool ok = vrc7_fm_state_capture(&nested, &nsf_vrc7)
            && mapper_state_write_raw(writer, nested.data, nested.size);
        nes_state_writer_destroy(&nested);
        if (!ok) return false;
    }
    if ((chips & NSF_SOUND_NAMCO163) && !MAPPER_WRITE_STATE(namco163_audio)) return false;
    if ((chips & NSF_SOUND_SUNSOFT5B) && !MAPPER_WRITE_STATE(sunsoft5b_audio)) return false;
    return true;
}

static bool mapper_state_read_nsf(NesStateReader *reader, bool apply) {
    uint8_t banks[10], song, multipliers[2];
    bool lower[2];
    uint32_t play_counter;
    uint64_t track_start;
    uint8_t chips = nsf_player.metadata.sound_chips;
    if (!nes_state_read_bytes(reader, banks, sizeof(banks))) return false;
    for (unsigned i = 0; i < 2; ++i)
        if (!nes_state_read_bool(reader, &lower[i])) return false;
    if (!nes_state_read_u8(reader, &song) || song >= nsf_player.metadata.total_songs
        || !nes_state_read_u32(reader, &play_counter)
        || !nes_state_read_u64(reader, &track_start)
        || !nes_state_read_bytes(reader, multipliers, sizeof(multipliers))) return false;
    if (chips & NSF_SOUND_MMC5) {
        if (!MAPPER_READ_STATE(mmc5)) return false;
    }
    if (chips & NSF_SOUND_VRC6) {
        if (!MAPPER_READ_STATE(vrc6)) return false;
    }
    if (chips & NSF_SOUND_VRC7) {
        uint32_t nested_size;
        if (!nes_state_read_u32(reader, &nested_size) || nested_size > nes_state_reader_remaining(reader))
            return false;
        NesStateReader nested;
        if (!nes_state_reader_slice(reader, nested_size, &nested)) return false;
        if (apply ? !vrc7_fm_state_apply(&nsf_vrc7, &nested)
                  : !vrc7_fm_state_validate(&nsf_vrc7, &nested)) return false;
    }
    if ((chips & NSF_SOUND_NAMCO163) && !MAPPER_READ_STATE(namco163_audio)) return false;
    if ((chips & NSF_SOUND_SUNSOFT5B) && !MAPPER_READ_STATE(sunsoft5b_audio)) return false;
    if (apply) {
        memcpy(nsf_player.banks, banks, sizeof(banks));
        memcpy(nsf_player.lower_program, lower, sizeof(lower));
        nsf_player.song = song;
        nsf_player.play_counter = play_counter;
        nsf_player.track_start_cycle = track_start;
        memcpy(nsf_player.mmc5_multiplier, multipliers, sizeof(multipliers));
    }
    return true;
}

static bool mapper_state_write_native(NesStateWriter *writer) {
    switch (C.mapper_no) {
        case 0: case 99: return true;
        case 1: case 155: return MAPPER_WRITE_STATE(mmc1);
        case 105: return MAPPER_WRITE_STATE(mmc1) && MAPPER_WRITE_STATE(m105);
        case 2: case 94: case 180: return MAPPER_WRITE_STATE(ux);
        case 3: return MAPPER_WRITE_STATE(cn);
        case 185: return MAPPER_WRITE_STATE(cn)
            && nes_state_write_bool(writer, cnrom185_chr_enabled)
            && nes_state_write_bool(writer, cnrom185_initial_ram_mapping);
        case 4: return MAPPER_WRITE_STATE(mmc3);
        case 5: return MAPPER_WRITE_STATE(mmc5);
        case 7: return MAPPER_WRITE_STATE(ao);
        case 9: return MAPPER_WRITE_STATE(mmc2);
        case 10: return MAPPER_WRITE_STATE(mmc4);
        case 11: case 144: return MAPPER_WRITE_STATE(colordreams);
        case 13: return MAPPER_WRITE_STATE(cprom);
        case 15: return MAPPER_WRITE_STATE(m15);
        case 16: case 153: case 157: case 159:
            return MAPPER_WRITE_STATE(bandai) && MAPPER_WRITE_STATE(bandai_eeprom);
        case 18: return MAPPER_WRITE_STATE(jaleco18);
        case 19: case 210:
            return MAPPER_WRITE_STATE(namco) && MAPPER_WRITE_STATE(namco_ppu_source)
                && MAPPER_WRITE_STATE(namco_ppu_offset) && MAPPER_WRITE_STATE(namco163_audio);
        case 21: case 22: case 23: case 25: case 27: case 183: return MAPPER_WRITE_STATE(vrc24);
        case 24: case 26: return MAPPER_WRITE_STATE(vrc6);
        case 28: return MAPPER_WRITE_STATE(m28);
        case 30: return MAPPER_WRITE_STATE(m30);
        case 32: return MAPPER_WRITE_STATE(irem32);
        case 33: return MAPPER_WRITE_STATE(taito33);
        case 48: return MAPPER_WRITE_STATE(taito48) && MAPPER_WRITE_STATE(taito48_irq);
        case 64: case 158: return MAPPER_WRITE_STATE(rambo1);
        case 65: return MAPPER_WRITE_STATE(irem65);
        case 66: return MAPPER_WRITE_STATE(gxrom);
        case 67: return MAPPER_WRITE_STATE(sunsoft3);
        case 68: return MAPPER_WRITE_STATE(sunsoft4);
        case 71: return MAPPER_WRITE_STATE(m71);
        case 72: case 78: case 87: case 92: case 101: case 140:
            return MAPPER_WRITE_STATE(jaleco_discrete);
        case 73: return MAPPER_WRITE_STATE(vrc3);
        case 75: case 151: return MAPPER_WRITE_STATE(vrc1);
        case 76: case 88: case 95: case 154: case 206: return MAPPER_WRITE_STATE(namco108);
        case 79: case 113: case 146: return MAPPER_WRITE_STATE(nina);
        case 85: return mapper_state_write_vrc7(writer);
        case 89: return MAPPER_WRITE_STATE(sunsoft89);
        case 90: case 209: case 211: return MAPPER_WRITE_STATE(jy);
        case 93: return MAPPER_WRITE_STATE(sunsoft93);
        case 96: return MAPPER_WRITE_STATE(m96);
        case 97: return MAPPER_WRITE_STATE(irem97);
        case 111: return MAPPER_WRITE_STATE(m111) && MAPPER_WRITE_STATE(m111_nt_ram);
        case 118: return MAPPER_WRITE_STATE(mmc3) && MAPPER_WRITE_STATE(txsrom_nt);
        case 184: return MAPPER_WRITE_STATE(sunsoft184);
        case 232: return MAPPER_WRITE_STATE(m232);
        default: return false;
    }
}

static bool mapper_state_read_native(NesStateReader *reader, bool apply) {
    bool chr_enabled, initial_mapping;
    bool old_eeprom_dirty[2] = {bandai_eeprom[0].dirty, bandai_eeprom[1].dirty};
    switch (C.mapper_no) {
        case 0: case 99: return true;
        case 1: case 155: return MAPPER_READ_STATE(mmc1);
        case 105: return MAPPER_READ_STATE(mmc1) && MAPPER_READ_STATE(m105);
        case 2: case 94: case 180: return MAPPER_READ_STATE(ux);
        case 3: return MAPPER_READ_STATE(cn);
        case 185:
            if (!MAPPER_READ_STATE(cn)
                || !nes_state_read_bool(reader, &chr_enabled)
                || !nes_state_read_bool(reader, &initial_mapping)) return false;
            if (apply) {
                cnrom185_chr_enabled = chr_enabled;
                cnrom185_initial_ram_mapping = initial_mapping;
            }
            return true;
        case 4: return MAPPER_READ_STATE(mmc3);
        case 5: return MAPPER_READ_STATE(mmc5);
        case 7: return MAPPER_READ_STATE(ao);
        case 9: return MAPPER_READ_STATE(mmc2);
        case 10: return MAPPER_READ_STATE(mmc4);
        case 11: case 144: return MAPPER_READ_STATE(colordreams);
        case 13: return MAPPER_READ_STATE(cprom);
        case 15: return MAPPER_READ_STATE(m15);
        case 16: case 153: case 157: case 159:
            if (!MAPPER_READ_STATE(bandai) || !MAPPER_READ_STATE(bandai_eeprom)) return false;
            if (apply) {
                bandai_eeprom[0].dirty = bandai_eeprom[0].dirty || old_eeprom_dirty[0];
                bandai_eeprom[1].dirty = bandai_eeprom[1].dirty || old_eeprom_dirty[1];
            }
            return true;
        case 18: return MAPPER_READ_STATE(jaleco18);
        case 19: case 210:
            return MAPPER_READ_STATE(namco) && MAPPER_READ_STATE(namco_ppu_source)
                && MAPPER_READ_STATE(namco_ppu_offset) && MAPPER_READ_STATE(namco163_audio);
        case 21: case 22: case 23: case 25: case 27: case 183: return MAPPER_READ_STATE(vrc24);
        case 24: case 26: return MAPPER_READ_STATE(vrc6);
        case 28: return MAPPER_READ_STATE(m28);
        case 30: return MAPPER_READ_STATE(m30);
        case 32: return MAPPER_READ_STATE(irem32);
        case 33: return MAPPER_READ_STATE(taito33);
        case 48: return MAPPER_READ_STATE(taito48) && MAPPER_READ_STATE(taito48_irq);
        case 64: case 158: return MAPPER_READ_STATE(rambo1);
        case 65: return MAPPER_READ_STATE(irem65);
        case 66: return MAPPER_READ_STATE(gxrom);
        case 67: return MAPPER_READ_STATE(sunsoft3);
        case 68: return MAPPER_READ_STATE(sunsoft4);
        case 71: return MAPPER_READ_STATE(m71);
        case 72: case 78: case 87: case 92: case 101: case 140:
            return MAPPER_READ_STATE(jaleco_discrete);
        case 73: return MAPPER_READ_STATE(vrc3);
        case 75: case 151: return MAPPER_READ_STATE(vrc1);
        case 76: case 88: case 95: case 154: case 206: return MAPPER_READ_STATE(namco108);
        case 79: case 113: case 146: return MAPPER_READ_STATE(nina);
        case 85: return mapper_state_read_vrc7(reader, apply);
        case 89: return MAPPER_READ_STATE(sunsoft89);
        case 90: case 209: case 211: return MAPPER_READ_STATE(jy);
        case 93: return MAPPER_READ_STATE(sunsoft93);
        case 96: return MAPPER_READ_STATE(m96);
        case 97: return MAPPER_READ_STATE(irem97);
        case 111: return MAPPER_READ_STATE(m111) && MAPPER_READ_STATE(m111_nt_ram);
        case 118: return MAPPER_READ_STATE(mmc3) && MAPPER_READ_STATE(txsrom_nt);
        case 184: return MAPPER_READ_STATE(sunsoft184);
        case 232: return MAPPER_READ_STATE(m232);
        default: return false;
    }
}

static bool mapper_state_write_board(NesStateWriter *writer) {
    NesStateWriter nested;
    nes_state_writer_init(&nested, NES_STATE_MAX_SIZE);
    NesStateResult result = board_state_capture(active_board, &nested);
    bool ok = result == NES_STATE_OK && mapper_state_write_raw(writer, nested.data, nested.size);
    nes_state_writer_destroy(&nested);
    return ok;
}

static NesStateResult mapper_state_read_board(NesStateReader *reader,
                                              BoardStateRestore **out_restore) {
    uint32_t size;
    NesStateReader nested;
    if (!nes_state_read_u32(reader, &size) || size > nes_state_reader_remaining(reader)
        || !nes_state_reader_slice(reader, size, &nested)) return NES_STATE_ERROR_CORRUPT;
    return board_state_validate(active_board, &nested, out_restore);
}

static bool mapper_state_write_payload(NesStateWriter *writer) {
    unsigned kind = mapper_state_kind();
    if (!mapper_state_write_config(writer) || !mapper_state_write_common(writer)) return false;
    if (kind == MAPPER_STATE_BOARD) return mapper_state_write_board(writer);
    if (kind == MAPPER_STATE_NSF) return mapper_state_write_nsf(writer);
    if (kind == MAPPER_STATE_FDS) return true;
    return mapper_state_write_native(writer);
}

static NesStateResult mapper_state_read_payload(NesStateReader *reader, bool apply,
                                                BoardStateRestore **board_restore) {
    unsigned kind = mapper_state_kind();
    if (!mapper_state_read_config(reader) || !mapper_state_read_common(reader, apply))
        return NES_STATE_ERROR_INCOMPATIBLE;
    if (kind == MAPPER_STATE_BOARD) {
        if (apply) {
            uint32_t size;
            if (!nes_state_read_u32(reader, &size) || size > nes_state_reader_remaining(reader))
                return NES_STATE_ERROR_CORRUPT;
            reader->offset += size;
        } else {
            NesStateResult result = mapper_state_read_board(reader, board_restore);
            if (result != NES_STATE_OK) return result;
        }
    } else if (kind == MAPPER_STATE_NSF) {
        if (!mapper_state_read_nsf(reader, apply)) return NES_STATE_ERROR_CORRUPT;
    } else if (kind == MAPPER_STATE_NATIVE) {
        if (!mapper_state_read_native(reader, apply)) return NES_STATE_ERROR_CORRUPT;
    }
    return nes_state_reader_remaining(reader) == 0 ? NES_STATE_OK : NES_STATE_ERROR_CORRUPT;
}

bool mapper_state_capture(NesStateWriter *writer) {
    return writer && mapper_state_write_payload(writer);
}

NesStateResult mapper_state_prepare(NesStateReader *reader, MapperStateRestore **out_restore) {
    if (!reader || !out_restore) return NES_STATE_ERROR_ARGUMENT;
    *out_restore = NULL;
    MapperStateRestore *restore = (MapperStateRestore *)nes_state_alloc(sizeof(*restore));
    if (!restore) return NES_STATE_ERROR_OUT_OF_MEMORY;
    memset(restore, 0, sizeof(*restore));
    restore->size = nes_state_reader_remaining(reader);
    if (restore->size) {
        restore->data = (uint8_t *)nes_state_alloc(restore->size);
        if (!restore->data) {
            nes_state_dealloc(restore);
            return NES_STATE_ERROR_OUT_OF_MEMORY;
        }
        memcpy(restore->data, reader->data + reader->offset, restore->size);
    }
    NesStateReader validate;
    nes_state_reader_init(&validate, restore->data, restore->size);
    NesStateResult result = mapper_state_read_payload(&validate, false, &restore->board);
    if (result != NES_STATE_OK) {
        board_state_restore_free(restore->board);
        nes_state_dealloc(restore->data);
        nes_state_dealloc(restore);
        return result;
    }
    reader->offset = reader->size;
    *out_restore = restore;
    return NES_STATE_OK;
}

void mapper_state_apply_prepared(MapperStateRestore *restore) {
    if (!restore) return;
    bool old_prg_dirty = prg_ram_dirty;
    bool old_chr_dirty = chr_ram_dirty;
    bool old_exram_dirty = mmc5_exram_dirty;
    bool old_namco_dirty = namco163_audio_dirty;
    bool old_flash_dirty = flash_dirty;
    NesStateReader reader;
    nes_state_reader_init(&reader, restore->data, restore->size);
    if (mapper_state_read_payload(&reader, true, NULL) != NES_STATE_OK) return;
    prg_ram_dirty = prg_ram_dirty || old_prg_dirty;
    chr_ram_dirty = chr_ram_dirty || old_chr_dirty;
    mmc5_exram_dirty = mmc5_exram_dirty || old_exram_dirty;
    namco163_audio_dirty = namco163_audio_dirty || old_namco_dirty;
    flash_dirty = flash_dirty || old_flash_dirty;
    if (restore->board) board_state_apply(active_board, restore->board);
}

void mapper_state_restore_free(MapperStateRestore *restore) {
    if (!restore) return;
    board_state_restore_free(restore->board);
    nes_state_dealloc(restore->data);
    nes_state_dealloc(restore);
}

#undef MAPPER_WRITE_STATE
#undef MAPPER_READ_STATE

#endif
