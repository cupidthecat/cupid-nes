/*
 * mapper_memory.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Physical backing for debugger edits. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_MAPPER_MEMORY_H
#define CUPID_MAPPER_MEMORY_H

static bool debug_prg_ram_location(RamBlock *ram, size_t offset, bool writable, NesMemoryLocation *out) {
    if (!ram || !ram->size) return false;
    return nes_memory_location(out, ram->data, ram->size, offset % ram->size,
                               ram == &prg_save_ram ? "PRG NVRAM" : "PRG RAM", writable,
                               ram == &prg_save_ram && battery_enabled ? &prg_ram_dirty : NULL);
}

static bool debug_chr_location(size_t offset, NesMemoryLocation *out) {
    bool saved = chr_nvram_contains(offset);
    if (!nes_memory_location(out, C.chr, C.chr_sz, offset,
                             C.chr_is_ram ? saved ? "CHR NVRAM" : "CHR RAM" : "CHR ROM", C.chr_is_ram,
                             saved && battery_enabled ? &chr_ram_dirty : NULL)) return false;
    if (saved) out->offset -= chr_nvram_offset();
    return true;
}

static bool debug_native_prg_rom(uint16_t addr, NesMemoryLocation *out) {
    if (addr < 0x8000 || !C.prg_sz) return false;
    size_t offset;
    if (cart->cpu_read == nrom_cpu_read) {
        if (C.prg_sz < PRG_BANK_16K) {
            if ((size_t)(addr - 0x8000) >= PRG_BANK_32K / C.prg_sz * C.prg_sz) return false;
            offset = (addr - 0x8000) % C.prg_sz;
        } else {
            size_t slot = (addr - 0x8000) / PRG_BANK_16K;
            offset = slot % (C.prg_sz / PRG_BANK_16K) * PRG_BANK_16K + (addr & 0x3FFF);
        }
    } else if (cart == &mapper_uxrom) {
        if (C.prg_sz < PRG_BANK_16K) {
            if ((size_t)(addr - 0x8000) >= PRG_BANK_32K / C.prg_sz * C.prg_sz) return false;
            offset = (addr - 0x8000) % C.prg_sz;
        } else {
            size_t banks = C.prg_sz / PRG_BANK_16K;
            size_t bank = addr < 0xC000 ? (C.mapper_no == 180 ? 0 : ux.bank % banks)
                                               : (C.mapper_no == 180 ? ux.bank % banks : banks - 1);
            offset = bank * PRG_BANK_16K + (addr & 0x3FFF);
        }
    } else if (cart == &mapper_mmc1) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        if (!banks) return false;
        unsigned mode = (mmc1.control >> 2) & 3, slot = (addr - 0x8000) >> 14;
        size_t outer = C.prg_sz == 0x80000 ? mmc1_extra_register() & 0x10 : 0;
        size_t bank;
        if (C.submapper == 5) bank = slot;
        else if (mode < 2) bank = ((mmc1.prg_bank & 0x0E) + slot) | outer;
        else if (mode == 2) bank = (slot ? mmc1.prg_bank & 0x0F : 0) | outer;
        else bank = (slot ? 0x0F : mmc1.prg_bank & 0x0F) | outer;
        offset = (bank % banks) * PRG_BANK_16K + (addr & 0x3FFF);
    } else if (cart == &mapper_mmc3 || cart == &mapper_txsrom) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        if (!banks) return false;
        size_t last = banks - 1, previous = banks > 1 ? banks - 2 : 0;
        size_t bank = addr < 0xA000 ? mmc3.prg_mode ? previous : mmc3.banks[6]
                    : addr < 0xC000 ? mmc3.banks[7]
                    : addr < 0xE000 ? mmc3.prg_mode ? mmc3.banks[6] : previous : last;
        offset = (bank % banks) * PRG_BANK_8K + (addr & 0x1FFF);
    } else if (cart == &mapper_mmc5 && !mmc5_ram_mapped(addr)) {
        if (C.prg_sz < PRG_BANK_8K) {
            if ((size_t)(addr - 0x8000) >= PRG_BANK_32K / C.prg_sz * C.prg_sz) return false;
            offset = (addr - 0x8000) % C.prg_sz;
        } else {
            offset = mmc5_map_prg_slot_to_bank((addr - 0x8000) >> 13) * PRG_BANK_8K + (addr & 0x1FFF);
        }
    } else return false;
    return nes_memory_location(out, C.prg, C.prg_sz, offset, "PRG ROM", false, NULL);
}

bool cart_debug_cpu_location(uint16_t addr, NesMemoryLocation *out) {
    if (!cart || !out) return false;
    if (active_board) return board_debug_location(active_board, true, addr, out);
    if (cart == &mapper_mmc5) {
        if (addr >= 0x5C00 && addr <= 0x5FFF && mmc5.exram_mode >= 2) {
            return nes_memory_location(out, mmc5_exram, sizeof(mmc5_exram), addr - 0x5C00, "MMC5 ExRAM",
                                       mmc5.exram_mode == 2, battery_enabled ? &mmc5_exram_dirty : NULL);
        }
        if (mmc5_ram_mapped(addr)) {
            size_t offset = 0;
            RamBlock *ram = mmc5_ram_location(addr, &offset);
            bool writable = (mmc5.prg_ram_protect1 & 3) == 2 && (mmc5.prg_ram_protect2 & 3) == 1;
            return debug_prg_ram_location(ram, offset, writable, out);
        }
    }
    if (addr >= 0x6000 && addr < 0x8000) {
        if (cart == &mapper_mmc1) {
            if (!C.mmc1a && (mmc1.prg_bank & 0x10)) return false;
            size_t offset;
            RamBlock *ram = mmc1_ram_location(addr, &offset);
            return debug_prg_ram_location(ram, offset, true, out);
        }
        if (cart == &mapper_mmc3 || cart == &mapper_txsrom) {
            if (C.submapper == 1) {
                unsigned read = addr & 0x200 ? 0x80 : 0x20, both = read | (read >> 1);
                if (addr < 0x7000 || !mmc3.ram_enabled || !(mmc3.ram_protect & read)) return false;
                return debug_prg_ram_location(default_prg_ram(), addr & 0x3FF,
                                              (mmc3.ram_protect & both) == both, out);
            }
            if (C.submapper != 3 && !(mmc3.ram_protect & 0x80)) return false;
            RamBlock *ram = default_prg_ram();
            if ((size_t)(addr - 0x6000) >= prg_ram_window_coverage(ram)) return false;
            return debug_prg_ram_location(ram, addr - 0x6000,
                                          C.submapper == 3 || !(mmc3.ram_protect & 0x40), out);
        }
        if (cart->cpu_read == nrom_cpu_read && cart->cpu_write == nrom_cpu_write) {
            RamBlock *ram = default_prg_ram();
            if ((size_t)(addr - 0x6000) >= prg_ram_window_coverage(ram)) return false;
            return debug_prg_ram_location(ram, addr - 0x6000, true, out);
        }
    }
    return debug_native_prg_rom(addr, out);
}

static bool debug_pattern_location(uint16_t addr, NesMemoryLocation *out) {
    size_t offset;
    if (cart->ppu_read == nrom_ppu_read) {
        offset = C.chr_is_ram ? chr_default_offset(addr, CHR_BANK_8K) : addr;
    } else if (cart->ppu_read == mmc1_ppu_read) {
        if (!mmc1_chr_offset(addr, &offset)) offset = chr_default_offset(addr, CHR_BANK_4K);
    } else if (cart->ppu_read == mmc3_ppu_read) {
        size_t page = shrunk_chr_page_size(CHR_BANK_1K);
        if (!page || addr / page >= 8) offset = chr_default_offset(addr, CHR_BANK_1K);
        else offset = mmc3_chr_bank_for_slot((uint8_t)(addr / page)) % (C.chr_sz / page) * page + addr % page;
    } else if (cart == &mapper_mmc5 && C.chr_sz) {
        CartPpuFetchSource previous = cart_ppu_fetch_source;
        cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
        offset = (mmc5_map_chr_bank_1k(addr) * CHR_BANK_1K + (addr & 0x3FF)) & (C.chr_sz - 1);
        cart_ppu_fetch_source = previous;
        if (!debug_chr_location(offset, out)) return false;
        if (C.chr_sz < CHR_BANK_1K) out->writable = false;
        return true;
    } else return false;
    return debug_chr_location(offset, out);
}

bool cart_debug_ppu_location(uint16_t addr, uint8_t *nt_ram, NesMemoryLocation *out) {
    if (!cart || !out || addr >= 0x3F00) return false;
    if (active_board) return board_debug_location(active_board, false, addr, out);
    if (addr < 0x2000) return debug_pattern_location(addr, out);
    size_t in = addr & 0x3FF, slot = ((addr - 0x2000) & 0xFFF) >> 10;
    size_t offset;
    if (cart == &mapper_m111) {
        offset = ((m111.bank_latch & 0x20) ? 8 : 0) * CHR_BANK_1K + ((addr - 0x2000) & 0x1FFF);
        return nes_memory_location(out, m111_nt_ram, sizeof(m111_nt_ram), offset, "Cartridge nametable RAM", true, NULL);
    }
    if (cart == &mapper_jy) {
        if (jy_advanced_nt() && (jy.disable_nt_ram || (jy.nt_low[slot] & 0x80) != (jy.nt_ram_select_bit & 0x80))) {
            if (C.chr_is_ram) return false;
            return debug_chr_location(((size_t)jy.nt_low[slot] | ((size_t)jy.nt_high[slot] << 8)) * CHR_BANK_1K + in, out);
        }
        offset = jy_ciram_page((unsigned)slot) * CHR_BANK_1K + in;
    } else if (cart == &mapper_txsrom) {
        offset = txsrom_nt[slot] * CHR_BANK_1K + in;
    } else if (cart == &mapper_unrom512 && unrom512_four_screen_chr) {
        offset = 0x6000 + ((addr - 0x2000) & 0x1FFF);
        if (C.chr_is_ram) return debug_chr_location(offset, out);
        RamBlock *ram = chr_save_ram.size ? &chr_save_ram : &chr_work_ram;
        return nes_memory_location(out, ram->data, ram->size, offset,
                                   ram == &chr_save_ram ? "CHR NVRAM" : "CHR RAM", true,
                                   ram == &chr_save_ram && battery_enabled ? &chr_ram_dirty : NULL);
    } else if (cart == &mapper_vrc6 && vrc6.ppu_initialized) {
        unsigned chunk = ((addr - 0x2000) & 0xFFF) >> 8;
        offset = vrc6.nt_offsets[chunk] + (addr & 0xFF);
        if (vrc6.nt_chr[chunk]) return debug_chr_location(offset, out);
    } else if (cart == &mapper_sunsoft4 && sunsoft4.use_chr_nt) {
        return debug_chr_location(sunsoft4_nt_chr_offset(addr), out);
    } else if (cart == &mapper_mmc5) {
        unsigned source = mmc5_nt_source(addr);
        if (source == 3 || (source == 2 && mmc5.exram_mode >= 2)) return false;
        if (source == 2) return nes_memory_location(out, mmc5_exram, sizeof(mmc5_exram), in, "MMC5 ExRAM", true,
                                                   battery_enabled ? &mmc5_exram_dirty : NULL);
        offset = source * CHR_BANK_1K + in;
    } else if (cart == &mapper_namco) {
        unsigned mapped = addr >= 0x3000 ? addr - 0x1000 : addr;
        unsigned chunk = mapped >> 8;
        offset = namco_ppu_offset[chunk] + (mapped & 0xFF);
        if (namco_ppu_source[chunk] == NAMCO_PPU_CHR) return debug_chr_location(offset, out);
        if (namco_ppu_source[chunk] != NAMCO_PPU_CIRAM) return false;
        offset &= 0x7FF;
    } else if (cart == &mapper_namco108 && C.mapper_no == 95 && namco108.nametables_selected) {
        offset = ((namco108.banks[slot < 2 ? 0 : 1] >> 5) & 1) * CHR_BANK_1K + in;
    } else if (cart == &mapper_rambo158) {
        offset = (rambo1.nt_map[slot] & 1) * CHR_BANK_1K + in;
    } else offset = base_nt_index(addr);
    return nes_memory_location(out, nt_ram, 0x1000, offset, "Nametable RAM", true, NULL);
}

#endif
