/* Mapper-aware inspection without PPU bus activity. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_MAPPER_DEBUG_H
#define CUPID_MAPPER_DEBUG_H

uint8_t cart_debug_chr(uint16_t addr, CartPpuFetchSource source) {
    addr &= 0x1FFF;
    CartPpuFetchSource previous = cart_ppu_fetch_source;
    cart_ppu_fetch_source = source;
    uint8_t value;
    if (cart == &mapper_mmc5) {
        size_t offset = mmc5_map_chr_bank_1k(addr) * CHR_BANK_1K + (addr & 0x3FF);
        value = mmc5_read_chr_raw(offset);
    } else {
        value = cart_ppu_peek(addr);
    }
    cart_ppu_fetch_source = previous;
    return value;
}

void cart_debug_bg_row(uint16_t nt_addr, uint16_t pattern_addr, unsigned row, uint8_t *low, uint8_t *high,
                       uint8_t *palette) {
    if (cart == &mapper_mmc5 && mmc5.exram_mode == 1) {
        uint8_t attr = mmc5_exram[nt_addr & 0x3FF];
        size_t bank = (attr & 0x3F) | ((size_t)mmc5.chr_upper << 6);
        size_t address = (bank << 12) | (pattern_addr & 0xFF0) | (row & 7);
        *palette = attr >> 6;
        *low = mmc5_read_chr_raw(address);
        *high = mmc5_read_chr_raw(address + 8);
    } else {
        *low = cart_debug_chr((uint16_t)(pattern_addr + row), CART_PPU_FETCH_BG);
        *high = cart_debug_chr((uint16_t)(pattern_addr + row + 8), CART_PPU_FETCH_BG);
    }
}

bool cart_debug_write_ppu(uint16_t addr, uint8_t value, uint8_t *nt_ram) {
    addr &= 0x3FFF;
    if (!cart || addr >= 0x3F00) {
        return false;
    }
    if (active_board) {
        return board_debug_write_ppu(active_board, addr, value);
    }
    if (addr < 0x2000) {
        if (cart != &mapper_fds && !C.chr_is_ram) {
            return false;
        }
        CartPpuFetchSource previous = cart_ppu_fetch_source;
        cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
        cart_ppu_write(addr, value);
        cart_ppu_fetch_source = previous;
        return cart_debug_chr(addr, CART_PPU_FETCH_CPU) == value;
    }
    if (cart == &mapper_mmc5) {
        uint8_t source = mmc5_nt_source(addr);
        if (source == 3 || (source == 2 && mmc5.exram_mode >= 2)) {
            return false;
        }
    }
    if (cart == &mapper_jy && jy_advanced_nt()) {
        unsigned slot = ((addr - 0x2000) & 0xFFF) >> 10;
        if (jy.disable_nt_ram || (jy.nt_low[slot] & 0x80) != (jy.nt_ram_select_bit & 0x80)) {
            return false;
        }
    }
    /* CHR-backed nametables follow their hardware write protection. */
    if (cart == &mapper_sunsoft4 && sunsoft4.use_chr_nt && !C.chr_is_ram) {
        return false;
    }
    if (cart == &mapper_vrc6 && vrc6.ppu_initialized && vrc6.nt_chr[((addr - 0x2000) & 0xFFF) >> 8] && !C.chr_is_ram) {
        return false;
    }
    if (cart == &mapper_namco && namco_ppu_source[addr >> 8] == NAMCO_PPU_CHR && !C.chr_is_ram) {
        return false;
    }
    cart_nt_write(addr, value, nt_ram);
    return cart_nt_peek(addr, nt_ram) == value;
}
#endif
