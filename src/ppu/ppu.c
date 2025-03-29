// ppu.c
#include "ppu.h"
#include "../rom/rom.h"
#include <stdbool.h>

// PPU Memory
uint8_t ppu_vram[PPU_VRAM_SIZE];
uint8_t ppu_palette[PPU_PALETTE_SIZE];

// PPU Registers
PPU ppu;

static uint16_t mirror_nametable_addr(uint16_t addr) {
    // addr is already masked to 0x3FFF in ppu_read/ppu_write
    // For addresses in 0x2000-0x3EFF (nametables and mirrors):
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t offset = addr - 0x2000;
        // Determine which nametable (0-3)
        int nametable = offset / 0x400;
        int innerOffset = offset % 0x400;
        // Use mirroring mode from the ROM header:
        if(mirroring_mode == 0) { // HORIZONTAL mirroring
            // NT0 and NT1 are unique; NT2 mirrors NT0, NT3 mirrors NT1.
            if(nametable == 0 || nametable == 2)
                return 0x2000 + innerOffset;
            else
                return 0x2400 + innerOffset;
        } else { // VERTICAL mirroring
            // NT0 and NT2 are unique; NT1 mirrors NT0, NT3 mirrors NT2.
            if(nametable == 0 || nametable == 1)
                return 0x2000 + innerOffset;
            else
                return 0x2800 + innerOffset;
        }
    }
    return addr; // Other areas are not modified
}

// PPU Register Read
uint8_t ppu_read(uint16_t addr) {
    addr &= 0x3FFF; // Mirror every 16KB

    // Handle palette mirroring: addresses 0x3F00-0x3F1F.
    if(addr >= 0x3F00 && addr < 0x3F20) {
        return ppu_palette[addr & 0x1F];
    }
    
    // Nametable read: apply mirroring.
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t effective_addr = mirror_nametable_addr(addr);
        return ppu_vram[effective_addr];
    }
    
    // Pattern tables and others.
    return ppu_vram[addr];
}

// PPU Register Write
void ppu_write(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF; // Mirror every 16KB

    if(addr >= 0x3F00 && addr < 0x3F20) {
        ppu_palette[addr & 0x1F] = value;
        return;
    }
    
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t effective_addr = mirror_nametable_addr(addr);
        ppu_vram[effective_addr] = value;
        return;
    }
    
    ppu_vram[addr] = value;
}

// Reset PPU to initial state
void ppu_reset(PPU* ppu) {
    ppu->ctrl = 0;
    ppu->mask = 0;
    ppu->status = 0xA0; // Status bits 5 and 7 are always set
    ppu->oam_addr = 0;
    ppu->scroll_x = 0;
    ppu->scroll_y = 0;
    ppu->vram_addr = 0;
    ppu->vram_latch = 0;
    
    // Clear OAM
    for (int i = 0; i < PPU_OAM_SIZE; i++) {
        ppu->oam[i] = 0xFF;
    }
}