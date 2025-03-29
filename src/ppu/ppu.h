#ifndef PPU_H
#define PPU_H

#include <stdint.h>

// PPU Memory Sizes
#define PPU_VRAM_SIZE 0x4000  // 16KB VRAM
#define PPU_OAM_SIZE 0x100    // 256 bytes OAM
#define PPU_PALETTE_SIZE 0x20 // 32 bytes palette RAM

// PPU Register Addresses
#define PPUCTRL   0x2000
#define PPUMASK   0x2001
#define PPUSTATUS 0x2002
#define OAMADDR   0x2003
#define OAMDATA   0x2004
#define PPUSCROLL 0x2005
#define PPUADDR   0x2006
#define PPUDATA   0x2007
#define OAMDMA    0x4014

// PPU Registers
typedef struct {
    uint8_t ctrl;       // PPUCTRL
    uint8_t mask;       // PPUMASK
    uint8_t status;     // PPUSTATUS
    uint8_t oam_addr;   // OAMADDR
    uint8_t scroll_x;   // PPUSCROLL X
    uint8_t scroll_y;   // PPUSCROLL Y
    uint16_t vram_addr; // PPUADDR
    uint8_t vram_latch; // Internal VRAM address latch
    uint8_t oam[PPU_OAM_SIZE]; // Object Attribute Memory
} PPU;

// PPU Memory
extern uint8_t ppu_vram[PPU_VRAM_SIZE];      // VRAM (pattern tables, nametables, attribute tables)
extern uint8_t ppu_palette[PPU_PALETTE_SIZE];  // Palette RAM

// Function prototypes
uint8_t ppu_read(uint16_t addr);
void ppu_write(uint16_t addr, uint8_t value);
void ppu_reset(PPU* ppu);

#endif // PPU_H
