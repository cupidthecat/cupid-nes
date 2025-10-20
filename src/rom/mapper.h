// rom/mapper.h
#ifndef MAPPER_H
#define MAPPER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "rom.h"

typedef struct Mapper {
    // CPU space ($6000–$FFFF typically)
    uint8_t (*cpu_read)(uint16_t addr);
    void     (*cpu_write)(uint16_t addr, uint8_t v);

    // PPU pattern space ($0000–$1FFF)
    uint8_t (*ppu_read)(uint16_t addr);
    void     (*ppu_write)(uint16_t addr, uint8_t v);

    // Optional hooks
    void     (*reset)(void);
    void     (*clock)(int cpu_cycles);   // for IRQ-capable mappers later
    Mirroring (*get_mirroring)(void);
} Mapper;

// Global “inserted” cart
extern Mapper *cart;

// Front door used by CPU/PPU
uint8_t cart_cpu_read (uint16_t addr);
void    cart_cpu_write(uint16_t addr, uint8_t v);
uint8_t cart_ppu_read (uint16_t addr);
void    cart_ppu_write(uint16_t addr, uint8_t v);

// Init from iNES 1.0 header + loaded PRG/CHR blobs
int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz);

// Current mirroring for PPU (keeps your existing mirroring_mode in sync)
Mirroring cart_get_mirroring(void);

#endif