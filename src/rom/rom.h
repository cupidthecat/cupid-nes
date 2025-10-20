// rom.h
#ifndef ROM_H
#define ROM_H
#include <stdint.h>
#include <stddef.h>

typedef struct __attribute__((packed)) {
    uint8_t signature[4];      // "NES\x1A"
    uint8_t prg_rom_chunks;    // count of 16KB units
    uint8_t chr_rom_chunks;    // count of 8KB  units
    uint8_t flags6;
    uint8_t flags7;
    uint8_t prg_ram_size;      // (iNES 1.0) in 8KB units
    uint8_t flags9;
    uint8_t flags10;
    uint8_t zero[5];
} iNESHeader;

typedef enum {
    MIRROR_HORIZONTAL = 0,
    MIRROR_VERTICAL   = 1,
    MIRROR_SINGLE0    = 2,
    MIRROR_SINGLE1    = 3,
    MIRROR_FOUR       = 4
} Mirroring;

// expose sizes so CPU/PPU can reason about mirroring
extern iNESHeader ines_header;
extern size_t     prg_size;
extern size_t     chr_size;
extern uint8_t    prg_rom[0x8000];  // fixed 32KB buffer
extern uint8_t   *chr_rom;

int load_rom(const char *filename);

// mirroring for PPU
extern int mirroring_mode;

#endif
