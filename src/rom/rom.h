#ifndef ROM_H
#define ROM_H

#include <stdint.h>

typedef struct {
    uint8_t signature[4];     // "NES" + 0x1A
    uint8_t prg_rom_chunks;   // 16KB units
    uint8_t chr_rom_chunks;   // 8KB units (0 means CHR-RAM)
    uint8_t flags6;           // Mapper low nibble, mirroring, battery, trainer
    uint8_t flags7;           // Mapper high nibble, VS/Playchoice, NES 2.0
    uint8_t prg_ram_size;     // 8KB units (0 implies 1)
    uint8_t flags9;           // TV system (0=NTSC, 1=PAL)
    uint8_t flags10;          // TV system + PRG-RAM presence
    uint8_t unused[5];        // Padding
} iNESHeader;

extern iNESHeader ines_header;

// Global variable for mirroring mode:
// 0 = horizontal, 1 = vertical.
extern int mirroring_mode;

extern uint8_t prg_rom[0x8000];
extern uint8_t *chr_rom;

int load_rom(const char *filename);

#define PRG_ROM_BANK_SIZE 0x4000  // 16KB banks
#define CHR_ROM_BANK_SIZE 0x2000  // 8KB banks

#endif // ROM_H
