// ppu.c
#include "ppu.h"
#include "../rom/rom.h"
#include "../../include/globals.h"
#include <stdbool.h>
#include <stdio.h>
#include "../rom/mapper.h"

// PPU Memory
uint8_t ppu_vram[PPU_VRAM_SIZE];
uint8_t ppu_palette[PPU_PALETTE_SIZE];

// PPU Registers
extern uint8_t ppu_vram[PPU_VRAM_SIZE];
extern uint8_t ppu_palette[PPU_PALETTE_SIZE];
PPU ppu;

// Helpers ----------------------------------------------------------------
static inline void set_open_bus(uint8_t v) { ppu.open_bus = v; }
static inline uint8_t get_open_bus(void)   { return ppu.open_bus; }

// $2000–$2007
uint8_t ppu_reg_read(uint16_t reg) {
    switch (reg & 7) {
        case 0: case 1: case 3: case 5: case 6: {
            uint8_t v = get_open_bus(); // PPU's open-bus latch
            return v;
        }

        case 2: { // PPUSTATUS
            uint8_t ob = get_open_bus();
            uint8_t ret = (ppu.status & 0xE0) | (ob & 0x1F);
        
            // ✅ latch exactly what was on the CPU data bus
            set_open_bus(ret);
            // side effects
            ppu.status &= ~0x80; // clear VBlank
            ppu.w = 0;           // reset $2005/$2006 toggle
            return ret;
        }
        case 4: { // OAMDATA
            uint8_t v = ppu.oam[ppu.oam_addr];
            set_open_bus(v);
            return v;
        }
        case 7: { // PPUDATA (read)
            uint16_t addr = ppu.v & 0x3FFF;
            uint8_t ret;
            if (addr >= 0x3F00 && addr < 0x4000) {
                // palette reads bypass the buffer
                ret = ppu_read(addr);
                set_open_bus(ret); // PPU latch sees palette byte
            } else {
                // CPU gets buffered old byte; bus sees the newly fetched one
                ret = ppu.ppudata_buffer;
                uint8_t fetched = ppu_read(addr);
                ppu.ppudata_buffer = fetched;
                set_open_bus(fetched); // PPU latch sees newly fetched byte
            }
            ppu.v += (ppu.ctrl & 0x04) ? 32 : 1;
            return ret;
        }
        default: {
            // This should never happen since reg & 7 covers all cases 0-7
            // But add it to satisfy the compiler
            uint8_t v = get_open_bus();
            return v;
        }
    }
}

void ppu_reg_write(uint16_t reg, uint8_t value) {
    set_open_bus(value); // any write to PPU updates open bus
    switch (reg & 7) {
        case 0: // PPUCTRL
            ppu.ctrl = value;
            // t: ....BA.. ........ = value & 0x03 (nametable select)
            ppu.t = (ppu.t & ~0x0C00) | ((value & 0x03) << 10);
            break;
        case 1: // PPUMASK
            ppu.mask = value;
            break;
        case 3: // OAMADDR
            ppu.oam_addr = value;
            break;
        case 4: // OAMDATA
            ppu.oam[ppu.oam_addr++] = value;
            break;
        case 5: { // PPUSCROLL
            if (ppu.w == 0) {
                // first write: X scroll and coarse X
                ppu.x = value & 0x07;
                ppu.t = (ppu.t & ~0x001F) | (value >> 3);
                ppu.w = 1;
            } else {
                // second write: coarse Y and fine Y
                ppu.t = (ppu.t & ~0x73E0)
                      | ((value & 0x07) << 12)   // fine Y (bits 12-14)
                      | ((value & 0xF8) << 2);   // coarse Y (bits 5-9)
                ppu.w = 0;
            }
            break;
        }
        case 6: { // PPUADDR
            if (ppu.w == 0) {
                // high byte (masked to 6 bits)
                ppu.t = (ppu.t & 0x00FF) | ((uint16_t)(value & 0x3F) << 8);
                ppu.w = 1;
            } else {
                // low byte; commit to v
                ppu.t = (ppu.t & 0xFF00) | value;
                ppu.v = ppu.t;
                ppu.w = 0;
            }
            break;
        }
        case 7: { // PPUDATA (write)
            uint16_t addr = ppu.v & 0x3FFF;
            ppu_write(addr, value);            
        
            // Common quirk: palette writes update the internal buffer as if reading $xF00
            if (addr >= 0x3F00 && addr < 0x4000) {
                ppu.ppudata_buffer = ppu_read(addr - 0x1000);
            }
            ppu.v += (ppu.ctrl & 0x04) ? 32 : 1;
            break;                               // <-- no return here
        }
    }
}

// $4014 OAM DMA — copy 256 bytes from page value<<8 in CPU RAM
extern uint8_t read_mem(uint16_t addr); // declare

void ppu_oam_dma(uint8_t page) {
    uint16_t base = ((uint16_t)page) << 8;
    for (int i = 0; i < 256; i++) {
        uint8_t b = read_mem((uint16_t)(base + i)); // must go through CPU bus!
        ppu.oam[(ppu.oam_addr + i) & 0xFF] = b;     // wrap in 256-byte OAM
    }
    // CPU stalls 513 or 514 cycles; OK to approximate for now
}

void ppu_begin_vblank(void) { ppu.status |= 0x80; }
void ppu_end_vblank(void)   { ppu.status &= ~0x80; }

//---------------------------------------------------------------------
// Sample background palettes (4 background palettes, 4 colors each)
// Note: Palette[0][0] is the universal background color.
// For simplicity these are dummy ARGB colors.
uint32_t bg_palettes[4][4] = {
    { 0xFF757575, 0xFF271B8F, 0xFF0000AB, 0xFF47009F },
    { 0xFF757575, 0xFF006400, 0xFF13A1A1, 0xFF000000 },
    { 0xFF757575, 0xFF8B0000, 0xFFB22222, 0xFFFF0000 },
    { 0xFF757575, 0xFF00008B, 0xFF0000CD, 0xFF4169E1 }
};

// Get the base address of the background pattern table based on PPUCTRL
static uint16_t get_bg_pattern_table_base() {
    // Bit 4 of PPUCTRL controls which pattern table is used for backgrounds
    return (ppu.ctrl & 0x10) ? 0x1000 : 0x0000;
}

static uint16_t mirror_nametable_addr(uint16_t addr) {
    // addr is already masked to 0x3FFF in ppu_read/ppu_write
    // For addresses in 0x2000-0x3EFF (nametables and mirrors):
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t offset = addr - 0x2000;
        // Determine which nametable (0-3)
        int nametable = offset / 0x400;
        int innerOffset = offset % 0x400;
        
        // Handle mirroring based on ROM configuration
        switch (cart_get_mirroring()) {
            case MIRROR_HORIZONTAL: // Horizontal mirroring
                // NT0 and NT1 are unique; NT2 mirrors NT0, NT3 mirrors NT1
                if(nametable == 0 || nametable == 2) return 0x2000 + innerOffset;
                else                                 return 0x2400 + innerOffset;
            
            case MIRROR_VERTICAL: // Vertical mirroring
                // NT0 and NT2 are unique; NT1 mirrors NT0, NT3 mirrors NT2
                if(nametable == 0 || nametable == 1) return 0x2000 + innerOffset;
                else                                 return 0x2800 + innerOffset;
            
            case MIRROR_SINGLE0: // Single-screen mirroring (all nametables use NT0)
                return 0x2000 + innerOffset;

            case MIRROR_SINGLE1: // Single-screen mirroring (all nametables use NT1)
                return 0x2400 + innerOffset;

            case MIRROR_FOUR:
                // (Four-screen uses external RAM; keep a simple fallback: no mirroring)
                return 0x2000 + offset; // simple pass-through into the 2KB VRAM window
            default:
                // Default to horizontal mirroring
                if(nametable == 0 || nametable == 2)
                    return 0x2000 + innerOffset;
                else
                    return 0x2400 + innerOffset;
        }
    }
    return addr; // Other areas are not modified
}

// render_background()
// Renders a full background (256×240 pixels) using the nametable and attribute table.
// - The nametable is located in VRAM at 0x2000–0x23BF (mirrored into 0x2400–0x2FFF).
// - The attribute table is located at 0x23C0–0x23FF for the first nametable.
// - This function assumes a single nametable (NROM configuration).
void render_background(uint32_t *framebuffer) {
    const int tilesX = 32;
    const int tilesY = 30;
    const int tileSize = 8;

    // Get the base address of the background pattern table
    uint16_t pattern_table_base = get_bg_pattern_table_base();

    for (int ty = 0; ty < tilesY; ty++) {
        for (int tx = 0; tx < tilesX; tx++) {
            uint16_t ntAddr = 0x2000 + ty * tilesX + tx;
            uint8_t tileIndex = ppu_vram[mirror_nametable_addr(ntAddr)];

            // Calculate attribute table address
            int attrX = tx / 4;
            int attrY = ty / 4;
            uint16_t attrAddr = 0x23C0 + attrY * 8 + attrX;
            uint8_t attribute = ppu_vram[mirror_nametable_addr(attrAddr)];

            // Determine palette index
            int quadX = (tx % 4) / 2;
            int quadY = (ty % 4) / 2;
            int shift = (quadY * 4) + (quadX * 2);
            uint8_t paletteIndex = (attribute >> shift) & 0x03;

            // Calculate tile address in pattern table
            int tileAddress = pattern_table_base + tileIndex * 16;

            // Render tile
            for (int row = 0; row < tileSize; row++) {
                uint8_t plane0 = chr_rom[tileAddress + row];
                uint8_t plane1 = chr_rom[tileAddress + row + 8];
                
                for (int col = 0; col < tileSize; col++) {
                    uint8_t bit0 = (plane0 >> (7 - col)) & 1;
                    uint8_t bit1 = (plane1 >> (7 - col)) & 1;
                    uint8_t pixelValue = (bit1 << 1) | bit0;

                    // Use ROM-loaded palette RAM instead of dummy bg_palettes
                    uint8_t palIndex = (pixelValue == 0)
                        ? ppu_palette[0]  // universal background color at $3F00
                        : ppu_palette[paletteIndex * 4 + pixelValue];
                    uint32_t color = get_color(palIndex);

                    int screenX = tx * tileSize + col;
                    int screenY = ty * tileSize + row;
                    
                    if (screenX < SCREEN_WIDTH && screenY < SCREEN_HEIGHT) {
                        framebuffer[screenY * SCREEN_WIDTH + screenX] = color;
                    }
                }
            }
        }
    }
}

void render_sprites(uint32_t* fb) {
    uint16_t base = (ppu.ctrl & 0x08) ? 0x1000 : 0x0000;
    for (int i = 0; i < 64; i++) {
        uint8_t y  = ppu.oam[i*4 + 0] + 1;
        uint8_t id = ppu.oam[i*4 + 1];
        uint8_t at = ppu.oam[i*4 + 2];
        uint8_t x  = ppu.oam[i*4 + 3];
        uint8_t pal = (at & 0x03);  // sprite palette 0..3

        int tileAddr = base + id * 16;
        for (int row = 0; row < 8; row++) {
            uint8_t p0 = chr_rom[tileAddr + row];
            uint8_t p1 = chr_rom[tileAddr + row + 8];
            for (int col = 0; col < 8; col++) {
                uint8_t bit0 = (p0 >> (7-col)) & 1;
                uint8_t bit1 = (p1 >> (7-col)) & 1;
                uint8_t px   = (bit1<<1) | bit0;
                if (!px) continue; // color 0 is transparent
                uint8_t idx = ppu_palette[0x10 + pal*4 + px];
                int sx = x + col, sy = y + row;
                if (sx<256 && sy<240) fb[sy*256 + sx] = get_color(idx);
            }
        }
    }
}

// PPU Register Read
uint8_t ppu_read(uint16_t addr) {
    addr &= 0x3FFF;

    // Handle palette mirroring
    if(addr >= 0x3F00 && addr < 0x3F20) {
        addr &= 0x1F;
        // Special case: address 0x3F10, 0x3F14, 0x3F18, 0x3F1C mirror 0x3F00
        if((addr & 0x03) == 0) addr &= 0x0F;
        return ppu_palette[addr];
    }

    // Nametables 0x2000–0x3EFF (with mirroring)
    if (addr >= 0x2000 && addr < 0x3F00) {
        uint16_t effective_addr = mirror_nametable_addr(addr);
        return ppu_vram[effective_addr];
    }

   // Pattern tables 0x0000–0x1FFF: cart (CHR-ROM/RAM, banked)
   return cart_ppu_read(addr & 0x1FFF);
}

// PPU Register Write
void ppu_write(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF;

    if(addr >= 0x3F00 && addr < 0x3F20) {
        addr &= 0x1F;
        // Special case: address 0x3F10, 0x3F14, 0x3F18, 0x3F1C mirror 0x3F00
        if((addr & 0x03) == 0) addr &= 0x0F;
        ppu_palette[addr] = value;
        return;
    }

    // Pattern tables (CHR). If the cart uses CHR-RAM, writes should stick; with CHR-ROM they
    // are typically ignored. For the test ROMs, allowing writes is fine.
    if (addr < 0x2000) { cart_ppu_write(addr & 0x1FFF, value); return; }

    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t effective_addr = mirror_nametable_addr(addr);
        ppu_vram[effective_addr] = value;
        return;
    }

    // 0x3F00–0x3FFF handled above; nothing else is writable here.
}

// Reset PPU to initial state
void ppu_reset(PPU* ppu) {
    ppu->ctrl = 0;
    ppu->mask = 0;
    ppu->status = 0x00;
    ppu->oam_addr = 0;
    ppu->scroll_x = 0;
    ppu->scroll_y = 0;
    ppu->v = 0;
    ppu->t = 0;
    ppu->x = 0;
    ppu->w = 0;
    ppu->ppudata_buffer = 0;
    ppu->open_bus = 0;
    
    // Clear OAM
    for (int i = 0; i < PPU_OAM_SIZE; i++) {
        ppu->oam[i] = 0xFF;
    }
}

// Convert a pixel value (0-3 or an index into the NES palette) to an ARGB color
uint32_t get_color(uint8_t idx) {
    idx &= 0x3F;

    // PPUMASK bit 0: grayscale
    if (ppu.mask & 0x01) idx &= 0x30;

    uint32_t c = nes_palette[idx];
    float r = (float)((c >> 16) & 0xFF);
    float g = (float)((c >>  8) & 0xFF);
    float b = (float)((c      ) & 0xFF);

    // Stronger, NES-like emphasis: emphasized channel stays,
    // the other two are attenuated noticeably.
    const float ATTEN = 0.60f;  // try 0.55–0.70 if you want to tweak

    if (ppu.mask & 0x20) {           // emphasize RED
        g *= ATTEN; b *= ATTEN;
    }
    if (ppu.mask & 0x40) {           // emphasize GREEN
        r *= ATTEN; b *= ATTEN;
    }
    if (ppu.mask & 0x80) {           // emphasize BLUE
        r *= ATTEN; g *= ATTEN;
    }

    // clamp
    int R = (int)(r < 0 ? 0 : (r > 255 ? 255 : r));
    int G = (int)(g < 0 ? 0 : (g > 255 ? 255 : g));
    int B = (int)(b < 0 ? 0 : (b > 255 ? 255 : b));
    return 0xFF000000u | (R << 16) | (G << 8) | B;
}

// Render the first pattern table (first 4KB of CHR-ROM) as a grid of 16x16 8x8 tiles.
// Each tile is decoded from 16 bytes (8 bytes for plane 0 and 8 bytes for plane 1).
// The resulting image is 128x128 pixels and is centered in the 256x240 screen.
void render_tiles() {
    if (!chr_rom) {
        // Fallback to test pattern if no CHR-ROM data
        for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
            framebuffer[i] = 0xFF0000FF; // solid blue, for example
        return;
    }

    const int tileWidth = 8;
    const int tileHeight = 8;
    const int tilesPerRow = 16;
    const int gridWidth = tilesPerRow * tileWidth; // 128 pixels
    const int gridHeight = tilesPerRow * tileHeight; // 128 pixels (16 rows)

    // Compute top-left corner to center the grid on the screen
    int offsetX = (SCREEN_WIDTH - gridWidth) / 2;
    int offsetY = (SCREEN_HEIGHT - gridHeight) / 2;

    // Clear framebuffer (set to black)
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++) {
        framebuffer[i] = 0xFF000000;
    }

    // For each tile in the first pattern table (4KB of data = 256 tiles)
    for (int tile = 0; tile < 256; tile++) {
        int tileX = (tile % tilesPerRow) * tileWidth;
        int tileY = (tile / tilesPerRow) * tileHeight;
        int tileOffset = tile * 16;  // each tile uses 16 bytes

        // For each row in the tile (0 to 7)
        for (int row = 0; row < tileHeight; row++) {
            // The two bit planes: first 8 bytes and second 8 bytes.
            uint8_t plane0 = chr_rom[tileOffset + row];
            uint8_t plane1 = chr_rom[tileOffset + row + 8];

            // For each pixel in the row (0 to 7)
            for (int col = 0; col < tileWidth; col++) {
                // The bit for this pixel is the (7-col) bit of each plane.
                uint8_t bit0 = (plane0 >> (7 - col)) & 1;
                uint8_t bit1 = (plane1 >> (7 - col)) & 1;
                uint8_t pixelValue = (bit1 << 1) | bit0;

                // Compute the position in the framebuffer:
                int screenX = offsetX + tileX + col;
                int screenY = offsetY + tileY + row;
                if (screenX < 0 || screenX >= SCREEN_WIDTH ||
                    screenY < 0 || screenY >= SCREEN_HEIGHT) {
                    continue;
                }
                framebuffer[screenY * SCREEN_WIDTH + screenX] = get_color(pixelValue);
            }
        }
    }

    // Print first few bytes of CHR-ROM
    printf("First 16 bytes of CHR-ROM:\n");
    for (int i = 0; i < 16; i++) {
        printf("%02X ", chr_rom[i]);
    }
    printf("\n\n");

    // Print pixel values for first few tiles
    printf("Pixel values for first 3 tiles:\n");
    for (int tile = 0; tile < 3; tile++) {
        printf("Tile %d:\n", tile);
        int tileOffset = tile * 16;
        for (int row = 0; row < 8; row++) {
            uint8_t plane0 = chr_rom[tileOffset + row];
            uint8_t plane1 = chr_rom[tileOffset + row + 8];
            for (int col = 0; col < 8; col++) {
                uint8_t bit0 = (plane0 >> (7 - col)) & 1;
                uint8_t bit1 = (plane1 >> (7 - col)) & 1;
                uint8_t pixelValue = (bit1 << 1) | bit0;
                printf("%d ", pixelValue);
            }
            printf("\n");
        }
        printf("\n");
    }

    // Print palette colors
    printf("NES Palette Colors:\n");
    for (int i = 0; i < 16; i++) {  // Print first 16 colors
        printf("Color %02d: 0x%08X\n", i, nes_palette[i]);
    }
}