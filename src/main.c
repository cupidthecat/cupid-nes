#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include "rom/rom.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include <time.h>

// Constants for NTSC NES timing:
const double CPU_FREQ = 1789773.0;              // CPU frequency in Hz
const double FRAME_TIME_MS = 1000.0 / 60.0;       // ~16.67 ms per frame
const double CPU_CYCLES_PER_FRAME = CPU_FREQ / 60.0;  // ~29796 cycles/frame

// Screen dimensions
#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 240

// Framebuffer for rendering (RGBA format)
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];


// Convert a 2-bit pixel value to an ARGB color (using a simple grayscale palette)
uint32_t get_color(uint8_t pixel) {
    // Simple mapping: 0->black, 1->dark gray, 2->light gray, 3->white.
    uint8_t intensity;
    switch(pixel) {
        case 0: intensity = 0x00; break;
        case 1: intensity = 0x55; break;
        case 2: intensity = 0xAA; break;
        case 3: intensity = 0xFF; break;
        default: intensity = 0x00; break;
    }
    // ARGB: alpha=0xFF, then intensity for red, green, blue.
    return (0xFF << 24) | (intensity << 16) | (intensity << 8) | intensity;
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
}

int main(int argc, char *argv[]) {
    if(argc < 2) {
        printf("Usage: %s <rom-file>\n", argv[0]);
        return 1;
    }
    
    printf("Loading ROM: %s\n", argv[1]);
    if(load_rom(argv[1]) != 0) {
        fprintf(stderr, "Failed to load ROM\n");
        return 1;
    }
    
    // Print header information for debugging
    printf("=== ROM Header Info ===\n");
    printf("Signature: %c%c%c 0x%02X\n", 
           ines_header.signature[0], 
           ines_header.signature[1], 
           ines_header.signature[2],
           ines_header.signature[3]);
    printf("PRG-ROM Chunks: %d\n", ines_header.prg_rom_chunks);
    printf("CHR-ROM Chunks: %d\n", ines_header.chr_rom_chunks);
    printf("Flags6: 0x%02X\n", ines_header.flags6);
    printf("Flags7: 0x%02X\n", ines_header.flags7);
    printf("Mirroring: %s\n", (mirroring_mode == 0 ? "Horizontal" : "Vertical"));
    printf("=======================\n");
    
    if (ines_header.prg_rom_chunks > 1 || (ines_header.flags6 & 0xF0)) {
        printf("WARNING: This ROM likely uses a mapper (mapper number: %d).\n",
            (ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4));
        printf("If you haven't implemented mapper support, expect memory mapping issues (and segfaults).\n");
    }
    
    // Initialize CPU
    CPU cpu = {0};
    printf("Resetting CPU...\n");
    cpu_reset(&cpu);
    printf("CPU state after reset:\n");
    printf("  PC: 0x%04X\n", cpu.pc);
    printf("  SP: 0x%02X\n", cpu.sp);
    printf("  A: 0x%02X\n", cpu.a);
    printf("  X: 0x%02X\n", cpu.x);
    printf("  Y: 0x%02X\n", cpu.y);
    printf("  Status: 0x%02X\n", cpu.status);

    // Initialize SDL video
    if(SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window *window = SDL_CreateWindow("Cupid NES Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH * 2, SCREEN_HEIGHT * 2, SDL_WINDOW_SHOWN);
    if(!window) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if(!renderer) {
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);
    if(!texture) {
        fprintf(stderr, "SDL_CreateTexture Error: %s\n", SDL_GetError());
        return 1;
    }

    bool running = true;
    SDL_Event e;
    
    while (running) {
        Uint32 frameStart = SDL_GetTicks();
        double cpuCyclesExecuted = 0.0;

        // Process events
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT)
                running = false;
        }

        // Execute CPU instructions until one frame's worth of cycles has passed
        while (cpuCyclesExecuted < CPU_CYCLES_PER_FRAME) {
            int cycles = cpu_step(&cpu);
            cpuCyclesExecuted += cycles;
        }

        // Instead of a test pattern, render the first pattern table (tile data)
        render_tiles();

        SDL_UpdateTexture(texture, NULL, framebuffer, SCREEN_WIDTH * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        Uint32 frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < FRAME_TIME_MS) {
            SDL_Delay((Uint32)(FRAME_TIME_MS - frameTime));
        }
    }
    
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
