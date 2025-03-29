#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include "rom/rom.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include "../include/globals.h"
#include <time.h>

// Constants for NTSC NES timing:
const double CPU_FREQ = 1789773.0;              // CPU frequency in Hz
const double FRAME_TIME_MS = 1000.0 / 60.0;       // ~16.67 ms per frame
const double CPU_CYCLES_PER_FRAME = CPU_FREQ / 60.0;  // ~29796 cycles/frame

// Framebuffer definition
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

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
