#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include "rom/rom.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include "joypad/joypad.h"
#include "../include/globals.h"
#include "apu/apu.h"
#include <time.h>
#include "rom/mapper.h"

// Constants for NTSC NES timing:
const double CPU_FREQ = 1789773.0;              // CPU frequency in Hz
const double FRAME_TIME_MS = 1000.0 / 60.0;       // ~16.67 ms per frame
const double CPU_CYCLES_PER_FRAME = CPU_FREQ / 60.0;  // ~29796 cycles/frame

// apu con
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_BUFFER_SAMPLES 1024

// Framebuffer definition
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

// Add joypad instances globally
Joypad pad1 = {0}, pad2 = {0};

// --- timing (NTSC) ---
const int TOTAL_SCANLINES = 262;
const int VBLANK_SCANLINES = 20;        // lines 241–260
const int VISIBLE_SCANLINES = TOTAL_SCANLINES - VBLANK_SCANLINES;

const int CPU_CYCLES_PER_FRAME_I = (int)CPU_CYCLES_PER_FRAME;
const int CYCLES_VISIBLE = CPU_CYCLES_PER_FRAME_I * VISIBLE_SCANLINES / TOTAL_SCANLINES;
const int CYCLES_VBLANK  = CPU_CYCLES_PER_FRAME_I - CYCLES_VISIBLE;

int main(int argc, char *argv[]) {
    SDL_AudioSpec want;                 // uninitialized; we'll memset later
    SDL_AudioSpec have;                 // "
    SDL_AudioDeviceID audio_dev = 0;    // keep handle so we can check it

    if(argc < 2) {
        printf("Usage: %s <rom-file>\n", argv[0]);
        return 1;
    }
    
    printf("Loading ROM: %s\n", argv[1]);
    if(load_rom(argv[1]) != 0) {
        fprintf(stderr, "Failed to load ROM\n");
        return 1;
    }
    ppu_reset(&ppu);
    apu_reset(&apu);
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
    
    printf("Mapper detected: %d\n", ((ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4)));


    // Initialize CPU
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
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }

   /* --- AUDIO SETUP (now safe for C90) --- */
    memset(&want, 0, sizeof want);
    memset(&have, 0, sizeof have);
    want.freq = AUDIO_SAMPLE_RATE;
    want.format = AUDIO_F32;     // float32 mono
    want.channels = 1;
    want.samples = AUDIO_BUFFER_SAMPLES;
    want.callback = apu_sdl_audio_callback;   // from apu.h

    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!audio_dev) {
        fprintf(stderr, "Warning: audio disabled (%s)\n", SDL_GetError());
    } else {
        apu_audio_init(have.freq);
        SDL_PauseAudioDevice(audio_dev, 0);
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
    
    // In main.c (inside the main loop)
    while (running) {
        Uint32 frameStart = SDL_GetTicks();
    
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT)
                running = false;
            
            // Add SDL key mapping
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                int down = (e.type == SDL_KEYDOWN);
    
                switch (e.key.keysym.sym) {
                    case SDLK_z:        joypad_set(&pad1, BTN_A,      down); break; // Z = A
                    case SDLK_x:        joypad_set(&pad1, BTN_B,      down); break; // X = B
                    case SDLK_RSHIFT:   joypad_set(&pad1, BTN_SELECT, down); break; // Right Shift = Select
                    case SDLK_RETURN:   joypad_set(&pad1, BTN_START,  down); break; // Enter = Start
                    case SDLK_UP:       joypad_set(&pad1, BTN_UP,     down); break;
                    case SDLK_DOWN:     joypad_set(&pad1, BTN_DOWN,   down); break;
                    case SDLK_LEFT:     joypad_set(&pad1, BTN_LEFT,   down); break;
                    case SDLK_RIGHT:    joypad_set(&pad1, BTN_RIGHT,  down); break;
                    default: break;
                }
            }
        }
    
    // --- visible period: no vblank ---
    ppu_end_vblank();
    ppu_predict_sprite0_split_for_frame();
    
    // Execute visible scanlines and check for sprite-0 hit at the right CPU cycle
    for (int c = 0; c < CYCLES_VISIBLE; ) {
        // Check if we should assert sprite-0 hit at this CPU cycle
        if (ppu.have_split && ppu.split_cpu_cycles >= 0 && c >= ppu.split_cpu_cycles) {
            if (!ppu.sprite_zero_hit && !(ppu.status & 0x80)) {
                ppu.status |= 0x40;          // Set bit 6: sprite-0 hit
                ppu.sprite_zero_hit = true;
            }
        }
        c += cpu_step(&cpu);
    }
    
        // render BEFORE vblank (while CPU is still processing visible scanlines)
        render_background(framebuffer);
        render_sprites(framebuffer);
        SDL_UpdateTexture(texture, NULL, framebuffer, SCREEN_WIDTH * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    
        // --- vblank period: set the flag, maybe fire NMI, and LET THE CPU RUN WHILE IT'S HIGH ---
        ppu_begin_vblank();
        for (int c = 0; c < CYCLES_VBLANK; ) c += cpu_step(&cpu);
    
        // DON'T call ppu_end_vblank() here - it happens at the start of next frame
    
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < FRAME_TIME_MS) SDL_Delay((Uint32)(FRAME_TIME_MS - frameTime));
    }

    
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

