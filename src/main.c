/*
 * main.c - Main entry point for Cupid NES Emulator
 *
 * Author: @frankischilling
 *
 * This file initializes the loaded cartridge, CPU, PPU, APU, SDL video and audio, input,
 * palette tools, frame pacing, and the main emulation loop. It also handles controller
 * input, reset events, palette file loading, and final emulator shutdown.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <SDL2/SDL.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "rom/rom.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include "joypad/joypad.h"
#include "../include/globals.h"
#include "apu/apu.h"
#include <time.h>
#include "rom/mapper.h"
#include <math.h>
#include "ui/palette_tool.h"
#include "system/timing.h"
#include "system/hardware.h"

#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_BUFFER_SAMPLES 1024

// SDL presents the framebuffer that the PPU fills.
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

Joypad pad1 = {0}, pad2 = {0};

static SDL_GameController *controllers[NES_INPUT_PLAYERS];

static void open_controller(int device) {
    if (!SDL_IsGameController(device)) return;
    SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(device);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (controllers[player] &&
            SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player])) == id)
            return;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) {
            controllers[player] = SDL_GameControllerOpen(device);
            return;
        }
    }
}

static void controller_event(const SDL_Event *event) {
    if (event->type == SDL_CONTROLLERDEVICEADDED) {
        open_controller(event->cdevice.which);
        return;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) continue;
        SDL_JoystickID id = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player]));
        if (event->type == SDL_CONTROLLERDEVICEREMOVED && event->cdevice.which == id) {
            SDL_GameControllerClose(controllers[player]);
            controllers[player] = NULL;
            joypad_player(player)->buttons = 0;
        } else if ((event->type == SDL_CONTROLLERBUTTONDOWN || event->type == SDL_CONTROLLERBUTTONUP)
                   && event->cbutton.which == id) {
            int button;
            switch (event->cbutton.button) {
                case SDL_CONTROLLER_BUTTON_A: button = BTN_A; break;
                case SDL_CONTROLLER_BUTTON_B: button = BTN_B; break;
                case SDL_CONTROLLER_BUTTON_BACK: button = BTN_SELECT; break;
                case SDL_CONTROLLER_BUTTON_START: button = BTN_START; break;
                case SDL_CONTROLLER_BUTTON_DPAD_UP: button = BTN_UP; break;
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN: button = BTN_DOWN; break;
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT: button = BTN_LEFT; break;
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: button = BTN_RIGHT; break;
                default: continue;
            }
            joypad_set_player(player, button, event->type == SDL_CONTROLLERBUTTONDOWN);
        }
    }
}

int main(int argc, char *argv[]) {
    SDL_AudioSpec want;
    SDL_AudioSpec have;
    SDL_AudioDeviceID audio_dev = 0;

    const char *rom_path = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--console") == 0) {
            if (++i == argc || !nes_set_console_model_name(argv[i])) {
                fprintf(stderr, "Console must be nes-001, nes-101, famicom, or av-famicom\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--adapter") == 0) {
            if (++i == argc || !joypad_set_adapter_name(argv[i])) {
                fprintf(stderr, "Adapter must be none, four-score, famicom-2, or famicom-4\n");
                return 1;
            }
        } else if (argv[i][0] == '-' || rom_path) {
            fprintf(stderr, "Unexpected argument: %s\n", argv[i]);
            return 1;
        } else {
            rom_path = argv[i];
        }
    }
    if (!rom_path) {
        printf("Usage: %s [--console MODEL] [--adapter TYPE] <rom-file>\n", argv[0]);
        return 1;
    }
    
    printf("Console: %s\n", nes_console_model_name());
    printf("Input adapter: %s\n", joypad_adapter_name());
    printf("Loading ROM: %s\n", rom_path);
    if(load_rom(rom_path) != 0) {
        fprintf(stderr, "Failed to load ROM\n");
        return 1;
    }
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    // Print ROM metadata at startup so mapper selection can be checked from the log.
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
    }
    
    printf("Mapper detected: %d\n", ((ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4)));


    printf("Resetting CPU...\n");
    cpu_power_on(&cpu);
    printf("CPU state after reset:\n");
    printf("  PC: 0x%04X\n", cpu.pc);
    printf("  SP: 0x%02X\n", cpu.sp);
    printf("  A: 0x%02X\n", cpu.a);
    printf("  X: 0x%02X\n", cpu.x);
    printf("  Y: 0x%02X\n", cpu.y);
    printf("  Status: 0x%02X\n", cpu.status);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }
    for (int device = 0; device < SDL_NumJoysticks(); ++device) open_controller(device);

    // Configure audio before starting the main loop.
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
        printf("=== Audio Info ===\n");
        printf("Requested: %d Hz, Got: %d Hz\n", want.freq, have.freq);
        printf("Requested: %d samples buffer, Got: %d samples\n", want.samples, have.samples);
        printf("Cycles per sample: %.6f\n", nes_timing()->cpu_hz / have.freq);
        printf("==================\n");
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

    palette_tool_init();
    const double performance_frequency = (double)SDL_GetPerformanceFrequency();
    double frame_deadline = (double)SDL_GetPerformanceCounter();
    
    while (running) {
        Uint32 frameStart = SDL_GetTicks();
        uint64_t frame_start_cycles = cpu_total_cycles;
    
        while (SDL_PollEvent(&e)) {
            controller_event(&e);
            if (e.type == SDL_QUIT)
                running = false;
            palette_tool_handle_event(&e, renderer);
            
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                int down = (e.type == SDL_KEYDOWN);
    
                switch (e.key.keysym.sym) {
                    case SDLK_F7:
                        if (down) { palette_tool_toggle_overlay(); }
                        break;
                    case SDLK_F6:
                        if (down) { ppu_palette_reset_default(); palette_tool_flash(true); }
                        break;
                    case SDLK_r:
                        if (down) {
                            if (audio_dev) SDL_LockAudioDevice(audio_dev);
                            ppu_soft_reset(&ppu);
                            apu_soft_reset(&apu);
                            cpu_soft_reset(&cpu);
                            if (audio_dev) SDL_UnlockAudioDevice(audio_dev);
                        }
                        break;
                    case SDLK_z:        joypad_set(&pad1, BTN_A,      down); break;
                    case SDLK_x:        joypad_set(&pad1, BTN_B,      down); break;
                    case SDLK_RSHIFT:   joypad_set(&pad1, BTN_SELECT, down); break;
                    case SDLK_RETURN:   joypad_set(&pad1, BTN_START,  down); break;
                    case SDLK_UP:       joypad_set(&pad1, BTN_UP,     down); break;
                    case SDLK_DOWN:     joypad_set(&pad1, BTN_DOWN,   down); break;
                    case SDLK_LEFT:     joypad_set(&pad1, BTN_LEFT,   down); break;
                    case SDLK_RIGHT:    joypad_set(&pad1, BTN_RIGHT,  down); break;
                    case SDLK_m:        joypad_set_microphone(down != 0); break;
                    default: break;
                }

                // Ctrl+V accepts the palette text formats handled by the parser.
                if (down && (e.key.keysym.sym == SDLK_v)) {
                    const SDL_Keymod mods = SDL_GetModState();
                    if ((mods & KMOD_CTRL) != 0) {
                        if (SDL_HasClipboardText()) {
                            char *txt = SDL_GetClipboardText();
                            if (txt) {
                                int rc = ppu_palette_load_hex_string(txt);
                                palette_tool_flash(rc == 0);
                                SDL_free(txt);
                                if (rc != 0) {
                                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_WARNING, "Palette Paste Error",
                                        "Clipboard text did not contain a valid palette.\n\nAccepts: 64 x RRGGBB tokens, or raw 192/1536 hex bytes.", NULL);
                                }
                            }
                        }
                    }
                }
            }

            // Dropped .pal files use the same validation as explicit palette loads.
            if (e.type == SDL_DROPFILE) {
                char *dropped_f = e.drop.file;
                if (dropped_f) {
                    int rc = ppu_palette_load_pal_file(dropped_f);
                    palette_tool_flash(rc == 0);
                    if (rc != 0) {
                        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Palette Load Error",
                            "Failed to load .pal file. Expected 192 or 1536 bytes.", NULL);
                    }
                    SDL_free(dropped_f);
                }
            }
        }
    
        // Run CPU steps until the PPU completes the current frame.
        start_frame();
        while (!ppu.frame_complete) {
            cpu_step(&cpu);
        }

        // Present the frame, then draw the palette UI on top.
        SDL_UpdateTexture(texture, NULL, framebuffer, SCREEN_WIDTH * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        int ww = 0, hh = 0; SDL_GetRendererOutputSize(renderer, &ww, &hh);
        if (palette_tool_is_visible()) { palette_tool_draw(renderer, ww, hh); }
        SDL_RenderPresent(renderer);
    
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        palette_tool_tick(frameTime);
        frame_deadline += (double)(cpu_total_cycles - frame_start_cycles)
                        * performance_frequency / nes_timing()->cpu_hz;
        double current_ticks = (double)SDL_GetPerformanceCounter();
        if (frame_deadline > current_ticks) {
            // Carry fractional milliseconds into the next deadline instead of
            // running every frame early after truncating SDL's delay argument.
            SDL_Delay((Uint32)((frame_deadline - current_ticks) * 1000.0 / performance_frequency));
        } else if (current_ticks - frame_deadline > performance_frequency * 0.25) {
            frame_deadline = current_ticks;
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    if (audio_dev) SDL_CloseAudioDevice(audio_dev);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        if (controllers[player]) SDL_GameControllerClose(controllers[player]);
    unload_rom();
    SDL_Quit();
    return 0;
}
