#ifndef GLOBALS_H
#define GLOBALS_H

// Screen dimensions
#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 240

// NES timing constants
#define CPU_FREQ 1789773.0              // CPU frequency in Hz
#define ACTUAL_FPS 60.0988              // Actual NTSC NES frame rate
#define CPU_CYCLES_PER_FRAME (CPU_FREQ / ACTUAL_FPS)  // ~29780.5 cycles/frame

// Framebuffer for rendering (RGBA format)
extern uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

#endif // GLOBALS_H
