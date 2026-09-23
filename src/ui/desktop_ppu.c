/* Visual PPU tools hosted by the desktop. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_internal.h"
#include "frontend_panels.h"
#include "frontend_commands.h"
#include "platform_frontend.h"
#include "output_guard.h"
#include "../debugger/ppu_inspector.h"
#include "../capture/capture_writer.h"
#include "../system/execution_policy.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct DesktopPpuViewer {
    DebugPpuImage image;
    SDL_Texture *texture;
    uint32_t pixels[512 * 480];
    unsigned width, height, selection, palette, source, mode, brush, page, rows;
    bool live, grid, valid, dirty, capture_requested, editing_address, dragging_scroll, step_requested;
    unsigned panel;
    uint32_t refreshed;
    unsigned undo_address, undo_count;
    uint8_t undo_before[16], undo_after[16];
    bool undo_oam;
    uint64_t undo_frame, undo_session, edit_session, step_cycles;
    char detail[256];
} DesktopPpuViewer;

static const char *const titles[] = {"Pattern tables", "Nametables",  "Sprites and OAM", "PPU debugger",
                                     "VRAM inspector", "Tile editor", "PPU palettes"};

enum {
    ACTION_PAUSE,
    ACTION_FRAME,
    ACTION_LIVE,
    ACTION_REFRESH,
    ACTION_GRID,
    ACTION_PREVIOUS,
    ACTION_NEXT,
    ACTION_SOURCE,
    ACTION_MODE,
    ACTION_EXPORT,
    ACTION_ADDRESS,
    ACTION_EDIT,
    ACTION_UNDO,
    ACTION_PALETTE = 20,
    ACTION_BRUSH = 32
};

static const Clay_Color ink = {232, 237, 247, 255}, muted = {160, 174, 195, 255}, accent = {179, 192, 255, 255};

bool desktop_ppu_panel(unsigned id) {
    return id >= DEBUG_PPU_PATTERNS && id <= DEBUG_PPU_PALETTE;
}

static bool empty_snapshot(void *data, FrontendPanelModel *model, char *error, size_t size) {
    (void)data;
    (void)error;
    (void)size;
    model->status = "Visual PPU inspection";
    return true;
}

bool desktop_ppu_register(void) {
    for (unsigned i = 0; i < 7; ++i) {
        FrontendPanelSpec spec = {DEBUG_PPU_PATTERNS + i, titles[i], "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                                  empty_snapshot,         NULL,      NULL};
        if (!frontend_panel_register(&spec)) {
            while (i) {
                frontend_panel_unregister(DEBUG_PPU_PATTERNS + --i);
            }
            return false;
        }
    }
    return true;
}

void desktop_ppu_unregister(void) {
    for (unsigned i = DEBUG_PPU_PATTERNS; i <= DEBUG_PPU_PALETTE; ++i) {
        frontend_panel_unregister(i);
    }
}

void desktop_ppu_destroy(FrontendDesktopUi *ui) {
    if (!ui->ppu_viewer) {
        return;
    }
    SDL_DestroyTexture(ui->ppu_viewer->texture);
    free(ui->ppu_viewer);
    ui->ppu_viewer = NULL;
}

static DesktopPpuViewer *viewer(FrontendDesktopUi *ui) {
    if (!ui->ppu_viewer) {
        ui->ppu_viewer = calloc(1, sizeof(*ui->ppu_viewer));
        if (ui->ppu_viewer) {
            ui->ppu_viewer->live = !ui->settings || ui->settings->ppu_viewer_live;
            ui->ppu_viewer->grid = !ui->settings || ui->settings->ppu_viewer_grid;
            ui->ppu_viewer->brush = 1;
            ui->ppu_viewer->dirty = true;
        }
    }
    return ui->ppu_viewer;
}

static void text(FrontendDesktopUi *ui, const char *value, int size, Clay_Color color) {
    CLAY_TEXT(desktop_clay_string(ui->clay, value), CLAY_TEXT_CONFIG({.fontSize = (uint16_t)size, .textColor = color}));
}

static void button(FrontendDesktopUi *ui, const char *name, unsigned action, bool selected, bool enabled) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_PPU_ACTION : HIT_NONE, (int)action, 0);
    CLAY(id,
         {.layout = {.sizing = {.width = CLAY_SIZING_FIT(), .height = CLAY_SIZING_FIXED(28)}, .padding = {8, 8, 5, 5}},
          .backgroundColor = selected ? (Clay_Color){59, 70, 101, 255} : (Clay_Color){39, 47, 64, 255},
          .border = {.color = {88, 105, 134, 255}, .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}}}) {
        text(ui, name, 12, enabled ? selected ? accent : ink : muted);
    }
}

static bool active(void) {
    return frontend_panel_session_active();
}

static const uint8_t *pattern_bytes(const DesktopPpuViewer *v) {
    return v->source == 1 ? v->image.background : v->source == 2 ? v->image.sprites : v->image.memory;
}

static uint8_t peek(bool oam, unsigned address) {
    if (oam) {
        uint8_t bytes[256];
        debugger_copy_oam(bytes);
        return bytes[address & 255];
    }
    return debugger_peek_ppu((uint16_t)address);
}

static bool editable(FrontendDesktopUi *ui) {
    if (!active() || ui->ppu_viewer->image.session != debugger_session_revision() || !ui->ppu_viewer->live ||
        nes_execution_policy() != NES_EXECUTION_LIVE || !ui->execution || !frontend_execution_paused(ui->execution)) {
        desktop_copy_status(
            ui, "Pause the game and enable Live to edit. Movie, rewind, and netplay sessions are read-only.");
        return false;
    }
    debugger_pause();
    frontend_execution_sync_debugger(ui->execution);
    return true;
}

static void remember(DesktopPpuViewer *v, bool oam, unsigned address, unsigned count) {
    v->undo_address = address;
    v->undo_count = count;
    v->undo_oam = oam;
    DebugPpuSnapshot now;
    debugger_get_ppu(&now);
    v->undo_frame = now.frame;
    v->undo_session = debugger_session_revision();
    for (unsigned i = 0; i < count; ++i) {
        v->undo_before[i] = peek(oam, address + i);
    }
}

static void changed(FrontendDesktopUi *ui) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    for (unsigned i = 0; i < v->undo_count; ++i) {
        v->undo_after[i] = peek(v->undo_oam, v->undo_address + i);
    }
    frontend_execution_clear_timeline(ui->execution);
    v->dirty = v->capture_requested = true;
    desktop_copy_status(ui, "Memory updated. The game may overwrite RAM when it resumes.");
}

static void refresh(FrontendDesktopUi *ui) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    if (!active()) {
        v->valid = false;
        v->undo_count = 0;
        return;
    }
    uint32_t now = SDL_GetTicks();
    if (v->image.session != debugger_session_revision() || v->panel != ui->panel_id) {
        v->dirty = v->capture_requested = true;
        v->undo_count = 0;
        v->selection = v->page = 0;
        v->panel = ui->panel_id;
        v->step_requested = false;
    }
    if (v->step_requested) {
        DebugPpuSnapshot state;
        debugger_get_ppu(&state);
        if (state.ppu_cycles != v->step_cycles) {
            v->capture_requested = true;
            v->step_requested = false;
        }
    }
    bool capture = !v->valid || v->capture_requested || (v->live && now - v->refreshed >= 33);
    if (!capture && !v->dirty) {
        return;
    }
    if (capture) {
        debug_ppu_capture(&v->image, ui->panel_id == DEBUG_PPU_NAMETABLES);
        v->refreshed = now;
    }
    v->valid = true;
    v->dirty = v->capture_requested = false;
    if (v->undo_frame != v->image.state.frame) {
        v->undo_count = 0;
    }
    unsigned old_width = v->width, old_height = v->height;
    v->width = 256;
    v->height = 128;
    if (ui->panel_id == DEBUG_PPU_PATTERNS) {
        debug_ppu_patterns(&v->image, pattern_bytes(v), v->palette, v->pixels);
    } else if (ui->panel_id == DEBUG_PPU_NAMETABLES) {
        v->width = 512;
        v->height = 480;
        debug_ppu_nametables(&v->image, v->mode != 0, v->pixels);
    } else if (ui->panel_id == DEBUG_PPU_SPRITES) {
        v->height = v->mode ? 240 : 128;
        debug_ppu_sprites(&v->image, v->mode != 0, v->pixels);
    } else if (ui->panel_id == DEBUG_PPU_TILE) {
        v->width = v->height = 8;
        unsigned address = (v->selection & 511) * 16;
        for (unsigned y = 0; y < 8; ++y) {
            for (unsigned x = 0; x < 8; ++x) {
                v->pixels[y * 8 + x] =
                    debug_ppu_color(&v->image, v->palette, debug_ppu_pixel(pattern_bytes(v) + address, x, y), false);
            }
        }
    } else if (ui->panel_id == DEBUG_PPU_PALETTE) {
        v->width = 16;
        v->height = 2;
        for (unsigned i = 0; i < 32; ++i) {
            v->pixels[i] = v->image.colors[v->image.memory[0x3F00 + i] & 63];
        }
    } else {
        return;
    }
    if (old_width != v->width || old_height != v->height) {
        SDL_DestroyTexture(v->texture);
        v->texture = NULL;
    }
    if (!v->texture) {
        v->texture = SDL_CreateTexture(ui->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                       (int)v->width, (int)v->height);
        if (v->texture) {
            SDL_SetTextureScaleMode(v->texture, SDL_ScaleModeNearest);
        }
    }
    if (v->texture) {
        SDL_UpdateTexture(v->texture, NULL, v->pixels, (int)v->width * 4);
    }
}

static void describe(FrontendDesktopUi *ui) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    if (ui->panel_id == DEBUG_PPU_PATTERNS || ui->panel_id == DEBUG_PPU_TILE) {
        snprintf(v->detail, sizeof(v->detail), "Tile $%02X | Table $%04X | Address $%04X | Palette %s%u",
                 v->selection & 255, v->selection >= 256 ? 0x1000 : 0, v->selection * 16, v->palette < 4 ? "BG" : "SP",
                 v->palette & 3);
    } else if (ui->panel_id == DEBUG_PPU_SPRITES) {
        DebugPpuSelection s = debug_ppu_sprite(&v->image, v->selection);
        snprintf(v->detail, sizeof(v->detail),
                 "OAM %u | X %u Y %u (raw %u) | %ux%u | Tile $%02X at $%04X\nPalette SP%u | Flip X: %s  Y: %s | %s "
                 "background",
                 v->selection, s.x, s.y, s.y - 1, 8u, s.height, s.tile, s.pattern, s.palette - 4,
                 s.flip_x ? "yes" : "no", s.flip_y ? "yes" : "no", s.behind ? "Behind" : "In front of");
    } else if (ui->panel_id == DEBUG_PPU_NAMETABLES) {
        unsigned x = (v->selection % 64) * 8, y = (v->selection / 64) * 8;
        DebugPpuSelection s = debug_ppu_nametable(&v->image, x, y);
        snprintf(v->detail, sizeof(v->detail),
                 "Nametable $%04X | Attribute $%04X | Tile $%02X at $%04X | Palette BG%u\nCurrent mapped banks; "
                 "raster-time bank changes are not a completed-frame reconstruction.",
                 s.nametable, s.attribute, s.tile, s.pattern, s.palette);
    } else if (ui->panel_id == DEBUG_PPU_VRAM) {
        bool oam = v->mode != 0;
        unsigned address = v->selection & (oam ? 255 : 0x3FFF);
        snprintf(v->detail, sizeof(v->detail),
                 "%s $%04X = $%02X | Click a byte; Enter edits; arrows move; wheel scrolls.", oam ? "OAM" : "PPU",
                 address, oam ? v->image.oam[address] : v->image.memory[address]);
    } else if (ui->panel_id == DEBUG_PPU_PALETTE) {
        unsigned a = 0x3F00 + (v->selection & 31);
        snprintf(v->detail, sizeof(v->detail), "$%04X = $%02X | $3F10/$14/$18/$1C mirror $3F00/$04/$08/$0C.", a,
                 v->image.memory[a]);
    }
}

static void registers(FrontendDesktopUi *ui) {
    DebugPpuImage *i = &ui->ppu_viewer->image;
    DebugPpuSnapshot *s = &i->state;
    char lines[8][160];
    snprintf(lines[0], 160, "PPUCTRL $2000: $%02X    NMI: %s    Increment: %u", s->ctrl,
             s->ctrl & 0x80 ? "enabled" : "disabled", s->ctrl & 4 ? 32 : 1);
    snprintf(lines[1], 160, "PPUMASK $2001: $%02X    Background: %s    Sprites: %s", s->mask,
             s->mask & 8 ? "on" : "off", s->mask & 16 ? "on" : "off");
    snprintf(lines[2], 160, "PPUSTATUS $2002: $%02X    VBlank: %u  Sprite 0: %u  Overflow: %u", s->status,
             !!(s->status & 0x80), !!(s->status & 0x40), !!(s->status & 0x20));
    snprintf(lines[3], 160, "VRAM v: $%04X    Temporary t: $%04X    Fine X: %u    Write latch: %u", s->v, s->t,
             s->fine_x, s->write_toggle);
    snprintf(lines[4], 160, "OAM address: $%02X    Background bank: $%04X    Sprite bank: $%04X", s->oam_addr,
             s->ctrl & 0x10 ? 0x1000 : 0, s->ctrl & 8 ? 0x1000 : 0);
    snprintf(lines[5], 160, "Sprite size: 8x%u    Base nametable: $%04X", s->ctrl & 0x20 ? 16 : 8,
             0x2000 + (s->ctrl & 3) * 0x400);
    snprintf(lines[6], 160, "Scanline: %d    Dot: %d    Frame: %llu", s->scanline, s->dot,
             (unsigned long long)s->frame);
    snprintf(lines[7], 160, "PPU cycles: %llu    Mirroring mode: %u", (unsigned long long)s->ppu_cycles,
             (unsigned)i->mirroring);
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 7}}) {
        for (unsigned n = 0; n < 8; ++n) {
            text(ui, lines[n], 13, n & 1 ? muted : ink);
        }
    }
}

static void memory_grid(FrontendDesktopUi *ui, float width, float height) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    v->rows = (unsigned)fmaxf(2, fminf(16, (height - 240) / 22));
    unsigned limit = v->mode ? 256 : 0x4000;
    if (v->selection < v->page || v->selection >= v->page + v->rows * 16) {
        v->page = (v->selection / 16) * 16;
    }
    if (v->page + v->rows * 16 > limit) {
        v->page = limit - v->rows * 16;
    }
    float cell = (width - 84) / 16;
    CLAY_AUTO_ID({.layout = {.childGap = 6}}) {
        CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 1}}) {
            for (unsigned row = 0; row < v->rows; ++row) {
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(22)},
                                         .childGap = 0}}) {
                    char value[16];
                    unsigned address = v->page + row * 16;
                    snprintf(value, sizeof(value), "%04X", address);
                    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(64)}}}) {
                        text(ui, value, 12, muted);
                    }
                    for (unsigned col = 0; col < 16; ++col) {
                        unsigned a = address + col;
                        Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_PPU_BYTE, (int)a, 0);
                        CLAY(id,
                             {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(cell), .height = CLAY_SIZING_FIXED(21)},
                                         .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
                              .backgroundColor =
                                  a == v->selection ? (Clay_Color){60, 75, 110, 255} : (Clay_Color){33, 40, 55, 255}}) {
                            snprintf(value, sizeof(value), "%02X", v->mode ? v->image.oam[a] : v->image.memory[a]);
                            text(ui, value, 12, ink);
                        }
                    }
                }
            }
        }
        Clay_ElementId scroll = desktop_clay_hit(ui->clay, HIT_PPU_SCROLL, 0, 0);
        float track = v->rows * 23.0f, thumb = fmaxf(18, track * v->rows * 16 / limit);
        float offset = limit > v->rows * 16 ? (track - thumb) * v->page / (limit - v->rows * 16) : 0;
        CLAY(scroll, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(14), .height = CLAY_SIZING_FIXED(track)}},
                      .backgroundColor = {25, 32, 46, 255}}) {
            CLAY_AUTO_ID({.floating = {.attachTo = CLAY_ATTACH_TO_PARENT, .zIndex = 7, .offset = {2, offset}},
                          .layout = {.sizing = {.width = CLAY_SIZING_FIXED(10), .height = CLAY_SIZING_FIXED(thumb)}},
                          .backgroundColor = {104, 124, 164, 255}}) {
            }
        }
    }
}

static void canvas_overlay(FrontendDesktopUi *ui, float scale) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    unsigned cell_x = 8, cell_y = 8;
    if (ui->panel_id == DEBUG_PPU_TILE || ui->panel_id == DEBUG_PPU_PALETTE) {
        cell_x = cell_y = 1;
    } else if (ui->panel_id == DEBUG_PPU_SPRITES) {
        cell_x = 16;
        cell_y = 32;
    }
    if (v->grid && !(ui->panel_id == DEBUG_PPU_SPRITES && v->mode)) {
        for (unsigned x = cell_x; x < v->width; x += cell_x) {
            CLAY_AUTO_ID(
                {.floating = {.attachTo = CLAY_ATTACH_TO_PARENT, .zIndex = 7, .offset = {x * scale, 0}},
                 .layout = {.sizing = {.width = CLAY_SIZING_FIXED(1), .height = CLAY_SIZING_FIXED(v->height * scale)}},
                 .backgroundColor = {130, 150, 180, 95}}) {
            }
        }
        for (unsigned y = cell_y; y < v->height; y += cell_y) {
            CLAY_AUTO_ID(
                {.floating = {.attachTo = CLAY_ATTACH_TO_PARENT, .zIndex = 7, .offset = {0, y * scale}},
                 .layout = {.sizing = {.width = CLAY_SIZING_FIXED(v->width * scale), .height = CLAY_SIZING_FIXED(1)}},
                 .backgroundColor = {130, 150, 180, 95}}) {
            }
        }
    }
    unsigned x = 0, y = 0, w = 8, h = 8;
    if (ui->panel_id == DEBUG_PPU_PATTERNS) {
        x = (v->selection / 256) * 128 + (v->selection % 16) * 8;
        y = (v->selection % 256) / 16 * 8;
    } else if (ui->panel_id == DEBUG_PPU_NAMETABLES) {
        x = (v->selection % 64) * 8;
        y = v->selection / 64 * 8;
    } else if (ui->panel_id == DEBUG_PPU_PALETTE) {
        x = v->selection % 16;
        y = v->selection / 16;
        w = h = 1;
    } else if (ui->panel_id == DEBUG_PPU_SPRITES) {
        DebugPpuSelection s = debug_ppu_sprite(&v->image, v->selection);
        x = v->mode ? s.x : (v->selection % 16) * 16 + 4;
        y = v->mode ? s.y : (v->selection / 16) * 32 + 8;
        h = s.height;
    }
    if (x < v->width && y < v->height) {
        if (x + w > v->width) {
            w = v->width - x;
        }
        if (y + h > v->height) {
            h = v->height - y;
        }
        CLAY_AUTO_ID(
            {.floating = {.attachTo = CLAY_ATTACH_TO_PARENT, .zIndex = 7, .offset = {x * scale, y * scale}},
             .layout = {.sizing = {.width = CLAY_SIZING_FIXED(w * scale), .height = CLAY_SIZING_FIXED(h * scale)}},
             .border = {.color = {255, 213, 99, 255}, .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}}}) {
        }
    }
}

void desktop_ppu_layout(FrontendDesktopUi *ui, float width, float height) {
    DesktopPpuViewer *v = viewer(ui);
    if (!v) {
        text(ui, "Could not allocate the PPU viewer.", 16, ink);
        return;
    }
    refresh(ui);
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        text(ui, titles[ui->panel_id - DEBUG_PPU_PATTERNS], 22, ink);
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
        }
        if (!ui->parent) {
            button(ui, "Close", 99, false, true);
        }
    }
    if (!v->valid) {
        text(ui, "Open a game to inspect its PPU memory.", 15, muted);
        return;
    }
    bool paused = ui->execution && frontend_execution_paused(ui->execution);
    CLAY_AUTO_ID({.layout = {.childGap = 6}}) {
        button(ui, paused ? "Resume" : "Pause", ACTION_PAUSE, paused, ui->execution != NULL);
        button(ui, "Frame", ACTION_FRAME, false, ui->execution != NULL);
        button(ui, v->live ? "Live" : "Frozen", ACTION_LIVE, v->live, true);
        button(ui, "Refresh", ACTION_REFRESH, false, true);
        if (ui->panel_id != DEBUG_PPU_VRAM && ui->panel_id != DEBUG_PPU_REGISTERS) {
            button(ui, "Grid", ACTION_GRID, v->grid, true);
            button(ui, "Export PNG", ACTION_EXPORT, false, true);
        }
    }
    if (ui->panel_id != DEBUG_PPU_REGISTERS) {
        CLAY_AUTO_ID({.layout = {.childGap = 5}}) {
            if (ui->panel_id == DEBUG_PPU_PATTERNS || ui->panel_id == DEBUG_PPU_TILE) {
                for (unsigned n = 0; n < 8; ++n) {
                    char label[8];
                    snprintf(label, sizeof(label), "%s%u", n < 4 ? "BG" : "SP", n & 3);
                    button(ui, label, ACTION_PALETTE + n, n == v->palette, true);
                }
            } else if (ui->panel_id == DEBUG_PPU_NAMETABLES) {
                button(ui, v->mode ? "Attributes" : "Tiles", ACTION_MODE, false, true);
            } else if (ui->panel_id == DEBUG_PPU_SPRITES) {
                button(ui, v->mode ? "Screen positions" : "OAM grid", ACTION_MODE, false, true);
            } else if (ui->panel_id == DEBUG_PPU_VRAM) {
                button(ui, v->mode ? "OAM memory" : "PPU memory", ACTION_MODE, false, true);
            }
            if (ui->panel_id == DEBUG_PPU_VRAM || ui->panel_id == DEBUG_PPU_TILE) {
                button(ui, "Address", ACTION_ADDRESS, false, true);
            }
            if (ui->panel_id == DEBUG_PPU_VRAM || ui->panel_id == DEBUG_PPU_PALETTE) {
                button(ui, "Edit byte", ACTION_EDIT, false, paused && v->live);
            }
        }
        CLAY_AUTO_ID({.layout = {.childGap = 6}}) {
            button(ui, "Previous", ACTION_PREVIOUS, false, ui->panel_id != DEBUG_PPU_REGISTERS);
            button(ui, "Next", ACTION_NEXT, false, ui->panel_id != DEBUG_PPU_REGISTERS);
            if (ui->panel_id == DEBUG_PPU_PATTERNS || ui->panel_id == DEBUG_PPU_TILE) {
                button(ui,
                       v->source == 0   ? "CPU banks"
                       : v->source == 1 ? "BG banks"
                                        : "Sprite banks",
                       ACTION_SOURCE, false, true);
            }
            if (ui->panel_id == DEBUG_PPU_TILE) {
                for (unsigned n = 0; n < 4; ++n) {
                    char name[8];
                    snprintf(name, 8, "%u", n);
                    button(ui, name, ACTION_BRUSH + n, v->brush == n, true);
                }
            }
            if (ui->panel_id == DEBUG_PPU_TILE || ui->panel_id == DEBUG_PPU_VRAM || ui->panel_id == DEBUG_PPU_PALETTE) {
                button(ui, "Undo", ACTION_UNDO, false, paused && v->undo_count != 0);
            }
        }
    }
    if (ui->panel_id == DEBUG_PPU_REGISTERS) {
        registers(ui);
    } else if (ui->panel_id == DEBUG_PPU_VRAM) {
        memory_grid(ui, width, height);
    } else {
        float scale = fminf(width / v->width, fmaxf(40, height - 238) / v->height);
        if (ui->panel_id == DEBUG_PPU_TILE) {
            scale = fminf(scale, 32);
        }
        if (scale >= 1 && ui->panel_id != DEBUG_PPU_PALETTE) {
            scale = floorf(scale);
        }
        Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_PPU_CANVAS, 0, 0);
        CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(v->width * scale),
                                        .height = CLAY_SIZING_FIXED(v->height * scale)}},
                  .image = {.imageData = v->texture},
                  .clip = {.horizontal = true, .vertical = true},
                  .border = {.color = {95, 113, 150, 255}, .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}}}) {
            canvas_overlay(ui, scale);
        }
    }
    describe(ui);
    if (ui->panel_id != DEBUG_PPU_REGISTERS) {
        text(ui, v->detail, 12, muted);
    }
    char footer[128];
    snprintf(footer, sizeof(footer), "%s snapshot | Frame %llu | Scanline %d, dot %d", v->live ? "Live" : "Frozen",
             (unsigned long long)v->image.state.frame, v->image.state.scanline, v->image.state.dot);
    text(ui, footer, 11, muted);
}

static void move(FrontendDesktopUi *ui, int direction) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    unsigned count = ui->panel_id == DEBUG_PPU_NAMETABLES ? 3840
                     : ui->panel_id == DEBUG_PPU_SPRITES  ? 64
                     : ui->panel_id == DEBUG_PPU_PALETTE  ? 32
                     : ui->panel_id == DEBUG_PPU_VRAM     ? (v->mode ? 256 : 0x4000)
                                                          : 512;
    v->selection = (unsigned)(((int)v->selection + (int)count + direction) % (int)count);
    v->dirty = true;
}

static void edit_value(FrontendDesktopUi *ui, bool address) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    if (!address && !editable(ui)) {
        return;
    }
    v->editing_address = address;
    v->edit_session = debugger_session_revision();
    unsigned a = ui->panel_id == DEBUG_PPU_PALETTE ? 0x3F00 + v->selection
                 : ui->panel_id == DEBUG_PPU_TILE  ? v->selection * 16
                                                   : v->selection;
    char value[12];
    snprintf(value, sizeof(value), address ? "%04X" : "%02X",
             address ? a : peek(v->mode != 0 && ui->panel_id == DEBUG_PPU_VRAM, a));
    desktop_start_text_edit(ui, 1, value);
}

bool desktop_ppu_commit(FrontendDesktopUi *ui, const char *value, char *error, size_t size) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    if (!v || !active() || v->edit_session != debugger_session_revision()) {
        snprintf(error, size, "The game changed. Close this edit and select the address again.");
        return false;
    }
    if (*value == '$') {
        ++value;
    }
    errno = 0;
    char *end;
    unsigned long number = strtoul(value, &end, 16);
    unsigned limit = v->editing_address ? ui->panel_id == DEBUG_PPU_TILE ? 0x1FFF : v->mode ? 255 : 0x3FFF : 255;
    if (errno || end == value || *end || number > limit || *value == '-') {
        snprintf(error, size, "Enter a hexadecimal value from 0 to %X.", limit);
        return false;
    }
    if (v->editing_address) {
        v->selection = ui->panel_id == DEBUG_PPU_TILE ? (unsigned)number / 16 : (unsigned)number;
    } else {
        if (!editable(ui)) {
            snprintf(error, size, "%s", ui->status);
            return false;
        }
        bool oam = ui->panel_id == DEBUG_PPU_VRAM && v->mode;
        unsigned address = ui->panel_id == DEBUG_PPU_PALETTE ? 0x3F00 + v->selection : v->selection;
        remember(v, oam, address, 1);
        if (!debug_ppu_write(oam, address, (uint8_t)number)) {
            v->undo_count = 0;
            snprintf(error, size, "This address is ROM, unmapped, or write-protected.");
            return false;
        }
        changed(ui);
    }
    v->dirty = true;
    return true;
}

static void action(FrontendDesktopUi *ui, unsigned id) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    if (id == 99) {
        ui->panel_open = false;
        return;
    }
    if (!active()) {
        return;
    }
    if (id == ACTION_PAUSE) {
        if (frontend_execution_paused(ui->execution)) {
            desktop_invoke_command(ui, FRONTEND_COMMAND_PAUSE);
        } else {
            debugger_pause();
            frontend_execution_sync_debugger(ui->execution);
        }
    } else if (id == ACTION_FRAME) {
        if (!frontend_execution_paused(ui->execution)) {
            debugger_pause();
            frontend_execution_sync_debugger(ui->execution);
        }
        DebugPpuSnapshot state;
        debugger_get_ppu(&state);
        v->step_cycles = state.ppu_cycles;
        v->step_requested = desktop_invoke_command(ui, FRONTEND_COMMAND_FRAME_ADVANCE);
    } else if (id == ACTION_LIVE) {
        v->live = !v->live;
    } else if (id == ACTION_GRID) {
        v->grid = !v->grid;
    } else if (id == ACTION_PREVIOUS || id == ACTION_NEXT) {
        move(ui, id == ACTION_PREVIOUS ? -1 : 1);
    } else if (id == ACTION_SOURCE) {
        v->source = (v->source + 1) % 3;
    } else if (id == ACTION_MODE) {
        v->mode ^= 1;
        v->selection = v->page = 0;
    } else if (id >= ACTION_PALETTE && id < ACTION_PALETTE + 8) {
        v->palette = id - ACTION_PALETTE;
    } else if (id >= ACTION_BRUSH && id < ACTION_BRUSH + 4) {
        v->brush = id - ACTION_BRUSH;
    } else if (id == ACTION_ADDRESS || id == ACTION_EDIT) {
        edit_value(ui, id == ACTION_ADDRESS);
    } else if (id == ACTION_UNDO && v->undo_count && editable(ui)) {
        bool match = v->undo_session == debugger_session_revision();
        DebugPpuSnapshot now;
        debugger_get_ppu(&now);
        match &= now.frame == v->undo_frame;
        for (unsigned i = 0; i < v->undo_count; ++i) {
            match &= peek(v->undo_oam, v->undo_address + i) == v->undo_after[i];
        }
        if (match) {
            for (unsigned i = 0; i < v->undo_count; ++i) {
                (void)debug_ppu_write(v->undo_oam, v->undo_address + i, v->undo_before[i]);
            }
            frontend_execution_clear_timeline(ui->execution);
        }
        desktop_copy_status(ui, match ? "Edit undone." : "Memory changed since the edit; undo was discarded.");
        v->undo_count = 0;
    } else if (id == ACTION_EXPORT) {
        char path[FRONTEND_SETTINGS_PATH_TEXT] = {0}, error[256] = {0};
        if (frontend_save_file_dialog(FRONTEND_SAVE_PNG, path, sizeof(path), error, sizeof(error)) &&
            frontend_output_path_allowed(path, ui->execution, NULL, 0, error, sizeof(error))) {
            NesCaptureFrame frame = {v->pixels, v->width, v->height, v->width};
            if (nes_capture_png(path, &frame) != NES_FILE_OK) {
                snprintf(error, sizeof(error), "Could not save the viewer image.");
            } else {
                desktop_copy_status(ui, "Viewer image saved.");
            }
        }
        if (error[0]) {
            desktop_copy_status(ui, error);
        }
    }
    if (id == ACTION_REFRESH || id == ACTION_PAUSE || id == ACTION_FRAME || id == ACTION_LIVE || id == ACTION_UNDO) {
        v->capture_requested = true;
    }
    v->dirty = true;
}

static void canvas_click(FrontendDesktopUi *ui, int x, int y, bool double_click) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    SDL_FRect bounds;
    if (!desktop_clay_bounds(ui->clay, HIT_PPU_CANVAS, 0, 0, &bounds) || bounds.w <= 0 || bounds.h <= 0) {
        return;
    }
    unsigned px = (unsigned)((x - bounds.x) * v->width / bounds.w),
             py = (unsigned)((y - bounds.y) * v->height / bounds.h);
    if (px >= v->width || py >= v->height) {
        return;
    }
    if (ui->panel_id == DEBUG_PPU_TILE) {
        if (!editable(ui)) {
            return;
        }
        if (v->source != 0) {
            desktop_copy_status(ui, "Select CPU banks to edit the mapped CHR memory.");
            return;
        }
        remember(v, false, v->selection * 16, 16);
        if (debug_ppu_paint(v->selection * 16, px, py, v->brush)) {
            changed(ui);
        } else {
            v->undo_count = 0;
            desktop_copy_status(ui, "This tile is ROM or write-protected. Only writable CHR memory can be painted.");
        }
    } else if (ui->panel_id == DEBUG_PPU_PATTERNS) {
        v->selection = (px / 128) * 256 + (py / 8) * 16 + (px % 128) / 8;
        if (double_click) {
            FrontendDesktopUi *tool = desktop_open_window(ui, 1, DEBUG_PPU_TILE);
            if (tool && viewer(tool)) {
                tool->ppu_viewer->selection = v->selection;
                tool->ppu_viewer->palette = v->palette;
                tool->ppu_viewer->source = v->source;
                tool->ppu_viewer->dirty = tool->ppu_viewer->capture_requested = true;
            }
        }
    } else if (ui->panel_id == DEBUG_PPU_NAMETABLES) {
        v->selection = (py / 8) * 64 + px / 8;
    } else if (ui->panel_id == DEBUG_PPU_PALETTE) {
        v->selection = py * 16 + px;
        if (double_click) {
            edit_value(ui, false);
        }
    } else if (ui->panel_id == DEBUG_PPU_SPRITES) {
        if (!v->mode) {
            v->selection = (py / 32) * 16 + px / 16;
        } else {
            for (unsigned n = 0; n < 64; ++n) {
                DebugPpuSelection s = debug_ppu_sprite(&v->image, n);
                if (px >= s.x && px < s.x + 8 && py >= s.y && py < s.y + s.height) {
                    v->selection = n;
                    break;
                }
            }
        }
    }
    v->dirty = true;
}

static void scroll_to(FrontendDesktopUi *ui, int y) {
    DesktopPpuViewer *v = ui->ppu_viewer;
    SDL_FRect rect;
    if (!desktop_clay_bounds(ui->clay, HIT_PPU_SCROLL, 0, 0, &rect) || rect.h <= 0) {
        return;
    }
    float position = fmaxf(0, fminf(1, (y - rect.y) / rect.h));
    unsigned limit = v->mode ? 256 : 0x4000;
    v->page = (unsigned)(position * (limit / 16 - v->rows)) * 16;
    v->selection = v->page;
    v->dirty = true;
}

bool desktop_ppu_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    DesktopPpuViewer *v = viewer(ui);
    if (!v) {
        return false;
    }
    if (event->type == SDL_MOUSEBUTTONUP) {
        v->dragging_scroll = false;
        return true;
    }
    if (event->type == SDL_MOUSEMOTION && v->dragging_scroll) {
        scroll_to(ui, (int)(event->motion.y / ui->ui_scale));
        return true;
    }
    if (event->type == SDL_MOUSEBUTTONDOWN && event->button.button == SDL_BUTTON_LEFT) {
        const DesktopHit *hit = desktop_clay_at(ui->clay, (float)event->button.x, (float)event->button.y);
        if (!hit) {
            return true;
        }
        if (active() && hit->kind == HIT_PPU_SCROLL) {
            v->dragging_scroll = true;
            scroll_to(ui, event->button.y);
        } else if (hit->kind == HIT_PPU_ACTION) {
            action(ui, (unsigned)hit->index);
        } else if (active() && hit->kind == HIT_PPU_CANVAS) {
            canvas_click(ui, event->button.x, event->button.y, event->button.clicks >= 2);
        } else if (active() && hit->kind == HIT_PPU_BYTE) {
            v->selection = (unsigned)hit->index;
            if (event->button.clicks >= 2) {
                edit_value(ui, false);
            }
        }
        return true;
    }
    if (event->type == SDL_MOUSEWHEEL) {
        if (active()) {
            move(ui, event->wheel.y > 0 ? -16 : 16);
        }
        return true;
    }
    if (event->type != SDL_KEYDOWN || event->key.repeat) {
        return false;
    }
    SDL_Scancode key = event->key.keysym.scancode;
    if (key == SDL_SCANCODE_ESCAPE) {
        ui->panel_open = false;
        return true;
    }
    if (!active()) {
        return true;
    }
    if (key == SDL_SCANCODE_LEFT || key == SDL_SCANCODE_RIGHT) {
        move(ui, key == SDL_SCANCODE_LEFT ? -1 : 1);
    } else if (key == SDL_SCANCODE_UP || key == SDL_SCANCODE_DOWN) {
        move(ui, (key == SDL_SCANCODE_UP ? -1 : 1) * (ui->panel_id == DEBUG_PPU_NAMETABLES ? 64 : 16));
    } else if (key == SDL_SCANCODE_RETURN && (ui->panel_id == DEBUG_PPU_VRAM || ui->panel_id == DEBUG_PPU_PALETTE)) {
        edit_value(ui, false);

    } else if (key == SDL_SCANCODE_Z && (event->key.keysym.mod & KMOD_CTRL)) {
        action(ui, ACTION_UNDO);
    }
    return true;
}
