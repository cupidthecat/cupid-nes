/*
 * gtk_ppu.c - Native PPU inspection and palette editing
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifdef CUPID_GTK
#ifndef GDK_VERSION_MIN_REQUIRED
#define GDK_VERSION_MIN_REQUIRED GDK_VERSION_4_8
#endif
#ifndef GDK_VERSION_MAX_ALLOWED
#define GDK_VERSION_MAX_ALLOWED GDK_VERSION_4_8
#endif
#include "gtk_internal.h"
#include "gtk_layout.h"
#include "gtk_desktop.h"
#include "frontend_commands.h"
#include "output_guard.h"
#include "palette_tool.h"
#include "../debugger/ppu_inspector.h"
#include "../capture/capture_writer.h"
#include "../system/execution_policy.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    CupidGtkTool *tool;
    GtkWidget *root, *canvas, *detail, *footer, *live, *grid, *selection;
    GtkWidget *value, *apply, *undo, *color, *memory[256], *row_labels[16];
    GtkWidget *source, *palette, *mode, *brush, *pause;
    GtkNativeDialog *dialog;
    DebugPpuImage image;
    uint32_t pixels[512 * 480], colors_before[64], colors_after[64];
    uint32_t emphasis_before[8][64], emphasis_after[8][64];
    bool emphasis_was, emphasis_now, color_undo, color_initialized;
    unsigned color_selected;
    unsigned panel, selected, width, height, page, undo_address, undo_count;
    uint8_t before[16], after[16];
    uint64_t undo_frame, undo_session, step_cycles;
    bool undo_oam, valid, dirty, capture, updating, runtime_palette, stepping;
    Uint32 refreshed;
} Ppu;

static void refresh(Ppu *v);

static unsigned choice(GtkWidget *widget) {
    return widget ? gtk_drop_down_get_selected(GTK_DROP_DOWN(widget)) : 0;
}

static bool live(Ppu *v) {
    return gtk_check_button_get_active(GTK_CHECK_BUTTON(v->live));
}

static bool oam(Ppu *v) {
    return v->panel == DEBUG_PPU_VRAM && choice(v->mode);
}

static void status(Ppu *v, const char *text) {
    cupid_gtk_tool_status(v->tool, text);
}

static unsigned count(Ppu *v) {
    if (v->runtime_palette) {
        return 64;
    }
    switch (v->panel) {
    case DEBUG_PPU_NAMETABLES:
        return 3840;
    case DEBUG_PPU_SPRITES:
        return 64;
    case DEBUG_PPU_PALETTE:
        return 32;
    case DEBUG_PPU_VRAM:
        return oam(v) ? 256 : 0x4000;
    default:
        return 512;
    }
}

static uint8_t peek(bool is_oam, unsigned address) {
    if (is_oam) {
        uint8_t bytes[256];
        debugger_copy_oam(bytes);
        return bytes[address & 255];
    }
    return debugger_peek_ppu((uint16_t)address);
}

static bool can_edit(Ppu *v) {
    return v->valid && frontend_panel_session_active() && live(v) && v->image.session == debugger_session_revision() &&
           nes_execution_policy() == NES_EXECUTION_LIVE && v->tool->ui.execution &&
           frontend_execution_paused(v->tool->ui.execution);
}

static bool editable(Ppu *v) {
    if (!can_edit(v)) {
        status(v, "Pause the game and enable Live to edit. Playback, rewind and netplay are read-only.");
        return false;
    }
    debugger_pause();
    frontend_execution_sync_debugger(v->tool->ui.execution);
    return true;
}

static void remember(Ppu *v, bool is_oam, unsigned address, unsigned n) {
    v->undo_oam = is_oam;
    v->undo_address = address;
    v->undo_count = n;
    DebugPpuSnapshot state;
    debugger_get_ppu(&state);
    v->undo_frame = state.frame;
    v->undo_session = debugger_session_revision();
    for (unsigned i = 0; i < n; ++i) {
        v->before[i] = peek(is_oam, address + i);
    }
}

static void changed(Ppu *v) {
    for (unsigned i = 0; i < v->undo_count; ++i) {
        v->after[i] = peek(v->undo_oam, v->undo_address + i);
    }
    frontend_execution_clear_timeline(v->tool->ui.execution);
    v->capture = v->dirty = true;
    status(v, "Memory updated. The game may overwrite RAM when resumed.");
    refresh(v);
}

static void remember_colors(Ppu *v) {
    ppu_palette_get(v->colors_before);
    memcpy(v->emphasis_before, ppu__emphasis_palettes, sizeof(v->emphasis_before));
    v->emphasis_was = ppu_palette_has_emphasis_tables();
}

static void colors_changed(Ppu *v) {
    ppu_palette_get(v->colors_after);
    memcpy(v->emphasis_after, ppu__emphasis_palettes, sizeof(v->emphasis_after));
    v->emphasis_now = ppu_palette_has_emphasis_tables();
    v->color_undo = true;
    v->color_initialized = false;
    v->dirty = true;
    refresh(v);
    status(v, "Display palette updated for this session. Export a PAL file to keep it.");
}

static void geometry(Ppu *v, int width, int height, double *scale, double *ox, double *oy) {
    *scale = fmin((double)width / v->width, (double)height / v->height);
    if (*scale >= 1) {
        *scale = floor(*scale);
    }
    *ox = (width - v->width * *scale) / 2;
    *oy = (height - v->height * *scale) / 2;
}

static void draw(GtkDrawingArea *area, cairo_t *cr, int width, int height, gpointer data) {
    (void)area;
    Ppu *v = data;
    if (!v->valid || !v->width || !v->height) {
        return;
    }
    double scale, ox, oy;
    geometry(v, width, height, &scale, &ox, &oy);
    if (scale <= 0) {
        return;
    }
    cairo_save(cr);
    cairo_translate(cr, ox, oy);
    cairo_scale(cr, scale, scale);
    cairo_surface_t *surface = cairo_image_surface_create_for_data((unsigned char *)v->pixels, CAIRO_FORMAT_ARGB32,
                                                                   (int)v->width, (int)v->height, (int)v->width * 4);
    cairo_set_source_surface(cr, surface, 0, 0);
    cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_NEAREST);
    cairo_rectangle(cr, 0, 0, v->width, v->height);
    cairo_fill(cr);
    cairo_surface_destroy(surface);
    unsigned cx = 8, cy = 8, sx = 0, sy = 0, sw = 8, sh = 8;
    if (v->runtime_palette || v->panel == DEBUG_PPU_PALETTE || v->panel == DEBUG_PPU_TILE) {
        cx = cy = 1;
    }
    if (v->panel == DEBUG_PPU_SPRITES) {
        cx = 16;
        cy = 32;
    }
    if (gtk_check_button_get_active(GTK_CHECK_BUTTON(v->grid)) && !(v->panel == DEBUG_PPU_SPRITES && choice(v->mode))) {
        cairo_set_source_rgba(cr, .55, .6, .7, .6);
        cairo_set_line_width(cr, 1 / scale);
        for (unsigned x = cx; x < v->width; x += cx) {
            cairo_move_to(cr, x, 0);
            cairo_line_to(cr, x, v->height);
        }
        for (unsigned y = cy; y < v->height; y += cy) {
            cairo_move_to(cr, 0, y);
            cairo_line_to(cr, v->width, y);
        }
        cairo_stroke(cr);
    }
    if (v->runtime_palette || v->panel == DEBUG_PPU_PALETTE) {
        sx = v->selected % 16;
        sy = v->selected / 16;
        sw = sh = 1;
    } else if (v->panel == DEBUG_PPU_PATTERNS) {
        sx = v->selected / 256 * 128 + v->selected % 16 * 8;
        sy = v->selected % 256 / 16 * 8;
    } else if (v->panel == DEBUG_PPU_NAMETABLES) {
        sx = v->selected % 64 * 8;
        sy = v->selected / 64 * 8;
    } else if (v->panel == DEBUG_PPU_SPRITES) {
        DebugPpuSelection s = debug_ppu_sprite(&v->image, v->selected);
        sx = choice(v->mode) ? s.x : v->selected % 16 * 16 + 4;
        sy = choice(v->mode) ? s.y : v->selected / 16 * 32 + 8;
        sh = s.height;
    }
    cairo_rectangle(cr, 0, 0, v->width, v->height);
    cairo_clip(cr);
    cairo_set_source_rgb(cr, 1, .8, .2);
    cairo_set_line_width(cr, 2 / scale);
    cairo_rectangle(cr, sx, sy, sw, sh);
    cairo_stroke(cr);
    cairo_restore(cr);
}

static void describe(Ppu *v) {
    char text[1536];
    DebugPpuSnapshot *s = &v->image.state;
    if (v->runtime_palette) {
        uint32_t c = v->pixels[v->selected];
        snprintf(text, sizeof(text),
                 "Color $%02X\nRGB #%06X\n\nChanges affect display colors, not game memory.\nPAL files contain 64 RGB "
                 "colors or eight emphasis tables.",
                 v->selected, c & 0xffffff);
        GdkRGBA color = {(float)((c >> 16) & 255) / 255, (float)((c >> 8) & 255) / 255, (float)(c & 255) / 255, 1};
        if (!v->color_initialized || v->color_selected != v->selected) {
            gtk_color_chooser_set_rgba(GTK_COLOR_CHOOSER(v->color), &color);
            v->color_selected = v->selected;
            v->color_initialized = true;
        }
    } else if (v->panel == DEBUG_PPU_REGISTERS) {
        snprintf(text, sizeof(text),
                 "PPUCTRL $2000: $%02X\nNMI: %s  Increment: %u\n\nPPUMASK $2001: $%02X\nBackground: %s  Sprites: "
                 "%s\n\nPPUSTATUS $2002: $%02X\nVBlank: %u  Sprite zero: %u  Overflow: %u\n\nVRAM v: $%04X  Temporary "
                 "t: $%04X\nFine X: %u  Write latch: %u\nOAM address: $%02X\n\nBackground bank: $%04X\nSprite bank: "
                 "$%04X\nSprite size: 8x%u\nBase nametable: $%04X\nMirroring: %u\n\nScanline: %d  Dot: %d\nFrame: "
                 "%llu\nPPU cycles: %llu",
                 s->ctrl, s->ctrl & 128 ? "on" : "off", s->ctrl & 4 ? 32 : 1, s->mask, s->mask & 8 ? "on" : "off",
                 s->mask & 16 ? "on" : "off", s->status, !!(s->status & 128), !!(s->status & 64), !!(s->status & 32),
                 s->v, s->t, s->fine_x, s->write_toggle, s->oam_addr, s->ctrl & 16 ? 0x1000 : 0,
                 s->ctrl & 8 ? 0x1000 : 0, s->ctrl & 32 ? 16 : 8, 0x2000 + (s->ctrl & 3) * 0x400,
                 (unsigned)v->image.mirroring, s->scanline, s->dot, (unsigned long long)s->frame,
                 (unsigned long long)s->ppu_cycles);
    } else if (v->panel == DEBUG_PPU_SPRITES) {
        DebugPpuSelection p = debug_ppu_sprite(&v->image, v->selected);
        snprintf(text, sizeof(text),
                 "OAM %u\nX %u  Y %u (raw %u)\nSize 8x%u\nTile $%02X  Address $%04X\nPalette SP%u\nFlip X: %s  Flip Y: "
                 "%s\n%s background",
                 v->selected, p.x, p.y, p.y - 1, p.height, p.tile, p.pattern, p.palette - 4, p.flip_x ? "yes" : "no",
                 p.flip_y ? "yes" : "no", p.behind ? "Behind" : "In front of");
    } else if (v->panel == DEBUG_PPU_NAMETABLES) {
        DebugPpuSelection p = debug_ppu_nametable(&v->image, v->selected % 64 * 8, v->selected / 64 * 8);
        snprintf(text, sizeof(text),
                 "Nametable $%04X\nAttribute $%04X\nTile $%02X  Pattern $%04X\nPalette BG%u\n\nCurrent mapped banks. "
                 "Raster-time bank changes are not a completed-frame reconstruction.",
                 p.nametable, p.attribute, p.tile, p.pattern, p.palette);
    } else if (v->panel == DEBUG_PPU_VRAM || v->panel == DEBUG_PPU_PALETTE) {
        unsigned a = v->panel == DEBUG_PPU_PALETTE ? 0x3f00 + v->selected : v->selected;
        snprintf(text, sizeof(text),
                 "%s $%04X = $%02X\n\nPause and enable Live to edit. ROM and protected pages cannot be written.%s",
                 oam(v) ? "OAM" : "PPU", a, oam(v) ? v->image.oam[a] : v->image.memory[a],
                 v->panel == DEBUG_PPU_PALETTE ? "\n\n$3F10/$14/$18/$1C mirror $3F00/$04/$08/$0C." : "");
    } else {
        snprintf(text, sizeof(text), "Tile $%02X\nTable $%04X\nAddress $%04X\nPalette %s%u\n\n%s", v->selected & 255,
                 v->selected >= 256 ? 0x1000 : 0, v->selected * 16, choice(v->palette) < 4 ? "BG" : "SP",
                 choice(v->palette) & 3,
                 v->panel == DEBUG_PPU_TILE
                     ? "Click a pixel to paint. Select CPU banks and pause the game. Only CHR RAM is writable."
                     : "Select a tile to inspect its address. Double-click to open it in the tile editor.");
    }
    gtk_label_set_text(GTK_LABEL(v->detail), text);
}

static void refresh(Ppu *v) {
    bool available = v->runtime_palette || frontend_panel_session_active();
    bool session_changed = !v->runtime_palette && v->image.session != debugger_session_revision();
    if (!available) {
        v->valid = false;
        v->undo_count = 0;
        gtk_label_set_text(GTK_LABEL(v->detail), "Open a game to inspect PPU memory.");
        if (v->apply) {
            gtk_widget_set_sensitive(v->apply, FALSE);
        }
        gtk_widget_set_sensitive(v->undo, FALSE);
        gtk_widget_queue_draw(v->canvas);
        return;
    }
    if (session_changed) {
        v->capture = true;
        v->undo_count = 0;
        v->selected = 0;
    }
    if (v->stepping) {
        DebugPpuSnapshot s;
        debugger_get_ppu(&s);
        if (s.ppu_cycles != v->step_cycles) {
            v->capture = true;
            v->stepping = false;
        }
    }
    bool capture = v->capture || !v->valid || (live(v) && SDL_GetTicks() - v->refreshed >= 33);
    if (v->apply) {
        gtk_widget_set_sensitive(v->apply, v->runtime_palette || can_edit(v));
    }
    gtk_widget_set_sensitive(v->undo, v->runtime_palette ? v->color_undo : can_edit(v) && v->undo_count);
    if (!capture && !v->dirty) {
        return;
    }
    v->updating = true;
    if (capture && !v->runtime_palette) {
        debug_ppu_capture(&v->image, v->panel == DEBUG_PPU_NAMETABLES);
    }
    v->refreshed = SDL_GetTicks();
    v->capture = v->dirty = false;
    v->valid = true;
    if (v->undo_frame != v->image.state.frame) {
        v->undo_count = 0;
    }
    v->selected %= count(v);
    if (v->selection) {
        gtk_spin_button_set_range(GTK_SPIN_BUTTON(v->selection), 0, count(v) - 1);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(v->selection), v->selected);
    }
    v->width = 256;
    v->height = 128;
    const uint8_t *chr = choice(v->source) == 1   ? v->image.background
                         : choice(v->source) == 2 ? v->image.sprites
                                                  : v->image.memory;
    if (v->runtime_palette) {
        v->width = 16;
        v->height = 4;
        ppu_palette_get(v->pixels);
    } else {
        switch (v->panel) {
        case DEBUG_PPU_PATTERNS:
            debug_ppu_patterns(&v->image, chr, choice(v->palette), v->pixels);
            break;
        case DEBUG_PPU_NAMETABLES:
            v->width = 512;
            v->height = 480;
            debug_ppu_nametables(&v->image, choice(v->mode), v->pixels);
            break;
        case DEBUG_PPU_SPRITES:
            v->height = choice(v->mode) ? 240 : 128;
            debug_ppu_sprites(&v->image, choice(v->mode), v->pixels);
            break;
        case DEBUG_PPU_TILE:
            v->width = v->height = 8;
            for (unsigned y = 0; y < 8; ++y) {
                for (unsigned x = 0; x < 8; ++x) {
                    v->pixels[y * 8 + x] = debug_ppu_color(&v->image, choice(v->palette),
                                                           debug_ppu_pixel(chr + v->selected * 16, x, y), false);
                }
            }
            break;
        case DEBUG_PPU_PALETTE:
            v->width = 16;
            v->height = 2;
            for (unsigned i = 0; i < 32; ++i) {
                v->pixels[i] = v->image.colors[v->image.memory[0x3f00 + i] & 63];
            }
            break;
        case DEBUG_PPU_VRAM:
            v->page = v->selected & ~255u;
            for (unsigned i = 0; i < 256; ++i) {
                char byte[8];
                unsigned a = v->page + i;
                snprintf(byte, sizeof(byte), "%02X", oam(v) ? v->image.oam[a] : v->image.memory[a]);
                gtk_button_set_label(GTK_BUTTON(v->memory[i]), byte);
                if (a == v->selected) {
                    gtk_widget_add_css_class(v->memory[i], "suggested-action");
                } else {
                    gtk_widget_remove_css_class(v->memory[i], "suggested-action");
                }
                if (!(i & 15)) {
                    snprintf(byte, sizeof(byte), "%04X", a);
                    gtk_label_set_text(GTK_LABEL(v->row_labels[i / 16]), byte);
                }
            }
            break;
        default:
            break;
        }
    }
    describe(v);
    char footer[160];
    snprintf(footer, sizeof(footer), "%s | Frame %llu | Scanline %d, dot %d", live(v) ? "Live" : "Frozen",
             (unsigned long long)v->image.state.frame, v->image.state.scanline, v->image.state.dot);
    gtk_label_set_text(GTK_LABEL(v->footer), v->runtime_palette ? "Display palette: 64 colors" : footer);
    if (v->pause) {
        gtk_button_set_label(GTK_BUTTON(v->pause),
                             v->tool->ui.execution && frontend_execution_paused(v->tool->ui.execution) ? "Resume"
                                                                                                       : "Pause");
    }
    if (v->apply) {
        gtk_widget_set_sensitive(v->apply, v->runtime_palette || can_edit(v));
    }
    gtk_widget_set_sensitive(v->undo, v->runtime_palette ? v->color_undo : can_edit(v) && v->undo_count);
    gtk_widget_queue_draw(v->canvas);
    v->updating = false;
}

void cupid_gtk_ppu_refresh(GtkWidget *root) {
    Ppu *v = g_object_get_data(G_OBJECT(root), "cupid-ppu");
    if (v) {
        refresh(v);
    }
}

static gboolean address_output(GtkSpinButton *spin, gpointer data) {
    (void)data;
    char text[12];
    snprintf(text, sizeof(text), "$%04X", gtk_spin_button_get_value_as_int(spin));
    gtk_editable_set_text(GTK_EDITABLE(spin), text);
    return TRUE;
}

static gint address_input(GtkSpinButton *spin, double *value, gpointer data) {
    Ppu *v = data;
    const char *text = gtk_editable_get_text(GTK_EDITABLE(spin));
    if (*text == '$') {
        ++text;
    }
    if (!*text || strlen(text) > 4) {
        return GTK_INPUT_ERROR;
    }
    unsigned address = 0;
    for (; *text; ++text) {
        int digit = g_ascii_xdigit_value(*text);
        if (digit < 0) {
            return GTK_INPUT_ERROR;
        }
        address = address * 16 + (unsigned)digit;
    }
    if (address >= count(v)) {
        return GTK_INPUT_ERROR;
    }
    *value = address;
    return TRUE;
}

static void selected(GtkSpinButton *spin, gpointer data) {
    Ppu *v = data;
    if (!v->updating) {
        v->selected = (unsigned)gtk_spin_button_get_value_as_int(spin);
        v->dirty = true;
        refresh(v);
    }
}

static void option(GObject *object, GParamSpec *spec, gpointer data) {
    (void)object;
    (void)spec;
    Ppu *v = data;
    if (!v->updating) {
        v->dirty = true;
        refresh(v);
    }
}

static void toggled(GtkCheckButton *button, gpointer data) {
    Ppu *v = data;
    v->dirty = true;
    if (GTK_WIDGET(button) == v->live) {
        v->capture = true;
    }
    refresh(v);
}

static void byte_clicked(GtkButton *button, gpointer data) {
    Ppu *v = data;
    v->selected = v->page + GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "offset"));
    v->dirty = true;
    refresh(v);
}

static void open_tile(GtkButton *button, gpointer data) {
    (void)button;
    Ppu *v = data;
    if (!v->valid || !v->tool->desktop) {
        return;
    }
    FrontendDesktopUi *opened = cupid_gtk_open(&v->tool->ui, 1, DEBUG_PPU_TILE);
    for (CupidGtkTool *tool = v->tool->desktop->tools; tool; tool = tool->next) {
        if (&tool->ui != opened || !tool->content) {
            continue;
        }
        Ppu *tile = g_object_get_data(G_OBJECT(tool->content), "cupid-ppu");
        if (!tile) {
            return;
        }
        tile->updating = true;
        tile->selected = v->selected;
        gtk_drop_down_set_selected(GTK_DROP_DOWN(tile->palette), choice(v->palette));
        gtk_drop_down_set_selected(GTK_DROP_DOWN(tile->source), choice(v->source));
        tile->updating = false;
        tile->dirty = true;
        refresh(tile);
        return;
    }
}

static void clicked(GtkGestureClick *gesture, int n, double x, double y, gpointer data) {
    (void)gesture;
    Ppu *v = data;
    if (!v->valid) {
        return;
    }
    double scale, ox, oy;
    geometry(v, gtk_widget_get_width(v->canvas), gtk_widget_get_height(v->canvas), &scale, &ox, &oy);
    if (scale <= 0 || x < ox || y < oy) {
        return;
    }
    unsigned px = (unsigned)((x - ox) / scale), py = (unsigned)((y - oy) / scale);
    if (px >= v->width || py >= v->height) {
        return;
    }
    if (v->runtime_palette || v->panel == DEBUG_PPU_PALETTE) {
        v->selected = py * 16 + px;
    } else if (v->panel == DEBUG_PPU_TILE) {
        if (!editable(v)) {
            return;
        }
        if (choice(v->source)) {
            status(v, "Select CPU banks to edit mapped CHR RAM.");
            return;
        }
        remember(v, false, v->selected * 16, 16);
        if (debug_ppu_paint(v->selected * 16, px, py, choice(v->brush))) {
            changed(v);
        } else {
            v->undo_count = 0;
            status(v, "This tile is ROM or write-protected.");
        }
    } else if (v->panel == DEBUG_PPU_PATTERNS) {
        v->selected = px / 128 * 256 + py / 8 * 16 + px % 128 / 8;
        if (n >= 2) {
            open_tile(NULL, v);
        }
    } else if (v->panel == DEBUG_PPU_NAMETABLES) {
        v->selected = py / 8 * 64 + px / 8;
    } else if (v->panel == DEBUG_PPU_SPRITES) {
        if (!choice(v->mode)) {
            v->selected = py / 32 * 16 + px / 16;
        } else {
            for (unsigned i = 0; i < 64; ++i) {
                DebugPpuSelection s = debug_ppu_sprite(&v->image, i);
                if (px >= s.x && px < s.x + 8 && py >= s.y && py < s.y + s.height) {
                    v->selected = i;
                    break;
                }
            }
        }
    }
    v->dirty = true;
    refresh(v);
}

static bool hex_byte(const char *text, unsigned *value) {
    if (*text == '$') {
        ++text;
    }
    if (!*text || strlen(text) > 2) {
        return false;
    }
    *value = 0;
    for (; *text; ++text) {
        int digit = g_ascii_xdigit_value(*text);
        if (digit < 0) {
            return false;
        }
        *value = *value * 16 + (unsigned)digit;
    }
    return true;
}

static void apply(GtkButton *button, gpointer data) {
    (void)button;
    Ppu *v = data;
    if (v->runtime_palette) {
        GdkRGBA c;
        gtk_color_chooser_get_rgba(GTK_COLOR_CHOOSER(v->color), &c);
        remember_colors(v);
        ppu_palette_set_color((int)v->selected, (uint8_t)lround(c.red * 255), (uint8_t)lround(c.green * 255),
                              (uint8_t)lround(c.blue * 255));
        colors_changed(v);
        return;
    }
    if (!editable(v)) {
        return;
    }
    unsigned value;
    if (!hex_byte(gtk_editable_get_text(GTK_EDITABLE(v->value)), &value)) {
        status(v, "Enter a byte in hexadecimal, 00 to FF.");
        return;
    }
    unsigned address = v->panel == DEBUG_PPU_PALETTE ? 0x3f00 + v->selected : v->selected;
    remember(v, oam(v), address, 1);
    if (!debug_ppu_write(oam(v), address, (uint8_t)value)) {
        v->undo_count = 0;
        status(v, "This address is ROM, unmapped, or write-protected.");
        return;
    }
    changed(v);
}

static void undo(GtkButton *button, gpointer data) {
    (void)button;
    Ppu *v = data;
    if (v->runtime_palette) {
        uint32_t current[64];
        ppu_palette_get(current);
        bool match = v->color_undo && !memcmp(current, v->colors_after, sizeof(current)) &&
                     ppu_palette_has_emphasis_tables() == v->emphasis_now &&
                     !memcmp(ppu__emphasis_palettes, v->emphasis_after, sizeof(v->emphasis_after));
        if (match) {
            memcpy(ppu__active_palette_base, v->colors_before, sizeof(current));
            memcpy(ppu__emphasis_palettes, v->emphasis_before, sizeof(v->emphasis_before));
            ppu__have_emphasis_tables = v->emphasis_was;
        }
        v->color_undo = false;
        v->color_initialized = false;
        status(v, match ? "Palette edit undone." : "Palette changed since the edit; undo discarded.");
    } else {
        if (!editable(v) || !v->undo_count) {
            return;
        }
        DebugPpuSnapshot state;
        debugger_get_ppu(&state);
        bool match = v->undo_session == debugger_session_revision() && v->undo_frame == state.frame;
        for (unsigned i = 0; i < v->undo_count; ++i) {
            match &= peek(v->undo_oam, v->undo_address + i) == v->after[i];
        }
        bool written = match;
        if (match) {
            for (unsigned i = 0; i < v->undo_count; ++i) {
                written &= debug_ppu_write(v->undo_oam, v->undo_address + i, v->before[i]);
            }
        }
        if (match) {
            frontend_execution_clear_timeline(v->tool->ui.execution);
        }
        v->undo_count = 0;
        status(v, written ? "Edit undone." : "Memory changed or is protected; undo discarded.");
    }
    v->dirty = v->capture = true;
    refresh(v);
}

static void command(GtkButton *button, gpointer data) {
    Ppu *v = data;
    unsigned id = GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "command"));
    if (id && (!frontend_panel_session_active() || !v->tool->ui.execution)) {
        return;
    }
    if (id == FRONTEND_COMMAND_FRAME_ADVANCE ||
        (id == FRONTEND_COMMAND_PAUSE && !frontend_execution_paused(v->tool->ui.execution))) {
        debugger_pause();
        frontend_execution_sync_debugger(v->tool->ui.execution);
        if (id == FRONTEND_COMMAND_PAUSE) {
            id = 0;
        }
    }
    if (id) {
        char error[256] = "";
        DebugPpuSnapshot s;
        debugger_get_ppu(&s);
        v->step_cycles = s.ppu_cycles;
        if (!frontend_command_invoke(id, error, sizeof(error))) {
            status(v, error);
        } else if (id == FRONTEND_COMMAND_FRAME_ADVANCE) {
            v->stepping = true;
        }
    }
    v->dirty = v->capture = true;
    refresh(v);
}

static void reset_palette(GtkButton *button, gpointer data) {
    (void)button;
    Ppu *v = data;
    remember_colors(v);
    ppu_palette_reset_default();
    colors_changed(v);
}

static void file_response(GtkNativeDialog *dialog, int response, gpointer data) {
    Ppu *v = data;
    unsigned mode = GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(dialog), "mode"));
    if (response == GTK_RESPONSE_ACCEPT) {
        GFile *file = gtk_file_chooser_get_file(GTK_FILE_CHOOSER(dialog));
        char *path = file ? g_file_get_path(file) : NULL;
        char error[256] = "";
        if (!path) {
            status(v, "Choose a local file.");
        } else if (mode == 1) {
            uint8_t *bytes = NULL;
            size_t size = 0;
            if (nes_file_read_all(path, 1536, &bytes, &size) != NES_FILE_OK || (size != 192 && size != 1536)) {
                status(v, "PAL files must contain 192 or 1536 bytes.");
            } else {
                remember_colors(v);
                for (unsigned e = 0; e < size / 192; ++e) {
                    for (unsigned i = 0; i < 64; ++i) {
                        unsigned a = (e * 64 + i) * 3;
                        uint32_t color =
                            0xff000000u | (uint32_t)bytes[a] << 16 | (uint32_t)bytes[a + 1] << 8 | bytes[a + 2];
                        if (!e) {
                            ppu__active_palette_base[i] = color;
                        }
                        if (size == 1536) {
                            ppu__emphasis_palettes[e][i] = color;
                        }
                    }
                }
                ppu__have_emphasis_tables = size == 1536;
                colors_changed(v);
            }
            free(bytes);
        } else if (!frontend_output_path_allowed(path, v->tool->ui.execution, NULL, 0, error, sizeof(error))) {
            status(v, error);
        } else if (mode == 2) {
            uint8_t bytes[1536];
            unsigned tables = ppu_palette_has_emphasis_tables() ? 8 : 1;
            for (unsigned e = 0; e < tables; ++e) {
                for (unsigned i = 0; i < 64; ++i) {
                    uint32_t c = tables == 8 ? ppu__emphasis_palettes[e][i] : ppu__active_palette_base[i];
                    unsigned a = (e * 64 + i) * 3;
                    bytes[a] = (uint8_t)(c >> 16);
                    bytes[a + 1] = (uint8_t)(c >> 8);
                    bytes[a + 2] = (uint8_t)c;
                }
            }
            status(v, nes_file_write_atomic(path, bytes, tables * 192) == NES_FILE_OK ? "Palette saved."
                                                                                      : "Could not save palette.");
        } else if (v->valid) {
            NesCaptureFrame frame = {v->pixels, v->width, v->height, v->width};
            status(v, nes_capture_png(path, &frame) == NES_FILE_OK ? "Viewer image saved." : "Could not save image.");
        }
        g_free(path);
        if (file) {
            g_object_unref(file);
        }
    }
    v->dialog = NULL;
    gtk_native_dialog_destroy(dialog);
    g_object_unref(dialog);
}

static void file_clicked(GtkButton *button, gpointer data) {
    Ppu *v = data;
    if (v->dialog) {
        gtk_native_dialog_show(v->dialog);
        return;
    }
    unsigned mode = GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "mode"));
    if (!mode && !v->valid) {
        return;
    }
    GtkFileChooserNative *chooser = gtk_file_chooser_native_new(
        mode == 1   ? "Import palette"
        : mode == 2 ? "Export palette"
                    : "Export viewer PNG",
        GTK_WINDOW(v->tool->window), mode == 1 ? GTK_FILE_CHOOSER_ACTION_OPEN : GTK_FILE_CHOOSER_ACTION_SAVE,
        mode == 1 ? "Open" : "Save", "Cancel");
    GtkFileFilter *filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, mode ? "PAL palette" : "PNG image");
    gtk_file_filter_add_pattern(filter, mode ? "*.pal" : "*.png");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(chooser), filter);
    g_object_unref(filter);
    if (mode != 1) {
        gtk_file_chooser_set_current_name(GTK_FILE_CHOOSER(chooser), mode ? "palette.pal" : "ppu.png");
    }
    v->dialog = GTK_NATIVE_DIALOG(chooser);
    g_object_set_data(G_OBJECT(chooser), "mode", GUINT_TO_POINTER(mode));
    g_signal_connect(chooser, "response", G_CALLBACK(file_response), v);
    gtk_native_dialog_show(v->dialog);
}

static void close_dialog(GtkWidget *widget, gpointer data) {
    (void)widget;
    Ppu *v = data;
    if (v->dialog) {
        g_signal_handlers_disconnect_by_data(v->dialog, v);
        gtk_native_dialog_destroy(v->dialog);
        g_object_unref(v->dialog);
        v->dialog = NULL;
    }
}

static void destroy(gpointer data) {
    close_dialog(NULL, data);
    g_free(data);
}

static GtkWidget *button(GtkWidget *box, const char *label, GCallback callback, Ppu *v) {
    GtkWidget *w = gtk_button_new_with_label(label);
    gtk_box_append(GTK_BOX(box), w);
    g_signal_connect(w, "clicked", callback, v);
    return w;
}

static GtkWidget *dropdown(GtkWidget *box, const char *label, const char *const *strings, Ppu *v) {
    gtk_box_append(GTK_BOX(box), cupid_gtk_label(label));
    GtkWidget *w = gtk_drop_down_new_from_strings(strings);
    gtk_box_append(GTK_BOX(box), w);
    g_signal_connect(w, "notify::selected", G_CALLBACK(option), v);
    return w;
}

GtkWidget *cupid_gtk_ppu_new(CupidGtkTool *tool) {
    Ppu *v = g_new0(Ppu, 1);
    v->tool = tool;
    v->panel = tool->id;
    v->runtime_palette = tool->kind == 3;
    v->dirty = v->capture = true;
    v->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    cupid_gtk_margins(v->root, 10);
    GtkWidget *bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_add_css_class(bar, "toolbar");
    gtk_box_append(GTK_BOX(v->root), bar);
    if (!v->runtime_palette) {
        v->pause = button(bar, "Pause", G_CALLBACK(command), v);
        g_object_set_data(G_OBJECT(v->pause), "command", GUINT_TO_POINTER(FRONTEND_COMMAND_PAUSE));
        GtkWidget *frame = button(bar, "Frame", G_CALLBACK(command), v);
        g_object_set_data(G_OBJECT(frame), "command", GUINT_TO_POINTER(FRONTEND_COMMAND_FRAME_ADVANCE));
    }
    button(bar, "Refresh", G_CALLBACK(command), v);
    v->live = gtk_check_button_new_with_label("Live");
    gtk_check_button_set_active(GTK_CHECK_BUTTON(v->live),
                                v->runtime_palette || !tool->ui.settings || tool->ui.settings->ppu_viewer_live);
    gtk_box_append(GTK_BOX(bar), v->live);
    g_signal_connect(v->live, "toggled", G_CALLBACK(toggled), v);
    v->grid = gtk_check_button_new_with_label("Grid");
    gtk_check_button_set_active(GTK_CHECK_BUTTON(v->grid), !tool->ui.settings || tool->ui.settings->ppu_viewer_grid);
    gtk_box_append(GTK_BOX(bar), v->grid);
    g_signal_connect(v->grid, "toggled", G_CALLBACK(toggled), v);
    if (v->runtime_palette || (v->panel != DEBUG_PPU_REGISTERS && v->panel != DEBUG_PPU_VRAM)) {
        button(bar, "Export PNG", G_CALLBACK(file_clicked), v);
    }
    GtkWidget *split = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_widget_set_vexpand(split, TRUE);
    gtk_box_append(GTK_BOX(v->root), split);
    GtkWidget *side = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    cupid_gtk_margins(side, 8);
    gtk_widget_set_size_request(side, 260, -1);
    gtk_paned_set_end_child(GTK_PANED(split), cupid_gtk_scroll(side));
    gtk_paned_set_resize_end_child(GTK_PANED(split), FALSE);
    gtk_paned_set_shrink_start_child(GTK_PANED(split), FALSE);
    gtk_paned_set_position(GTK_PANED(split), 550);
    GtkWidget *sidebar = side;
    const char *selection_title = v->runtime_palette || v->panel == DEBUG_PPU_PALETTE ? "Selected color"
                                  : v->panel == DEBUG_PPU_SPRITES                     ? "Selected sprite"
                                  : v->panel == DEBUG_PPU_NAMETABLES                  ? "Selected nametable tile"
                                  : v->panel == DEBUG_PPU_VRAM                        ? "Memory address"
                                                                                      : "Selected tile";
    if (v->panel != DEBUG_PPU_REGISTERS || v->runtime_palette) {
        side = cupid_gtk_group(sidebar, selection_title);
    }
    v->canvas = gtk_drawing_area_new();
    gtk_widget_set_size_request(v->canvas, 256, 240);
    gtk_widget_set_hexpand(v->canvas, TRUE);
    gtk_widget_set_vexpand(v->canvas, TRUE);
    gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(v->canvas), draw, v, NULL);
    GtkGesture *gesture = gtk_gesture_click_new();
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(gesture), 1);
    g_signal_connect(gesture, "pressed", G_CALLBACK(clicked), v);
    gtk_widget_add_controller(v->canvas, GTK_EVENT_CONTROLLER(gesture));
    gtk_paned_set_start_child(GTK_PANED(split), v->canvas);
    if (!v->runtime_palette && v->panel == DEBUG_PPU_VRAM) {
        GtkWidget *memory = gtk_grid_new();
        gtk_grid_set_row_spacing(GTK_GRID(memory), 2);
        gtk_grid_set_column_spacing(GTK_GRID(memory), 2);
        for (unsigned row = 0; row < 16; ++row) {
            v->row_labels[row] = cupid_gtk_label("");
            gtk_grid_attach(GTK_GRID(memory), v->row_labels[row], 0, (int)row, 1, 1);
            for (unsigned col = 0; col < 16; ++col) {
                unsigned i = row * 16 + col;
                v->memory[i] = gtk_button_new_with_label("00");
                gtk_widget_add_css_class(v->memory[i], "monospace");
                g_object_set_data(G_OBJECT(v->memory[i]), "offset", GUINT_TO_POINTER(i));
                g_signal_connect(v->memory[i], "clicked", G_CALLBACK(byte_clicked), v);
                gtk_grid_attach(GTK_GRID(memory), v->memory[i], (int)col + 1, (int)row, 1, 1);
            }
        }
        /* Keep the hidden canvas alive for common refresh/lifetime handling. */
        g_object_ref(v->canvas);
        g_object_set_data_full(G_OBJECT(v->root), "hidden-canvas", v->canvas, g_object_unref);
        gtk_paned_set_start_child(GTK_PANED(split), cupid_gtk_scroll(memory));
    }
    if (v->panel != DEBUG_PPU_REGISTERS || v->runtime_palette) {
        gtk_box_append(GTK_BOX(side),
                       cupid_gtk_label(v->panel == DEBUG_PPU_VRAM ? "Address (hexadecimal)" : "Selection index"));
        v->selection = gtk_spin_button_new_with_range(0, count(v) - 1, 1);
        gtk_box_append(GTK_BOX(side), v->selection);
        g_signal_connect(v->selection, "value-changed", G_CALLBACK(selected), v);
        if (v->panel == DEBUG_PPU_VRAM && !v->runtime_palette) {
            gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(v->selection), FALSE);
            gtk_spin_button_set_increments(GTK_SPIN_BUTTON(v->selection), 1, 256);
            g_signal_connect(v->selection, "output", G_CALLBACK(address_output), v);
            g_signal_connect(v->selection, "input", G_CALLBACK(address_input), v);
        }
    }
    if (!v->runtime_palette && (v->panel == DEBUG_PPU_PATTERNS || v->panel == DEBUG_PPU_TILE)) {
        const char *palettes[] = {"BG0", "BG1", "BG2", "BG3", "SP0", "SP1", "SP2", "SP3", NULL};
        const char *sources[] = {"CPU banks", "Background banks", "Sprite banks", NULL};
        v->palette = dropdown(side, "Palette", palettes, v);
        v->source = dropdown(side, "Source", sources, v);
        if (v->panel == DEBUG_PPU_TILE) {
            const char *brushes[] = {"0", "1", "2", "3", NULL};
            v->brush = dropdown(side, "Paint color", brushes, v);
        }
    }
    if (!v->runtime_palette &&
        (v->panel == DEBUG_PPU_NAMETABLES || v->panel == DEBUG_PPU_SPRITES || v->panel == DEBUG_PPU_VRAM)) {
        const char *modes[] = {v->panel == DEBUG_PPU_NAMETABLES ? "Tiles"
                               : v->panel == DEBUG_PPU_SPRITES  ? "OAM grid"
                                                                : "PPU memory",
                               v->panel == DEBUG_PPU_NAMETABLES ? "Attributes"
                               : v->panel == DEBUG_PPU_SPRITES  ? "Screen positions"
                                                                : "OAM memory",
                               NULL};
        v->mode = dropdown(side, "View", modes, v);
    }
    if (v->runtime_palette) {
        v->color = gtk_color_button_new();
        gtk_color_chooser_set_use_alpha(GTK_COLOR_CHOOSER(v->color), FALSE);
        gtk_box_append(GTK_BOX(side), v->color);
        v->apply = button(side, "Apply color", G_CALLBACK(apply), v);
        button(side, "Restore default palette", G_CALLBACK(reset_palette), v);
        GtkWidget *load = button(side, "Import PAL", G_CALLBACK(file_clicked), v);
        g_object_set_data(G_OBJECT(load), "mode", GUINT_TO_POINTER(1));
        GtkWidget *save = button(side, "Export PAL", G_CALLBACK(file_clicked), v);
        g_object_set_data(G_OBJECT(save), "mode", GUINT_TO_POINTER(2));
    } else if (v->panel == DEBUG_PPU_VRAM || v->panel == DEBUG_PPU_PALETTE) {
        v->value = gtk_entry_new();
        gtk_entry_set_placeholder_text(GTK_ENTRY(v->value), "Hex byte, 00–FF");
        gtk_entry_set_max_length(GTK_ENTRY(v->value), 3);
        gtk_box_append(GTK_BOX(side), v->value);
        v->apply = button(side, "Write byte", G_CALLBACK(apply), v);
    }
    if (!v->runtime_palette && v->panel == DEBUG_PPU_PATTERNS) {
        button(side, "Open tile editor", G_CALLBACK(open_tile), v);
    }
    v->undo = button(side, "Undo last edit", G_CALLBACK(undo), v);
    side = cupid_gtk_group(sidebar, v->panel == DEBUG_PPU_REGISTERS ? "PPU registers" : "Selection details");
    v->detail = cupid_gtk_label("");
    gtk_label_set_wrap(GTK_LABEL(v->detail), TRUE);
    gtk_label_set_selectable(GTK_LABEL(v->detail), TRUE);
    gtk_label_set_max_width_chars(GTK_LABEL(v->detail), 42);
    gtk_box_append(GTK_BOX(side), v->detail);
    if (!v->runtime_palette && v->panel == DEBUG_PPU_REGISTERS) {
        gtk_widget_set_visible(v->canvas, FALSE);
        gtk_widget_set_visible(v->undo, FALSE);
        gtk_paned_set_position(GTK_PANED(split), 0);
        gtk_paned_set_resize_end_child(GTK_PANED(split), TRUE);
        gtk_label_set_max_width_chars(GTK_LABEL(v->detail), 90);
        gtk_widget_add_css_class(v->detail, "monospace");
    }
    v->footer = cupid_gtk_label("");
    gtk_box_append(GTK_BOX(v->root), v->footer);
    g_signal_connect(v->root, "unmap", G_CALLBACK(close_dialog), v);
    g_object_set_data_full(G_OBJECT(v->root), "cupid-ppu", v, destroy);
    refresh(v);
    return v->root;
}
#endif
