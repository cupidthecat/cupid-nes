/*
 * desktop_hex.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Hex editor grid and input. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_hex.h"
#include "desktop_internal.h"
#include "desktop_memory.h"
#include "hex_frontend.h"
#include "../debugger/memory_editor.h"

#include <stdio.h>
#include <stdlib.h>

static const Clay_Color raised = {33, 39, 54, 255}, ink = {232, 237, 247, 255};
static const Clay_Color muted = {151, 165, 190, 255}, selected = {52, 57, 85, 255};
static const Clay_Color accent = {170, 180, 255, 255}, cursor_color = {79, 79, 122, 255};

static void text(FrontendDesktopUi *ui, const char *value, unsigned size, Clay_Color color) {
    CLAY_TEXT(desktop_clay_string(ui->clay, value ? value : ""),
              CLAY_TEXT_CONFIG({.fontSize = (uint16_t)size, .textColor = color, .wrapMode = CLAY_TEXT_WRAP_NONE}));
}

static int index_of(const FrontendPanelModel *model, unsigned id) {
    for (size_t i = 0; i < model->count; ++i) if (model->controls[i].id == id) return (int)i;
    return -1;
}

static void controls(FrontendDesktopUi *ui, const FrontendPanelModel *model, const unsigned *ids, size_t count) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 6}}) {
        for (size_t i = 0; i < count; ++i) {
            int at = index_of(model, ids[i]);
            if (at < 0) continue;
            const FrontendPanelControl *c = &model->controls[at];
            bool action = c->type == FRONTEND_PANEL_ACTION;
            const char *value = c->value;
            if (c->type == FRONTEND_PANEL_CHOICE && c->selected >= 0 && (size_t)c->selected < c->item_count)
                value = c->items[c->selected];
            if (c->type == FRONTEND_PANEL_CHECKBOX) value = c->selected ? "On" : "Off";
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(1.0f / (float)count)},
                                     .layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 3}}) {
                if (!action) text(ui, c->label, 11, muted);
                Clay_ElementId hit = desktop_clay_hit(ui->clay, c->enabled ? HIT_PANEL : HIT_NONE, at, 0);
                CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(1), .height = CLAY_SIZING_FIXED(26)},
                                     .padding = {6, 6, 3, 3}, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                            .backgroundColor = ui->panel_row == at || Clay_PointerOver(hit) ? selected : raised,
                            .clip = {.horizontal = true, .vertical = true}}) {
                    text(ui, action ? c->label : value, 12, c->enabled ? ink : muted);
                }
            }
        }
    }
}

static void note(FrontendDesktopUi *ui, const char *value, Clay_Color color, float height) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(height)}}}) {
        CLAY_TEXT(desktop_clay_string(ui->clay, value ? value : ""),
                  CLAY_TEXT_CONFIG({.fontSize = 11, .textColor = color, .wrapMode = CLAY_TEXT_WRAP_WORDS}));
    }
}

static void model_note(FrontendDesktopUi *ui, const FrontendPanelModel *model, unsigned id, Clay_Color color) {
    int at = index_of(model, id);
    note(ui, at >= 0 ? model->controls[at].value : "", color, 16);
}

static void grid(FrontendDesktopUi *ui, const FrontendPanelModel *model) {
    int cursor_at = index_of(model, HEX_SELECTION);
    uint32_t cursor = cursor_at >= 0 ? (uint32_t)model->controls[cursor_at].selected : 0;
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
        for (int row = -1; row < HEX_ROWS; ++row) {
            int at = row < 0 ? -1 : index_of(model, HEX_ROW + (unsigned)row);
            const FrontendPanelControl *c = at >= 0 ? &model->controls[at] : NULL;
            uint32_t address = c ? (uint32_t)strtoul(c->label + 1, NULL, 16) : 0;
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(17)},
                                     .childGap = 8, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                          .backgroundColor = row < 0 || (row & 1) ? raised : (Clay_Color){0}}) {
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(56)}}}) {
                    text(ui, c ? c->label : "Address", 11, muted);
                }
                for (int ascii = 0; ascii < 2; ++ascii) {
                    CLAY_AUTO_ID({.layout = {.sizing = {.width = ascii ? CLAY_SIZING_FIXED(176) : CLAY_SIZING_GROW(),
                                                       .height = CLAY_SIZING_GROW()}}}) {
                        for (unsigned col = 0; col < HEX_COLUMNS; ++col) {
                            bool present = c && c->enabled && c->item_count == HEX_COLUMNS + 2 && c->items[col][0] != ' ';
                            bool marked = present && c->items[HEX_COLUMNS + 1][col] == '*';
                            bool chosen = present && ((unsigned)c->selected & (1u << col));
                            char value[5] = "";
                            if (row < 0) {
                                if (!ascii) snprintf(value, sizeof(value), "%02X", col);
                                else if (col < 5) { value[0] = "ASCII"[col]; value[1] = '\0'; }
                            } else if (present && ascii) {
                                value[0] = c->items[HEX_COLUMNS][col];
                                value[1] = '\0';
                            } else if (present) {
                                snprintf(value, sizeof(value), "%s%s", c->items[col], marked ? "*" : "");
                            }
                            Clay_ElementId hit = desktop_clay_hit(ui->clay, present ? HIT_HEX_BYTE : HIT_NONE,
                                                                  (int)(address + col), ascii);
                            CLAY(hit, {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(1.0f / HEX_COLUMNS),
                                                             .height = CLAY_SIZING_GROW()},
                                                 .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
                                        .backgroundColor = present && cursor == address + col ? cursor_color :
                                                           chosen || Clay_PointerOver(hit) ? selected : (Clay_Color){0}}) {
                                text(ui, value, ascii ? 11 : 12, row < 0 ? muted : marked ? accent : ink);
                            }
                        }
                    }
                }
            }
        }
    }
}

bool desktop_hex_layout(FrontendDesktopUi *ui, float width, float height) {
    if (width < 760 || height < 730) return false;
    FrontendPanelControl items[64];
    FrontendPanelModel model = {.controls = items, .capacity = 64};
    char error[256] = {0};
    if (!frontend_panel_snapshot(HEX_FRONTEND_PANEL, &model, error, sizeof(error))) {
        note(ui, error, muted, 28);
        return true;
    }
    desktop_memory_focus(ui, &model, 0);
    static const unsigned setup[] = {HEX_SPACE, HEX_ADDRESS, HEX_COUNT, HEX_SELECT, HEX_LIVE};
    static const unsigned actions[] = {HEX_PREV_PAGE, HEX_NEXT_PAGE, HEX_REFRESH, HEX_BASELINE, HEX_COPY, HEX_PASTE};
    static const unsigned edit[] = {HEX_BYTES, HEX_TEXT_MODE, HEX_APPLY};
    static const unsigned find[] = {HEX_PATTERN, HEX_WRAP, HEX_FIND, HEX_WATCH};
    static const unsigned files[] = {HEX_IMPORT_PATH, HEX_IMPORT, HEX_EXPORT_PATH, HEX_EXPORT};
    ui->visible_rows = HEX_ROWS;
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_GROW()},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 12}}) {
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
                text(ui, "Hex / Memory Editor", 22, ink);
            }
            if (!ui->parent) {
                Clay_ElementId close = desktop_clay_hit(ui->clay, HIT_CLOSE, 0, 0);
                CLAY(close, {.layout = {.padding = {8, 8, 3, 3}}, .backgroundColor = raised}) {
                    text(ui, "Close", 12, ink);
                }
            }
        }
        controls(ui, &model, setup, sizeof(setup) / sizeof(*setup));
        controls(ui, &model, actions, sizeof(actions) / sizeof(*actions));
        model_note(ui, &model, HEX_INFO, muted);
        grid(ui, &model);
        model_note(ui, &model, HEX_SELECTION, accent);
        model_note(ui, &model, HEX_BACKING, ink);
        controls(ui, &model, edit, sizeof(edit) / sizeof(*edit));
        controls(ui, &model, find, sizeof(find) / sizeof(*find));
        controls(ui, &model, files, sizeof(files) / sizeof(*files));
        note(ui, model.status && *model.status ? model.status :
             "Ctrl+G: address   Ctrl+F: find   Ctrl+C/V: copy/paste   Enter on a byte: edit", muted, 28);
    }
    return true;
}

static bool invoke(FrontendDesktopUi *ui, unsigned id, int value) {
    char error[256] = {0};
    bool ok = frontend_panel_action(HEX_FRONTEND_PANEL, id, NULL, value, error, sizeof(error));
    if (!ok && error[0]) desktop_copy_status(ui, error);
    return ok;
}

static void pick(FrontendDesktopUi *ui, uint32_t address, bool extend) {
    if (!invoke(ui, extend ? HEX_EXTEND : HEX_PICK, (int)address)) return;
    FrontendPanelControl items[64];
    FrontendPanelModel model = {.controls = items, .capacity = 64};
    if (!frontend_panel_snapshot(HEX_FRONTEND_PANEL, &model, NULL, 0)) return;
    int at = index_of(&model, HEX_ROW + (address % DEBUG_MEMORY_PAGE_SIZE) / HEX_COLUMNS);
    if (at >= 0) ui->panel_row = at;
}

static bool key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *event) {
    SDL_Scancode sc = event->keysym.scancode;
    bool ctrl = (event->keysym.mod & KMOD_CTRL) != 0, shift = (event->keysym.mod & KMOD_SHIFT) != 0;
    if (event->keysym.mod & (KMOD_ALT | KMOD_GUI)) return false;
    if (ctrl && (sc == SDL_SCANCODE_C || sc == SDL_SCANCODE_V)) {
        if (!event->repeat) (void)invoke(ui, sc == SDL_SCANCODE_C ? HEX_COPY : HEX_PASTE, 0);
        return true;
    }
    FrontendPanelControl items[64];
    FrontendPanelModel model = {.controls = items, .capacity = 64};
    if (!frontend_panel_snapshot(HEX_FRONTEND_PANEL, &model, NULL, 0)) return false;
    if (ctrl && (sc == SDL_SCANCODE_G || sc == SDL_SCANCODE_F)) {
        unsigned id = sc == SDL_SCANCODE_G ? HEX_ADDRESS : HEX_PATTERN;
        int at = index_of(&model, id);
        if (at >= 0) { ui->panel_row = at; desktop_start_text_edit(ui, id, items[at].value); }
        return true;
    }
    if (sc == SDL_SCANCODE_F3 && !event->repeat) { (void)invoke(ui, HEX_FIND, 0); return true; }
    if (sc == SDL_SCANCODE_TAB) {
        desktop_memory_focus(ui, &model, shift ? -1 : 1);
        return true;
    }
    int cursor_at = index_of(&model, HEX_SELECTION), space_at = index_of(&model, HEX_SPACE);
    uint32_t low, high;
    if (cursor_at < 0 || space_at < 0 || !debug_memory_bounds((DebugMemorySpace)items[space_at].selected, &low, &high))
        return false;
    uint32_t cursor = (uint32_t)items[cursor_at].selected;
    if (ctrl && sc == SDL_SCANCODE_A) {
        pick(ui, low, false);
        pick(ui, high, true);
        return true;
    }
    bool grid_focus = ui->panel_row >= 0 && (size_t)ui->panel_row < model.count && items[ui->panel_row].id >= HEX_ROW;
    if (grid_focus && sc == SDL_SCANCODE_RETURN) {
        int at = index_of(&model, HEX_BYTES);
        if (at >= 0) desktop_start_text_edit(ui, HEX_BYTES, items[at].value);
        return true;
    }
    int delta = 0;
    if (sc == SDL_SCANCODE_PAGEUP) delta = -DEBUG_MEMORY_PAGE_SIZE;
    else if (sc == SDL_SCANCODE_PAGEDOWN) delta = DEBUG_MEMORY_PAGE_SIZE;
    else if (!grid_focus) return false;
    else if (sc == SDL_SCANCODE_LEFT) delta = -1;
    else if (sc == SDL_SCANCODE_RIGHT) delta = 1;
    else if (sc == SDL_SCANCODE_UP) delta = -HEX_COLUMNS;
    else if (sc == SDL_SCANCODE_DOWN) delta = HEX_COLUMNS;
    else if (sc == SDL_SCANCODE_HOME) { pick(ui, ctrl ? low : cursor - (cursor - low) % HEX_COLUMNS, shift); return true; }
    else if (sc == SDL_SCANCODE_END) {
        uint32_t last = cursor - (cursor - low) % HEX_COLUMNS + HEX_COLUMNS - 1;
        pick(ui, ctrl || last > high ? high : last, shift);
        return true;
    } else return false;
    int64_t next = (int64_t)cursor + delta;
    pick(ui, next < low ? low : next > high ? high : (uint32_t)next, shift);
    return true;
}

bool desktop_hex_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!ui->panel_open || ui->panel_id != HEX_FRONTEND_PANEL || ui->edit_text_active || ui->capture_binding ||
        ui->settings_open || ui->choice_open || ui->info_open || ui->open_menu >= 0 || desktop_palette_visible(ui)) {
        ui->hex_selecting = false;
        return false;
    }
    if (event->type == SDL_KEYDOWN) return key(ui, &event->key);
    if (event->type == SDL_MOUSEBUTTONUP ||
        (event->type == SDL_WINDOWEVENT && event->window.event == SDL_WINDOWEVENT_FOCUS_LOST)) ui->hex_selecting = false;
    if (event->type == SDL_MOUSEBUTTONDOWN && event->button.button == SDL_BUTTON_LEFT) {
        ui->hex_selecting = false;
        const DesktopHit *hit = desktop_clay_at(ui->clay, (float)event->button.x, (float)event->button.y);
        if (!hit || hit->kind != HIT_HEX_BYTE) return false;
        pick(ui, (uint32_t)hit->index, (SDL_GetModState() & KMOD_SHIFT) != 0);
        ui->hex_selecting = true;
        return true;
    }
    if (event->type == SDL_MOUSEMOTION && ui->hex_selecting && (event->motion.state & SDL_BUTTON_LMASK)) {
        float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
        const DesktopHit *hit = desktop_clay_at(ui->clay, event->motion.x / scale, event->motion.y / scale);
        if (hit && hit->kind == HIT_HEX_BYTE) pick(ui, (uint32_t)hit->index, true);
        return true;
    }
    return false;
}
