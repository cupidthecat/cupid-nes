/* Desktop interface built with Clay. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../rom/fds.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "desktop_internal.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "palette_tool.h"
#include <stdio.h>
#include <string.h>
static const Clay_Color surface = {24, 28, 39, 255}, raised = {33, 39, 54, 255}, selected = {52, 57, 85, 255};
static const Clay_Color ink = {232, 237, 247, 255}, muted = {151, 165, 190, 255}, accent = {170, 180, 255, 255};
static const char *const menus[] = {"File", "Emulation", "View", "Audio", "Media", "Tools", "Help"};
static const char *const categories[] = {"General",     "Emulation",        "Video",           "Audio",
                                         "Controllers", "Media / firmware", "Files / storage", "Advanced hardware"};

static void label(FrontendDesktopUi *ui, const char *text, int size, Clay_Color color) {
    CLAY_TEXT(desktop_clay_string(ui->clay, text), CLAY_TEXT_CONFIG({.fontSize = (uint16_t)size, .textColor = color}));
}

static void line(FrontendDesktopUi *ui, const char *text, int size, Clay_Color color) {
    CLAY_TEXT(desktop_clay_string(ui->clay, text),
              CLAY_TEXT_CONFIG({.fontSize = (uint16_t)size, .textColor = color, .wrapMode = CLAY_TEXT_WRAP_NONE}));
}

static void spacer(void) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
    }
}

static void button(FrontendDesktopUi *ui, const char *text, int kind, int index, int direction, bool active,
                   bool enabled) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? kind : HIT_NONE, index, direction);
    bool hover = Clay_PointerOver(id);
    CLAY(id, {.layout = {.padding = {12, 12, 8, 8}, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
              .backgroundColor = active             ? selected
                                 : hover && enabled ? (Clay_Color){47, 55, 75, 255}
                                                    : raised,
              .cornerRadius = CLAY_CORNER_RADIUS(5)}) {
        line(ui, text, 14, enabled ? active ? accent : ink : muted);
    }
}

static void heading(FrontendDesktopUi *ui, const char *title, const char *subtitle) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                             .childAlignment = {.y = CLAY_ALIGN_Y_CENTER},
                             .childGap = 12}}) {
        CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM,
                                 .childGap = 5,
                                 .sizing = {.width = CLAY_SIZING_GROW()}}}) {
            label(ui, title, 24, ink);
            if (subtitle) {
                label(ui, subtitle, 13, muted);
            }
        }
        button(ui, "Close", HIT_CLOSE, 0, 0, false, true);
    }
}

static int scroll_start(int row, int count, int *start, int visible) {
    if (row < *start) {
        *start = row;
    }
    if (row >= *start + visible) {
        *start = row - visible + 1;
    }
    if (*start > count - visible) {
        *start = count - visible;
    }
    if (*start < 0) {
        *start = 0;
    }
    return *start;
}

static void scroll_footer(FrontendDesktopUi *ui, int count, int start) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                             .childAlignment = {.y = CLAY_ALIGN_Y_CENTER},
                             .childGap = 6}}) {
        char text[64];
        snprintf(text, sizeof(text), "%d–%d of %d", start + 1,
                 start + ui->visible_rows < count ? start + ui->visible_rows : count, count);
        label(ui, text, 12, muted);
        spacer();
        button(ui, "↑", HIT_SCROLL, 0, -1, false, start > 0);
        button(ui, "↓", HIT_SCROLL, 0, 1, false, start + ui->visible_rows < count);
    }
}

static void settings(FrontendDesktopUi *ui, float height) {
    heading(ui, "Settings", "Changes take effect when you choose Apply or OK.");
    ui->visible_rows = (int)((height - 260) / 40);
    if (ui->visible_rows < 2) {
        ui->visible_rows = 2;
    }
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_GROW()}, .childGap = 16}}) {
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(150), .height = CLAY_SIZING_GROW()},
                                 .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                 .childGap = 5}}) {
            for (int i = 0; i < 8; ++i) {
                Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_CATEGORY, i, 0);
                CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .padding = {10, 8, 7, 7}},
                          .backgroundColor = i == ui->settings_category ? selected : surface,
                          .cornerRadius = CLAY_CORNER_RADIUS(5)}) {
                    label(ui, categories[i], 12, i == ui->settings_category ? accent : muted);
                }
            }
        }
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_GROW()},
                                 .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                 .childGap = 6}}) {
            int count = desktop_setting_rows(ui),
                start = scroll_start(ui->settings_row, count, &ui->settings_scroll, ui->visible_rows);
            for (int i = start; i < count && i < start + ui->visible_rows; ++i) {
                char name[96], value[256];
                desktop_setting_text(ui, i, name, sizeof(name), value, sizeof(value));
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(34)},
                                         .childGap = 5,
                                         .childAlignment = {.y = CLAY_ALIGN_Y_CENTER},
                                         .padding = {8, 0, 0, 0}},
                              .backgroundColor = i == ui->settings_row ? selected : surface,
                              .cornerRadius = CLAY_CORNER_RADIUS(4)}) {
                    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(32)},
                                             .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                                  .clip = {.horizontal = true, .vertical = true}}) {
                        line(ui, name, 13, ink);
                    }
                    button(ui, "−", HIT_SETTING, i, -1, false, true);
                    Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_SETTING, i, 1);
                    CLAY(id,
                         {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(.32f), .height = CLAY_SIZING_FIXED(32)},
                                     .padding = {6, 6, 8, 6}},
                          .clip = {.horizontal = true, .vertical = true},
                          .backgroundColor = raised,
                          .cornerRadius = CLAY_CORNER_RADIUS(4)}) {
                        line(ui, value, 13, accent);
                    }
                    button(ui, "+", HIT_SETTING, i, 1, false, true);
                }
            }
            scroll_footer(ui, count, start);
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                                     .childGap = 6,
                                     .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
                label(ui, "Arrows adjust • Ctrl+Tab changes category", 12, muted);
                spacer();
                bool browse = (ui->settings_category == 5 &&
                               (ui->settings_row <= 3 || ui->settings_row == 9 || ui->settings_row == 10)) ||
                              (ui->settings_category == 6 && ui->settings_row >= 1 && ui->settings_row <= 4);
                button(ui, "Browse…", HIT_BROWSE, 0, 0, false, browse);
            }
        }
    }
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 8}}) {
        const char *names[] = {"Apply", "OK", "Cancel", "Defaults", "Reset all"};
        for (int i = 0; i < 5; ++i) {
            button(ui, names[i], HIT_SETTINGS_BUTTON, i, 0, ui->settings_focus == 1 ? ui->settings_button == i : i == 1,
                   true);
        }
    }
    if (ui->staged.cli_overrides) {
        label(ui, "Explicit launch overrides remain effective.", 12, muted);
    }
}

static void panel(FrontendDesktopUi *ui, float height) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    char error[256] = {0};
    FrontendPanelInfo info = {0};
    if (!frontend_panel_snapshot(ui->panel_id, &model, error, sizeof(error))) {
        heading(ui, "Tool unavailable", error);
        return;
    }
    frontend_panel_get(ui->panel_id, &info);
    heading(ui, info.title, info.category);
    if (ui->panel_row >= (int)model.count) {
        ui->panel_row = model.count ? (int)model.count - 1 : 0;
    }
    ui->visible_rows = (int)((height - 224) / 40);
    if (ui->visible_rows < 2) {
        ui->visible_rows = 2;
    }
    int start = scroll_start(ui->panel_row, (int)model.count, &ui->panel_scroll, ui->visible_rows);
    char detail[1024] = "";
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_GROW()},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM,
                             .childGap = 6}}) {
        for (int i = start; i < (int)model.count && i < start + ui->visible_rows; ++i) {
            FrontendPanelControl *c = &controls[i];
            const char *value = c->value ? c->value : "";
            bool choice = c->type == FRONTEND_PANEL_CHOICE || c->type == FRONTEND_PANEL_LIST;
            if (choice && c->selected >= 0 && (size_t)c->selected < c->item_count) {
                value = c->items[c->selected];
            }
            if (c->type == FRONTEND_PANEL_CHECKBOX) {
                value = c->selected ? "On" : "Off";
            }
            if (i == ui->panel_row) {
                snprintf(detail, sizeof(detail), "%s: %s", c->label, value);
            }
            Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_PANEL, i, 0);
            CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(34)},
                                 .padding = {10, 0, 0, 0},
                                 .childGap = 6,
                                 .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                      .backgroundColor = i == ui->panel_row ? selected : raised,
                      .cornerRadius = CLAY_CORNER_RADIUS(4)}) {
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(32)},
                                         .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                              .clip = {.horizontal = true, .vertical = true}}) {
                    line(ui, c->label, 14, c->enabled ? ink : muted);
                }
                if (choice) {
                    button(ui, "−", HIT_PANEL, i, -1, false, c->enabled);
                }
                CLAY_AUTO_ID(
                    {.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(.40f), .height = CLAY_SIZING_FIXED(30)},
                                .padding = {5, 5, 7, 5}},
                     .clip = {.horizontal = true, .vertical = true}}) {
                    line(ui, value, 13, accent);
                }
                if (!c->read_only) {
                    button(ui,
                           choice                               ? "+"
                           : c->type == FRONTEND_PANEL_TEXT     ? "Edit"
                           : c->type == FRONTEND_PANEL_CHECKBOX ? "Toggle"
                                                                : "Run",
                           HIT_PANEL, i, 1, false, c->enabled);
                }
            }
        }
        scroll_footer(ui, (int)model.count, start);
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(50)},
                                 .padding = CLAY_PADDING_ALL(8)},
                      .backgroundColor = raised,
                      .cornerRadius = CLAY_CORNER_RADIUS(5),
                      .clip = {.horizontal = true, .vertical = true}}) {
            label(ui, detail, 13, accent);
        }
    }
    label(ui, model.status ? model.status : "Arrows move or adjust • Enter selects • Escape closes", 13, muted);
}

static void information(FrontendDesktopUi *ui) {
    heading(ui, ui->log_open ? "Recent messages" : "Game information", NULL);
    if (ui->log_open) {
        for (unsigned i = 0; i < ui->log_count; ++i) {
            label(ui, ui->log_lines[i], 14, muted);
        }
        return;
    }
    char line[256];
    snprintf(line, sizeof(line), "Metadata: %s", rom_metadata_source_name());
    label(ui, line, 16, ink);
    snprintf(line, sizeof(line), "Region: %s    •    Console: %s", nes_region_name(nes_timing()->region),
             nes_console_model_name());
    label(ui, line, 16, ink);
    if (!rom_is_fds() && !rom_is_studybox() && !rom_is_nsf()) {
        snprintf(line, sizeof(line), "Mapper: %d / submapper %u", rom_mapper_number(&ines_header),
                 (ines_header.flags7 & 0x0Cu) == 0x08u ? (unsigned)(ines_header.prg_ram_size >> 4) : 0u);
        label(ui, line, 16, ink);
    }
    if (ui->sessions && ui->sessions->session && ui->sessions->session->active) {
        label(ui, ui->sessions->session->current.path, 14, accent);
    }
}

static void palette(FrontendDesktopUi *ui) {
    heading(ui, "Palette editor", "Select a swatch. Changes appear immediately.");
    uint32_t colors[64];
    ppu_palette_get(colors);
    CLAY_AUTO_ID(
        {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        for (int row = 0; row < 4; ++row) {
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 4}}) {
                for (int col = 0; col < 16; ++col) {
                    int index = row * 16 + col;
                    uint32_t c = colors[index];
                    Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_COLOR, index, 0);
                    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(24)},
                                         .padding = CLAY_PADDING_ALL(2)},
                              .backgroundColor = index == ui->palette_index ? ink : surface,
                              .cornerRadius = CLAY_CORNER_RADIUS(3)}) {
                        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_GROW()}},
                                      .backgroundColor = {(float)((c >> 16) & 255), (float)((c >> 8) & 255),
                                                          (float)(c & 255), 255}}) {
                        }
                    }
                }
            }
        }
    }
    uint32_t c = colors[ui->palette_index];
    char text[64];
    snprintf(text, sizeof(text), "Color $%02X  ·  #%06X", ui->palette_index, (unsigned)(c & 0xFFFFFF));
    label(ui, text, 18, ink);
    const char *channels[] = {"Red", "Green", "Blue"};
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}, .childGap = 14}}) {
        for (int i = 0; i < 3; ++i) {
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                                     .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                     .childGap = 4}}) {
                int value = (c >> (16 - i * 8)) & 255;
                snprintf(text, sizeof(text), "%s  %d", channels[i], value);
                label(ui, text, 14, ink);
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                                         .childGap = 6,
                                         .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
                    button(ui, "−", HIT_CHANNEL, i, -1, false, true);
                    Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_CHANNEL, i, 0);
                    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(22)},
                                         .padding = CLAY_PADDING_ALL(3)},
                              .backgroundColor = raised,
                              .cornerRadius = CLAY_CORNER_RADIUS(4)}) {
                        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_PERCENT(value / 255.0f),
                                                            .height = CLAY_SIZING_GROW()}},
                                      .backgroundColor = accent,
                                      .cornerRadius = CLAY_CORNER_RADIUS(3)}) {
                        }
                    }
                    button(ui, "+", HIT_CHANNEL, i, 1, false, true);
                }
            }
        }
    }
    CLAY_AUTO_ID({.layout = {.childGap = 8}}) {
        button(ui, "Load palette…", HIT_COMMAND, 0x1B01, 0, false, true);
        button(ui, "Reset palette", HIT_COMMAND, 0x1B02, 0, false, true);
    }
}

static void welcome(FrontendDesktopUi *ui) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_GROW()},
                             .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER},
                             .padding = CLAY_PADDING_ALL(24)},
                  .backgroundColor = {14, 18, 27, 255}}) {
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(0, 620)},
                                 .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                 .childGap = 14,
                                 .padding = CLAY_PADDING_ALL(28)},
                      .backgroundColor = surface,
                      .cornerRadius = CLAY_CORNER_RADIUS(12)}) {
            label(ui, "CUPID NES", 13, accent);
            label(ui, "Ready when you are.", 32, ink);
            label(ui, "Open a game or drop a file into this window.", 16, muted);
            CLAY_AUTO_ID({.layout = {.childGap = 8}}) {
                button(ui, "Open game…", HIT_COMMAND, FRONTEND_COMMAND_OPEN, 0, true, true);
                button(ui, "Settings", HIT_COMMAND, FRONTEND_COMMAND_SETTINGS, 0, false, true);
            }
            size_t count = frontend_session_recent_count(ui->idle_session);
            if (count) {
                label(ui, "RECENT GAMES", 12, muted);
            }
            for (size_t i = 0; i < count && i < 4; ++i) {
                const FrontendImageRequest *recent = frontend_session_recent(ui->idle_session, i);
                const char *name = recent->archive_member[0] ? recent->archive_member : recent->path;
                const char *base = name;
                for (const char *p = name; *p; ++p) {
                    if (*p == '/' || *p == '\\') {
                        base = p + 1;
                    }
                }
                button(ui, base, HIT_RECENT, (int)i, 0, false, true);
            }
        }
    }
}

void desktop_layout(FrontendDesktopUi *ui, const char *title, const char *region, const char *run_state) {
    if (!ui->clay) {
        return;
    }
    int width, height;
    SDL_GetWindowSize(ui->window, &width, &height);
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    float w = width / scale, h = height / scale;
    desktop_clay_begin(ui->clay, w, h, scale);
    CLAY(CLAY_ID("desktop"), {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(w), .height = CLAY_SIZING_FIXED(h)},
                                         .layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
        if (!ui->parent) {
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(32)},
                                     .padding = {10, 10, 0, 0},
                                     .childGap = 2,
                                     .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                          .backgroundColor = surface}) {
                for (int i = 0; i < 7; ++i) {
                    button(ui, menus[i], HIT_MENU, i, 0, ui->open_menu == i, true);
                }
            }
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(48)},
                                     .padding = {12, 12, 7, 7},
                                     .childGap = 8,
                                     .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                          .backgroundColor = surface,
                          .border = {.color = raised, .width = {.bottom = 1}}}) {
                const char *names[] = {"Open", "Pause", "Reset", "Settings"};
                unsigned commands[] = {FRONTEND_COMMAND_OPEN, FRONTEND_COMMAND_PAUSE, FRONTEND_COMMAND_SOFT_RESET,
                                       FRONTEND_COMMAND_SETTINGS};
                for (int i = 0; i < 4; ++i) {
                    FrontendCommandInfo info = {0};
                    bool enabled = frontend_command_get(commands[i], &info) && info.enabled;
                    button(ui, i == 1 && info.checked ? "Resume" : names[i], HIT_COMMAND, (int)commands[i], 0, i == 0,
                           enabled);
                }
                spacer();
                label(ui, "CUPID", 14, muted);
            }
            if (!title || !*title) {
                welcome(ui);
            } else {
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_GROW()}}}) {
                }
            }
        } else {
            CLAY_AUTO_ID({.layout = {.sizing = {.height = CLAY_SIZING_GROW()}}}) {
            }
        }
        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(28)},
                                 .padding = {12, 12, 6, 6},
                                 .childGap = 12,
                                 .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                      .backgroundColor = surface,
                      .clip = {.horizontal = true, .vertical = true}}) {
            char status[512];
            bool message = ui->status[0] && SDL_GetTicks() < ui->status_until;
            snprintf(status, sizeof(status), "%.100s  •  %s  •  %s%s%s%s", title && *title ? title : "No game loaded",
                     region ? region : "Ready", run_state ? run_state : "Idle",
                     ui->capture && ui->capture->session.info.recording ? "  •  Recording" : "",
                     ui->execution && nes_movie_mode(ui->execution->movie) != NES_MOVIE_IDLE ? "  •  Movie active" : "",
                     fds_active() ? fds_loading_fast_forward() ? "  •  Disk loading"
                                    : fds_disk_dirty()         ? "  •  Disk modified"
                                                               : "  •  Disk"
                                  : "");
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
                label(ui, message ? ui->status : status, 12, message ? accent : muted);
            }
            if (ui->settings && ui->settings->show_fps && !message) {
                snprintf(status, sizeof(status), "%.1f FPS", ui->fps);
                label(ui, status, 12, accent);
            }
        }
        if (ui->open_menu >= 0) {
            DesktopMenuItem items[128];
            int count = desktop_menu_items(ui, items), visible = (int)((h - 120) / 34);
            scroll_start(ui->menu_row, count, &ui->menu_scroll, visible);
            float x = 10 + ui->open_menu * 70;
            if (x + 440 > w) {
                x = w - 440;
            }
            CLAY_AUTO_ID({.floating = {.attachTo = CLAY_ATTACH_TO_ROOT, .offset = {x, 32}, .zIndex = 1},
                          .layout = {.sizing = {.width = CLAY_SIZING_FIXED(430)},
                                     .padding = CLAY_PADDING_ALL(6),
                                     .childGap = 2,
                                     .layoutDirection = CLAY_TOP_TO_BOTTOM},
                          .backgroundColor = raised,
                          .cornerRadius = CLAY_CORNER_RADIUS(6)}) {
                for (int i = ui->menu_scroll; i < count && i < ui->menu_scroll + visible; ++i) {
                    Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_MENU_ROW, i, 0);
                    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(32)},
                                         .padding = {8, 8, 8, 8},
                                         .childGap = 12},
                              .backgroundColor = ui->menu_row == i || Clay_PointerOver(id) ? selected : raised,
                              .cornerRadius = CLAY_CORNER_RADIUS(4)}) {
                        CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}},
                                      .clip = {.horizontal = true, .vertical = true}}) {
                            line(ui, items[i].label, 14, items[i].enabled ? ink : (Clay_Color){105, 115, 135, 255});
                        }
                        line(ui, items[i].enabled ? items[i].shortcut : "Unavailable", 12, muted);
                    }
                }
                if (count > visible) {
                    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()},
                                             .childGap = 6,
                                             .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
                        label(ui, "Scroll for more", 12, muted);
                        spacer();
                        button(ui, "↑", HIT_SCROLL, 0, -1, false, ui->menu_scroll > 0);
                        button(ui, "↓", HIT_SCROLL, 0, 1, false, ui->menu_scroll + visible < count);
                    }
                }
            }
        }
        if (ui->settings_open || ui->panel_open || ui->info_open || desktop_palette_visible(ui)) {
            desktop_clay_block(ui->clay);
            float mw = ui->parent ? w : w - 32;
            if (!ui->parent && mw > 1000) {
                mw = 1000;
            }
            float mh = ui->parent ? h - 28 : h - 64;
            if (!ui->parent && mh > 760) {
                mh = 760;
            }
            CLAY_AUTO_ID({.floating = {.attachTo = CLAY_ATTACH_TO_ROOT, .zIndex = 2},
                          .layout = {.sizing = {.width = CLAY_SIZING_FIXED(w), .height = CLAY_SIZING_FIXED(h - 28)},
                                     .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
                          .backgroundColor = {4, 7, 13, 175}}) {
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(mw), .height = CLAY_SIZING_FIXED(mh)},
                                         .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                         .padding = CLAY_PADDING_ALL(20),
                                         .childGap = 14},
                              .backgroundColor = surface,
                              .cornerRadius = CLAY_CORNER_RADIUS(10),
                              .clip = {.horizontal = true, .vertical = true}}) {
                    if (ui->settings_open) {
                        settings(ui, mh);
                    } else if (ui->panel_open) {
                        panel(ui, mh);
                    } else if (ui->info_open) {
                        information(ui);
                    } else {
                        palette(ui);
                    }
                }
            }
        }
        if (ui->edit_text_active || ui->capture_binding) {
            desktop_clay_block(ui->clay);
            CLAY_AUTO_ID({.floating = {.attachTo = CLAY_ATTACH_TO_ROOT, .zIndex = 3},
                          .layout = {.sizing = {.width = CLAY_SIZING_FIXED(w), .height = CLAY_SIZING_FIXED(h - 28)},
                                     .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER},
                                     .padding = CLAY_PADDING_ALL(32)},
                          .backgroundColor = {4, 7, 13, 210}}) {
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(0, 760)},
                                         .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                         .childGap = 16,
                                         .padding = CLAY_PADDING_ALL(24)},
                              .backgroundColor = surface,
                              .cornerRadius = CLAY_CORNER_RADIUS(10)}) {
                    label(ui, ui->capture_binding ? "Set binding" : "Edit value", 24, ink);
                    label(ui,
                          ui->capture_binding ? ui->capture_gamepad
                                                    ? "Press a controller button. Backspace clears the binding."
                                                    : "Press a key combination. Backspace clears the binding."
                                              : "Type to replace • Ctrl+A selects all • Enter saves • Escape cancels",
                          14, muted);
                    if (ui->edit_text_active) {
                        CLAY_AUTO_ID(
                            {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(100)},
                                        .padding = CLAY_PADDING_ALL(12)},
                             .backgroundColor = ui->edit_select_all ? selected : raised,
                             .cornerRadius = CLAY_CORNER_RADIUS(5),
                             .clip = {.horizontal = true, .vertical = true}}) {
                            label(ui, ui->edit_text, 16, accent);
                        }
                    }
                    CLAY_AUTO_ID({.layout = {.childGap = 8}}) {
                        if (ui->edit_text_active) {
                            button(ui, "Save value", HIT_EDIT_OK, 0, 0, true, true);
                        }
                        button(ui, "Cancel", HIT_EDIT_CANCEL, 0, 0, false, true);
                    }
                }
            }
        }
    }
    desktop_clay_end(ui->clay);
}
