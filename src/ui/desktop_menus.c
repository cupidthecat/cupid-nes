/* Hierarchical application menus. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_internal.h"
#include "frontend_commands.h"
#include "state_frontend.h"
#include "../debugger/debugger.h"
#include "../cheats/cheats.h"
#include <stdio.h>
#include <string.h>

enum {
    GROUP_STATES = 1,
    GROUP_RECENT,
    GROUP_SPEED,
    GROUP_PALETTE,
    GROUP_DISK,
    GROUP_TAPE,
    GROUP_BARCODE,
    GROUP_ARCADE,
    GROUP_DEBUG,
    GROUP_CHEATS,
    GROUP_REPLAY,
    GROUP_NETPLAY,
    GROUP_CAPTURE,
    GROUP_GRAPHICS,
    GROUP_STORAGE,
    GROUP_MUSIC,
    GROUP_OTHER,
    GROUP_DEBUG_EXECUTION,
    GROUP_SCRIPT,
    GROUP_PPU
};

static const char *const names[] = {
    "",          "Save states",    "Recent games",  "Speed",     "Palette",      "Disk system",
    "Tape",      "Barcode reader", "Arcade inputs", "Debugging", "Cheats",       "Rewind and movies",
    "Netplay",   "Capture",        "HD graphics",   "Storage",   "Music player", "Other tools",
    "Execution", "Scripts", "PPU tools"};

static unsigned group(const DesktopMenuItem *item, int menu, unsigned parent) {
    unsigned id = item->id;
    if (parent == GROUP_DEBUG) {
        if (desktop_ppu_panel(id)) return GROUP_PPU;
        if (id == DEBUGGER_FRONTEND_COMMAND || (id >= DEBUGGER_STEP_INTO_COMMAND && id <= DEBUGGER_PAUSE_COMMAND)) {
            return GROUP_DEBUG_EXECUTION;
        }
        if (id == DEBUGGER_LUA_FRONTEND_COMMAND || id == DEBUGGER_LUA_LOAD_COMMAND ||
            id == DEBUGGER_LUA_UNLOAD_COMMAND) {
            return GROUP_SCRIPT;
        }
        return 0;
    }
    if (parent) {
        return 0;
    }
    if (menu == 0) {
        if (item->kind == 2) {
            return GROUP_RECENT;
        }
        if (id >= STATE_COMMAND_SAVE_SLOT && id <= STATE_COMMAND_LOAD_FILE) {
            return GROUP_STATES;
        }
    }
    if (menu == 1 && id >= FRONTEND_COMMAND_FAST_FORWARD_HOLD && id <= FRONTEND_COMMAND_SPEED_DOUBLE) {
        return GROUP_SPEED;
    }
    if (menu == 2 && id >= 0x1B00 && id <= 0x1B02) {
        return GROUP_PALETTE;
    }
    if (menu == 4) {
        if (id == DEVICE_PANEL_DISK || id == DEVICE_COMMAND_DISK_TOGGLE || id == DEVICE_COMMAND_DISK_NEXT) {
            return GROUP_DISK;
        }
        if (id == DEVICE_PANEL_TAPE || (id >= DEVICE_COMMAND_TAPE_PLAY && id <= DEVICE_COMMAND_TAPE_STOP)) {
            return GROUP_TAPE;
        }
        if (id == DEVICE_PANEL_BARCODE || id == DEVICE_COMMAND_BARCODE_SCAN) {
            return GROUP_BARCODE;
        }
        if (id == DEVICE_PANEL_VS) {
            return GROUP_ARCADE;
        }
    }
    if (menu == 5) {
        if (id == CHEATS_FRONTEND_COMMAND || id == CHEATS_FRONTEND_PANEL || id == CHEATS_ADD_COMMAND ||
            id == CHEATS_TOGGLE_COMMAND) {
            return GROUP_CHEATS;
        }
        if (desktop_ppu_panel(id) || (id >= 0x1340 && id <= 0x1382)) {
            return GROUP_DEBUG;
        }
        if (id >= 0x1700 && id < 0x1800) {
            return GROUP_REPLAY;
        }
        if (id >= 0x1900 && id < 0x1A00) {
            return GROUP_NETPLAY;
        }
        if (id >= 0x1600 && id < 0x1700) {
            return GROUP_CAPTURE;
        }
        if (id >= 0x1800 && id < 0x1900) {
            return GROUP_GRAPHICS;
        }
        if ((id >= 0x1310 && id <= 0x1313) || (id >= 0x1C00 && id < 0x1D00)) {
            return GROUP_STORAGE;
        }
        if ((id >= 0x1500 && id < 0x1600) || strstr(item->label, "Music") || strstr(item->label, "music")) {
            return GROUP_MUSIC;
        }
        return GROUP_OTHER;
    }
    return 0;
}

int desktop_menu_level(FrontendDesktopUi *ui, int depth, DesktopMenuItem out[128]) {
    DesktopMenuItem rows[128];
    int count = desktop_menu_all(ui, rows);
    unsigned parent = 0;
    for (int d = 0; d < depth; ++d) {
        unsigned path = ui->menu_path[d];
        if (path & 0x80000000u) {
            continue;
        } else {
            int kept = 0;
            for (int i = 0; i < count; ++i) {
                if (group(&rows[i], ui->open_menu, parent) == path) {
                    rows[kept++] = rows[i];
                }
            }
            count = kept;
            parent = path;
        }
    }
    int used = 0;
    for (int i = 0; i < count; ++i) {
        unsigned section = group(&rows[i], ui->open_menu, parent);
        if (!section) {
            out[used++] = rows[i];
        } else {
            bool present = false;
            for (int j = 0; j < used; ++j) {
                if (out[j].kind == 7 && out[j].id == section) {
                    present = true;
                }
            }
            if (!present) {
                out[used] = (DesktopMenuItem){.id = section, .kind = 7, .enabled = true};
                snprintf(out[used++].label, sizeof(out[0].label), "%s", names[section]);
            }
        }
    }
    /* Page ranges apply to the grouped level, not to its underlying commands. */
    int first_page = depth;
    while (first_page > 0 && (ui->menu_path[first_page - 1] & 0x80000000u)) {
        --first_page;
    }
    for (int d = first_page; d < depth; ++d) {
        unsigned path = ui->menu_path[d];
        int start = (int)((path >> 8) & 255), length = (int)(path & 255);
        if (start >= used) {
            return 0;
        }
        if (length > used - start) {
            length = used - start;
        }
        memmove(out, out + start, (size_t)length * sizeof(*out));
        used = length;
    }
    if (used <= 10) {
        return used;
    }
    /* Large recent/custom lists form small nested ranges, never tall scrolling menus. */
    memcpy(rows, out, (size_t)used * sizeof(*rows));
    int span = used > 80 ? 16 : 8, pages = 0;
    for (int start = 0; start < used; start += span) {
        int length = used - start < span ? used - start : span;
        out[pages] = (DesktopMenuItem){
            .id = 0x80000000u | ((unsigned)start << 8) | (unsigned)length, .kind = 7, .enabled = true};
        snprintf(out[pages++].label, sizeof(out[0].label), "%s %d–%d", parent ? names[parent] : "Items", start + 1,
                 start + length);
    }
    return pages;
}

int desktop_menu_items(FrontendDesktopUi *ui, DesktopMenuItem items[128]) {
    return desktop_menu_level(ui, ui->menu_depth, items);
}
