/*
 * gtk_layout.c - Window dimensions and control groups for each desktop tool
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_layout.h"
#include "memory_search_frontend.h"
#include "watch_frontend.h"
#include "hex_frontend.h"
#include "debug_frontend.h"
#include "../debugger/debugger.h"
#include "../cheats/cheats.h"

CupidGtkLayout cupid_gtk_layout(unsigned id) {
    static const CupidGtkLayout layouts[] = {{0x1310, 560, 420, true, "State files and slots"},
                                             {DEBUGGER_FRONTEND_PANEL, 1100, 740, false, "CPU and breakpoints"},
                                             {DEBUGGER_VIEWERS_FRONTEND_PANEL, 860, 600, false, "Memory inspection"},
                                             {DEBUGGER_LUA_FRONTEND_PANEL, 800, 560, true, "Lua script"},
                                             {MEMORY_SEARCH_PANEL, 1060, 720, false, "Search criteria"},
                                             {CHEAT_FINDER_PANEL, 1060, 720, false, "Cheat search"},
                                             {WATCH_FRONTEND_PANEL, 1000, 650, true, "Watch properties"},
                                             {HEX_FRONTEND_PANEL, 1040, 700, false, "Memory operations"},
                                             {0x1389, 980, 700, false, "Assembly"},
                                             {CHEATS_FRONTEND_PANEL, 780, 560, true, "Cheat properties"},
                                             {CHEATS_GAME_GENIE_PANEL, 460, 440, true, "Code converter"},
                                             {0x1500, 650, 480, true, "Track playback"},
                                             {0x1600, 620, 510, true, "Recording"},
                                             {0x1700, 700, 530, true, "Movie playback and recording"},
                                             {0x1750, 1140, 760, false, "Input timeline"},
                                             {0x1800, 720, 590, true, "HD packs"},
                                             {0x1900, 500, 480, true, "Connection"},
                                             {0x1a00, 500, 400, true, "Disk drive"},
                                             {0x1a01, 530, 430, true, "Tape transport"},
                                             {0x1a02, 440, 260, true, "Barcode"},
                                             {0x1a03, 540, 420, true, "Arcade cabinet"},
                                             {0x1c00, 660, 410, true, "Storage location"},
                                             {0x1d00, 850, 650, false, "Pattern tables"},
                                             {0x1d01, 960, 650, false, "Nametables"},
                                             {0x1d02, 820, 600, false, "Sprites"},
                                             {0x1d03, 560, 480, true, "PPU registers"},
                                             {0x1d04, 920, 620, false, "VRAM"},
                                             {0x1d05, 680, 540, false, "Tile editor"},
                                             {0x1d06, 760, 510, false, "Palette"},
                                             {0x2400, 900, 620, false, "Code coverage"},
                                             {0x2401, 900, 620, true, "CPU profiling"},
                                             {0x2402, 960, 640, true, "Event filters"},
                                             {0x2403, 680, 500, true, "Stack navigation"},
                                             {0x2404, 880, 620, false, "Symbol files"},
                                             {0x2405, 920, 650, false, "Source navigation"},
                                             {0x2406, 800, 560, true, "Reference search"},
                                             {0x2407, 800, 580, false, "Text extraction"},
                                             {0x2408, 940, 650, true, "Trace recording"},
                                             {0x2500, 850, 600, true, "Container file"},
                                             {0x2501, 690, 590, true, "Movie preferences"},
                                             {0x2502, 540, 430, true, "Video encoding"},
                                             {0x2600, 760, 620, true, "Game overrides"},
                                             {0x2640, 560, 380, true, "Update check"},
                                             {0x2680, 800, 600, true, "Command-line reference"},
                                             {0x26a0, 580, 430, true, "Game recovery"},
                                             {0x2700, 850, 590, true, "History playback"},
                                             {0x2720, 760, 530, true, "Frame measurements"},
                                             {0x2740, 800, 580, false, "HD draft"},
                                             {0x2780, 650, 510, true, "Shader configuration"},
                                             {0x27c0, 560, 430, true, "Audio output"},
                                             {0x2800, 590, 340, true, "Movie conversion"},
                                             {0x2900, 840, 600, false, "Database selection"},
                                             {0x2a00, 1060, 440, true, "Keyboard"},
                                             {0x2b00, 700, 570, true, "Cartridge image"},
                                             {0x2c00, 540, 400, true, "CPU overclock"}};
    for (unsigned i = 0; i < G_N_ELEMENTS(layouts); ++i) {
        if (layouts[i].id == id) {
            return layouts[i];
        }
    }
    return (CupidGtkLayout){id, 900, 620, false, "Options"};
}

const char *cupid_gtk_control_group(unsigned panel, unsigned id) {
    if (panel == CHEATS_GAME_GENIE_PANEL) {
        return id == 1 || id >= 5 ? "Game Genie code" : "Decoded values";
    }
    if (panel == CHEATS_FRONTEND_PANEL) {
        return id >= 10 ? "Cheat file" : "Selected cheat";
    }
    if (panel == MEMORY_SEARCH_PANEL || panel == CHEAT_FINDER_PANEL) {
        if (id <= 6) {
            return "Memory range and format";
        }
        if (id <= 13) {
            return "Compare values";
        }
        if (id >= 17 && id <= 24) {
            return "Selected address";
        }
        return "Results";
    }
    if (panel == WATCH_FRONTEND_PANEL) {
        if (id <= 8) {
            return "Watch properties";
        }
        if (id >= 15 && id <= 17) {
            return "Export";
        }
        return "Watch list";
    }
    if (panel == HEX_FRONTEND_PANEL) {
        if (id <= HEX_NEXT_PAGE) {
            return "Address and display";
        }
        if (id <= HEX_PASTE) {
            return "Edit selection";
        }
        if (id <= HEX_WRAP) {
            return "Find bytes";
        }
        if (id <= HEX_EXPORT) {
            return "Import and export";
        }
        return "Selection";
    }
    if (panel == DEBUGGER_FRONTEND_PANEL) {
        if (id <= DEBUG_CONTROL_STEP_OUT) {
            return "CPU";
        }
        if (id <= DEBUG_CONTROL_BREAK_REMOVE) {
            return "Breakpoints";
        }
        return "Trace";
    }
    if (panel == 0x2b00) {
        if (id < 100) {
            return "Image file";
        }
        if (id < 108) {
            return "ROM and mapper";
        }
        if (id < 112 || id == 119) {
            return "RAM sizes";
        }
        return "Console and timing";
    }
    if (panel == 0x1900) {
        return id >= 0x1920 ? "Player assignments" : "Connection";
    }
    if (panel == 0x1600) {
        return id <= 3 || (id >= 7 && id <= 9) ? "Capture files" : id < 0x1600 ? "Output options" : "Recording";
    }
    if (panel == 0x1700) {
        return id <= 5 ? "Rewind and run-ahead" : "Input movie";
    }
    if (panel == 0x1800) {
        return id < 0x1806 ? "Installed packs" : id < 0x180b || id == 0x180e ? "Import and export" : "Frame capture";
    }
    if (panel == 0x2501) {
        return id >= 20 || (id >= 5 && id <= 9) ? "Subtitles and overlays"
               : id >= 10                       ? "Backups"
                                                : "Playback and recording";
    }
    if (panel == 0x2600) {
        return id < 100 ? "Game profile" : "Overrides";
    }
    return cupid_gtk_layout(panel).fields;
}

const char *cupid_gtk_setting_group(int category, int row) {
    switch (category) {
    case 0:
        return row < 3 ? "Startup" : row < 5 ? "Focus" : row < 8 ? "Window" : "PPU viewers";
    case 1:
        return row < 3 ? "Speed and timing" : "Rewind and latency";
    case 2:
        if (row < 7) {
            return "Display";
        }
        if (row < 19) {
            return "Overscan";
        }
        if (row < 25) {
            return "Filters";
        }
        return "NTSC picture";
    case 3:
        return row < 5 ? "Output" : row < 15 ? "Channel mixer" : "Expansion mixer";
    case 4:
        if (row < 6) {
            return "Devices";
        }
        if (row < 24) {
            return "Controller mapping";
        }
        return row < 24 + FRONTEND_SHORTCUT_COUNT ? "Keyboard shortcuts" : "Controller shortcuts";
    case 5:
        return row < 4 ? "Firmware" : row < 9 ? "Disk system" : row < 11 ? "Tape" : "Music player";
    case 6:
        return row < 2 ? "Save states" : "Capture";
    case 7:
        return row < 8 ? "Console" : row < 15 ? "PPU behavior" : row < 18 ? "Cartridge" : "Power-on state";
    default:
        return "General";
    }
}

GtkWidget *cupid_gtk_group(GtkWidget *parent, const char *title) {
    GtkWidget *frame = gtk_frame_new(title);
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    cupid_gtk_margins(box, 10);
    gtk_frame_set_child(GTK_FRAME(frame), box);
    gtk_box_append(GTK_BOX(parent), frame);
    gtk_widget_set_hexpand(frame, TRUE);
    return box;
}
