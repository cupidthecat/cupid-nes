/*
 * desktop_menu_accuracy.c
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/desktop_internal.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include <stdio.h>
#include <string.h>

static int failures;
static unsigned invoked;

static const struct {
    unsigned id;
    const char *group;
} tools[] = {{0x2400, "Debugging"},
             {0x2408, "Debugging"},
             {0x2500, "Capture"},
             {0x2501, "Rewind and movies"},
             {0x2502, "Capture"},
             {0x2600, "Game and recovery"},
             {0x26a0, "Game and recovery"},
             {0x2640, "Application"},
             {0x2680, "Application"},
             {0x2700, "Rewind and movies"},
             {0x2720, "Picture, sound and timing"},
             {0x2740, "HD graphics"},
             {0x2780, "Picture, sound and timing"},
             {0x27c0, "Picture, sound and timing"},
             {0x2800, "Rewind and movies"},
             {0x2900, "Cheats"},
             {0x2a00, "Input tools"},
             {0x2b00, "Storage"},
             {0x2c00, "Picture, sound and timing"}};

#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "Desktop menu line %d: %s\n", __LINE__, #x);                                               \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

static bool command(void *context, char *error, size_t size) {
    (void)error;
    (void)size;
    invoked = *(unsigned *)context;
    return true;
}

static void render(FrontendDesktopUi *ui) {
    SDL_SetRenderDrawColor(ui->renderer, 14, 18, 27, 255);
    SDL_RenderClear(ui->renderer);
    desktop_layout(ui, "Menu regression", "NTSC", "Paused");
    CHECK(desktop_clay_error_count(ui->clay) == 0);
}

static void bounds(FrontendDesktopUi *ui, int kind, int row, int depth) {
    SDL_FRect r;
    bool found = desktop_clay_bounds(ui->clay, kind, row, depth, &r);
    CHECK(found);
    if (found) {
        CHECK(r.x >= 0 && r.y >= 0 && r.w > 0 && r.h > 0);
        CHECK((r.x + r.w) * ui->ui_scale <= 901);
        CHECK((r.y + r.h) * ui->ui_scale <= 681);
    }
}

static void walk(FrontendDesktopUi *ui, int depth, unsigned seen[100]) {
    DesktopMenuItem rows[128];
    int count = desktop_menu_level(ui, depth, rows);
    CHECK(count > 0 && count <= 10);
    ui->menu_depth = depth;
    render(ui);
    for (int i = 0; i < count; ++i) {
        bounds(ui, HIT_MENU_ROW, i, depth);
    }
    for (int i = 0; i < count; ++i) {
        if (rows[i].kind == 7) {
            CHECK(depth < 4);
            if (depth < 4) {
                ui->menu_path[depth] = rows[i].id;
                ui->menu_parent_rows[depth] = i;
                walk(ui, depth + 1, seen);
            }
        } else if (rows[i].id >= 0x3000 && rows[i].id < 0x3064) {
            ++seen[rows[i].id - 0x3000];
            CHECK(frontend_command_invoke(rows[i].id, NULL, 0));
            CHECK(invoked == rows[i].id);
        }
    }
    ui->menu_depth = depth;
}

static void screenshot(FrontendDesktopUi *ui, const char *name, int scale) {
    char path[128];
    snprintf(path, sizeof(path), "build/desktop-menu-%s-%d.bmp", name, scale);
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, 900, 680, 32, SDL_PIXELFORMAT_ARGB8888);
    CHECK(surface != NULL);
    if (surface) {
        CHECK(SDL_RenderReadPixels(ui->renderer, NULL, surface->format->format, surface->pixels, surface->pitch) == 0);
        CHECK(SDL_SaveBMP(surface, path) == 0);
        SDL_FreeSurface(surface);
    }
}

static void grouped(FrontendDesktopUi *ui, int depth, const char *group, unsigned *seen) {
    DesktopMenuItem rows[128];
    int count = desktop_menu_level(ui, depth, rows);
    for (int i = 0; i < count; ++i) {
        if (rows[i].kind == 7) {
            CHECK(depth < 4);
            if (depth < 4) {
                ui->menu_path[depth] = rows[i].id;
                grouped(ui, depth + 1, rows[i].id & 0x80000000u ? group : rows[i].label, seen);
            }
        } else {
            for (unsigned j = 0; j < sizeof(tools) / sizeof(tools[0]); ++j) {
                if (tools[j].id == rows[i].id) {
                    CHECK(group && !strcmp(group, tools[j].group));
                    CHECK(!rows[i].enabled);
                    ++seen[j];
                }
            }
        }
    }
}

int test_desktop_menu_accuracy(void) {
    failures = 0;
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_Window *window = SDL_CreateWindow("Menu checks", 0, 0, 900, 680, SDL_WINDOW_HIDDEN);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    CHECK(renderer != NULL);
    if (!renderer) {
        if (window) {
            SDL_DestroyWindow(window);
        }
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        return failures;
    }
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, window, renderer, &settings, NULL, NULL, NULL);
    frontend_commands_reset();
    frontend_panels_reset();
    unsigned ids[100];
    for (unsigned i = 0; i < 100; ++i) {
        char label[64];
        ids[i] = 0x3000 + i;
        snprintf(label, sizeof(label), "Registered tool %u", i + 1);
        FrontendCommandSpec spec = {ids[i], label, "Tools", "", 0, command, &ids[i]};
        CHECK(frontend_command_register(&spec));
    }
    const float scales[] = {1, 1.5f, 2};
    for (int s = 0; s < 3; ++s) {
        ui.ui_scale = scales[s];
        ui.open_menu = 5;
        ui.menu_depth = ui.menu_row = 0;
        unsigned seen[100] = {0};
        walk(&ui, 0, seen);
        for (int i = 0; i < 100; ++i) {
            CHECK(seen[i] == 1);
        }
        ui.menu_depth = 0;
        render(&ui);
        for (int i = 0; i < 7; ++i) {
            bounds(&ui, HIT_MENU, i, 0);
        }
        CHECK(desktop_clay_text_visible(ui.clay, "Help"));
        screenshot(&ui, "tools", (int)(scales[s] * 100));
        ui.open_menu = -1;
        ui.settings_open = true;
        ui.staged = settings;
        ui.settings_category = 2;
        ui.settings_row = ui.settings_scroll = 0;
        render(&ui);
        for (int i = 0; i < 8; ++i) {
            bounds(&ui, HIT_CATEGORY, i, 0);
        }
        for (int i = 0; i < 5; ++i) {
            bounds(&ui, HIT_SETTINGS_BUTTON, i, 0);
        }
        bounds(&ui, HIT_SETTING, 0, 0);
        screenshot(&ui, "settings", (int)(scales[s] * 100));
        ui.choice_open = true;
        ui.choice_panel = false;
        ui.choice_row = 20;
        ui.choice_page = 0;
        render(&ui);
        for (int i = 0; i < 20; ++i) {
            bounds(&ui, HIT_CHOICE, i, 0);
        }
        bounds(&ui, HIT_CHOICE_PAGE, 0, 1);
        screenshot(&ui, "choices", (int)(scales[s] * 100));
        ui.choice_open = ui.settings_open = false;
        ui.settings_open = true;
        ui.settings_category = 5;
        ui.settings_row = ui.settings_scroll = 0;
        render(&ui);
        bounds(&ui, HIT_CLEAR_SETTING, 0, 0);
        ui.settings_open = false;
    }
    frontend_commands_reset();
    for (unsigned i = 0; i < sizeof(tools) / sizeof(tools[0]); ++i) {
        ids[i] = tools[i].id;
        FrontendCommandSpec spec = {ids[i],  "Feature tool", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION,
                                    command, &ids[i]};
        CHECK(frontend_command_register(&spec));
    }
    ui.open_menu = 5;
    for (int s = 0; s < 3; ++s) {
        unsigned seen[sizeof(tools) / sizeof(tools[0])] = {0};
        ui.ui_scale = scales[s];
        grouped(&ui, 0, NULL, seen);
        for (unsigned i = 0; i < sizeof(tools) / sizeof(tools[0]); ++i) {
            CHECK(seen[i] == 1);
        }
        ui.menu_depth = 0;
        render(&ui);
        screenshot(&ui, "groups", (int)(scales[s] * 100));
    }
    DesktopMenuItem invalid[128];
    CHECK(desktop_menu_level(&ui, -1, invalid) == 0);
    CHECK(desktop_menu_level(&ui, 5, invalid) == 0);
    frontend_desktop_shutdown(&ui);
    frontend_commands_reset();
    frontend_panels_reset();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    printf("Desktop menus and compact layout: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
