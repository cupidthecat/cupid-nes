/*
 * desktop_windows.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Independent desktop tool windows. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_internal.h"
#include "desktop_keyboard.h"
#include "frontend_panels.h"
#include "memory_search_frontend.h"
#include "watch_frontend.h"
#include "hex_frontend.h"
#include "palette_tool.h"
#include "netplay_frontend.h"
#include "../system/timing.h"
#include <stdlib.h>
#include <string.h>

void desktop_window_context(const FrontendDesktopUi *ui, const char **title, const char **region, const char **state) {
    const FrontendDesktopUi *root = ui->parent ? ui->parent : ui;
    const FrontendSession *session = root->sessions ? root->sessions->session : NULL;
    if (!session || !session->active) {
        *title = NULL;
        *region = "Ready";
        *state = "Idle";
        return;
    }
    *title = session->current_result.title;
    *region = nes_region_name(nes_timing()->region);
    const FrontendExecutionRuntime *execution = root->execution;
    if (execution && (nes_netplay_mode(execution->netplay) == NES_NETPLAY_CONNECTED ||
                      nes_netplay_mode(execution->netplay) == NES_NETPLAY_LISTENING)) {
        *state = frontend_netplay_status(execution->network);
    } else if (execution && execution->rewind_held) {
        *state = "Rewinding";
    } else {
        *state = execution && frontend_execution_paused(execution) ? "Paused" : "Running";
    }
}

FrontendDesktopUi *desktop_open_window(FrontendDesktopUi *ui, int kind, unsigned id) {
    FrontendDesktopUi *root = ui->parent ? ui->parent : ui;
    for (FrontendDesktopUi *tool = root->tools; tool; tool = tool->next) {
        if ((kind == 0 && tool->settings_open) || (kind == 1 && tool->panel_open && tool->panel_id == id) ||
            (kind == 2 && tool->info_open && tool->log_open == (id != 0)) || (kind == 3 && tool->palette_window)) {
            SDL_RaiseWindow(tool->window);
            return tool;
        }
    }
    const char *title = kind == 0 ? "Settings" : id ? "Recent messages" : "Game information";
    if (kind == 3) {
        title = "Palette editor";
    }
    FrontendPanelInfo info;
    if (kind == 1 && frontend_panel_get(id, &info)) {
        title = info.title;
    }
    FrontendDesktopUi *tool = calloc(1, sizeof(*tool));
    int height = kind == 1 && (memory_tools_panel(id) || id == WATCH_FRONTEND_PANEL || id == HEX_FRONTEND_PANEL) ? 820 :
                 kind == 1 && desktop_tas_panel(id) ? 760 : 680;
    SDL_Window *window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 900, height,
                                          SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED) : NULL;
    if (window && !renderer) {
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (!tool || !window || !renderer) {
        desktop_copy_status(root, "Could not create a tool window");
        if (renderer) {
            SDL_DestroyRenderer(renderer);
        }
        if (window) {
            SDL_DestroyWindow(window);
        }
        free(tool);
        return NULL;
    }
    frontend_desktop_init(tool, window, renderer, root->settings, root->execution, root->sessions, root->settings_path);
    if (!tool->clay) {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        free(tool);
        desktop_copy_status(root, "Could not allocate the tool interface");
        return NULL;
    }
    tool->parent = root;
    tool->audio = root->audio;
    tool->video = root->video;
    tool->capture = root->capture;
    tool->devices = root->devices;
    tool->music = root->music;
    tool->next = root->tools;
    root->tools = tool;
    if (kind == 0) {
        desktop_settings_open(tool, true);
    } else if (kind == 1) {
        tool->panel_id = id;
        tool->panel_open = true;
    } else if (kind == 3) {
        tool->palette_window = true;
    } else {
        tool->info_open = true;
        tool->log_open = id != 0;
        tool->log_count = root->log_count;
        memcpy(tool->log_lines, root->log_lines, sizeof(tool->log_lines));
    }
    desktop_sync_scale(tool);
    SDL_SetRenderDrawColor(renderer, 24, 28, 39, 255);
    SDL_RenderClear(renderer);
    desktop_layout(tool, "", "", "");
    SDL_RenderPresent(renderer);
    SDL_RaiseWindow(window);
    return tool;
}

bool frontend_desktop_open_panel(FrontendDesktopUi *ui, unsigned id) {
    if (!ui) {
        return false;
    }

    FrontendPanelInfo info;
    if (!frontend_panel_get(id, &info) || !info.enabled) {
        desktop_copy_status(ui, "This tool is unavailable for the current session.");
        return false;
    }

    FrontendDesktopUi *root = ui->parent ? ui->parent : ui;
    if (root->native_windows || ui->parent) {
        if (!desktop_open_window(root, 1, id)) {
            return false;
        }

        if (root->panel_open && root->panel_id == id) {
            root->panel_open = false;
        }

        return true;
    }

    ui->panel_id = id;
    ui->panel_row = ui->panel_scroll = 0;
    ui->panel_open = true;
    return true;
}

static Uint32 event_window(const SDL_Event *event) {
    switch (event->type) {
    case SDL_WINDOWEVENT:
        return event->window.windowID;
    case SDL_KEYDOWN:
    case SDL_KEYUP:
        return event->key.windowID;
    case SDL_TEXTINPUT:
        return event->text.windowID;
    case SDL_TEXTEDITING:
        return event->edit.windowID;
    case SDL_MOUSEMOTION:
        return event->motion.windowID;
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:
        return event->button.windowID;
    case SDL_MOUSEWHEEL:
        return event->wheel.windowID;
    case SDL_DROPFILE:
        return event->drop.windowID;
    case SDL_FINGERDOWN:
    case SDL_FINGERUP:
    case SDL_FINGERMOTION:
        return event->tfinger.windowID;
    default:
        return 0;
    }
}

static void destroy_tool(FrontendDesktopUi *tool) {
    desktop_keyboard_release(tool);
    if (tool->edit_text_active) {
        SDL_StopTextInput();
    }
    desktop_ppu_destroy(tool);
    desktop_tas_destroy(tool);
    desktop_clay_destroy(tool->clay);
    SDL_DestroyRenderer(tool->renderer);
    SDL_DestroyWindow(tool->window);
    free(tool);
}

bool desktop_route_window(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (ui->parent) {
        return false;
    }
    Uint32 id = event_window(event);
    for (FrontendDesktopUi **link = &ui->tools; *link; link = &(*link)->next) {
        FrontendDesktopUi *tool = *link;
        bool controller = (event->type == SDL_CONTROLLERBUTTONDOWN || event->type == SDL_CONTROLLERBUTTONUP) &&
                          tool->focused &&
                          (tool->settings_open || tool->edit_text_active || tool->capture_binding || tool->choice_open);
        if (id != SDL_GetWindowID(tool->window) && !controller) {
            continue;
        }
        bool close = event->type == SDL_WINDOWEVENT && event->window.event == SDL_WINDOWEVENT_CLOSE;
        if (!close) {
            (void)frontend_desktop_handle_event(tool, event);
        }
        if (close || (!tool->settings_open && !tool->panel_open && !tool->info_open && !tool->palette_window)) {
            *link = tool->next;
            destroy_tool(tool);
        }
        return true;
    }
    /* Ignore late events from a destroyed tool, never deliver them to the game. */
    return id && ui->window && id != SDL_GetWindowID(ui->window);
}

void frontend_desktop_update_activity(FrontendDesktopUi *ui) {
    if (!ui || ui->parent || !ui->execution || !ui->settings) {
        return;
    }
    bool focused = ui->focused;
    bool modal = ui->settings_open || (ui->panel_open && !desktop_tas_panel(ui->panel_id)) || ui->info_open || ui->edit_text_active || ui->capture_binding ||
                 ui->open_menu >= 0 || palette_tool_is_visible();
    for (FrontendDesktopUi *tool = ui->tools; tool; tool = tool->next) {
        focused |= tool->focused;
        /* Inspection tools are modeless, including their editable controls. */
        modal |= tool->settings_open;
    }
    unsigned reasons = ui->execution->suspend_reasons & ~(FRONTEND_SUSPEND_UI | FRONTEND_SUSPEND_FOCUS);
    if (modal && ui->settings->pause_on_ui) {
        reasons |= FRONTEND_SUSPEND_UI;
    }
    if (!focused && ui->settings->pause_on_focus_loss) {
        reasons |= FRONTEND_SUSPEND_FOCUS;
    }
    frontend_execution_set_suspension(ui->execution, reasons);
}

void desktop_render_windows(FrontendDesktopUi *ui) {
    Uint32 now = SDL_GetTicks();
    for (FrontendDesktopUi *tool = ui->tools; tool; tool = tool->next) {
        if (now - tool->tool_rendered < 33) {
            continue;
        }
        tool->tool_rendered = now;
        SDL_SetRenderDrawColor(tool->renderer, 24, 28, 39, 255);
        SDL_RenderClear(tool->renderer);
        desktop_layout(tool, "", "", "");
        SDL_RenderPresent(tool->renderer);
    }
}

void desktop_close_windows(FrontendDesktopUi *ui) {
    while (ui->tools) {
        FrontendDesktopUi *tool = ui->tools;
        ui->tools = tool->next;
        destroy_tool(tool);
    }
}
