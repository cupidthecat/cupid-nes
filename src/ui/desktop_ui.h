/*
 * desktop_ui.h - SDL desktop menus, toolbar, status, and settings interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_DESKTOP_UI_H
#define FRONTEND_DESKTOP_UI_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#include "frontend_execution.h"
#include "frontend_session.h"
#include "audio_runtime.h"
#include "session_actions.h"
#include "settings.h"
#include "video_runtime.h"
#include "capture_frontend.h"
#include "device_frontend.h"

typedef enum {
    FRONTEND_IDLE_ERROR = -1,
    FRONTEND_IDLE_QUIT = 0,
    FRONTEND_IDLE_OPEN = 1
} FrontendIdleResult;

typedef struct FrontendDesktopUi {
    struct FrontendDesktopUi *parent, *tools, *next;
    bool native_windows, focused, palette_window;
    uint32_t tool_rendered;
    struct DesktopClay *clay;
    struct DesktopPpuViewer *ppu_viewer;
    const FrontendSession *idle_session;
    int idle_recent_index;
    int visible_rows;
    int palette_index;
    bool edit_select_all;
    SDL_Window *window;
    SDL_Renderer *renderer;
    FrontendSettings *settings;
    FrontendExecutionRuntime *execution;
    FrontendSessionActions *sessions;
    FrontendAudioRuntime *audio;
    FrontendVideoRuntime *video;
    NesCaptureFrontend *capture;
    FrontendDeviceRuntime *devices;
    NsfPlayer *music;
    const char *settings_path;
    FrontendSettings staged;
    bool settings_open;
    bool info_open;
    bool log_open;
    unsigned log_count;
    char log_lines[12][256];
    bool panel_open;
    bool quit_requested;
    bool paused_for_ui;
    bool paused_for_focus;
    bool capture_binding;
    bool capture_shortcut;
    bool capture_gamepad;
    bool confirm_restore_all;
    int open_menu;
    int menu_row;
    int menu_scroll;
    int menu_depth, menu_parent_rows[4];
    unsigned menu_path[4];
    bool choice_open, choice_panel, edit_number;
    int choice_row, choice_index, choice_page;
    int dragging_scroll;
    int panel_row;
    int panel_scroll;
    int settings_focus;
    int settings_button;
    int settings_category;
    int settings_row;
    int settings_scroll;
    unsigned settings_player;
    unsigned capture_index;
    unsigned panel_id;
    unsigned edit_control, prompt_command;
    float ui_scale;
    bool edit_text_active;
    char edit_text[FRONTEND_SETTINGS_PATH_TEXT];
    char status[256];
    uint32_t status_until;
    double fps;
} FrontendDesktopUi;

FrontendIdleResult frontend_desktop_idle_open(FrontendSettings *settings,
                                              const FrontendSession *session,
                                              const char *settings_path,
                                              FrontendImageRequest *request,
                                              SDL_Window **window,
                                              SDL_Renderer **renderer,
                                              char *error, size_t error_size);
void frontend_desktop_init(FrontendDesktopUi *ui, SDL_Window *window,
                           SDL_Renderer *renderer, FrontendSettings *settings,
                           FrontendExecutionRuntime *execution,
                           FrontendSessionActions *sessions,
                           const char *settings_path);
void frontend_desktop_set_runtime(FrontendDesktopUi *ui,
                                  FrontendAudioRuntime *audio,
                                  FrontendVideoRuntime *video);
bool frontend_desktop_register_commands(FrontendDesktopUi *ui);
bool frontend_desktop_handle_event(FrontendDesktopUi *ui, const SDL_Event *event);
void frontend_desktop_render(FrontendDesktopUi *ui, int video_width, int video_height,
                             const char *title, const char *region, const char *run_state);
void frontend_desktop_compute_game_rect(int window_width, int window_height,
                                        int video_width, int video_height,
                                        bool integer_scaling, SDL_Rect *rect);
void frontend_desktop_game_rect(const FrontendDesktopUi *ui, int ww, int wh,
                                 int vw, int vh, bool integer_scaling, SDL_Rect *rect);
bool frontend_desktop_apply_features(FrontendDesktopUi *ui, const FrontendSettings *settings, char *error, size_t size);
bool frontend_desktop_input_captured(const FrontendDesktopUi *ui);
bool frontend_desktop_quit_requested(const FrontendDesktopUi *ui);
void frontend_desktop_set_status(FrontendDesktopUi *ui, const char *message);
void frontend_desktop_set_fps(FrontendDesktopUi *ui, double fps);
void frontend_desktop_update_window_settings(FrontendDesktopUi *ui);
void frontend_desktop_update_activity(FrontendDesktopUi *ui);
void frontend_desktop_shutdown(FrontendDesktopUi *ui);

#endif
