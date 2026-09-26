/*
 * gtk_ui_smoke.c - Native widget lifecycle, resizing and render checks
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/platform_frontend.h"
#include "../ui/gtk_desktop.h"
#include "../ui/assembler_frontend.h"
#include "../ui/cheat_frontend.h"
#include "../ui/debug_frontend.h"
#include "../ui/debug_tools_frontend.h"
#include "../ui/memory_search_frontend.h"
#include "../ui/watch_frontend.h"
#include "../ui/hex_frontend.h"
#include "../ui/tas_frontend.h"
#include "../ui/desktop_tas_internal.h"
#include "../ui/gtk_assembler.h"
#include "../ui/frontend_commands.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../rom/rom.h"
#include "../debugger/debugger.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
Joypad pad1 = {0}, pad2 = {0};

bool test_gtk_input_accuracy(FrontendDesktopUi *ui);

static void pump(FrontendDesktopUi *ui) {
    for (unsigned i = 0; i < 20; i++) {
        frontend_desktop_render(ui, 256, 240, "UI regression fixture", "NTSC", "Paused");
        SDL_Delay(10);
    }
}

static bool capture(GtkWidget *widget, const char *path) {
    int w = gtk_widget_get_width(widget), h = gtk_widget_get_height(widget);
    if (w <= 0 || h <= 0) {
        return false;
    }
    GdkPaintable *paintable = gtk_widget_paintable_new(widget);
    GtkSnapshot *snapshot = gtk_snapshot_new();
    gdk_paintable_snapshot(paintable, GDK_SNAPSHOT(snapshot), w, h);
    GskRenderNode *node = gtk_snapshot_free_to_node(snapshot);
    g_object_unref(paintable);
    if (!node) {
        return false;
    }
    GskRenderer *renderer = gtk_native_get_renderer(gtk_widget_get_native(widget));
    GdkTexture *texture = gsk_renderer_render_texture(renderer, node, NULL);
    gsk_render_node_unref(node);
    bool ok = texture && gdk_texture_save_to_png(texture, path);
    g_clear_object(&texture);
    return ok;
}

#define CHECK(expression)                                                                                              \
    do {                                                                                                               \
        if (!(expression)) {                                                                                           \
            fprintf(stderr, "GTK interaction failed at line %d: %s\n", __LINE__, #expression);                         \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

static GtkWidget *find_widget(GtkWidget *root, GType type) {
    if (G_TYPE_CHECK_INSTANCE_TYPE(root, type)) {
        return root;
    }
    for (GtkWidget *child = gtk_widget_get_first_child(root); child; child = gtk_widget_get_next_sibling(child)) {
        GtkWidget *found = find_widget(child, type);
        if (found) {
            return found;
        }
    }
    return NULL;
}

static CupidGtkTool *find_tool(FrontendDesktopUi *ui, FrontendDesktopUi *model) {
    for (CupidGtkTool *tool = ui->gtk->tools; tool; tool = tool->next) {
        if (&tool->ui == model) {
            return tool;
        }
    }
    return NULL;
}

static GtkWidget *find_register_entry(GtkWidget *root) {
    if (GTK_IS_LABEL(root) && g_str_has_prefix(gtk_label_get_text(GTK_LABEL(root)), "Set register")) {
        return find_widget(gtk_widget_get_parent(root), GTK_TYPE_ENTRY);
    }
    for (GtkWidget *child = gtk_widget_get_first_child(root); child; child = gtk_widget_get_next_sibling(child)) {
        GtkWidget *entry = find_register_entry(child);
        if (entry) {
            return entry;
        }
    }
    return NULL;
}

static gboolean cancel_file_dialog(gpointer data) {
    GtkNativeDialog *dialog = g_object_get_data(G_OBJECT(data), "cupid-file-dialog");
    if (dialog) g_signal_emit_by_name(dialog, "response", GTK_RESPONSE_CANCEL);
    return G_SOURCE_REMOVE;
}

static bool file_dialog_interactions(FrontendDesktopUi *ui) {
    char path[128] = "unchanged", error[128] = "";
    cupid_gtk_files_install(GTK_WINDOW(ui->gtk->window));
    guint source = g_idle_add(cancel_file_dialog, ui->gtk->window);
    bool accepted = frontend_open_file_dialog(FRONTEND_OPEN_IMAGE, path, sizeof(path), error, sizeof(error));
    if (g_main_context_find_source_by_id(NULL, source)) g_source_remove(source);
    CHECK(!accepted && !error[0] && !strcmp(path, "unchanged"));
    CHECK(!g_object_get_data(G_OBJECT(ui->gtk->window), "cupid-file-dialog"));
    return true;
}

static bool register_interactions(FrontendDesktopUi *ui) {
    CHECK(debugger_is_paused() && frontend_execution_paused(ui->execution));
    CupidGtkTool *tool = find_tool(ui, cupid_gtk_open(ui, 1, DEBUGGER_FRONTEND_PANEL));
    CHECK(tool);
    pump(ui);
    GtkWidget *entry = find_register_entry(tool->content);
    CHECK(entry);
    uint16_t initial = cpu.pc;
    gtk_editable_set_text(GTK_EDITABLE(entry), "PC=0010");
    pump(ui);
    CHECK(cpu.pc == initial);
    g_signal_emit_by_name(entry, "activate");
    CHECK(cpu.pc == 0x0010);
    char restore[32];
    snprintf(restore, sizeof(restore), "PC=%04X", initial);
    gtk_editable_set_text(GTK_EDITABLE(entry), restore);
    g_signal_emit_by_name(entry, "activate");
    CHECK(cpu.pc == initial);
    gtk_widget_set_visible(tool->window, FALSE);
    return true;
}

static bool assembler_action(unsigned action, const char *value, int selected) {
    char error[256] = "";
    bool ok = frontend_panel_action(ASSEMBLER_FRONTEND_PANEL, action, value, selected, error, sizeof(error));
    if (!ok) {
        fprintf(stderr, "Assembler action: %s\n", error);
    }
    return ok;
}

static bool assembler_apply_enabled(void) {
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = G_N_ELEMENTS(controls)};
    if (!frontend_panel_snapshot(ASSEMBLER_FRONTEND_PANEL, &model, NULL, 0)) {
        return false;
    }
    for (size_t i = 0; i < model.count; ++i) {
        if (controls[i].id == ASSEMBLER_APPLY) {
            return controls[i].enabled;
        }
    }
    return false;
}

static bool assembler_interactions(FrontendDesktopUi *ui) {
    CHECK(assembler_action(ASSEMBLER_SPACE, NULL, 1));
    CHECK(assembler_action(ASSEMBLER_ADDRESS, "$0600", 0));
    CHECK(assembler_action(ASSEMBLER_SOURCE,
                           "; Native multiline assembler\nstart: LDA #$2A\nSTA $0200\nINX\nBNE start\nRTS", 0));
    CHECK(assembler_action(ASSEMBLER_PREVIEW, NULL, 0));
    CHECK(assembler_apply_enabled());
    CHECK(assembler_action(ASSEMBLER_APPLY, NULL, 0));
    CHECK(read_mem(0x600) == 0xA9 && read_mem(0x601) == 0x2A && read_mem(0x607) == 0xF8);
    CupidGtkTool *tool = find_tool(ui, cupid_gtk_open(ui, 1, ASSEMBLER_FRONTEND_PANEL));
    CHECK(tool);
    pump(ui);
    GtkWidget *source = find_widget(tool->content, GTK_TYPE_TEXT_VIEW);
    CHECK(source && gtk_text_view_get_editable(GTK_TEXT_VIEW(source)));
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(source));
    CHECK(gtk_text_buffer_get_line_count(buffer) == 6);
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(buffer, &end);
    gtk_text_buffer_begin_user_action(buffer);
    gtk_text_buffer_insert(buffer, &end, "\nNOP", -1);
    gtk_text_buffer_end_user_action(buffer);
    CHECK(!assembler_apply_enabled() && gtk_text_buffer_get_can_undo(buffer));
    gtk_text_buffer_undo(buffer);
    CHECK(gtk_text_buffer_get_line_count(buffer) == 6 && gtk_text_buffer_get_can_redo(buffer));
    CHECK(assembler_action(ASSEMBLER_PREVIEW, NULL, 0));
    cupid_gtk_assembler_refresh(tool->content);
    gtk_widget_set_visible(tool->window, FALSE);
    return true;
}

static void preview_pattern(void) {
    for (unsigned y = 0; y < 240; ++y) {
        for (unsigned x = 0; x < 256; ++x) {
            framebuffer[y * 256 + x] = 0xFF000000u | ((x ^ y) << 16) | (y << 8) | x;
        }
    }
}

static bool tas_fixture(FrontendDesktopUi *ui, const char *out) {
    char path[4096], error[256] = "";
    g_snprintf(path, sizeof(path), "%s/fixture.ctas", out);
    CHECK(tas_frontend_open(ui->execution, path, true, error, sizeof(error)));
    NesTasSession *session = nes_movie_tas(ui->execution->movie);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    NesTasProject *project = nes_tas_session_project(session);
    CHECK(project && nes_tas_edit_begin(project) == NES_TAS_OK);
    bool ok = nes_tas_insert_frames(project, 0, 10000) == NES_TAS_OK;
    for (size_t i = 0; ok && i < 240; ++i) {
        NesFm2Frame frame = {0};
        frame.pads[0] = (i % 32 < 24 ? 0x80 : 0) | (i % 12 < 3 ? 1 : 0);
        frame.pads[1] = i % 16 < 8 ? 2 : 0;
        ok = nes_tas_set_frame(project, i, &frame) == NES_TAS_OK;
        if (ok) {
            ok = nes_tas_annotate_lag(project, i, i % 7 ? NES_TAS_LAG_NO : NES_TAS_LAG_YES) == NES_TAS_OK;
        }
    }
    if (ok) {
        ok = nes_tas_marker_set(project, 0, "Start: run right") == NES_TAS_OK;
    }
    if (ok) {
        ok = nes_tas_marker_set(project, 24, "Jump timing") == NES_TAS_OK;
    }
    if (ok) {
        ok =
            nes_tas_navigation_add(project, 24, "First jump", "Review the jump input and landing.", NULL) == NES_TAS_OK;
    }
    if (ok) {
        ok = nes_tas_edit_end(project) == NES_TAS_OK;
    } else {
        nes_tas_edit_cancel(project);
    }
    CHECK(ok && tas_frontend_changed(ui->execution, error, sizeof(error)));
    preview_pattern();
    return true;
}

static GtkEventController *find_controller(GtkWidget *widget, const char *name) {
    GListModel *controllers = gtk_widget_observe_controllers(widget);
    GtkEventController *found = NULL;
    for (guint i = 0; i < g_list_model_get_n_items(controllers); ++i) {
        GtkEventController *controller = g_list_model_get_item(controllers, i);
        if (!g_strcmp0(gtk_event_controller_get_name(controller), name)) {
            found = controller;
            break;
        }
        g_object_unref(controller);
    }
    g_object_unref(controllers);
    return found;
}

static bool click_cell(FrontendDesktopUi *ui, GtkWidget *tree, size_t frame, GtkTreeViewColumn *column) {
    GtkTreePath *path = gtk_tree_path_new_from_indices((int)frame, -1);
    gtk_tree_view_scroll_to_cell(GTK_TREE_VIEW(tree), path, column, TRUE, 0, 0);
    pump(ui);
    GdkRectangle area;
    gtk_tree_view_get_cell_area(GTK_TREE_VIEW(tree), path, column, &area);
    gtk_tree_path_free(path);
    int ignored, y;
    /* GTK 4 reports cell-area x already adjusted for horizontal scrolling.
     * Only the vertical header offset needs conversion for controller signals. */
    gtk_tree_view_convert_bin_window_to_widget_coords(GTK_TREE_VIEW(tree), 0, area.y + area.height / 2, &ignored, &y);
    GtkEventController *click = find_controller(tree, "cupid-tas-input");
    CHECK(click);
    g_signal_emit_by_name(click, "pressed", 1, (double)(area.x + area.width / 2), (double)y);
    g_signal_emit_by_name(click, "released", 1, (double)(area.x + area.width / 2), (double)y);
    g_object_unref(click);
    return true;
}

static bool tas_action(CupidGtkTool *tool, unsigned action) {
    CHECK(gtk_widget_activate_action(tool->content, "tas.run", "u", action));
    return true;
}

static void count_model_change(GtkTreeModel *model, GtkTreePath *path, GtkTreeIter *iter, gpointer data) {
    (void)model;
    (void)path;
    (void)iter;
    ++*(unsigned *)data;
}

static bool tas_interactions(FrontendDesktopUi *ui) {
    CupidGtkTool *tool = find_tool(ui, cupid_gtk_open(ui, 1, TAS_PANEL));
    CHECK(tool);
    pump(ui);
    CHECK(!tool->ui.clay);
    GtkWidget *tree = find_widget(tool->content, GTK_TYPE_TREE_VIEW);
    GtkWidget *picture = find_widget(tool->content, GTK_TYPE_PICTURE);
    GtkWidget *entry = find_widget(tool->content, GTK_TYPE_ENTRY);
    CHECK(tree && picture && entry);
    GtkTreeModel *rows = gtk_tree_view_get_model(GTK_TREE_VIEW(tree));
    CHECK(gtk_tree_model_iter_n_children(rows, NULL) == 10000);
    GtkTreeIter iter;
    CHECK(gtk_tree_model_iter_nth_child(rows, &iter, NULL, 9999));
    guint64 index = 0;
    gtk_tree_model_get(rows, &iter, 0, &index, -1);
    CHECK(index == 9999);
    unsigned changes = 0;
    gulong observer = g_signal_connect(rows, "row-changed", G_CALLBACK(count_model_change), &changes);
    framebuffer[0] ^= 0x00FFFFFF;
    cupid_gtk_tas_video(tool->content);
    g_signal_handler_disconnect(rows, observer);
    CHECK(changes == 0 && gtk_tree_view_get_model(GTK_TREE_VIEW(tree)) == rows);
    GdkPaintable *paintable = gtk_picture_get_paintable(GTK_PICTURE(picture));
    CHECK(GDK_IS_TEXTURE(paintable));
    uint32_t downloaded[256 * 240];
    gdk_texture_download(GDK_TEXTURE(paintable), (guchar *)downloaded, 256 * sizeof(uint32_t));
    CHECK(!memcmp(downloaded, framebuffer, sizeof(downloaded)));
    NesTasProject *project = desktop_tas_project_writable(&tool->ui, false);
    CHECK(project);
    GtkTreeViewColumn *input = NULL;
    GList *columns = gtk_tree_view_get_columns(GTK_TREE_VIEW(tree));
    for (GList *item = columns; item; item = item->next) {
        if (GPOINTER_TO_INT(g_object_get_data(G_OBJECT(item->data), "tas-column")) == TAS_GRID_PAD_BASE + 7) {
            input = item->data;
        }
    }
    g_list_free(columns);
    CHECK(input);
    uint8_t before = nes_tas_project_frame(project, 0)->pads[0];
    CHECK(click_cell(ui, tree, 0, input));
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == (before ^ 0x80));
    CHECK(tas_action(tool, TAS_ACTION_UNDO));
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == before);
    CHECK(tas_action(tool, TAS_ACTION_REDO));
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == (before ^ 0x80));
    CHECK(click_cell(ui, tree, 0, gtk_tree_view_get_column(GTK_TREE_VIEW(tree), 0)));
    GtkEventController *keys = find_controller(tree, "cupid-tas-keys");
    CHECK(keys);
    gboolean handled = FALSE;
    g_signal_emit_by_name(keys, "key-pressed", GDK_KEY_Down, 0u, GDK_SHIFT_MASK, &handled);
    g_object_unref(keys);
    CHECK(handled && nes_tas_selection_count(project) == 2);
    CHECK(tas_action(tool, TAS_ACTION_INSERT_MANY));
    CHECK(tool->ui.edit_text_active && gtk_widget_get_visible(entry));
    gtk_editable_set_text(GTK_EDITABLE(entry), "invalid");
    g_signal_emit_by_name(entry, "activate");
    CHECK(tool->ui.edit_text_active && nes_tas_project_frame_count(project) == 10000);
    gtk_editable_set_text(GTK_EDITABLE(entry), "3");
    g_signal_emit_by_name(entry, "activate");
    CHECK(!tool->ui.edit_text_active && nes_tas_project_frame_count(project) == 10003);
    CHECK(tas_action(tool, TAS_ACTION_UNDO));
    CHECK(nes_tas_project_frame_count(project) == 10000);
    CHECK(tas_action(tool, TAS_ACTION_READ_ONLY));
    before = nes_tas_project_frame(project, 0)->pads[0];
    CHECK(click_cell(ui, tree, 0, input));
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == before && !desktop_tas_project_writable(&tool->ui, false));
    CHECK(tas_action(tool, TAS_ACTION_READ_ONLY));
    CHECK(tas_action(tool, TAS_ACTION_SPLICE_OPEN));
    CHECK(tool->ui.tas_editor->splice.preview_valid);
    CHECK(tas_action(tool, TAS_ACTION_SIDEBAR_INPUT));
    gtk_adjustment_set_value(gtk_scrollable_get_hadjustment(GTK_SCROLLABLE(tree)), 0);
    preview_pattern();
    cupid_gtk_tas_video(tool->content);
    /* Retain a disposed root to verify late presentation/refresh calls are safe. */
    GtkWidget *probe = cupid_gtk_tas_new(tool);
    g_object_ref_sink(probe);
    g_object_run_dispose(G_OBJECT(probe));
    cupid_gtk_tas_video(probe);
    cupid_gtk_tas_refresh(probe);
    g_object_unref(probe);
    gtk_widget_set_visible(tool->window, FALSE);
    return true;
}

static void pause_fixture(FrontendExecutionRuntime *execution) {
    debugger_pause();
    execution_control_set_paused(&execution->execution, true);
    frontend_execution_sync_debugger(execution);
}

static bool capture_tool(FrontendDesktopUi *ui, CupidGtkTool *tool, unsigned panel, const char *out) {
    gtk_widget_set_visible(tool->window, TRUE);
    gtk_window_present(GTK_WINDOW(tool->window));
    pump(ui);
    for (unsigned size = 0; size < 2; ++size) {
        gtk_window_set_default_size(GTK_WINDOW(tool->window), size ? 760 : 1280, size ? 540 : 850);
        pump(ui);
        char path[4096];
        g_snprintf(path, sizeof(path), "%s/panel-%04x-%s.png", out, panel, size ? "small" : "large");
        /* A remapped window can have an allocation before its first snapshot.
         * Wait for a drawable frame rather than capturing an empty paintable. */
        bool captured = false;
        for (unsigned attempt = 0; attempt < 10 && !captured; ++attempt) {
            gtk_widget_queue_draw(tool->window);
            pump(ui);
            captured = gtk_widget_get_mapped(tool->window) && capture(tool->window, path);
        }
        CHECK(captured);
    }
    gtk_widget_set_visible(tool->window, FALSE);
    return true;
}

int main(int argc, char **argv) {
    const char *out = argc > 1 ? argv[1] : "build/gtk-smoke";
    g_mkdir_with_parents(out, 0755);
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_TIMER) != 0) {
        return 1;
    }
    uint8_t *rom = calloc(1, 16 + 32768 + 8192);
    memcpy(rom, "NES\032", 4);
    rom[4] = 2;
    rom[5] = 1;
    rom[16 + 0x7ffc] = 0;
    rom[16 + 0x7ffd] = 0x80;
    memset(rom + 16, 0xea, 32768);
    rom[16 + 0x7ffc] = 0;
    rom[16 + 0x7ffd] = 0x80;
    if (load_rom_memory(rom, 16 + 32768 + 8192) != 0) {
        free(rom);
        return 1;
    }
    free(rom);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) {
        return 1;
    }
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    settings.pause_on_focus_loss = false;
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    debugger_pause();
    execution_control_set_paused(&execution.execution, true);
    frontend_execution_sync_debugger(&execution);
    frontend_commands_reset();
    frontend_panels_reset();
    SDL_Window *window = SDL_CreateWindow("GTK rendering fixture", 0, 0, 900, 700, SDL_WINDOW_HIDDEN);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, window, renderer, &settings, &execution, NULL, NULL);
    ui.native_windows = true;
    if (!ui.gtk || ui.clay) {
        fprintf(stderr, "GTK host was not selected\n");
        return 1;
    }
    FrontendVideoRuntime video = {0};
    video.settings = &settings;
    video.frame.pixels = framebuffer;
    video.frame.width = 256;
    video.frame.height = 240;
    video.frame.screens = 1;
    ui.video = &video;
    preview_pattern();
    DebugFrontend *debug = debug_frontend_create(&execution);
    CheatFrontend *cheats = cheat_frontend_create(out);
    MemoryToolsFrontend *memory = memory_tools_create(cheats);
    bool ok = true;
#define REGISTER(call)                                                                                                 \
    do {                                                                                                               \
        if (!(call)) {                                                                                                 \
            fprintf(stderr, "Registration failed: %s\n", #call);                                                       \
            ok = false;                                                                                                \
        }                                                                                                              \
    } while (0)
    REGISTER(frontend_execution_register_commands(&execution));
    REGISTER(frontend_desktop_register_commands(&ui));
    REGISTER(debug_frontend_register_ui(debug));
    REGISTER(cheat_frontend_register_ui(cheats));
    REGISTER(memory_tools_register(memory));
#undef REGISTER
    frontend_panel_set_session_active(true);
    frontend_command_set_session_active(true);
    pump(&ui);
    if (ok) {
        ok = test_gtk_input_accuracy(&ui);
    }
    if (ok) {
        ok = file_dialog_interactions(&ui) && register_interactions(&ui);
    }
    if (ok) {
        ok = assembler_interactions(&ui);
    }
    if (ok) {
        ok = tas_fixture(&ui, out) && tas_interactions(&ui);
    }
    unsigned count = 0;
    pause_fixture(&execution);
    /* Capture the populated TAS timeline while its session is active. Restore
     * live mode for the debugger/assembler captures; pausing alone deliberately
     * does not bypass movie-mode memory protection. */
    if (ok) {
        CupidGtkTool *tas = find_tool(&ui, cupid_gtk_open(&ui, 1, TAS_PANEL));
        ok = tas && capture_tool(&ui, tas, TAS_PANEL, out);
        if (ok) {
            ++count;
        }
    }
    if (ok) {
        char error[256] = "";
        ok = tas_frontend_discard(&execution, error, sizeof(error));
        if (!ok) {
            fprintf(stderr, "Cannot restore live fixture: %s\n", error);
        }
    }
    pause_fixture(&execution);
    preview_pattern();
    if (ok) {
        ok = assembler_action(ASSEMBLER_PREVIEW, NULL, 0) && assembler_apply_enabled();
        if (!ok) {
            fprintf(stderr, "Assembler Apply must be enabled in the paused live fixture\n");
        }
    }
    for (size_t i = 0; ok && i < frontend_panel_count(); i++) {
        FrontendPanelInfo panel;
        if (!frontend_panel_at(i, &panel) || panel.id == TAS_PANEL) {
            continue;
        }
        FrontendDesktopUi *model = cupid_gtk_open(&ui, 1, panel.id);
        if (!model) {
            ok = false;
            break;
        }
        CupidGtkTool *tool = find_tool(&ui, model);
        if (!tool) {
            ok = false;
            break;
        }
        ok = capture_tool(&ui, tool, panel.id, out);
        ++count;
    }
    cupid_gtk_open(&ui, 0, 0);
    pump(&ui);
    char path[4096];
    g_snprintf(path, sizeof(path), "%s/settings.png", out);
    ok = capture(ui.gtk->tools->window, path) && ok;
    frontend_desktop_shutdown(&ui);
    memory_tools_destroy(memory);
    cheat_frontend_destroy(cheats);
    debug_frontend_destroy(debug);
    tas_frontend_unregister();
    desktop_ppu_unregister();
    frontend_execution_shutdown(&execution);
    unload_rom();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    printf("GTK: register/TAS/assembler interactions; %u tool windows rendered at two sizes; settings rendered: %s\n",
           count, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
