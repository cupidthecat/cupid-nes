/*
 * gtk_video_accuracy.c - Accelerated viewport and live game information checks
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <gdk/win32/gdkwin32.h>
#endif

#define CHECK(expression) do { \
    if (!(expression)) { \
        fprintf(stderr, "GTK video check failed at line %d: %s\n", __LINE__, #expression); \
        return false; \
    } \
} while (0)

static void pump(FrontendDesktopUi *ui) {
    for (unsigned i = 0; i < 30; ++i) {
        frontend_desktop_render(ui, 256, 240, "Video fixture", "NTSC", "Paused");
        cupid_gtk_dispatch();
        SDL_Delay(10);
    }
}

static bool contains(GtkWidget *view, const char *expected) {
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
    GtkTextIter start, end;
    gtk_text_buffer_get_bounds(buffer, &start, &end);
    char *text = gtk_text_buffer_get_text(buffer, &start, &end, FALSE);
    bool found = strstr(text, expected) != NULL;
    if (!found) fprintf(stderr, "Expected '%s' in: %s\n", expected, text);
    g_free(text);
    return found;
}

bool test_gtk_information(FrontendDesktopUi *ui) {
    FrontendSessionActions *saved = ui->sessions;
    FrontendSession *session = g_new0(FrontendSession, 1);
    FrontendSessionActions actions = {0};
    actions.session = session;
    ui->sessions = &actions;
    session->active = true;
    g_strlcpy(session->current_result.title, "First game.nes", sizeof(session->current_result.title));
    FrontendDesktopUi *model = cupid_gtk_open(ui, 2, 0);
    CupidGtkTool *tool = ui->gtk->tools;
    while (tool && &tool->ui != model) tool = tool->next;
    CHECK(tool && tool->info_view);
    CHECK(contains(tool->info_view, "First game.nes"));
    CHECK(contains(tool->info_view, "Video renderer:"));
    CHECK(contains(tool->info_view, "Desktop compositor:"));
    gtk_widget_set_visible(tool->window, FALSE);
    g_strlcpy(session->current_result.title, "Replacement game.nes", sizeof(session->current_result.title));
    CHECK(cupid_gtk_open(ui, 2, 0) == model);
    CHECK(contains(tool->info_view, "Replacement game.nes"));
    g_strlcpy(session->current_result.title, "Live replacement.nes", sizeof(session->current_result.title));
    ui->gtk->refreshed = SDL_GetTicks() - 100;
    pump(ui);
    CHECK(contains(tool->info_view, "Live replacement.nes"));
    session->active = false;
    ui->gtk->refreshed = SDL_GetTicks() - 100;
    pump(ui);
    CHECK(contains(tool->info_view, "No game loaded"));
    CHECK(contains(tool->info_view, "Ready | Idle"));
    gtk_widget_set_visible(tool->window, FALSE);
    ui->sessions = saved;
    /* The tool's copied model must not retain this fixture's session. */
    tool->ui.sessions = saved;
    g_free(session);
    puts("GTK game information reload, live refresh and unload: PASS");
    return true;
}

#ifdef _WIN32
static const uint32_t colors[] = {0x002266aa, 0x00aa6633, 0x0033aa66, 0x00aa3366};

static bool check_pixels(FrontendDesktopUi *ui, const char *out, unsigned mode) {
    GdkSurface *surface = gtk_native_get_surface(GTK_NATIVE(ui->gtk->window));
    HWND parent = gdk_win32_surface_get_handle(surface);
    RECT client;
    GetClientRect(parent, &client);
    graphene_rect_t bounds;
    CHECK(gtk_widget_compute_bounds(ui->gtk->picture, ui->gtk->window, &bounds));
    HWND child = FindWindowExW(parent, NULL, L"STATIC", L"Cupid game view");
    RECT child_bounds;
    CHECK(child && GetWindowRect(child, &child_bounds));
    MapWindowPoints(NULL, parent, (POINT *)&child_bounds, 2);
    /* Derive the decoration inset independently from the surface transform.
     * Checking only texture pixels missed a child shifted over the toolbar. */
    double sx = (double)client.right / gdk_surface_get_width(surface);
    double sy = (double)client.bottom / gdk_surface_get_height(surface);
    int inset_x = (client.right - (int)lround(gtk_widget_get_width(ui->gtk->window) * sx)) / 2;
    int inset_y = (client.bottom - (int)lround(gtk_widget_get_height(ui->gtk->window) * sy)) / 2;
    CHECK(abs(child_bounds.left - ((int)lround(bounds.origin.x * sx) + inset_x)) <= 1);
    CHECK(abs(child_bounds.top - ((int)lround(bounds.origin.y * sy) + inset_y)) <= 1);
    CHECK(child_bounds.left >= 0 && child_bounds.top >= 0);
    CHECK(child_bounds.right <= client.right && child_bounds.bottom <= client.bottom);
    CHECK(abs(child_bounds.right - child_bounds.left - (int)lround(bounds.size.width * sx)) <= 1);
    CHECK(abs(child_bounds.bottom - child_bounds.top - (int)lround(bounds.size.height * sy)) <= 1);
    uint32_t *pixels = NULL;
    unsigned width = 0, height = 0, vw, vh;
    CHECK(cupid_gtk_native_video_read(ui->gtk, &pixels, &width, &height));
    frontend_video_runtime_display_size(ui->video, &vw, &vh);
    SDL_Rect game;
    frontend_desktop_game_rect(ui, (int)width, (int)height, (int)vw, (int)vh,
                               ui->settings->integer_scaling, &game);
    CHECK(game.w > 0 && game.h > 0);
    /* Alpha is intentionally zero in the source: the game is opaque. */
    for (unsigned quadrant = 0; quadrant < 4; ++quadrant) {
        unsigned x = game.x + game.w * (quadrant % 2 ? 3 : 1) / 4;
        unsigned y = game.y + game.h * (quadrant / 2 ? 3 : 1) / 4;
        CHECK((pixels[y * width + x] & 0xffffff) == colors[quadrant]);
    }
    if (game.x > 0 || game.y > 0) CHECK((pixels[0] & 0xffffff) == 0x060709);
    GBytes *bytes = g_bytes_new_take(pixels, (size_t)width * height * sizeof(*pixels));
    GdkTexture *texture = gdk_memory_texture_new((int)width, (int)height, GDK_MEMORY_B8G8R8A8,
                                                bytes, (size_t)width * sizeof(*pixels));
    char path[4096];
    g_snprintf(path, sizeof(path), "%s/native-%u.png", out, mode);
    bool saved = gdk_texture_save_to_png(texture, path);
    g_object_unref(texture);
    g_bytes_unref(bytes);
    CHECK(saved);
    return true;
}
#endif

bool test_gtk_native_video(FrontendDesktopUi *ui, const char *out) {
#ifdef _WIN32
    uint32_t *source = g_new(uint32_t, 256 * 240);
    for (unsigned y = 0; y < 240; ++y)
        for (unsigned x = 0; x < 256; ++x) source[y * 256 + x] = colors[(y >= 120) * 2 + (x >= 128)];
    const uint32_t *saved_pixels = ui->video->frame.pixels;
    ui->video->frame.pixels = source;
    bool saved_integer = ui->settings->integer_scaling;
    bool saved_linear = ui->settings->bilinear_interpolation;
    for (unsigned mode = 0; mode < 4; ++mode) {
        if (mode == 1) gtk_window_maximize(GTK_WINDOW(ui->gtk->window));
        if (mode == 2) ui->settings->fullscreen = true;
        if (mode == 3) ui->settings->fullscreen = false;
        ui->settings->integer_scaling = mode == 3;
        ui->settings->bilinear_interpolation = mode == 2;
        pump(ui);
        CHECK(cupid_gtk_native_video_name(ui->gtk));
        CHECK(check_pixels(ui, out, mode));
    }
    gtk_window_unmaximize(GTK_WINDOW(ui->gtk->window));
    const int sizes[][2] = {{640, 480}, {1100, 500}, {520, 800}, {900, 700}};
    for (unsigned i = 0; i < G_N_ELEMENTS(sizes); ++i) {
        gtk_window_set_default_size(GTK_WINDOW(ui->gtk->window), sizes[i][0], sizes[i][1]);
        pump(ui);
        CHECK(check_pixels(ui, out, 6 + i));
    }
    HWND parent = gdk_win32_surface_get_handle(gtk_native_get_surface(GTK_NATIVE(ui->gtk->window)));
    HWND child = FindWindowExW(parent, NULL, L"STATIC", L"Cupid game view");
    CHECK(child && IsWindowVisible(child) && !IsWindowEnabled(child));
    CHECK(GetWindowLongPtrW(parent, GWL_STYLE) & WS_CLIPCHILDREN);
    uint32_t padded[80 * 48];
    memset(padded, 0, sizeof(padded));
    for (unsigned y = 0; y < 48; ++y)
        for (unsigned x = 0; x < 64; ++x) padded[y * 80 + x] = colors[(y >= 24) * 2 + (x >= 32)];
    CHECK(cupid_gtk_native_video_present(ui->gtk, padded, 64, 48, 80));
    CHECK(check_pixels(ui, out, 5));
    frontend_panel_set_session_active(false);
    pump(ui);
    CHECK(!IsWindowVisible(child));
    frontend_panel_set_session_active(true);
    pump(ui);
    CHECK(IsWindowVisible(child));
    SDL_Event reset = {0};
    reset.type = SDL_RENDER_DEVICE_RESET;
    CHECK(cupid_gtk_event(ui, &reset));
    CHECK(!ui->gtk->native_video);
    pump(ui);
    CHECK(cupid_gtk_native_video_name(ui->gtk));
    CHECK(check_pixels(ui, out, 4));
    /* Force the driver-error branch and verify that software drawing resumes. */
    cupid_gtk_native_video_destroy(ui->gtk);
    ui->gtk->native_video_failed = true;
    uint64_t before = ui->gtk->drawn_frames;
    pump(ui);
    CHECK(!ui->gtk->native_video && ui->gtk->drawn_frames > before);
    ui->gtk->native_video_failed = false;
    ui->settings->integer_scaling = saved_integer;
    ui->settings->bilinear_interpolation = saved_linear;
    gtk_window_unmaximize(GTK_WINDOW(ui->gtk->window));
    ui->video->frame.pixels = saved_pixels;
    g_free(source);
#else
    (void)ui; (void)out;
#endif
    return true;
}
