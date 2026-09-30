/*
 * gtk_video_accuracy.c - Accelerated viewport and live game information checks
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../video/shader_preset.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <glib/gstdio.h>
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

bool test_gtk_picture_pixels(void) {
    GtkWidget *picture = g_object_ref_sink(gtk_picture_new());
    /* Packed RGB, partial alpha and padded rows must all remain opaque. */
    uint32_t pixels[8] = {0x00112233, 0x80445566, 0, 0, 0x00778899, 0xffaabbcc, 0, 0};
    cupid_gtk_picture(picture, pixels, 2, 2, 4 * sizeof(uint32_t));
    GdkTexture *texture = GDK_TEXTURE(gtk_picture_get_paintable(GTK_PICTURE(picture)));
    CHECK(texture && gdk_texture_get_width(texture) == 2 && gdk_texture_get_height(texture) == 2);
    g_object_ref(texture);
    uint32_t download[4];
    gdk_texture_download(texture, (guchar *)download, 2 * sizeof(uint32_t));
    CHECK(download[0] == 0xff112233 && download[1] == 0xff445566);
    CHECK(download[2] == 0xff778899 && download[3] == 0xffaabbcc);
    memset(pixels, 0, sizeof(pixels));
    cupid_gtk_picture(picture, pixels, 2, 2, 4 * sizeof(uint32_t));
    gdk_texture_download(texture, (guchar *)download, 2 * sizeof(uint32_t));
    CHECK(download[0] == 0xff112233 && download[3] == 0xffaabbcc);
    g_object_unref(texture);
    g_object_unref(picture);
    puts("GTK preview opacity, padded rows and retained texture: PASS");
    return true;
}

bool test_gtk_shader_context(FrontendDesktopUi *ui) {
    if (GSK_IS_CAIRO_RENDERER(gtk_native_get_renderer(GTK_NATIVE(ui->gtk->window)))) return true;
    GError *error = NULL;
    GdkSurface *surface = gtk_native_get_surface(GTK_NATIVE(ui->gtk->window));
    GdkGLContext *context = gdk_surface_create_gl_context(surface, &error);
    CHECK(context && gdk_gl_context_realize(context, &error));
    gdk_gl_context_make_current(context);
    CHECK(gdk_gl_context_get_current() == context);
    char *directory = g_dir_make_tmp("cupid-gtk-shader-XXXXXX", &error);
    CHECK(directory);
    char *source = g_build_filename(directory, "pass.glsl", NULL);
    char *preset = g_build_filename(directory, "preset.glslp", NULL);
    const char *glsl =
        "#version 130\n#ifdef VERTEX\n"
        "in vec4 VertexCoord; in vec2 TexCoord; out vec2 uv; uniform mat4 MVPMatrix;\n"
        "void main() { gl_Position=MVPMatrix*VertexCoord; uv=TexCoord; }\n"
        "#elif defined(FRAGMENT)\n"
        "in vec2 uv; out vec4 FragColor; uniform sampler2D Texture;\n"
        "void main() { FragColor=vec4(texture(Texture,uv).rgb,1.0); }\n#endif\n";
    CHECK(g_file_set_contents(source, glsl, -1, &error));
    CHECK(g_file_set_contents(preset, "shaders=1\nshader0=pass.glsl\n", -1, &error));
    NesShaderPreset *shader = nes_shader_create();
    char why[512] = {0};
    bool loaded = nes_shader_load(shader, preset, why, sizeof(why));
    if (!loaded) fprintf(stderr, "GTK shader load: %s\n", why);
    CHECK(loaded && gdk_gl_context_get_current() == context);
    uint32_t pixels[4] = {0xff2266aa, 0xffaa6633, 0xff33aa66, 0xffaa3366};
    NesVideoPresentationFrame input = {0}, output = {0};
    input.pixels = pixels;
    input.width = input.height = 2;
    input.screens = 1;
    for (unsigned i = 0; i < 3; ++i) {
        CHECK(nes_shader_render(shader, &input, 2, 2, &output, why, sizeof(why)));
        CHECK(output.width == 2 && output.height == 2 && !memcmp(output.pixels, pixels, sizeof(pixels)));
        CHECK(gdk_gl_context_get_current() == context);
    }
    /* A rejected replacement destroys its partial GPU state and restores GTK. */
    CHECK(g_file_set_contents(source, "invalid GLSL", -1, &error));
    CHECK(!nes_shader_reload(shader, why, sizeof(why)) && gdk_gl_context_get_current() == context);
    CHECK(nes_shader_render(shader, &input, 2, 2, &output, why, sizeof(why)));
    CHECK(!memcmp(output.pixels, pixels, sizeof(pixels)) && gdk_gl_context_get_current() == context);
    CHECK(g_file_set_contents(source, glsl, -1, &error));
    CHECK(nes_shader_reload(shader, why, sizeof(why)) && gdk_gl_context_get_current() == context);
    nes_shader_destroy(shader);
    CHECK(gdk_gl_context_get_current() == context);
    gdk_gl_context_clear_current();
    g_object_unref(context);
    CHECK(remove(preset) == 0 && remove(source) == 0);
    CHECK(g_rmdir(directory) == 0);
    g_free(preset);
    g_free(source);
    g_free(directory);
    puts("GTK shader pixels and GL context restoration: PASS");
    return true;
}

static bool check_snapshot_pixels(FrontendDesktopUi *ui, GskRenderNode *node) {
    CupidGtkDesktop *d = ui->gtk;
    unsigned width = (unsigned)gtk_widget_get_width(d->picture);
    unsigned height = (unsigned)gtk_widget_get_height(d->picture);
    graphene_rect_t viewport = GRAPHENE_RECT_INIT(0, 0, width, height);
    GskRenderer *renderer = gtk_native_get_renderer(GTK_NATIVE(d->window));
    GdkTexture *texture = gsk_renderer_render_texture(renderer, node, &viewport);
    CHECK(texture && gdk_texture_get_width(texture) == (int)width && gdk_texture_get_height(texture) == (int)height);
    uint32_t *pixels = g_new(uint32_t, (size_t)width *height);
    gdk_texture_download(texture, (guchar *)pixels, width * sizeof(*pixels));
    SDL_Rect game;
    frontend_desktop_game_rect(ui, (int)width, (int)height, 64, 48, ui->settings->integer_scaling, &game);
    const uint32_t colors[] = {0xff2266aa, 0xffaa6633, 0xff33aa66, 0xffaa3366};
    bool same = true;
    for (unsigned quadrant = 0; quadrant < 4; ++quadrant) {
        unsigned x = game.x + game.w * (quadrant % 2 ? 3 : 1) / 4;
        unsigned y = game.y + game.h * (quadrant / 2 ? 3 : 1) / 4;
        if (pixels[(size_t)y * width + x] != colors[quadrant]) {
            fprintf(stderr, "Snapshot quadrant %u at %u,%u: %08x expected %08x\n", quadrant, x, y,
                    pixels[(size_t)y * width + x], colors[quadrant]);
        }
        same &= pixels[(size_t)y * width + x] == colors[quadrant];
    }
    if (game.x > 0 || game.y > 0) {
        /* Cairo and GPU compositors round the float background differently. */
        same &= pixels[0] == 0xff060708 || pixels[0] == 0xff060709;
    }
    g_free(pixels);
    g_object_unref(texture);
    CHECK(same);
    return true;
}

bool test_gtk_game_pixels(FrontendDesktopUi *ui) {
    uint32_t source[64 * 48];
    const uint32_t colors[] = {0x002266aa, 0x80aa6633, 0x0033aa66, 0xffaa3366};
    const uint32_t *saved = ui->video->frame.pixels;
    unsigned width = ui->video->frame.width, height = ui->video->frame.height;
    bool integer = ui->settings->integer_scaling, linear = ui->settings->bilinear_interpolation;
    FrontendAspectMode aspect = ui->settings->aspect_mode;
    ui->video->frame.pixels = source;
    ui->video->frame.width = 64;
    ui->video->frame.height = 48;
    ui->settings->aspect_mode = FRONTEND_ASPECT_SOURCE;
    for (unsigned mode = 0; mode < 2; ++mode) {
        for (unsigned y = 0; y < 48; ++y) {
            for (unsigned x = 0; x < 64; ++x) {
                source[y * 64 + x] = colors[(y >= 24) * 2 + (x >= 32)];
            }
        }
        ui->settings->integer_scaling = mode == 0;
        ui->settings->bilinear_interpolation = mode == 1;
        pump(ui);
        GtkSnapshot *snapshot = gtk_snapshot_new();
        GTK_WIDGET_GET_CLASS(ui->gtk->picture)->snapshot(ui->gtk->picture, snapshot);
        GskRenderNode *node = gtk_snapshot_free_to_node(snapshot);
        CHECK(node && check_snapshot_pixels(ui, node));
        /* GTK may retain a snapshot across the next emulated frame. */
        memset(source, 0, sizeof(source));
        pump(ui);
        CHECK(check_snapshot_pixels(ui, node));
        gsk_render_node_unref(node);
    }
    ui->settings->integer_scaling = integer;
    ui->settings->bilinear_interpolation = linear;
    ui->settings->aspect_mode = aspect;
    ui->video->frame.pixels = saved;
    ui->video->frame.width = width;
    ui->video->frame.height = height;
    pump(ui);
    puts("GTK rendered colors, letterboxing, filtering and retained snapshots: PASS");
    return true;
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
