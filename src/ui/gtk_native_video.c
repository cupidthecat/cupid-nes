/*
 * gtk_native_video.c - Accelerated game surface inside the Windows desktop
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#ifdef _WIN32
#include <gdk/win32/gdkwin32.h>
#include <math.h>
#include <string.h>

typedef struct CupidGtkNativeVideo {
    HWND handle;
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    unsigned width, height;
    SDL_Rect bounds;
    SDL_Rect game;
    char name[80];
} CupidGtkNativeVideo;

void cupid_gtk_native_video_destroy(CupidGtkDesktop *d) {
    CupidGtkNativeVideo *v = d->native_video;
    if (!v) return;
    SDL_DestroyTexture(v->texture);
    SDL_DestroyRenderer(v->renderer);
    SDL_DestroyWindow(v->window);
    DestroyWindow(v->handle);
    g_free(v);
    d->native_video = NULL;
}

void cupid_gtk_native_video_hide(CupidGtkDesktop *d) {
    if (d->native_video && IsWindowVisible(d->native_video->handle))
        ShowWindow(d->native_video->handle, SW_HIDE);
}

const char *cupid_gtk_native_video_name(CupidGtkDesktop *d) {
    return d->native_video ? d->native_video->name : NULL;
}

static bool create_video(CupidGtkDesktop *d, HWND parent) {
    CupidGtkNativeVideo *v = g_new0(CupidGtkNativeVideo, 1);
    d->native_video = v;
    /* A disabled child passes mouse and keyboard input through to GTK. */
    v->handle = CreateWindowExW(WS_EX_NOACTIVATE, L"STATIC", L"Cupid game view",
        WS_CHILD | WS_DISABLED | WS_CLIPSIBLINGS, 0, 0, 1, 1, parent, NULL, GetModuleHandleW(NULL), NULL);
    if (!v->handle) return false;
    v->window = SDL_CreateWindowFrom(v->handle);
    if (!v->window) return false;
    for (int i = 0; i < SDL_GetNumRenderDrivers(); ++i) {
        SDL_RendererInfo info;
        if (!SDL_GetRenderDriverInfo(i, &info) && !strcmp(info.name, "direct3d11")) {
            v->renderer = SDL_CreateRenderer(v->window, i, SDL_RENDERER_ACCELERATED);
            break;
        }
    }
    if (!v->renderer) v->renderer = SDL_CreateRenderer(v->window, -1, SDL_RENDERER_ACCELERATED);
    if (!v->renderer) return false;
    SDL_RendererInfo info;
    if (SDL_GetRendererInfo(v->renderer, &info) || !(info.flags & SDL_RENDERER_ACCELERATED)) return false;
    g_snprintf(v->name, sizeof(v->name), "SDL %s (accelerated)", info.name);
    /* The desktop compositor presents this window. A second blocking VSync
     * clock here would compete with the emulation deadline and audio clock. */
    (void)SDL_RenderSetVSync(v->renderer, 0);
    SetWindowLongPtrW(parent, GWL_STYLE, GetWindowLongPtrW(parent, GWL_STYLE) | WS_CLIPCHILDREN);
    return true;
}

bool cupid_gtk_native_video_present(CupidGtkDesktop *d, const uint32_t *pixels,
                                   unsigned width, unsigned height, unsigned stride) {
    if (d->native_video_failed || !pixels || !width || !height || stride < width) return false;
    /* An explicit software override remains a way to diagnose driver problems. */
    if (!g_strcmp0(g_getenv("GSK_RENDERER"), "cairo")) return false;
    GtkNative *native = GTK_NATIVE(d->window);
    GdkSurface *surface = gtk_native_get_surface(native);
    GskRenderer *renderer = gtk_native_get_renderer(native);
    if (!surface || !GDK_IS_WIN32_SURFACE(surface) || !GSK_IS_CAIRO_RENDERER(renderer)) return false;
    graphene_rect_t bounds;
    if (!gtk_widget_compute_bounds(d->picture, d->window, &bounds) ||
        bounds.size.width < 1 || bounds.size.height < 1) return false;
    HWND parent = gdk_win32_surface_get_handle(surface);
    RECT client;
    if (!GetClientRect(parent, &client)) return false;
    int sw = gdk_surface_get_width(surface), sh = gdk_surface_get_height(surface);
    if (sw <= 0 || sh <= 0) return false;
    double tx, ty;
    gtk_native_get_surface_transform(native, &tx, &ty);
    double sx = (double)(client.right - client.left) / sw;
    double sy = (double)(client.bottom - client.top) / sh;
    /* GtkWindow's content allocation excludes its client-side shadow margins.
     * The child HWND uses surface coordinates, so include those margins. */
    SDL_Rect r = {(int)lround((bounds.origin.x + tx) * sx), (int)lround((bounds.origin.y + ty) * sy),
                  (int)lround(bounds.size.width * sx), (int)lround(bounds.size.height * sy)};
    if (r.w <= 0 || r.h <= 0) return false;
    if (!d->native_video && !create_video(d, parent)) goto failed;
    CupidGtkNativeVideo *v = d->native_video;
    if (memcmp(&v->bounds, &r, sizeof(r))) {
        if (!SetWindowPos(v->handle, NULL, r.x, r.y, r.w, r.h, SWP_NOACTIVATE | SWP_NOZORDER)) goto failed;
        v->bounds = r;
    }
    if (!v->texture || v->width != width || v->height != height) {
        SDL_Texture *texture = SDL_CreateTexture(v->renderer, SDL_PIXELFORMAT_ARGB8888,
                                                SDL_TEXTUREACCESS_STREAMING, (int)width, (int)height);
        if (!texture) goto failed;
        SDL_DestroyTexture(v->texture);
        v->texture = texture;
        v->width = width;
        v->height = height;
        SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);
    }
    SDL_SetTextureScaleMode(v->texture, d->ui->settings->bilinear_interpolation ? SDL_ScaleModeLinear : SDL_ScaleModeNearest);
    if (SDL_UpdateTexture(v->texture, NULL, pixels, (int)(stride * sizeof(*pixels)))) goto failed;
    unsigned vw, vh;
    frontend_video_runtime_display_size(d->ui->video, &vw, &vh);
    SDL_Rect game;
    frontend_desktop_game_rect(d->ui, r.w, r.h, (int)vw, (int)vh, d->ui->settings->integer_scaling, &game);
    v->game = game;
    SDL_SetRenderDrawColor(v->renderer, 6, 7, 9, 255);
    if (SDL_RenderClear(v->renderer) || SDL_RenderCopy(v->renderer, v->texture, NULL, &game)) goto failed;
    if (!IsWindowVisible(v->handle)) ShowWindow(v->handle, SW_SHOWNA);
    SDL_RenderPresent(v->renderer);
    cupid_gtk_record_draw(d);
    return true;
failed:
    cupid_gtk_native_video_destroy(d);
    d->native_video_failed = true;
    return false;
}

bool cupid_gtk_native_video_read(CupidGtkDesktop *d, uint32_t **pixels, unsigned *width, unsigned *height) {
    CupidGtkNativeVideo *v = d->native_video;
    if (!v || !pixels || !width || !height) return false;
    *width = (unsigned)v->bounds.w;
    *height = (unsigned)v->bounds.h;
    *pixels = g_try_new(uint32_t, (size_t)*width * *height);
    if (!*pixels) return false;
    /* Swapchain backbuffers are undefined after Present; redraw before readback. */
    if (SDL_RenderClear(v->renderer) || SDL_RenderCopy(v->renderer, v->texture, NULL, &v->game) ||
        SDL_RenderReadPixels(v->renderer, NULL, SDL_PIXELFORMAT_ARGB8888, *pixels, (int)(*width * sizeof(**pixels)))) {
        g_free(*pixels);
        *pixels = NULL;
        return false;
    }
    SDL_RenderPresent(v->renderer);
    return true;
}
#else
void cupid_gtk_native_video_destroy(CupidGtkDesktop *d) { (void)d; }
void cupid_gtk_native_video_hide(CupidGtkDesktop *d) { (void)d; }
const char *cupid_gtk_native_video_name(CupidGtkDesktop *d) { (void)d; return NULL; }
bool cupid_gtk_native_video_present(CupidGtkDesktop *d, const uint32_t *p, unsigned w, unsigned h, unsigned s) {
    (void)d; (void)p; (void)w; (void)h; (void)s; return false;
}
bool cupid_gtk_native_video_read(CupidGtkDesktop *d, uint32_t **p, unsigned *w, unsigned *h) {
    (void)d; (void)p; (void)w; (void)h; return false;
}
#endif
