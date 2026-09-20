/* Clay layout and SDL rendering. SPDX-License-Identifier: GPL-3.0-or-later */
#include "clay_backend.h"
#include "font_atlas.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
struct DesktopClay {
    Clay_Context *context;
    void *arena;
    DesktopFont *font;
    SDL_Renderer *renderer;
    char strings[131072];
    size_t used;
    DesktopHit hits[512];
    int count, first;
    float old_x, old_y;
};
static Clay_Dimensions measure(Clay_StringSlice text, Clay_TextElementConfig *config, void *data) {
    return (Clay_Dimensions){
        desktop_font_measure((DesktopFont *)data, text.chars, text.length, config->fontSize),
        (float)config->fontSize};
}
static void layout_error(Clay_ErrorData error) {
    SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Layout: %.*s", error.errorText.length, error.errorText.chars);
}
DesktopClay *desktop_clay_create(SDL_Renderer *renderer) {
    DesktopClay *clay = calloc(1, sizeof(*clay));
    if (!clay)
        return NULL;
    Clay_SetCurrentContext(NULL);
    uint32_t bytes = Clay_MinMemorySize();
    clay->arena = malloc(bytes);
    clay->renderer = renderer;
    clay->font = desktop_font_create(renderer);
    if (!clay->arena || !clay->font) {
        desktop_clay_destroy(clay);
        return NULL;
    }
    clay->context = Clay_Initialize(Clay_CreateArenaWithCapacityAndMemory(bytes, clay->arena),
                                    (Clay_Dimensions){640, 480}, (Clay_ErrorHandler){layout_error, 0});
    Clay_SetMeasureTextFunction(measure, clay->font);
    return clay;
}
void desktop_clay_destroy(DesktopClay *clay) {
    if (clay) {
        if (Clay_GetCurrentContext() == clay->context)
            Clay_SetCurrentContext(NULL);
        desktop_font_destroy(clay->font);
        free(clay->arena);
        free(clay);
    }
}
void desktop_clay_begin(DesktopClay *clay, float width, float height, float scale) {
    Clay_SetCurrentContext(clay->context);
    Clay_SetLayoutDimensions((Clay_Dimensions){width, height});
    int x, y;
    Uint32 buttons = SDL_GetMouseState(&x, &y);
    Clay_SetPointerState((Clay_Vector2){x / scale, y / scale}, (buttons & SDL_BUTTON_LMASK) != 0);
    clay->used = 0;
    clay->count = clay->first = 0;
    SDL_RenderGetScale(clay->renderer, &clay->old_x, &clay->old_y);
    SDL_RenderSetScale(clay->renderer, clay->old_x * scale, clay->old_y * scale);
    Clay_BeginLayout();
}
Clay_String desktop_clay_string(DesktopClay *clay, const char *text) {
    if (!text)
        text = "";
    size_t length = strlen(text);
    if (length + 1 > sizeof(clay->strings) - clay->used)
        return CLAY_STRING("...");
    char *copy = clay->strings + clay->used;
    memcpy(copy, text, length + 1);
    clay->used += length + 1;
    return (Clay_String){.length = (int32_t)length, .chars = copy};
}
Clay_ElementId desktop_clay_hit(DesktopClay *clay, int kind, int index, int direction) {
    Clay_ElementId id = Clay_GetElementIdWithIndex(CLAY_STRING("control"), (uint32_t)clay->count);
    if (clay->count < 512)
        clay->hits[clay->count++] =
            (DesktopHit){.id = id, .kind = kind, .index = index, .direction = direction};
    return id;
}
void desktop_clay_block(DesktopClay *clay) { clay->first = clay->count; }
const DesktopHit *desktop_clay_at(DesktopClay *clay, float x, float y) {
    if (!clay)
        return NULL;
    for (int i = clay->count - 1; i >= clay->first; --i) {
        DesktopHit *hit = &clay->hits[i];
        Clay_BoundingBox b = hit->bounds;
        if (x >= b.x && y >= b.y && x < b.x + b.width && y < b.y + b.height)
            return hit;
    }
    return NULL;
}
bool desktop_clay_bounds(DesktopClay *clay, int kind, int index, int direction, SDL_FRect *bounds) {
    if (!clay || !bounds)
        return false;
    for (int i = clay->first; i < clay->count; ++i) {
        DesktopHit *h = &clay->hits[i];
        if (h->kind == kind && h->index == index && h->direction == direction) {
            *bounds = (SDL_FRect){h->bounds.x, h->bounds.y, h->bounds.width, h->bounds.height};
            return true;
        }
    }
    return false;
}
static SDL_Color color(Clay_Color c) { return (SDL_Color){(Uint8)c.r, (Uint8)c.g, (Uint8)c.b, (Uint8)c.a}; }
static void rectangle(SDL_Renderer *renderer, Clay_BoundingBox b, Clay_Color c, float radius) {
    SDL_Color tint = color(c);
    SDL_SetRenderDrawColor(renderer, tint.r, tint.g, tint.b, tint.a);
    if (radius < 1) {
        SDL_FRect rect = {b.x, b.y, b.width, b.height};
        SDL_RenderFillRectF(renderer, &rect);
        return;
    }
    radius = fminf(radius, fminf(b.width, b.height) / 2);
    SDL_Vertex vertices[37 * 3];
    int count = 0;
    SDL_FPoint center = {b.x + b.width / 2, b.y + b.height / 2};
    SDL_FPoint previous = {b.x + b.width - radius, b.y};
    for (int corner = 0; corner < 4; ++corner)
        for (int step = 0; step <= 8; ++step) {
            float angle = (-90 + corner * 90 + step * 90.0f / 8) * 0.01745329252f;
            float cx = b.x + (corner < 2 ? b.width - radius : radius),
                  cy = b.y + (corner == 0 || corner == 3 ? radius : b.height - radius);
            SDL_FPoint next = {cx + cosf(angle) * radius, cy + sinf(angle) * radius};
            vertices[count++] = (SDL_Vertex){center, tint, {0, 0}};
            vertices[count++] = (SDL_Vertex){previous, tint, {0, 0}};
            vertices[count++] = (SDL_Vertex){next, tint, {0, 0}};
            previous = next;
        }
    vertices[count++] = (SDL_Vertex){center, tint, {0, 0}};
    vertices[count++] = (SDL_Vertex){previous, tint, {0, 0}};
    vertices[count++] = (SDL_Vertex){{b.x + b.width - radius, b.y}, tint, {0, 0}};
    SDL_RenderGeometry(renderer, NULL, vertices, count, NULL, 0);
}
void desktop_clay_end(DesktopClay *clay) {
    Clay_RenderCommandArray commands = Clay_EndLayout(1.0f / 60);
    SDL_Renderer *renderer = clay->renderer;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_Rect clips[32];
    int depth = 0;
    for (int32_t i = 0; i < commands.length; ++i) {
        Clay_RenderCommand *c = Clay_RenderCommandArray_Get(&commands, i);
        Clay_BoundingBox b = c->boundingBox;
        switch (c->commandType) {
        case CLAY_RENDER_COMMAND_TYPE_RECTANGLE:
            rectangle(renderer, b, c->renderData.rectangle.backgroundColor,
                      c->renderData.rectangle.cornerRadius.topLeft);
            break;
        case CLAY_RENDER_COMMAND_TYPE_TEXT: {
            Clay_TextRenderData *t = &c->renderData.text;
            desktop_font_draw(clay->font, renderer, t->stringContents.chars, t->stringContents.length, b.x,
                              b.y, t->fontSize, color(t->textColor));
            break;
        }
        case CLAY_RENDER_COMMAND_TYPE_SCISSOR_START: {
            SDL_Rect next = {(int)b.x, (int)b.y, (int)ceilf(b.width), (int)ceilf(b.height)};
            if (depth)
                SDL_IntersectRect(&clips[depth - 1], &next, &next);
            if (depth < 32)
                clips[depth++] = next;
            SDL_RenderSetClipRect(renderer, &next);
            break;
        }
        case CLAY_RENDER_COMMAND_TYPE_SCISSOR_END:
            if (depth)
                --depth;
            SDL_RenderSetClipRect(renderer, depth ? &clips[depth - 1] : NULL);
            break;
        case CLAY_RENDER_COMMAND_TYPE_BORDER: {
            Clay_BorderRenderData *d = &c->renderData.border;
            rectangle(renderer, (Clay_BoundingBox){b.x, b.y, b.width, d->width.top}, d->color, 0);
            rectangle(renderer,
                      (Clay_BoundingBox){b.x, b.y + b.height - d->width.bottom, b.width, d->width.bottom},
                      d->color, 0);
            rectangle(renderer, (Clay_BoundingBox){b.x, b.y, d->width.left, b.height}, d->color, 0);
            rectangle(renderer,
                      (Clay_BoundingBox){b.x + b.width - d->width.right, b.y, d->width.right, b.height},
                      d->color, 0);
            break;
        }
        default:
            break;
        }
    }
    for (int i = 0; i < clay->count; ++i)
        clay->hits[i].bounds = Clay_GetElementData(clay->hits[i].id).boundingBox;
    SDL_RenderSetClipRect(renderer, NULL);
    SDL_RenderSetScale(renderer, clay->old_x, clay->old_y);
}
