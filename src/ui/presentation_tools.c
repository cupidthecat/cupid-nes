/*
 * presentation_tools.c - Shader, HD draft, and native audio panels
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "presentation_tools.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "output_guard.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct FrontendPresentationTools {
    FrontendVideoRuntime *video;
    FrontendAudioRuntime *audio;
    void (*changed)(void *);
    void *context;
    char status[512], member[1024], line[16385], line_number[24], parameter_value[40], buffer[24];
    char discovered[128][1024];
    const char *presets[128], *parameters[64], *lines[128], *devices[129];
    size_t preset_count, selected_line, page, selected_parameter;
    SDL_Window *preview;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    unsigned width, height;
};

static bool fail(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

static bool add(FrontendPanelModel *model, unsigned id, FrontendPanelControlType type, const char *label,
                const char *value, const char *const *items, size_t count, int selected, bool enabled) {
    if (model->count >= model->capacity) {
        return false;
    }
    model->controls[model->count++] = (FrontendPanelControl){.id = id,
                                                             .type = type,
                                                             .label = label,
                                                             .value = value,
                                                             .items = items,
                                                             .item_count = count,
                                                             .selected = selected,
                                                             .enabled = enabled};
    return true;
}

static bool numeric(const char *text, size_t maximum, size_t *out) {
    if (!text || !*text || text[0] == '-') {
        return false;
    }
    char *end;
    errno = 0;
    unsigned long long value = strtoull(text, &end, 10);
    if (errno || *end || value > maximum) {
        return false;
    }
    *out = (size_t)value;
    return true;
}

static void close_preview(FrontendPresentationTools *tools) {
    SDL_DestroyTexture(tools->texture);
    SDL_DestroyRenderer(tools->renderer);
    SDL_DestroyWindow(tools->preview);
    tools->texture = NULL;
    tools->renderer = NULL;
    tools->preview = NULL;
    tools->video->builder_preview = false;
    (void)nes_video_trace_use(NES_VIDEO_TRACE_BUILDER, false);
}

static bool hd_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendPresentationTools *tools = context;
    if (model->capacity < 13) {
        return fail(error, size, "HD builder panel requires 13 controls");
    }
    model->count = 0;
    size_t count = nes_hd_builder_line_count(tools->video->builder);
    if (tools->selected_line > count) {
        tools->selected_line = count;
    }
    tools->page = tools->selected_line / 128 * 128;
    size_t visible = count > tools->page ? count - tools->page : 0;
    if (visible > 128) {
        visible = 128;
    }
    for (size_t i = 0; i < visible; ++i) {
        tools->lines[i] = nes_hd_builder_line(tools->video->builder, tools->page + i);
    }
    snprintf(tools->line_number, sizeof(tools->line_number), "%zu", tools->selected_line + 1);
    add(model, 0x2741, FRONTEND_PANEL_ACTION, "New 2x draft", NULL, NULL, 0, 0, true);
    add(model, 0x2742, FRONTEND_PANEL_FILE_OPEN, "Open pack ZIP", NULL, NULL, 0, FRONTEND_OPEN_HD_PACK, true);
    add(model, 0x2743, FRONTEND_PANEL_LIST, "Definition lines (current page)", NULL, tools->lines, visible,
        (int)(tools->selected_line - tools->page), true);
    add(model, 0x2744, FRONTEND_PANEL_TEXT, "Line number (count + 1 appends)", tools->line_number, NULL, 0, 0, true);
    add(model, 0x2745, FRONTEND_PANEL_TEXT, "Format-109 definition line", tools->line, NULL, 0, 0, true);
    add(model, 0x2746, FRONTEND_PANEL_ACTION, "Apply line", NULL, NULL, 0, 0, true);
    add(model, 0x2747, FRONTEND_PANEL_ACTION, "Delete line", NULL, NULL, 0, 0, tools->selected_line < count);
    add(model, 0x2748, FRONTEND_PANEL_TEXT, "Asset member name", tools->member, NULL, 0, 0, true);
    add(model, 0x2749, FRONTEND_PANEL_FILE_OPEN, "Import PNG / WAV / OGG / palette", NULL, NULL, 0,
        FRONTEND_OPEN_MEMORY, true);
    add(model, 0x274a, FRONTEND_PANEL_ACTION, "Validate draft", NULL, NULL, 0, 0, true);
    add(model, 0x274b, FRONTEND_PANEL_ACTION, "Open / close live preview", NULL, NULL, 0, 0, true);
    add(model, 0x274c, FRONTEND_PANEL_FILE_SAVE, "Export validated pack ZIP", NULL, NULL, 0, FRONTEND_SAVE_HD_PACK,
        true);
    add(model, 0x274d, FRONTEND_PANEL_TEXT, "Supported authoring",
        "Tiles, backgrounds, conditions, additions, fallbacks, layers, metadata, audio", NULL, 0, 0, false);
    model->status = tools->video->builder_error[0] ? tools->video->builder_error : tools->status;
    return true;
}

static bool hd_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    FrontendPresentationTools *tools = context;
    NesHdBuilder *builder = tools->video->builder;
    bool result = false;
    if (id == 0x2741) {
        result = nes_hd_builder_new(builder, 2, error, size);
        tools->selected_line = 0;
    } else if (id == 0x2742) {
        result = nes_hd_builder_open(builder, value, error, size);
        tools->selected_line = 0;
    } else if (id == 0x2743 || id == 0x2744) {
        size_t index = 0, count = nes_hd_builder_line_count(builder);
        if (id == 0x2743) {
            if (selected < 0 || (size_t)selected + tools->page >= count) {
                return false;
            }
            index = (size_t)selected + tools->page;
        } else {
            if (!numeric(value, count + 1, &index) || !index) {
                return fail(error, size, "Line number is outside the draft");
            }
            --index;
        }
        tools->selected_line = index;
        const char *line = nes_hd_builder_line(builder, index);
        snprintf(tools->line, sizeof(tools->line), "%s", line ? line : "");
        return true;
    } else if (id == 0x2745 || id == 0x2748) {
        char *target = id == 0x2745 ? tools->line : tools->member;
        size_t capacity = id == 0x2745 ? sizeof(tools->line) : sizeof(tools->member);
        if (!value || strlen(value) >= capacity) {
            return fail(error, size, "Text is too long");
        }
        snprintf(target, capacity, "%s", value);
        return true;
    } else if (id == 0x2746 || id == 0x2747) {
        result = nes_hd_builder_set_line(builder, tools->selected_line, id == 0x2746 ? tools->line : NULL, error, size);
    } else if (id == 0x2749) {
        result = nes_hd_builder_import_asset(builder, tools->member, value, error, size);
    } else if (id == 0x274a) {
        result = nes_hd_builder_validate(builder, error, size);
    } else if (id == 0x274c) {
        if (!frontend_output_path_allowed(value, tools->audio->execution, NULL, 0, error, size)) {
            return false;
        }
        result = nes_hd_builder_export(builder, value, error, size);
    } else if (id == 0x274b) {
        if (tools->preview) {
            close_preview(tools);
            return true;
        }
        if (!nes_hd_builder_validate(builder, error, size) || !nes_video_trace_use(NES_VIDEO_TRACE_BUILDER, true)) {
            return false;
        }
        tools->preview = SDL_CreateWindow("HD Draft Preview", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 768, 720,
                                          SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        if (tools->preview) {
            tools->renderer = SDL_CreateRenderer(tools->preview, -1, 0);
        }
        if (!tools->renderer) {
            fail(error, size, SDL_GetError());
            close_preview(tools);
            return false;
        }
        tools->video->builder_preview = true;
        return true;
    }
    if (result) {
        if (id == 0x2741 || id == 0x2742) {
            memset(&tools->video->builder_frame, 0, sizeof(tools->video->builder_frame));
        }
        snprintf(tools->status, sizeof(tools->status), "Draft updated; validate edits before preview or export.");
    }
    return result;
}

static bool shader_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendPresentationTools *tools = context;
    NesShaderPreset *shader = tools->video->shader;
    if (model->capacity < 8) {
        return fail(error, size, "Shader panel requires eight controls");
    }
    model->count = 0;
    size_t count = nes_shader_parameter_count(shader);
    for (size_t i = 0; i < count && i < 64; ++i) {
        NesShaderParameter p;
        nes_shader_parameter(shader, i, &p);
        tools->parameters[i] = p.label;
    }
    if (tools->selected_parameter >= count) {
        tools->selected_parameter = 0;
    }
    NesShaderParameter parameter;
    if (nes_shader_parameter(shader, tools->selected_parameter, &parameter)) {
        snprintf(tools->parameter_value, sizeof(tools->parameter_value), "%.6g", (double)parameter.value);
    }
    add(model, 0x2781, FRONTEND_PANEL_FILE_OPEN, "Load GLSL preset (.glslp)", tools->video->shader_requested_path, NULL,
        0, FRONTEND_OPEN_MEMORY, true);
    add(model, 0x2782, FRONTEND_PANEL_DIRECTORY, "Discover preset directory", NULL, NULL, 0, 0, true);
    add(model, 0x2783, FRONTEND_PANEL_CHOICE, "Discovered presets", NULL, tools->presets, tools->preset_count, -1,
        tools->preset_count > 0);
    add(model, 0x2784, FRONTEND_PANEL_ACTION, "Reload preset", NULL, NULL, 0, 0,
        tools->video->shader_requested_path[0] != 0);
    add(model, 0x2785, FRONTEND_PANEL_CHECKBOX, "Enable GPU preset", NULL, NULL, 0, nes_shader_enabled(shader), true);
    add(model, 0x2786, FRONTEND_PANEL_CHOICE, "Parameter", NULL, tools->parameters, count,
        (int)tools->selected_parameter, count > 0);
    add(model, 0x2787, FRONTEND_PANEL_TEXT, "Parameter value", tools->parameter_value, NULL, 0, 0, count > 0);
    add(model, 0x2788, FRONTEND_PANEL_TEXT, "GPU format", "GLSL multi-pass; software output stays available on failure",
        NULL, 0, 0, false);
    model->status = tools->video->shader_error[0] ? tools->video->shader_error : tools->status;
    return true;
}

static bool shader_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    FrontendPresentationTools *tools = context;
    NesShaderPreset *shader = tools->video->shader;
    bool result = false;
    if (id == 0x2781) {
        result = frontend_video_runtime_load_shader(tools->video, value, error, size);
    } else if (id == 0x2782) {
        result = nes_shader_discover(value, tools->discovered, 128, &tools->preset_count, error, size);
        for (size_t i = 0; i < tools->preset_count; ++i) {
            tools->presets[i] = tools->discovered[i];
        }
    } else if (id == 0x2783 && selected >= 0 && (size_t)selected < tools->preset_count) {
        result = frontend_video_runtime_load_shader(tools->video, tools->presets[selected], error, size);
    } else if (id == 0x2784) {
        result = frontend_video_runtime_load_shader(tools->video, tools->video->shader_requested_path, error, size);
    } else if (id == 0x2785) {
        if (selected && (!tools->video->shader_requested_path[0] || tools->video->shader_error[0] ||
                         strcmp(tools->video->shader_requested_path, nes_shader_path(shader)))) {
            result = frontend_video_runtime_load_shader(tools->video, tools->video->shader_requested_path, error, size);
        } else {
            nes_shader_enable(shader, selected != 0);
            result = true;
        }
    } else if (id == 0x2786 && selected >= 0 && (size_t)selected < nes_shader_parameter_count(shader)) {
        tools->selected_parameter = (size_t)selected;
        return true;
    } else if (id == 0x2787 && value) {
        char *end = NULL;
        float number = strtof(value, &end);
        if (end == value || *end || !isfinite(number)) {
            return fail(error, size, "Enter a finite parameter value");
        }
        result = nes_shader_set_parameter(shader, tools->selected_parameter, number, error, size);
    }
    if (result && tools->changed) {
        tools->changed(tools->context);
    }
    return result;
}

static const char *const backends[] = {"default", "wasapi", "directsound"};

static bool audio_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendPresentationTools *tools = context;
    if (model->capacity < 4) {
        return fail(error, size, "Audio panel requires four controls");
    }
    model->count = 0;
    int current = 0;
    for (int i = 0; i < 3; ++i) {
        if (!strcmp(frontend_audio_runtime_backend(tools->audio), backends[i])) {
            current = i;
        }
    }
    int devices = SDL_GetNumAudioDevices(0);
    if (devices < 0) {
        devices = 0;
    }
    if (devices > 128) {
        devices = 128;
    }
    tools->devices[0] = "System default";
    int selected = 0;
    for (int i = 0; i < devices; ++i) {
        tools->devices[i + 1] = SDL_GetAudioDeviceName(i, 0);
        if (tools->devices[i + 1] && !strcmp(tools->video->settings->audio_device, tools->devices[i + 1])) {
            selected = i + 1;
        }
    }
    snprintf(tools->buffer, sizeof(tools->buffer), "%u", tools->video->settings->audio_buffer_samples);
    add(model, 0x27c1, FRONTEND_PANEL_CHOICE, "Windows output backend", NULL, backends, 3, current, true);
    add(model, 0x27c2, FRONTEND_PANEL_CHOICE, "Output device", NULL, tools->devices, (size_t)devices + 1, selected,
        true);
    add(model, 0x27c3, FRONTEND_PANEL_TEXT, "Buffer samples (64 to 8192)", tools->buffer, NULL, 0, 0, true);
    const char *driver = SDL_GetCurrentAudioDriver();
    add(model, 0x27c4, FRONTEND_PANEL_TEXT, "Active SDL native driver (shared mode)",
        driver ? driver : "No audio driver", NULL, 0, 0, false);
    model->status = tools->status;
    return true;
}

static bool audio_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    FrontendPresentationTools *tools = context;
    FrontendSettings next = *tools->video->settings;
    bool result = false;
    if (id == 0x27c1 && selected >= 0 && selected < 3) {
        next.audio_device[0] = 0;
        result = frontend_audio_runtime_select_backend(tools->audio, &next, backends[selected], error, size);
    } else if (id == 0x27c2 && selected >= 0 && selected <= SDL_GetNumAudioDevices(0)) {
        const char *name = selected ? SDL_GetAudioDeviceName(selected - 1, 0) : "";
        if (!name || strlen(name) >= sizeof(next.audio_device)) {
            return fail(error, size, "Audio device name is unavailable or too long");
        }
        snprintf(next.audio_device, sizeof(next.audio_device), "%s", name);
        result = frontend_audio_runtime_apply(tools->audio, &next, false, error, size);
    } else if (id == 0x27c3) {
        size_t samples;
        if (!numeric(value, 8192, &samples) || samples < 64) {
            return fail(error, size, "Buffer must be from 64 to 8192 samples");
        }
        next.audio_buffer_samples = (unsigned)samples;
        result = frontend_audio_runtime_apply(tools->audio, &next, false, error, size);
    }
    if (result) {
        *tools->video->settings = next;
    }
    if (tools->changed) {
        tools->changed(tools->context);
    }
    return result;
}

FrontendPresentationTools *frontend_presentation_tools_create(FrontendVideoRuntime *video, FrontendAudioRuntime *audio,
                                                              void (*changed)(void *), void *context) {
    if (!video || !audio) {
        return NULL;
    }
    FrontendPresentationTools *tools = calloc(1, sizeof(*tools));
    if (!tools) {
        return NULL;
    }
    tools->video = video;
    tools->audio = audio;
    tools->changed = changed;
    tools->context = context;
    FrontendPanelSpec specs[] = {
        {0x2740, "HD Pack Builder", "Tools", FRONTEND_PANEL_NEEDS_SESSION, hd_snapshot, hd_action, tools},
        {0x2780, "Shader Presets", "Video", 0, shader_snapshot, shader_action, tools},
        {0x27c0, "Native Audio Output", "Audio", 0, audio_snapshot, audio_action, tools}};
    for (size_t i = 0; i < 3; ++i) {
        if (!frontend_panel_register(&specs[i])) {
            for (size_t j = 0; j < i; ++j) {
                frontend_panel_unregister(specs[j].id);
            }
            free(tools);
            return NULL;
        }
    }
    return tools;
}

void frontend_presentation_tools_destroy(FrontendPresentationTools *tools) {
    if (!tools) {
        return;
    }
    close_preview(tools);
    frontend_panel_unregister(0x2740);
    frontend_panel_unregister(0x2780);
    frontend_panel_unregister(0x27c0);
    free(tools);
}

bool frontend_presentation_tools_event(FrontendPresentationTools *tools, const SDL_Event *event) {
    if (!tools || !tools->preview || !event) {
        return false;
    }
    if (event->type == SDL_WINDOWEVENT && event->window.windowID == SDL_GetWindowID(tools->preview)) {
        if (event->window.event == SDL_WINDOWEVENT_CLOSE) {
            close_preview(tools);
        }
        return true;
    }
    return false;
}

void frontend_presentation_tools_tick(FrontendPresentationTools *tools) {
    if (!tools || !tools->preview) {
        return;
    }
    if (!tools->video->builder_preview) {
        close_preview(tools);
        return;
    }
    NesHdFrame *frame = &tools->video->builder_frame;
    if (!frame->pixels || !frame->width || !frame->height) {
        return;
    }
    if (!tools->texture || tools->width != frame->width || tools->height != frame->height) {
        SDL_Texture *next = SDL_CreateTexture(tools->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                              (int)frame->width, (int)frame->height);
        if (!next) {
            return;
        }
        SDL_DestroyTexture(tools->texture);
        tools->texture = next;
        tools->width = frame->width;
        tools->height = frame->height;
    }
    if (SDL_UpdateTexture(tools->texture, NULL, frame->pixels, (int)(frame->width * sizeof(uint32_t))) != 0) {
        return;
    }
    SDL_RenderClear(tools->renderer);
    SDL_RenderCopy(tools->renderer, tools->texture, NULL, NULL);
    SDL_RenderPresent(tools->renderer);
}

bool frontend_presentation_restore_settings(FrontendVideoRuntime *video, FrontendAudioRuntime *audio,
                                            const FrontendSettings *settings, char *error, size_t size) {
    if (!video || !audio || !settings || settings->shader_parameter_count > NES_SHADER_MAX_PARAMETERS) {
        return fail(error, size, "Presentation settings are incomplete");
    }
    char audio_error[512] = {0};
    bool audio_ok = frontend_audio_runtime_select_backend(audio, settings, settings->audio_backend, audio_error,
                                                          sizeof(audio_error));
    bool shader_ok = true;
    if (!settings->shader_path[0]) {
        video->shader_requested_path[0] = 0;
        video->shader_error[0] = 0;
        nes_shader_enable(video->shader, false);
    } else {
        shader_ok = frontend_video_runtime_load_shader(video, settings->shader_path, video->shader_error,
                                                       sizeof(video->shader_error));
        if (shader_ok) {
            NesShaderParameterValue values[NES_SHADER_MAX_PARAMETERS];
            for (size_t i = 0; i < settings->shader_parameter_count; ++i) {
                memcpy(values[i].name, settings->shader_parameters[i].name, sizeof(values[i].name));
                values[i].value = settings->shader_parameters[i].value;
            }
            shader_ok = nes_shader_apply_parameters(video->shader, values, settings->shader_parameter_count,
                                                    video->shader_error, sizeof(video->shader_error));
        }
        nes_shader_enable(video->shader, shader_ok && settings->shader_enabled);
    }
    if (!audio_ok || !shader_ok) {
        if (error && size) {
            snprintf(error, size, "%s%s%s", audio_error, !audio_ok && !shader_ok ? "; " : "",
                     shader_ok ? "" : video->shader_error);
        }
        return false;
    }
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool frontend_presentation_capture_settings(const FrontendVideoRuntime *video, const FrontendAudioRuntime *audio,
                                            FrontendSettings *settings) {
    if (!video || !audio || !settings) {
        return false;
    }
    NesShaderParameterValue values[NES_SHADER_MAX_PARAMETERS];
    size_t count = 0;
    bool loaded =
        video->shader_requested_path[0] && !strcmp(video->shader_requested_path, nes_shader_path(video->shader));
    if (loaded && !nes_shader_capture_parameters(video->shader, values, NES_SHADER_MAX_PARAMETERS, &count)) {
        return false;
    }
    snprintf(settings->audio_backend, sizeof(settings->audio_backend), "%s", frontend_audio_runtime_backend(audio));
    snprintf(settings->shader_path, sizeof(settings->shader_path), "%s", video->shader_requested_path);
    settings->shader_enabled = nes_shader_enabled(video->shader);
    if (loaded || !settings->shader_path[0]) {
        settings->shader_parameter_count = count;
        for (size_t i = 0; i < count; ++i) {
            memcpy(settings->shader_parameters[i].name, values[i].name, sizeof(values[i].name));
            settings->shader_parameters[i].value = values[i].value;
        }
    }
    return true;
}
