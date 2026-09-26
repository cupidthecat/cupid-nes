/*
 * shader_preset.h - GPU GLSL preset presentation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_SHADER_PRESET_H
#define CUPID_SHADER_PRESET_H
#include "presentation.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct NesShaderPreset NesShaderPreset;

typedef struct {
    const char *name, *label;
    float value, minimum, maximum, step;
} NesShaderParameter;

enum { NES_SHADER_MAX_PARAMETERS = 64 };

typedef struct {
    char name[64];
    float value;
} NesShaderParameterValue;

/* Named settings records survive parameter reordering between preset reloads.
 * Apply is transactional; unknown names, duplicates and invalid values fail. */
bool nes_shader_apply_parameters(NesShaderPreset *shader, const NesShaderParameterValue *values, size_t count,
                                 char *error, size_t size);
bool nes_shader_capture_parameters(const NesShaderPreset *shader, NesShaderParameterValue *values, size_t capacity,
                                   size_t *count);
NesShaderPreset *nes_shader_create(void);
void nes_shader_destroy(NesShaderPreset *shader);
/* Loading validates resources and compiles every pass before replacing the
 * active preset. The caller can disable on failure for ordinary rendering. */
bool nes_shader_load(NesShaderPreset *shader, const char *path, char *error, size_t size);
bool nes_shader_reload(NesShaderPreset *shader, char *error, size_t size);
void nes_shader_enable(NesShaderPreset *shader, bool enabled);
bool nes_shader_enabled(const NesShaderPreset *shader);
const char *nes_shader_path(const NesShaderPreset *shader);
size_t nes_shader_parameter_count(const NesShaderPreset *shader);
/* Sorted regular .glslp files in one directory; no recursive or symlink scan. */
bool nes_shader_discover(const char *directory, char (*paths)[1024], size_t capacity, size_t *count, char *error,
                         size_t size);
bool nes_shader_parameter(const NesShaderPreset *shader, size_t index, NesShaderParameter *parameter);
bool nes_shader_set_parameter(NesShaderPreset *shader, size_t index, float value, char *error, size_t size);
/* Readback makes the same GPU output available to SDL presentation and capture.
 * Viewport dimensions affect viewport-scaled passes only. */
bool nes_shader_render(NesShaderPreset *shader, const NesVideoPresentationFrame *input, unsigned viewport_width,
                       unsigned viewport_height, NesVideoPresentationFrame *output, char *error, size_t size);
#ifdef __cplusplus
}
#endif
#endif
