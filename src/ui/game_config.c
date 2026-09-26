/*
 * game_config.c - Per-image settings layered over global preferences
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "game_config.h"
#include "recovery_store.h"
#include "frontend_panels.h"
#include "../system/execution_policy.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { FIELD_BOOL, FIELD_INT, FIELD_UINT, FIELD_DOUBLE, FIELD_PATH, FIELD_CHEAT } FieldType;

typedef struct {
    const char *name, *help;
    size_t offset, size;
    FieldType type;
    double minimum, maximum;
    uint32_t cli;
} Field;

#define FIELD(n, h, f, t, lo, hi, c)                                                                                   \
    {n, h, offsetof(FrontendSettings, f), sizeof(((FrontendSettings *)0)->f), t, lo, hi, c}
static const Field fields[] = {
    FIELD("region", "0 auto, 1 NTSC, 2 PAL, 3 Dendy", region_mode, FIELD_INT, 0, 3, FRONTEND_OVERRIDE_REGION),
    FIELD("console", "0 NES-001, 1 NES-101, 2 Famicom, 3 AV Famicom", console_model, FIELD_INT, 0, 3,
          FRONTEND_OVERRIDE_CONSOLE),
    FIELD("composite", "0 direct, 1 composite", ntsc_composite, FIELD_BOOL, 0, 1, FRONTEND_OVERRIDE_VIDEO_FILTER),
    FIELD("speed", "Playback speed (0.05-16)", speed, FIELD_DOUBLE, .05, 16, FRONTEND_OVERRIDE_SPEED),
    FIELD("adapter", "Adapter enum: see Input Settings", input.adapter, FIELD_INT, 0, 3, FRONTEND_OVERRIDE_ADAPTER),
    FIELD("port1", "Port device enum: see Input Settings", input.ports[0], FIELD_INT, 0, NES_PORT_VIRTUAL_BOY,
          FRONTEND_OVERRIDE_PORT1),
    FIELD("port2", "Port device enum: see Input Settings", input.ports[1], FIELD_INT, 0, NES_PORT_VIRTUAL_BOY,
          FRONTEND_OVERRIDE_PORT2),
    FIELD("expansion", "Expansion device enum: see Input Settings", input.expansion, FIELD_INT, 0,
          NES_EXPANSION_FCNS_CONTROLLER, FRONTEND_OVERRIDE_EXPANSION),
    FIELD("zapper_radius", "Light-sensor radius (0-255)", zapper_radius, FIELD_UINT, 0, 255,
          FRONTEND_OVERRIDE_ZAPPER_RADIUS),
    FIELD("cpu_revision", "0 early, 1 late", cpu_revision, FIELD_INT, 0, 1, FRONTEND_OVERRIDE_CPU_REVISION),
    FIELD("ppu_revision", "0 early, 1 E+", ppu_revision, FIELD_INT, 0, 1, FRONTEND_OVERRIDE_PPU_REVISION),
    FIELD("ram_power", "0 default, 1 zero, 2 ones, 3 random", ram_power_state, FIELD_INT, 0, 3,
          FRONTEND_OVERRIDE_RAM_POWER),
    FIELD("cart_dips", "Cartridge DIP byte (0-255)", cart_dips, FIELD_UINT, 0, 255, FRONTEND_OVERRIDE_CART_DIPS),
    FIELD("mmc3_revision_a", "0 standard, 1 revision A", mmc3_revision_a, FIELD_BOOL, 0, 1,
          FRONTEND_OVERRIDE_MMC3_REVISION),
    FIELD("muted", "Mute listening audio (0/1)", muted, FIELD_BOOL, 0, 1, 0),
    FIELD("master_volume", "Listening volume (0-100 percent)", audio_mix.master_volume, FIELD_UINT, 0, 100, 0),
    FIELD("pulse1_volume", "Pulse 1 volume (0-200 percent)", audio_mix.volume[NES_AUDIO_PULSE1], FIELD_UINT, 0, 200, 0),
    FIELD("pulse2_volume", "Pulse 2 volume (0-200 percent)", audio_mix.volume[NES_AUDIO_PULSE2], FIELD_UINT, 0, 200, 0),
    FIELD("triangle_volume", "Triangle volume (0-200 percent)", audio_mix.volume[NES_AUDIO_TRIANGLE], FIELD_UINT, 0,
          200, 0),
    FIELD("noise_volume", "Noise volume (0-200 percent)", audio_mix.volume[NES_AUDIO_NOISE], FIELD_UINT, 0, 200, 0),
    FIELD("dmc_volume", "DMC volume (0-200 percent)", audio_mix.volume[NES_AUDIO_DMC], FIELD_UINT, 0, 200, 0),
    FIELD("fullscreen", "Fullscreen (0/1)", fullscreen, FIELD_BOOL, 0, 1, 0),
    FIELD("integer_scaling", "Integer scaling (0/1)", integer_scaling, FIELD_BOOL, 0, 1, 0),
    FIELD("bilinear", "Bilinear interpolation (0/1)", bilinear_interpolation, FIELD_BOOL, 0, 1, 0),
    FIELD("aspect", "0 source, 1 4:3", aspect_mode, FIELD_INT, 0, 1, 0),
    FIELD("background", "Show background (0/1)", presentation.show_background, FIELD_BOOL, 0, 1, 0),
    FIELD("sprites", "Show sprites (0/1)", presentation.show_sprites, FIELD_BOOL, 0, 1, 0),
    FIELD("fds_bios", "8 KiB firmware path", fds_bios_path, FIELD_PATH, 0, 0, 0),
    FIELD("studybox_bios", "256 KiB firmware path", studybox_bios_path, FIELD_PATH, 0, 0, 0),
    FIELD("epsm_adpcm", "8 KiB percussion ROM path", epsm_adpcm_path, FIELD_PATH, 0, 0, 0),
    FIELD("fcns_kanji", "256 KiB character ROM path", fcns_kanji_path, FIELD_PATH, 0, 0, 0),
    FIELD("disk_overlay", "Writable disk overlay path", disk_overlay_path, FIELD_PATH, 0, 0, 0),
    FIELD("disk_write_protect", "Write protection (0/1)", fds_write_protected, FIELD_BOOL, 0, 1, 0),
    FIELD("disk_save_mode", "0 in-place, 1 overlay", disk_save_mode, FIELD_INT, 0, 1, 0),
    FIELD("shader_path", "GLSL preset path", shader_path, FIELD_PATH, 0, 0, 0),
    FIELD("shader_enabled", "Enable shader preset (0/1)", shader_enabled, FIELD_BOOL, 0, 1, 0),
    FIELD("audio_backend", "default, wasapi, or directsound", audio_backend, FIELD_PATH, 0, 0, 0),
    {"cheat_file", "Cheat file loaded with this image; empty disables automatic cheats", 0, 1024, FIELD_CHEAT, 0, 0,
     0}};
#undef FIELD
_Static_assert(sizeof(fields) / sizeof(fields[0]) <= GAME_CONFIG_FIELDS, "field capacity");

size_t game_config_field_count(void) {
    return sizeof(fields) / sizeof(fields[0]);
}

const char *game_config_field_name(size_t field) {
    return field < game_config_field_count() ? fields[field].name : NULL;
}

int game_config_find_field(const char *name) {
    for (size_t i = 0; name && i < game_config_field_count(); ++i) {
        if (!strcmp(fields[i].name, name)) {
            return (int)i;
        }
    }
    return -1;
}

static bool assign(const Field *f, const char *text, FrontendSettings *settings) {
    if (!text || (strlen(text) >= f->size && (f->type == FIELD_PATH || f->type == FIELD_CHEAT))) {
        return false;
    }
    if (strpbrk(text, "\r\n")) {
        return false;
    }
    unsigned char *p = settings ? (unsigned char *)settings + f->offset : NULL;
    if (f->type == FIELD_CHEAT) {
        return true;
    }
    if (f->type == FIELD_PATH) {
        if (p) {
            strcpy((char *)p, text);
        }
        return true;
    }
    if (!*text) {
        return false;
    }
    char *end;
    errno = 0;
    double value = strtod(text, &end);
    if (errno || *end || !isfinite(value) || value < f->minimum || value > f->maximum ||
        (f->type != FIELD_DOUBLE && value != floor(value))) {
        return false;
    }
    if (!p) {
        return true;
    }
    if (f->type == FIELD_BOOL) {
        bool b = value != 0;
        memcpy(p, &b, sizeof(b));
    } else if (f->type == FIELD_DOUBLE) {
        memcpy(p, &value, sizeof(value));
    } else if (f->type == FIELD_UINT) {
        unsigned v = (unsigned)value;
        memcpy(p, &v, sizeof(v));
    } else {
        int v = (int)value;
        memcpy(p, &v, sizeof(v));
    }
    return true;
}

bool game_config_set(GameConfig *config, size_t field, const char *value) {
    if (!config || field >= game_config_field_count() || !value || strlen(value) >= 1024 ||
        !assign(&fields[field], value, NULL)) {
        return false;
    }
    strcpy(config->values[field], value);
    config->present |= UINT64_C(1) << field;
    return true;
}

void game_config_reset(GameConfig *config, size_t field) {
    if (config && field < game_config_field_count()) {
        config->present &= ~(UINT64_C(1) << field);
        config->values[field][0] = 0;
    }
}

bool game_config_resolve(const FrontendSettings *global, const GameConfig *config, uint64_t cli_fields,
                         FrontendSettings *effective, char *cheat_path, size_t cheat_capacity, char *error,
                         size_t error_size) {
    if (!global || !config || !effective) {
        return false;
    }
    FrontendSettings result = *global;
    if (cheat_path && cheat_capacity) {
        cheat_path[0] = 0;
    }
    for (size_t i = 0; i < game_config_field_count(); ++i) {
        uint64_t bit = UINT64_C(1) << i;
        if (!(config->present & bit) || (cli_fields & bit) || (global->cli_overrides & fields[i].cli)) {
            continue;
        }
        if (!assign(&fields[i], config->values[i], &result)) {
            return false;
        }
        if (fields[i].type == FIELD_CHEAT && cheat_path) {
            if (strlen(config->values[i]) >= cheat_capacity) {
                return false;
            }
            strcpy(cheat_path, config->values[i]);
        }
        if (!strcmp(fields[i].name, "shader_path")) {
            result.shader_parameter_count = 0;
        }
        if (!strcmp(fields[i].name, "adapter")) {
            result.saved_input_overrides |= NES_INPUT_OVERRIDE_ADAPTER;
        }
        if (!strcmp(fields[i].name, "port1")) {
            result.saved_input_overrides |= NES_INPUT_OVERRIDE_PORT1;
        }
        if (!strcmp(fields[i].name, "port2")) {
            result.saved_input_overrides |= NES_INPUT_OVERRIDE_PORT2;
        }
        if (!strcmp(fields[i].name, "expansion")) {
            result.saved_input_overrides |= NES_INPUT_OVERRIDE_EXPANSION;
        }
    }
    result.audio_mix.muted = result.muted;
    if (!frontend_settings_validate(&result, error, error_size)) {
        return false;
    }
    *effective = result;
    return true;
}

bool game_config_save(const char *directory, const char *key, const GameConfig *config, char *error,
                      size_t error_size) {
    char path[4096];
    if (!config || !recovery_path(directory, key, ".game.ini", path, sizeof(path))) {
        return false;
    }
    char *text = malloc(GAME_CONFIG_FIELDS * 1152);
    if (!text) {
        return false;
    }
    size_t used = 10;
    memcpy(text, "version=1\n", used);
    bool valid = (config->present >> game_config_field_count()) == 0;
    for (size_t i = 0; valid && i < game_config_field_count(); ++i) {
        if (!(config->present & (UINT64_C(1) << i))) {
            continue;
        }
        valid = assign(&fields[i], config->values[i], NULL);
        if (valid) {
            used += (size_t)sprintf(text + used, "%s=%s\n", fields[i].name, config->values[i]);
        }
    }
    NesFileResult result = valid ? nes_file_write_atomic(path, text, used) : NES_FILE_INVALID_ARGUMENT;
    free(text);
    if (error && error_size) {
        snprintf(error, error_size, "%s", result == NES_FILE_OK ? "" : nes_file_result_message(result));
    }
    return result == NES_FILE_OK;
}

bool game_config_load(const char *directory, const char *key, GameConfig *config, char *error, size_t error_size) {
    char path[4096];
    if (!config || !recovery_path(directory, key, ".game.ini", path, sizeof(path))) {
        return false;
    }
    uint8_t *bytes = NULL;
    size_t size = 0;
    NesFileResult result = nes_file_read_all(path, GAME_CONFIG_FIELDS * 1152, &bytes, &size);
    if (result == NES_FILE_NOT_FOUND) {
        memset(config, 0, sizeof(*config));
        return true;
    }
    if (result != NES_FILE_OK) {
        if (error && error_size) {
            snprintf(error, error_size, "%s", nes_file_result_message(result));
        }
        return false;
    }
    char *text = malloc(size + 1);
    GameConfig *parsed = calloc(1, sizeof(*parsed));
    if (!text || !parsed) {
        free(text);
        free(parsed);
        free(bytes);
        return false;
    }
    memcpy(text, bytes, size);
    text[size] = 0;
    free(bytes);
    bool valid = size >= 10 && !memcmp(text, "version=1\n", 10) && !memchr(text, 0, size);
    char *line = text + (valid ? 10 : size);
    while (valid && *line) {
        char *end = strchr(line, '\n');
        if (!end) {
            valid = false;
            break;
        }
        *end = 0;
        char *equals = strchr(line, '=');
        if (!equals) {
            valid = false;
            break;
        }
        *equals++ = 0;
        int field = game_config_find_field(line);
        valid =
            field >= 0 && !(parsed->present & (UINT64_C(1) << field)) && game_config_set(parsed, (size_t)field, equals);
        line = end + 1;
    }
    if (valid) {
        *config = *parsed;
    }
    free(parsed);
    free(text);
    if (error && error_size) {
        snprintf(error, error_size, "%s", valid ? "" : "Invalid game configuration; previous settings retained");
    }
    return valid;
}

bool game_config_prepare(GameConfigFrontend *frontend, const FrontendImageResult *image, FrontendSettings *effective,
                         char *cheat_path, size_t cheat_capacity, char *error, size_t error_size) {
    if (!frontend || !image || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return false;
    }
    FrontendSession *session = calloc(1, sizeof(*session));
    if (!session) {
        return false;
    }
    session->active = true;
    session->current_result = *image;
    strcpy(session->current.archive_member, image->archive_member);
    char key[41];
    bool valid = recovery_game_key(session, key);
    free(session);
    if (!valid) {
        return false;
    }
    GameConfig *config = calloc(1, sizeof(*config));
    if (!config) {
        return false;
    }
    bool ok = game_config_load(frontend->directory, key, config, error, error_size) &&
              game_config_resolve(frontend->global, config, frontend->cli_fields, effective, cheat_path, cheat_capacity,
                                  error, error_size);
    if (ok) {
        for (size_t i = 0; i < game_config_field_count(); ++i) {
            if ((frontend->cli_fields & (UINT64_C(1) << i)) && fields[i].type != FIELD_CHEAT) {
                memcpy((char *)effective + fields[i].offset, (char *)&frontend->launch + fields[i].offset,
                       fields[i].size);
            }
        }
        frontend->config = *config;
        strcpy(frontend->key, key);
        frontend->selected = 0;
    }
    free(config);
    return ok;
}

static void inherited_value(const Field *field, const FrontendSettings *settings, char *text, size_t size) {
    const unsigned char *p = (const unsigned char *)settings + field->offset;
    if (field->type == FIELD_CHEAT) {
        snprintf(text, size, "Default per-game cheats");
    } else if (field->type == FIELD_PATH) {
        snprintf(text, size, "%s", (const char *)p);
    } else if (field->type == FIELD_BOOL) {
        bool b;
        memcpy(&b, p, sizeof(b));
        snprintf(text, size, "%u", b);
    } else if (field->type == FIELD_DOUBLE) {
        double d;
        memcpy(&d, p, sizeof(d));
        snprintf(text, size, "%.6g", d);
    } else if (field->type == FIELD_UINT) {
        unsigned u;
        memcpy(&u, p, sizeof(u));
        snprintf(text, size, "%u", u);
    } else {
        int i;
        memcpy(&i, p, sizeof(i));
        snprintf(text, size, "%d", i);
    }
}

static bool panel_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    GameConfigFrontend *r = context;
    for (size_t i = 0; i < game_config_field_count(); ++i) {
        bool cli = (r->cli_fields & (UINT64_C(1) << i)) || (r->global->cli_overrides & fields[i].cli);
        snprintf(r->names[i], sizeof(r->names[i]), "%s [%s]", fields[i].name,
                 cli                                      ? "command line"
                 : r->config.present & (UINT64_C(1) << i) ? "override"
                                                          : "inherited");
        r->items[i] = r->names[i];
    }
    size_t selected = (size_t)r->selected;
    inherited_value(&fields[selected], r->global, r->inherited, sizeof(r->inherited));
    bool editable = nes_execution_policy() == NES_EXECUTION_LIVE && r->key[0] &&
                    !(r->cli_fields & (UINT64_C(1) << selected)) && !(r->global->cli_overrides & fields[selected].cli);
    FrontendPanelControl controls[] = {
        {1, FRONTEND_PANEL_LIST, "Setting and source", NULL, r->items, game_config_field_count(), r->selected, true,
         true},
        {2, FRONTEND_PANEL_TEXT, fields[selected].help, r->value, NULL, 0, 0, editable, false},
        {3, FRONTEND_PANEL_TEXT, "Inherited global value", r->inherited, NULL, 0, 0, true, true},
        {4, FRONTEND_PANEL_ACTION, "Save override for next load", NULL, NULL, 0, 0, editable, false},
        {5, FRONTEND_PANEL_ACTION, "Reset to inherited value", NULL, NULL, 0, 0, editable, false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }
    model->status = r->status[0] ? r->status : "Changes take effect when this image is opened again";
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool panel_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    GameConfigFrontend *r = context;
    if (id == 1 && selected >= 0 && (size_t)selected < game_config_field_count()) {
        r->selected = selected;
        if (r->config.present & (UINT64_C(1) << selected)) {
            strcpy(r->value, r->config.values[selected]);
        } else {
            inherited_value(&fields[selected], r->global, r->value, sizeof(r->value));
        }
        return true;
    }
    size_t field = (size_t)r->selected;
    if (nes_execution_policy() != NES_EXECUTION_LIVE || !r->key[0] || (r->cli_fields & (UINT64_C(1) << field)) ||
        (r->global->cli_overrides & fields[field].cli)) {
        return false;
    }
    if (id == 2 && value && strlen(value) < sizeof(r->value)) {
        strcpy(r->value, value);
        return true;
    }
    if (id != 4 && id != 5) {
        return false;
    }
    GameConfig *candidate = malloc(sizeof(*candidate));
    if (!candidate) {
        return false;
    }
    *candidate = r->config;
    bool ok = true;
    if (id == 4) {
        ok = game_config_set(candidate, field, r->value);
    } else {
        game_config_reset(candidate, field);
    }
    FrontendSettings effective;
    ok = ok && game_config_resolve(r->global, candidate, r->cli_fields, &effective, NULL, 0, error, size);
    ok = ok && game_config_save(r->directory, r->key, candidate, error, size);
    if (ok) {
        r->config = *candidate;
        snprintf(r->status, sizeof(r->status), "Saved. Reopen the image to apply these settings.");
    }
    free(candidate);
    return ok;
}

bool game_config_init(GameConfigFrontend *r, FrontendSettings *global, const char *directory) {
    if (!r || !global || !directory || strlen(directory) >= sizeof(r->directory)) {
        return false;
    }
    memset(r, 0, sizeof(*r));
    r->global = global;
    strcpy(r->directory, directory);
    r->launch = *global;
    return true;
}

bool game_config_register_ui(GameConfigFrontend *r, FrontendSettings *global, const char *directory) {
    if (!r || !global || !directory) {
        return false;
    }
    FrontendPanelSpec spec = {GAME_CONFIG_PANEL, "Game Configuration", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                              panel_snapshot,    panel_action,         r};
    return frontend_panel_register(&spec);
}

void game_config_unregister_ui(void) {
    frontend_panel_unregister(GAME_CONFIG_PANEL);
}

bool game_config_save_globals(GameConfigFrontend *r, const char *path, const FrontendSettings *effective,
                              FrontendSettingsReport *report) {
    if (!r || !r->global || !effective) {
        return false;
    }
    FrontendSettings global = *effective;
    for (size_t i = 0; i < game_config_field_count(); ++i) {
        if (fields[i].type == FIELD_CHEAT) {
            continue;
        }
        if ((r->config.present & (UINT64_C(1) << i)) || (r->cli_fields & (UINT64_C(1) << i)) ||
            (effective->cli_overrides & fields[i].cli)) {
            memcpy((char *)&global + fields[i].offset, (char *)r->global + fields[i].offset, fields[i].size);
        }
    }
    int shader_field = game_config_find_field("shader_path");
    if (shader_field >= 0 && (r->config.present & (UINT64_C(1) << shader_field))) {
        global.shader_parameter_count = r->global->shader_parameter_count;
        memcpy(global.shader_parameters, r->global->shader_parameters, sizeof(global.shader_parameters));
    }
    if (!frontend_settings_save(path, &global, report)) {
        return false;
    }
    *r->global = global;
    return true;
}
