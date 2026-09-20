/*
 * settings.c - Persistent desktop application settings
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "settings.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    SETTINGS_FILE_LIMIT = 256 * 1024,
    SETTINGS_SAVE_CAPACITY = 192 * 1024
};

typedef struct {
    const char *name;
    int value;
} NameValue;

static const NameValue region_names[] = {
    {"auto", NES_REGION_MODE_AUTO}, {"ntsc", NES_REGION_MODE_NTSC},
    {"pal", NES_REGION_MODE_PAL}, {"dendy", NES_REGION_MODE_DENDY}
};

static const NameValue console_names[] = {
    {"nes-001", NES_CONSOLE_NES001}, {"nes-101", NES_CONSOLE_NES101},
    {"famicom", NES_CONSOLE_HVC001}, {"av-famicom", NES_CONSOLE_HVC101}
};

static const NameValue adapter_names[] = {
    {"none", NES_ADAPTER_NONE}, {"four-score", NES_ADAPTER_FOUR_SCORE},
    {"famicom-2", NES_ADAPTER_FAMICOM_TWO}, {"famicom-4", NES_ADAPTER_FAMICOM_FOUR}
};

static const NameValue port_names[] = {
    {"pad", NES_PORT_GAMEPAD}, {"none", NES_PORT_NONE},
    {"arkanoid", NES_PORT_ARKANOID}, {"power-pad-a", NES_PORT_POWER_PAD_A},
    {"power-pad-b", NES_PORT_POWER_PAD_B}, {"zapper", NES_PORT_ZAPPER},
    {"subor-mouse", NES_PORT_SUBOR_MOUSE}, {"snes-pad", NES_PORT_SNES_CONTROLLER},
    {"snes-mouse", NES_PORT_SNES_MOUSE}, {"ntt-keypad", NES_PORT_NTT_KEYPAD},
    {"virtual-boy", NES_PORT_VIRTUAL_BOY}
};

static const NameValue expansion_names[] = {
    {"none", NES_EXPANSION_NONE}, {"arkanoid", NES_EXPANSION_ARKANOID},
    {"family-trainer-a", NES_EXPANSION_FAMILY_TRAINER_A},
    {"family-trainer-b", NES_EXPANSION_FAMILY_TRAINER_B},
    {"zapper", NES_EXPANSION_ZAPPER}, {"family-basic", NES_EXPANSION_FAMILY_BASIC},
    {"turbo-file", NES_EXPANSION_TURBO_FILE}, {"battle-box", NES_EXPANSION_BATTLE_BOX},
    {"subor-keyboard", NES_EXPANSION_SUBOR_KEYBOARD},
    {"hori-track", NES_EXPANSION_HORI_TRACK},
    {"konami-hyper-shot", NES_EXPANSION_KONAMI_HYPER_SHOT},
    {"bandai-hyper-shot", NES_EXPANSION_BANDAI_HYPER_SHOT},
    {"party-tap", NES_EXPANSION_PARTY_TAP}, {"pachinko", NES_EXPANSION_PACHINKO},
    {"exciting-boxing", NES_EXPANSION_EXCITING_BOXING},
    {"jissen-mahjong", NES_EXPANSION_JISSEN_MAHJONG},
    {"barcode-battler", NES_EXPANSION_BARCODE_BATTLER},
    {"oeka-kids-tablet", NES_EXPANSION_OEKA_KIDS_TABLET},
    {"fcns", NES_EXPANSION_FCNS_CONTROLLER}
};

static const char *const player_button_names[8] = {
    "a", "b", "select", "start", "up", "down", "left", "right"
};

static const char *const shortcut_names[FRONTEND_SHORTCUT_COUNT] = {
    "pause", "frame-advance", "soft-reset", "power-cycle", "reload",
    "fast-forward-hold", "fast-forward-toggle", "speed-half",
    "speed-normal", "speed-double"
};

static const NameValue gamepad_names[] = {
    {"a", SDL_CONTROLLER_BUTTON_A}, {"b", SDL_CONTROLLER_BUTTON_B},
    {"x", SDL_CONTROLLER_BUTTON_X}, {"y", SDL_CONTROLLER_BUTTON_Y},
    {"back", SDL_CONTROLLER_BUTTON_BACK}, {"guide", SDL_CONTROLLER_BUTTON_GUIDE},
    {"start", SDL_CONTROLLER_BUTTON_START},
    {"left-stick", SDL_CONTROLLER_BUTTON_LEFTSTICK},
    {"right-stick", SDL_CONTROLLER_BUTTON_RIGHTSTICK},
    {"left-shoulder", SDL_CONTROLLER_BUTTON_LEFTSHOULDER},
    {"right-shoulder", SDL_CONTROLLER_BUTTON_RIGHTSHOULDER},
    {"dpad-up", SDL_CONTROLLER_BUTTON_DPAD_UP},
    {"dpad-down", SDL_CONTROLLER_BUTTON_DPAD_DOWN},
    {"dpad-left", SDL_CONTROLLER_BUTTON_DPAD_LEFT},
    {"dpad-right", SDL_CONTROLLER_BUTTON_DPAD_RIGHT}
};

static bool text_equal_ci(const char *a, const char *b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return false;
    }
    return *a == *b;
}

static char *trim(char *text) {
    while (*text && isspace((unsigned char)*text)) ++text;
    char *end = text + strlen(text);
    while (end > text && isspace((unsigned char)end[-1])) --end;
    *end = '\0';
    return text;
}

static bool parse_name_value(const NameValue *table, size_t count,
                             const char *name, int *value) {
    if (!name || !value) return false;
    for (size_t i = 0; i < count; ++i) {
        if (text_equal_ci(name, table[i].name)) {
            *value = table[i].value;
            return true;
        }
    }
    return false;
}

static const char *name_for_value(const NameValue *table, size_t count, int value) {
    for (size_t i = 0; i < count; ++i)
        if (table[i].value == value) return table[i].name;
    return "unknown";
}

const char *frontend_shortcut_name(FrontendShortcut shortcut) {
    return shortcut < FRONTEND_SHORTCUT_COUNT ? shortcut_names[shortcut] : "unknown";
}

bool frontend_shortcut_from_name(const char *name, FrontendShortcut *shortcut) {
    if (!name || !shortcut) return false;
    for (unsigned i = 0; i < FRONTEND_SHORTCUT_COUNT; ++i) {
        if (text_equal_ci(name, shortcut_names[i])) {
            *shortcut = (FrontendShortcut)i;
            return true;
        }
    }
    return false;
}

const char *frontend_player_button_name(unsigned button) {
    return button < 8 ? player_button_names[button] : "unknown";
}

bool frontend_player_button_from_name(const char *name, unsigned *button) {
    if (!name || !button) return false;
    for (unsigned i = 0; i < 8; ++i) {
        if (text_equal_ci(name, player_button_names[i])) {
            *button = i;
            return true;
        }
    }
    return false;
}

const char *frontend_gamepad_button_name(SDL_GameControllerButton button) {
    if (button == SDL_CONTROLLER_BUTTON_INVALID) return "none";
    return name_for_value(gamepad_names, sizeof(gamepad_names) / sizeof(gamepad_names[0]),
                          button);
}

bool frontend_gamepad_button_from_name(const char *name,
                                       SDL_GameControllerButton *button) {
    if (!name || !button) return false;
    if (text_equal_ci(name, "none")) {
        *button = SDL_CONTROLLER_BUTTON_INVALID;
        return true;
    }
    int value;
    if (!parse_name_value(gamepad_names, sizeof(gamepad_names) / sizeof(gamepad_names[0]),
                          name, &value)) return false;
    *button = (SDL_GameControllerButton)value;
    return true;
}

static void clear_binding(FrontendHostBinding *binding) {
    binding->key = SDL_SCANCODE_UNKNOWN;
    binding->modifiers = KMOD_NONE;
    binding->gamepad_button = SDL_CONTROLLER_BUTTON_INVALID;
}

static void set_player_default(FrontendBindingProfile *profile, unsigned player,
                               unsigned button, SDL_Scancode key,
                               SDL_GameControllerButton pad) {
    profile->players[player][button].key = key;
    profile->players[player][button].modifiers = KMOD_NONE;
    profile->players[player][button].gamepad_button = pad;
}

static void set_shortcut_default(FrontendBindingProfile *profile,
                                 FrontendShortcut shortcut, SDL_Scancode key,
                                 SDL_Keymod modifiers) {
    profile->shortcuts[shortcut].key = key;
    profile->shortcuts[shortcut].modifiers = modifiers;
    profile->shortcuts[shortcut].gamepad_button = SDL_CONTROLLER_BUTTON_INVALID;
}

bool frontend_binding_profile_preset(FrontendBindingProfile *profile,
                                     const char *name, const char *preset) {
    if (!profile || !name || !*name || strlen(name) >= sizeof(profile->name) || !preset)
        return false;
    memset(profile, 0, sizeof(*profile));
    strcpy(profile->name, name);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        for (unsigned button = 0; button < 8; ++button)
            clear_binding(&profile->players[player][button]);
    for (unsigned shortcut = 0; shortcut < FRONTEND_SHORTCUT_COUNT; ++shortcut)
        clear_binding(&profile->shortcuts[shortcut]);

    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        set_player_default(profile, player, BTN_A, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_A);
        set_player_default(profile, player, BTN_B, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_B);
        set_player_default(profile, player, BTN_SELECT, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_BACK);
        set_player_default(profile, player, BTN_START, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_START);
        set_player_default(profile, player, BTN_UP, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_DPAD_UP);
        set_player_default(profile, player, BTN_DOWN, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        set_player_default(profile, player, BTN_LEFT, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        set_player_default(profile, player, BTN_RIGHT, SDL_SCANCODE_UNKNOWN, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
    }

    if (text_equal_ci(preset, "default")) {
        set_player_default(profile, 0, BTN_A, SDL_SCANCODE_Z, SDL_CONTROLLER_BUTTON_A);
        set_player_default(profile, 0, BTN_B, SDL_SCANCODE_X, SDL_CONTROLLER_BUTTON_B);
        set_player_default(profile, 0, BTN_SELECT, SDL_SCANCODE_RSHIFT, SDL_CONTROLLER_BUTTON_BACK);
        set_player_default(profile, 0, BTN_START, SDL_SCANCODE_RETURN, SDL_CONTROLLER_BUTTON_START);
        set_player_default(profile, 0, BTN_UP, SDL_SCANCODE_UP, SDL_CONTROLLER_BUTTON_DPAD_UP);
        set_player_default(profile, 0, BTN_DOWN, SDL_SCANCODE_DOWN, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        set_player_default(profile, 0, BTN_LEFT, SDL_SCANCODE_LEFT, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        set_player_default(profile, 0, BTN_RIGHT, SDL_SCANCODE_RIGHT, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
    } else if (text_equal_ci(preset, "wasd")) {
        set_player_default(profile, 0, BTN_A, SDL_SCANCODE_J, SDL_CONTROLLER_BUTTON_A);
        set_player_default(profile, 0, BTN_B, SDL_SCANCODE_K, SDL_CONTROLLER_BUTTON_B);
        set_player_default(profile, 0, BTN_SELECT, SDL_SCANCODE_U, SDL_CONTROLLER_BUTTON_BACK);
        set_player_default(profile, 0, BTN_START, SDL_SCANCODE_I, SDL_CONTROLLER_BUTTON_START);
        set_player_default(profile, 0, BTN_UP, SDL_SCANCODE_W, SDL_CONTROLLER_BUTTON_DPAD_UP);
        set_player_default(profile, 0, BTN_DOWN, SDL_SCANCODE_S, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        set_player_default(profile, 0, BTN_LEFT, SDL_SCANCODE_A, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        set_player_default(profile, 0, BTN_RIGHT, SDL_SCANCODE_D, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
    } else {
        return false;
    }

    set_shortcut_default(profile, FRONTEND_SHORTCUT_PAUSE, SDL_SCANCODE_P, KMOD_CTRL);
    set_shortcut_default(profile, FRONTEND_SHORTCUT_FRAME_ADVANCE, SDL_SCANCODE_PERIOD, KMOD_CTRL);
    set_shortcut_default(profile, FRONTEND_SHORTCUT_SOFT_RESET, SDL_SCANCODE_R, KMOD_CTRL);
    set_shortcut_default(profile, FRONTEND_SHORTCUT_POWER_CYCLE, SDL_SCANCODE_R,
                         (SDL_Keymod)(KMOD_CTRL | KMOD_SHIFT));
    set_shortcut_default(profile, FRONTEND_SHORTCUT_RELOAD, SDL_SCANCODE_R,
                         (SDL_Keymod)(KMOD_CTRL | KMOD_ALT));
    set_shortcut_default(profile, FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, SDL_SCANCODE_F, KMOD_CTRL);
    set_shortcut_default(profile, FRONTEND_SHORTCUT_FAST_FORWARD_TOGGLE, SDL_SCANCODE_F,
                         (SDL_Keymod)(KMOD_CTRL | KMOD_SHIFT));
    set_shortcut_default(profile, FRONTEND_SHORTCUT_SPEED_HALF, SDL_SCANCODE_1, KMOD_CTRL);
    set_shortcut_default(profile, FRONTEND_SHORTCUT_SPEED_NORMAL, SDL_SCANCODE_2, KMOD_CTRL);
    set_shortcut_default(profile, FRONTEND_SHORTCUT_SPEED_DOUBLE, SDL_SCANCODE_3, KMOD_CTRL);
    return true;
}

void frontend_settings_defaults(FrontendSettings *settings) {
    if (!settings) return;
    memset(settings, 0, sizeof(*settings));
    settings->version = FRONTEND_SETTINGS_VERSION;
    settings->region_mode = NES_REGION_MODE_AUTO;
    settings->console_model = NES_CONSOLE_NES001;
    settings->speed = 1.0;
    settings->fast_forward_speed = 4.0;
    settings->input.adapter = NES_ADAPTER_NONE;
    settings->input.ports[0] = NES_PORT_GAMEPAD;
    settings->input.ports[1] = NES_PORT_GAMEPAD;
    settings->input.expansion = NES_EXPANSION_NONE;
    settings->profile_count = 1;
    (void)frontend_binding_profile_preset(&settings->profiles[0], "default", "default");
    strcpy(settings->active_profile, "default");
}

static bool valid_profile_name(const char *name) {
    if (!name || !*name || strlen(name) >= FRONTEND_SETTINGS_PROFILE_NAME) return false;
    for (const unsigned char *p = (const unsigned char *)name; *p; ++p)
        if (!isalnum(*p) && *p != '-' && *p != '_') return false;
    return true;
}

static FrontendBindingProfile *find_profile(FrontendSettings *settings, const char *name) {
    if (!settings || !name) return NULL;
    for (size_t i = 0; i < settings->profile_count; ++i)
        if (strcmp(settings->profiles[i].name, name) == 0) return &settings->profiles[i];
    return NULL;
}

static const FrontendBindingProfile *find_profile_const(const FrontendSettings *settings,
                                                        const char *name) {
    return find_profile((FrontendSettings *)settings, name);
}

bool frontend_settings_add_profile(FrontendSettings *settings, const char *name,
                                   const char *preset) {
    if (!settings || !valid_profile_name(name) || find_profile(settings, name)
        || settings->profile_count >= FRONTEND_SETTINGS_MAX_PROFILES) return false;
    FrontendBindingProfile *profile = &settings->profiles[settings->profile_count];
    if (!frontend_binding_profile_preset(profile, name, preset ? preset : "default")) return false;
    ++settings->profile_count;
    return true;
}

bool frontend_settings_select_profile(FrontendSettings *settings, const char *name) {
    if (!settings || !find_profile(settings, name)) return false;
    strcpy(settings->active_profile, name);
    return true;
}

FrontendBindingProfile *frontend_settings_active_profile(FrontendSettings *settings) {
    return settings ? find_profile(settings, settings->active_profile) : NULL;
}

const FrontendBindingProfile *frontend_settings_active_profile_const(
    const FrontendSettings *settings) {
    return settings ? find_profile_const(settings, settings->active_profile) : NULL;
}

void frontend_settings_mark_cli_override(FrontendSettings *settings,
                                         FrontendSettingOverride override_flag) {
    if (settings) settings->cli_overrides |= (uint32_t)override_flag;
}

bool frontend_settings_cli_overridden(const FrontendSettings *settings,
                                      FrontendSettingOverride override_flag) {
    return settings && (settings->cli_overrides & (uint32_t)override_flag) != 0;
}

static bool parse_double_range(const char *text, double min, double max, double *value) {
    if (!text || !*text || !value) return false;
    char *end = NULL;
    errno = 0;
    double parsed = strtod(text, &end);
    if (errno || end == text || *end || !isfinite(parsed) || parsed < min || parsed > max)
        return false;
    *value = parsed;
    return true;
}

static bool parse_unsigned_range(const char *text, unsigned max, unsigned *value) {
    if (!text || !*text || !value || text[0] == '-') return false;
    char *end = NULL;
    errno = 0;
    unsigned long parsed = strtoul(text, &end, 10);
    if (errno || end == text || *end || parsed > max) return false;
    *value = (unsigned)parsed;
    return true;
}

static SDL_Keymod normalized_modifiers(SDL_Keymod mods) {
    return (SDL_Keymod)(mods & (KMOD_CTRL | KMOD_SHIFT | KMOD_ALT | KMOD_GUI));
}

static bool key_binding_matches(const FrontendHostBinding *binding,
                                const SDL_KeyboardEvent *event) {
    if (!binding || !event || binding->key == SDL_SCANCODE_UNKNOWN) return false;
    return binding->key == event->keysym.scancode
        && normalized_modifiers(binding->modifiers)
            == normalized_modifiers((SDL_Keymod)event->keysym.mod);
}

bool frontend_profile_player_key(const FrontendBindingProfile *profile,
                                 const SDL_KeyboardEvent *event,
                                 unsigned *player, unsigned *button) {
    if (!profile || !event || !player || !button) return false;
    for (unsigned p = 0; p < NES_INPUT_PLAYERS; ++p) {
        for (unsigned b = 0; b < 8; ++b) {
            if (key_binding_matches(&profile->players[p][b], event)) {
                *player = p;
                *button = b;
                return true;
            }
        }
    }
    return false;
}

bool frontend_profile_player_gamepad(const FrontendBindingProfile *profile,
                                     unsigned player,
                                     SDL_GameControllerButton gamepad_button,
                                     unsigned *button) {
    if (!profile || player >= NES_INPUT_PLAYERS || !button
        || gamepad_button == SDL_CONTROLLER_BUTTON_INVALID) return false;
    for (unsigned b = 0; b < 8; ++b) {
        if (profile->players[player][b].gamepad_button == gamepad_button) {
            *button = b;
            return true;
        }
    }
    return false;
}

bool frontend_profile_shortcut_key(const FrontendBindingProfile *profile,
                                   const SDL_KeyboardEvent *event,
                                   FrontendShortcut *shortcut) {
    if (!profile || !event || !shortcut) return false;
    for (unsigned s = 0; s < FRONTEND_SHORTCUT_COUNT; ++s) {
        if (key_binding_matches(&profile->shortcuts[s], event)) {
            *shortcut = (FrontendShortcut)s;
            return true;
        }
    }
    return false;
}

bool frontend_profile_shortcut_gamepad(const FrontendBindingProfile *profile,
                                       SDL_GameControllerButton gamepad_button,
                                       FrontendShortcut *shortcut) {
    if (!profile || !shortcut || gamepad_button == SDL_CONTROLLER_BUTTON_INVALID) return false;
    for (unsigned s = 0; s < FRONTEND_SHORTCUT_COUNT; ++s) {
        if (profile->shortcuts[s].gamepad_button == gamepad_button) {
            *shortcut = (FrontendShortcut)s;
            return true;
        }
    }
    return false;
}

static bool parse_key_binding(const char *text, SDL_Scancode *scancode,
                              SDL_Keymod *modifiers) {
    if (!text || !scancode || !modifiers) return false;
    if (text_equal_ci(text, "none")) {
        *scancode = SDL_SCANCODE_UNKNOWN;
        *modifiers = KMOD_NONE;
        return true;
    }
    char copy[128];
    if (strlen(text) >= sizeof(copy)) return false;
    strcpy(copy, text);
    SDL_Keymod mods = KMOD_NONE;
    char *last = copy;
    char *part = copy;
    while (part) {
        char *next = strchr(part, '+');
        if (next) *next++ = '\0';
        char *token = trim(part);
        if (!next) {
            last = token;
            break;
        }
        if (text_equal_ci(token, "ctrl")) mods = (SDL_Keymod)(mods | KMOD_CTRL);
        else if (text_equal_ci(token, "shift")) mods = (SDL_Keymod)(mods | KMOD_SHIFT);
        else if (text_equal_ci(token, "alt")) mods = (SDL_Keymod)(mods | KMOD_ALT);
        else if (text_equal_ci(token, "gui")) mods = (SDL_Keymod)(mods | KMOD_GUI);
        else return false;
        part = next;
    }
    SDL_Scancode code = SDL_GetScancodeFromName(last);
    if (code == SDL_SCANCODE_UNKNOWN && !text_equal_ci(last, "Unknown")) return false;
    *scancode = code;
    *modifiers = normalized_modifiers(mods);
    return true;
}

static bool format_key_binding(const FrontendHostBinding *binding,
                               char *text, size_t size) {
    if (!binding || !text || !size) return false;
    if (binding->key == SDL_SCANCODE_UNKNOWN) {
        return snprintf(text, size, "none") > 0;
    }
    const char *key = SDL_GetScancodeName(binding->key);
    if (!key || !*key) return false;
    int used = 0;
    SDL_Keymod mods = normalized_modifiers(binding->modifiers);
    if (mods & KMOD_CTRL) used += snprintf(text + used, size - (size_t)used, "Ctrl+");
    if (mods & KMOD_SHIFT) used += snprintf(text + used, size - (size_t)used, "Shift+");
    if (mods & KMOD_ALT) used += snprintf(text + used, size - (size_t)used, "Alt+");
    if (mods & KMOD_GUI) used += snprintf(text + used, size - (size_t)used, "Gui+");
    if (used < 0 || (size_t)used >= size) return false;
    int tail = snprintf(text + used, size - (size_t)used, "%s", key);
    return tail >= 0 && (size_t)(used + tail) < size;
}

static FrontendBindingProfile *get_or_add_profile(FrontendSettings *settings,
                                                  const char *name) {
    FrontendBindingProfile *profile = find_profile(settings, name);
    if (profile) return profile;
    if (!frontend_settings_add_profile(settings, name, "default")) return NULL;
    return find_profile(settings, name);
}

static bool parse_profile_key(FrontendSettings *settings, const char *key,
                              const char *value) {
    if (strncmp(key, "profile.", 8) != 0) return false;
    char copy[192];
    if (strlen(key) >= sizeof(copy)) return false;
    strcpy(copy, key + 8);
    char *profile_name = copy;
    char *kind = strchr(profile_name, '.');
    if (!kind) return false;
    *kind++ = '\0';
    if (!valid_profile_name(profile_name)) return false;
    FrontendBindingProfile *profile = get_or_add_profile(settings, profile_name);
    if (!profile) return false;

    if (strncmp(kind, "player.", 7) == 0) {
        char *player_text = kind + 7;
        char *button_text = strchr(player_text, '.');
        if (!button_text) return false;
        *button_text++ = '\0';
        char *binding_kind = strchr(button_text, '.');
        if (!binding_kind) return false;
        *binding_kind++ = '\0';
        unsigned player;
        if (!parse_unsigned_range(player_text, NES_INPUT_PLAYERS, &player)
            || player < 1 || player > NES_INPUT_PLAYERS) return false;
        unsigned button;
        if (!frontend_player_button_from_name(button_text, &button)) return false;
        FrontendHostBinding *binding = &profile->players[player - 1][button];
        if (strcmp(binding_kind, "key") == 0)
            return parse_key_binding(value, &binding->key, &binding->modifiers);
        if (strcmp(binding_kind, "pad") == 0)
            return frontend_gamepad_button_from_name(value, &binding->gamepad_button);
        return false;
    }
    if (strncmp(kind, "shortcut.", 9) == 0) {
        char *shortcut_text = kind + 9;
        char *binding_kind = strrchr(shortcut_text, '.');
        if (!binding_kind) return false;
        *binding_kind++ = '\0';
        FrontendShortcut shortcut;
        if (!frontend_shortcut_from_name(shortcut_text, &shortcut)) return false;
        FrontendHostBinding *binding = &profile->shortcuts[shortcut];
        if (strcmp(binding_kind, "key") == 0)
            return parse_key_binding(value, &binding->key, &binding->modifiers);
        if (strcmp(binding_kind, "pad") == 0)
            return frontend_gamepad_button_from_name(value, &binding->gamepad_button);
        return false;
    }
    return false;
}

static bool set_known_setting(FrontendSettings *settings, const char *key,
                              const char *value, unsigned file_version,
                              FrontendSettingsReport *report, bool *known) {
    *known = true;
    int parsed;
    if (strcmp(key, "region") == 0) {
        if (!parse_name_value(region_names, 4, value, &parsed)) return false;
        settings->region_mode = (NesRegionMode)parsed;
    } else if (strcmp(key, "console") == 0) {
        if (!parse_name_value(console_names, 4, value, &parsed)) return false;
        settings->console_model = (NesConsoleModel)parsed;
    } else if (strcmp(key, "video_filter") == 0) {
        if (text_equal_ci(value, "direct")) settings->ntsc_composite = false;
        else if (text_equal_ci(value, "ntsc-composite")) settings->ntsc_composite = true;
        else return false;
    } else if (strcmp(key, "speed") == 0) {
        if (!parse_double_range(value, 0.1, 16.0, &settings->speed)) return false;
    } else if (strcmp(key, "fast_forward_speed") == 0) {
        if (!parse_double_range(value, 0.1, 16.0, &settings->fast_forward_speed)) return false;
    } else if (file_version == 1 && strcmp(key, "speed_percent") == 0) {
        double percent;
        if (!parse_double_range(value, 10.0, 1600.0, &percent)) return false;
        settings->speed = percent / 100.0;
        report->migrated = true;
    } else if (file_version == 1 && strcmp(key, "fast_forward_percent") == 0) {
        double percent;
        if (!parse_double_range(value, 10.0, 1600.0, &percent)) return false;
        settings->fast_forward_speed = percent / 100.0;
        report->migrated = true;
    } else if (strcmp(key, "adapter") == 0) {
        if (!parse_name_value(adapter_names, 4, value, &parsed)) return false;
        settings->input.adapter = (NesInputAdapter)parsed;
        settings->saved_input_overrides |= NES_INPUT_OVERRIDE_ADAPTER;
    } else if (strcmp(key, "port1") == 0 || strcmp(key, "port2") == 0) {
        if (!parse_name_value(port_names, sizeof(port_names) / sizeof(port_names[0]),
                              value, &parsed)) return false;
        unsigned port = key[4] == '2' ? 1 : 0;
        settings->input.ports[port] = (NesPortDevice)parsed;
        settings->saved_input_overrides |= port ? NES_INPUT_OVERRIDE_PORT2
                                                : NES_INPUT_OVERRIDE_PORT1;
    } else if (strcmp(key, "expansion") == 0) {
        if (!parse_name_value(expansion_names,
                              sizeof(expansion_names) / sizeof(expansion_names[0]),
                              value, &parsed)) return false;
        settings->input.expansion = (NesExpansionDevice)parsed;
        settings->saved_input_overrides |= NES_INPUT_OVERRIDE_EXPANSION;
    } else if (strcmp(key, "zapper_radius") == 0) {
        if (!parse_unsigned_range(value, NES_ZAPPER_MAX_RADIUS, &settings->zapper_radius))
            return false;
    } else if (strcmp(key, "active_profile") == 0) {
        if (!valid_profile_name(value)) return false;
        if (strlen(value) >= sizeof(settings->active_profile)) return false;
        strcpy(settings->active_profile, value);
    } else if (strncmp(key, "device.", 7) == 0) {
        unsigned player;
        if (!parse_unsigned_range(key + 7, NES_INPUT_PLAYERS, &player)
            || player < 1 || player > NES_INPUT_PLAYERS
            || strlen(value) >= FRONTEND_SETTINGS_GUID_TEXT) return false;
        if (text_equal_ci(value, "auto")) settings->device_guid[player - 1][0] = '\0';
        else strcpy(settings->device_guid[player - 1], value);
    } else if (strncmp(key, "profile.", 8) == 0) {
        if (!parse_profile_key(settings, key, value)) return false;
    } else if (strcmp(key, "version") == 0) {
        /* Checked before parsing the remaining keys. */
    } else {
        *known = false;
    }
    return true;
}

static void report_error(FrontendSettingsReport *report, unsigned line,
                         NesFileResult file_result, const char *message,
                         const char *key) {
    if (!report) return;
    report->line = line;
    report->file_result = file_result;
    if (key)
        snprintf(report->message, sizeof(report->message), "Line %u: invalid %s", line, key);
    else
        snprintf(report->message, sizeof(report->message), "%s", message ? message : "Settings error");
}

bool frontend_settings_load(const char *path, FrontendSettings *settings,
                            FrontendSettingsReport *report) {
    if (!settings || !path) return false;
    if (report) memset(report, 0, sizeof(*report));
    FrontendSettings loaded;
    frontend_settings_defaults(&loaded);
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult file_result = nes_file_read_all(path, SETTINGS_FILE_LIMIT, &data, &size);
    if (report) report->file_result = file_result;
    if (file_result == NES_FILE_NOT_FOUND) {
        *settings = loaded;
        if (report) {
            report->found = false;
            snprintf(report->message, sizeof(report->message), "No saved settings");
        }
        return true;
    }
    if (file_result != NES_FILE_OK) {
        if (report)
            snprintf(report->message, sizeof(report->message), "Cannot read settings: %s",
                     nes_file_result_message(file_result));
        return false;
    }
    if (report) report->found = true;

    char *text = (char *)malloc(size + 1);
    if (!text) {
        free(data);
        report_error(report, 0, NES_FILE_OUT_OF_MEMORY, "Out of memory", NULL);
        return false;
    }
    memcpy(text, data, size);
    text[size] = '\0';
    free(data);

    unsigned line_number = 0;
    unsigned file_version = 0;
    bool saw_version = false;
    char *cursor = text;
    while (cursor && *cursor) {
        char *next = strchr(cursor, '\n');
        if (next) *next++ = '\0';
        ++line_number;
        char *line = trim(cursor);
        if (*line && *line != '#' && *line != ';') {
            char *equals = strchr(line, '=');
            if (!equals) {
                report_error(report, line_number, NES_FILE_OK, NULL, "setting");
                free(text);
                return false;
            }
            *equals++ = '\0';
            char *key = trim(line);
            char *value = trim(equals);
            if (!saw_version) {
                unsigned parsed_version;
                if (strcmp(key, "version") != 0
                    || !parse_unsigned_range(value, FRONTEND_SETTINGS_VERSION, &parsed_version)
                    || parsed_version < 1) {
                    report_error(report, line_number, NES_FILE_OK,
                                 "Settings file must begin with a supported version", NULL);
                    free(text);
                    return false;
                }
                file_version = parsed_version;
                saw_version = true;
                loaded.version = FRONTEND_SETTINGS_VERSION;
                if (file_version != FRONTEND_SETTINGS_VERSION && report) report->migrated = true;
            } else {
                bool known;
                if (!set_known_setting(&loaded, key, value, file_version, report, &known)) {
                    report_error(report, line_number, NES_FILE_OK, NULL, key);
                    free(text);
                    return false;
                }
                if (!known && report) ++report->unknown_settings;
            }
        }
        cursor = next;
    }
    free(text);
    if (!saw_version) {
        report_error(report, 0, NES_FILE_OK, "Settings file is empty", NULL);
        return false;
    }
    if (!find_profile(&loaded, loaded.active_profile)) {
        report_error(report, line_number, NES_FILE_OK, "Active input profile does not exist", NULL);
        return false;
    }
    *settings = loaded;
    if (report) {
        if (report->migrated)
            snprintf(report->message, sizeof(report->message),
                     "Loaded and migrated settings version %u", file_version);
        else if (report->unknown_settings)
            snprintf(report->message, sizeof(report->message),
                     "Loaded settings with %u unknown entr%s ignored",
                     report->unknown_settings, report->unknown_settings == 1 ? "y" : "ies");
        else
            snprintf(report->message, sizeof(report->message), "Loaded settings");
    }
    return true;
}

static bool append_text(char *buffer, size_t capacity, size_t *used,
                        const char *format, ...) {
    if (!buffer || !used || *used >= capacity) return false;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(buffer + *used, capacity - *used, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= capacity - *used) return false;
    *used += (size_t)written;
    return true;
}

bool frontend_settings_save(const char *path, const FrontendSettings *settings,
                            FrontendSettingsReport *report) {
    if (report) memset(report, 0, sizeof(*report));
    if (!path || !settings || !frontend_settings_active_profile_const(settings)) {
        report_error(report, 0, NES_FILE_INVALID_ARGUMENT, "Invalid settings", NULL);
        return false;
    }
    char *buffer = (char *)malloc(SETTINGS_SAVE_CAPACITY);
    if (!buffer) {
        report_error(report, 0, NES_FILE_OUT_OF_MEMORY, "Out of memory", NULL);
        return false;
    }
    size_t used = 0;
    bool ok = append_text(buffer, SETTINGS_SAVE_CAPACITY, &used,
                          "version=%u\nregion=%s\nconsole=%s\nvideo_filter=%s\n"
                          "speed=%.6g\nfast_forward_speed=%.6g\n"
                          "adapter=%s\nport1=%s\nport2=%s\nexpansion=%s\n"
                          "zapper_radius=%u\nactive_profile=%s\n",
                          FRONTEND_SETTINGS_VERSION,
                          name_for_value(region_names, 4, settings->region_mode),
                          name_for_value(console_names, 4, settings->console_model),
                          settings->ntsc_composite ? "ntsc-composite" : "direct",
                          settings->speed, settings->fast_forward_speed,
                          name_for_value(adapter_names, 4, settings->input.adapter),
                          name_for_value(port_names, sizeof(port_names) / sizeof(port_names[0]),
                                         settings->input.ports[0]),
                          name_for_value(port_names, sizeof(port_names) / sizeof(port_names[0]),
                                         settings->input.ports[1]),
                          name_for_value(expansion_names,
                                         sizeof(expansion_names) / sizeof(expansion_names[0]),
                                         settings->input.expansion),
                          settings->zapper_radius, settings->active_profile);
    for (unsigned player = 0; ok && player < NES_INPUT_PLAYERS; ++player)
        ok = append_text(buffer, SETTINGS_SAVE_CAPACITY, &used, "device.%u=%s\n", player + 1,
                         settings->device_guid[player][0]
                             ? settings->device_guid[player] : "auto");

    for (size_t p = 0; ok && p < settings->profile_count; ++p) {
        const FrontendBindingProfile *profile = &settings->profiles[p];
        for (unsigned player = 0; ok && player < NES_INPUT_PLAYERS; ++player) {
            for (unsigned button = 0; ok && button < 8; ++button) {
                char key[128];
                const FrontendHostBinding *binding = &profile->players[player][button];
                ok = format_key_binding(binding, key, sizeof(key))
                    && append_text(buffer, SETTINGS_SAVE_CAPACITY, &used,
                                   "profile.%s.player.%u.%s.key=%s\n"
                                   "profile.%s.player.%u.%s.pad=%s\n",
                                   profile->name, player + 1,
                                   frontend_player_button_name(button), key,
                                   profile->name, player + 1,
                                   frontend_player_button_name(button),
                                   frontend_gamepad_button_name(binding->gamepad_button));
            }
        }
        for (unsigned shortcut = 0; ok && shortcut < FRONTEND_SHORTCUT_COUNT; ++shortcut) {
            char key[128];
            const FrontendHostBinding *binding = &profile->shortcuts[shortcut];
            ok = format_key_binding(binding, key, sizeof(key))
                && append_text(buffer, SETTINGS_SAVE_CAPACITY, &used,
                               "profile.%s.shortcut.%s.key=%s\n"
                               "profile.%s.shortcut.%s.pad=%s\n",
                               profile->name, frontend_shortcut_name((FrontendShortcut)shortcut),
                               key, profile->name,
                               frontend_shortcut_name((FrontendShortcut)shortcut),
                               frontend_gamepad_button_name(binding->gamepad_button));
        }
    }
    if (!ok) {
        free(buffer);
        report_error(report, 0, NES_FILE_TOO_LARGE, "Settings serialization is too large", NULL);
        return false;
    }
    NesFileResult result = nes_file_write_atomic(path, buffer, used);
    free(buffer);
    if (report) report->file_result = result;
    if (result != NES_FILE_OK) {
        if (report)
            snprintf(report->message, sizeof(report->message), "Cannot save settings: %s",
                     nes_file_result_message(result));
        return false;
    }
    if (report) {
        report->found = true;
        snprintf(report->message, sizeof(report->message), "Saved settings");
    }
    return true;
}

bool frontend_settings_apply_core(const FrontendSettings *settings,
                                  char *error, size_t error_size) {
    if (!settings) return false;
    NesInputConfiguration previous = {
        .adapter = joypad_adapter(),
        .ports = {joypad_port_device(0), joypad_port_device(1)},
        .expansion = joypad_expansion_device()
    };
    uint8_t previous_overrides = joypad_configuration_overrides();
    unsigned previous_radius = joypad_zapper_radius();
    NesRegionMode previous_region = nes_region_mode();
    NesConsoleModel previous_console = nes_console_model();

    NesInputConfiguration selected = settings->input;
    if (settings->cli_overrides & FRONTEND_OVERRIDE_ADAPTER)
        selected.adapter = previous.adapter;
    if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT1)
        selected.ports[0] = previous.ports[0];
    if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT2)
        selected.ports[1] = previous.ports[1];
    if (settings->cli_overrides & FRONTEND_OVERRIDE_EXPANSION)
        selected.expansion = previous.expansion;

    bool valid = ((settings->cli_overrides & FRONTEND_OVERRIDE_REGION)
                  || nes_set_region_mode(settings->region_mode))
        && ((settings->cli_overrides & FRONTEND_OVERRIDE_CONSOLE)
            || nes_set_console_model(settings->console_model))
        && joypad_apply_configuration(&selected)
        && ((settings->cli_overrides & FRONTEND_OVERRIDE_ZAPPER_RADIUS)
            || joypad_set_zapper_radius(settings->zapper_radius))
        && joypad_configuration_valid();
    if (valid) {
        uint8_t overrides = settings->saved_input_overrides;
        if (settings->cli_overrides & FRONTEND_OVERRIDE_ADAPTER)
            overrides |= NES_INPUT_OVERRIDE_ADAPTER;
        if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT1)
            overrides |= NES_INPUT_OVERRIDE_PORT1;
        if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT2)
            overrides |= NES_INPUT_OVERRIDE_PORT2;
        if (settings->cli_overrides & FRONTEND_OVERRIDE_EXPANSION)
            overrides |= NES_INPUT_OVERRIDE_EXPANSION;
        joypad_set_configuration_overrides(overrides);
        return true;
    }
    (void)nes_set_region_mode(previous_region);
    (void)nes_set_console_model(previous_console);
    (void)joypad_apply_configuration(&previous);
    (void)joypad_set_zapper_radius(previous_radius);
    joypad_set_configuration_overrides(previous_overrides);
    if (error && error_size)
        snprintf(error, error_size, "Saved controller connector settings conflict");
    return false;
}
