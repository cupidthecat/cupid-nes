/*
 * frontend_commands.c - Shared desktop frontend command registry
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "frontend_commands.h"
#include <stdio.h>
#include <string.h>

enum {
    FRONTEND_COMMAND_CAPACITY = 128,
    FRONTEND_COMMAND_TEXT = 64,
    FRONTEND_COMMAND_SHORTCUT = 32
};

typedef struct {
    bool used;
    unsigned id;
    char label[FRONTEND_COMMAND_TEXT];
    char menu[FRONTEND_COMMAND_TEXT];
    char shortcut[FRONTEND_COMMAND_SHORTCUT];
    unsigned flags;
    bool enabled;
    bool checked;
    FrontendCommandHandler handler;
    void *userdata;
} CommandEntry;

static CommandEntry commands[FRONTEND_COMMAND_CAPACITY];
static size_t command_count;

static CommandEntry *find_command(unsigned id) {
    for (size_t i = 0; i < FRONTEND_COMMAND_CAPACITY; ++i)
        if (commands[i].used && commands[i].id == id) return &commands[i];
    return NULL;
}

static const CommandEntry *command_at_index(size_t index) {
    size_t seen = 0;
    for (size_t i = 0; i < FRONTEND_COMMAND_CAPACITY; ++i) {
        if (!commands[i].used) continue;
        if (seen++ == index) return &commands[i];
    }
    return NULL;
}

static bool copy_text(char *dst, size_t size, const char *src) {
    if (!dst || !size || !src) return false;
    size_t length = strlen(src);
    if (length >= size) return false;
    memcpy(dst, src, length + 1);
    return true;
}

static void export_info(const CommandEntry *entry, FrontendCommandInfo *info) {
    info->id = entry->id;
    info->label = entry->label;
    info->menu = entry->menu;
    info->shortcut = entry->shortcut;
    info->flags = entry->flags;
    info->enabled = entry->enabled;
    info->checked = entry->checked;
}

void frontend_commands_reset(void) {
    memset(commands, 0, sizeof(commands));
    command_count = 0;
}

bool frontend_command_register(const FrontendCommandSpec *spec) {
    if (!spec || !spec->id || !spec->label || !spec->handler || find_command(spec->id))
        return false;
    CommandEntry *entry = NULL;
    for (size_t i = 0; i < FRONTEND_COMMAND_CAPACITY; ++i) {
        if (!commands[i].used) {
            entry = &commands[i];
            break;
        }
    }
    if (!entry || !copy_text(entry->label, sizeof(entry->label), spec->label)
        || !copy_text(entry->menu, sizeof(entry->menu), spec->menu ? spec->menu : "")
        || !copy_text(entry->shortcut, sizeof(entry->shortcut),
                      spec->shortcut ? spec->shortcut : "")) {
        if (entry) memset(entry, 0, sizeof(*entry));
        return false;
    }
    entry->used = true;
    entry->id = spec->id;
    entry->flags = spec->flags;
    entry->enabled = true;
    entry->handler = spec->handler;
    entry->userdata = spec->userdata;
    ++command_count;
    return true;
}

bool frontend_command_unregister(unsigned id) {
    CommandEntry *entry = find_command(id);
    if (!entry) return false;
    memset(entry, 0, sizeof(*entry));
    --command_count;
    return true;
}

bool frontend_command_invoke(unsigned id, char *error, size_t error_size) {
    CommandEntry *entry = find_command(id);
    if (!entry) {
        if (error && error_size) snprintf(error, error_size, "Unknown frontend command %u", id);
        return false;
    }
    if (!entry->enabled) {
        if (error && error_size) snprintf(error, error_size, "%s is unavailable", entry->label);
        return false;
    }
    if (error && error_size) error[0] = '\0';
    return entry->handler(entry->userdata, error, error_size);
}

bool frontend_command_set_enabled(unsigned id, bool enabled) {
    CommandEntry *entry = find_command(id);
    if (!entry) return false;
    entry->enabled = enabled;
    return true;
}

bool frontend_command_set_checked(unsigned id, bool checked) {
    CommandEntry *entry = find_command(id);
    if (!entry || !(entry->flags & FRONTEND_COMMAND_CHECKABLE)) return false;
    entry->checked = checked;
    return true;
}

bool frontend_command_get(unsigned id, FrontendCommandInfo *info) {
    const CommandEntry *entry = find_command(id);
    if (!entry || !info) return false;
    export_info(entry, info);
    return true;
}

size_t frontend_command_count(void) {
    return command_count;
}

bool frontend_command_at(size_t index, FrontendCommandInfo *info) {
    const CommandEntry *entry = command_at_index(index);
    if (!entry || !info) return false;
    export_info(entry, info);
    return true;
}
