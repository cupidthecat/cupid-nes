/*
 * update_checker.c - Worker lifecycle and persisted release acknowledgements
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "update_checker.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool save(UpdateChecker *c, char *error, size_t size) {
    char text[160];
    int length =
        snprintf(text, sizeof(text), "version=1\nautomatic=%u\nacknowledged=%s\n", c->automatic, c->acknowledged);
    NesFileResult result = nes_file_write_atomic(c->preferences, text, (size_t)length);
    if (error && size) {
        snprintf(error, size, "%s", result == NES_FILE_OK ? "" : nes_file_result_message(result));
    }
    return result == NES_FILE_OK;
}

bool update_checker_init(UpdateChecker *c, const char *version, const char *preferences, char *error, size_t size) {
    int ignored;
    if (!c || !version || !preferences || !update_version_compare(version, version, &ignored) ||
        strlen(preferences) >= sizeof(c->preferences)) {
        return false;
    }
    memset(c, 0, sizeof(*c));
    strcpy(c->current, version);
    strcpy(c->preferences, preferences);
    c->fetch = update_fetch_native;
    uint8_t *bytes = NULL;
    size_t length = 0;
    NesFileResult result = nes_file_read_all(preferences, 159, &bytes, &length);
    if (result == NES_FILE_NOT_FOUND) {
        return true;
    }
    if (result != NES_FILE_OK) {
        if (error && size) {
            snprintf(error, size, "%s", nes_file_result_message(result));
        }
        return false;
    }
    char text[160];
    memcpy(text, bytes, length);
    text[length] = 0;
    free(bytes);
    const char prefix[] = "version=1\nautomatic=";
    bool valid = !memchr(text, 0, length) && !strncmp(text, prefix, sizeof(prefix) - 1);
    const char *p = text + (valid ? sizeof(prefix) - 1 : length);
    valid = valid && (*p == '0' || *p == '1') && !strncmp(p + 1, "\nacknowledged=", 14);
    if (valid) {
        c->automatic = *p == '1';
        p += 15;
        const char *end = strchr(p, '\n');
        valid = end && !end[1] && (size_t)(end - p) < sizeof(c->acknowledged);
        if (valid) {
            memcpy(c->acknowledged, p, (size_t)(end - p));
            valid = !c->acknowledged[0] || update_version_compare(c->acknowledged, c->acknowledged, &ignored);
        }
    }
    if (!valid) {
        c->automatic = false;
        c->acknowledged[0] = 0;
        if (error && size) {
            snprintf(error, size, "Invalid update preferences; automatic checks disabled");
        }
    }
    return valid;
}

static int worker(void *context) {
    UpdateChecker *c = context;
    char *body = malloc(UPDATE_METADATA_LIMIT + 1);
    unsigned status = 0;
    c->worker_status = UPDATE_ERROR;
    c->worker_message[0] = 0;
    memset(&c->worker_release, 0, sizeof(c->worker_release));
    if (!body) {
        snprintf(c->worker_message, sizeof(c->worker_message), "Not enough memory to check updates");
    } else if (!c->fetch(c->fetch_context, body, UPDATE_METADATA_LIMIT + 1, &status, c->worker_message,
                         sizeof(c->worker_message))) {
        if (!c->worker_message[0]) {
            snprintf(c->worker_message, sizeof(c->worker_message), "Offline or update server unavailable");
        }
    } else if (status != 200) {
        snprintf(c->worker_message, sizeof(c->worker_message),
                 status == 403 || status == 429 ? "Update server rate limit reached (HTTP %u); try later"
                                                : "Update server returned HTTP %u",
                 status);
    } else if (!update_metadata_parse(body, strlen(body), &c->worker_release)) {
        snprintf(c->worker_message, sizeof(c->worker_message), "Update server returned malformed release metadata");
    } else {
        int order = 0;
        if (!strcmp(c->current, "0.0.0-dev")) {
            c->worker_status = UPDATE_CURRENT;
            snprintf(c->worker_message, sizeof(c->worker_message),
                     "Development build: latest release %s; version ordering unavailable", c->worker_release.version);
        } else if (update_version_compare(c->worker_release.version, c->current, &order)) {
            c->worker_status = order > 0 ? UPDATE_AVAILABLE : UPDATE_CURRENT;
            snprintf(c->worker_message, sizeof(c->worker_message),
                     order > 0 ? "Update available: %s" : "Up to date (latest release %s)", c->worker_release.version);
        }
    }
    free(body);
    SDL_AtomicSet(&c->finished, 1);
    return 0;
}

bool update_checker_start(UpdateChecker *c, bool automatic, char *error, size_t size) {
    if (!c || !c->fetch) {
        return false;
    }
    if (automatic && (!c->automatic || c->attempted)) {
        return true;
    }
    if (c->thread) {
        return true;
    }
    c->automatic_request = automatic;
    c->attempted = true;
    c->notify = !automatic;
    c->status = UPDATE_CHECKING;
    SDL_AtomicSet(&c->finished, 0);
    snprintf(c->message, sizeof(c->message), "Checking for updates...");
    c->thread = SDL_CreateThread(worker, "release-check", c);
    if (!c->thread) {
        c->status = UPDATE_ERROR;
        snprintf(c->message, sizeof(c->message), "Could not start update check: %s", SDL_GetError());
        if (error && size) {
            snprintf(error, size, "%s", c->message);
        }
        return false;
    }
    return true;
}

void update_checker_poll(UpdateChecker *c) {
    if (!c || !c->thread || !SDL_AtomicGet(&c->finished)) {
        return;
    }
    SDL_WaitThread(c->thread, NULL);
    c->thread = NULL;
    c->status = c->worker_status;
    c->release = c->worker_release;
    strcpy(c->message, c->worker_message);
    c->notify = !c->automatic_request || (c->status == UPDATE_AVAILABLE && strcmp(c->acknowledged, c->release.version));
}

bool update_checker_set_automatic(UpdateChecker *c, bool automatic, char *error, size_t size) {
    if (!c) {
        return false;
    }
    bool previous = c->automatic;
    c->automatic = automatic;
    if (save(c, error, size)) {
        return true;
    }
    c->automatic = previous;
    return false;
}

bool update_checker_acknowledge(UpdateChecker *c, char *error, size_t size) {
    if (!c || !c->release.version[0]) {
        return false;
    }
    char previous[96];
    strcpy(previous, c->acknowledged);
    strcpy(c->acknowledged, c->release.version);
    if (!save(c, error, size)) {
        strcpy(c->acknowledged, previous);
        return false;
    }
    c->notify = false;
    return true;
}

static bool check(void *context, char *error, size_t size) {
    return update_checker_start(context, false, error, size);
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    UpdateChecker *c = context;
    update_checker_poll(c);
    FrontendPanelControl controls[] = {
        {1, FRONTEND_PANEL_TEXT, "Running version", c->current, NULL, 0, 0, true, true},
        {2, FRONTEND_PANEL_TEXT, "Latest release", c->release.version, NULL, 0, 0, true, true},
        {3, FRONTEND_PANEL_CHECKBOX, "Check automatically on startup", NULL, NULL, 0, c->automatic, true, false},
        {4, FRONTEND_PANEL_ACTION, "Check now", NULL, NULL, 0, 0, c->status != UPDATE_CHECKING, false},
        {5, FRONTEND_PANEL_ACTION, "Open release page", NULL, NULL, 0, 0, c->release.url[0] != 0, false},
        {6, FRONTEND_PANEL_ACTION, "Acknowledge this release", NULL, NULL, 0, 0, c->release.version[0] != 0, false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }
    model->status = c->message[0] ? c->message : "Checks download release metadata only; installation is manual";
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)value;
    UpdateChecker *c = context;
    if (id == 3) {
        return update_checker_set_automatic(c, selected != 0, error, size);
    }
    if (id == 4) {
        return check(c, error, size);
    }
    if (id == 6) {
        return update_checker_acknowledge(c, error, size);
    }
    if (id == 5 && c->release.url[0]) {
        if (SDL_OpenURL(c->release.url) != 0) {
            if (error && size) {
                snprintf(error, size, "%s", SDL_GetError());
            }
            return false;
        }
        return update_checker_acknowledge(c, error, size);
    }
    return false;
}

bool update_checker_register_ui(UpdateChecker *c) {
    FrontendPanelSpec panel = {UPDATE_PANEL, "Check for Updates", "Help", 0, snapshot, action, c};
    FrontendCommandSpec command = {UPDATE_CHECK_COMMAND, "Check for Updates", "Help", NULL, 0, check, c};
    if (!frontend_panel_register(&panel)) {
        return false;
    }
    if (!frontend_command_register(&command)) {
        frontend_panel_unregister(UPDATE_PANEL);
        return false;
    }
    return true;
}

void update_checker_shutdown(UpdateChecker *c) {
    if (!c) {
        return;
    }
    /* The HTTPS request is bounded by transport timeouts. No worker accesses freed storage. */
    if (c->thread) {
        SDL_WaitThread(c->thread, NULL);
        c->thread = NULL;
    }
    frontend_panel_unregister(UPDATE_PANEL);
    frontend_command_unregister(UPDATE_CHECK_COMMAND);
}
