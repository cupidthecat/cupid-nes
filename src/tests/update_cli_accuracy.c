/*
 * update_cli_accuracy.c - Option table and asynchronous update regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/update_checker.h"
#include "../ui/cli_help.h"
#include "../ppu/ppu.h"
#include "../ui/settings.h"
#include "../ui/frontend_panels.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <string.h>
#define CHECK(c)                                                                                                       \
    do {                                                                                                               \
        if (!(c)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #c);                                                    \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

typedef struct {
    const char *body;
    unsigned status, calls;
    bool offline;
} Response;

static bool fetch(void *context, char *body, size_t capacity, unsigned *status, char *error, size_t size) {
    Response *r = context;
    ++r->calls;
    if (r->offline) {
        snprintf(error, size, "offline");
        return false;
    }
    *status = r->status;
    snprintf(body, capacity, "%s", r->body);
    return true;
}

static bool completed(UpdateChecker *checker) {
    Uint32 started = SDL_GetTicks();
    while (checker->thread && SDL_GetTicks() - started < 3000) {
        update_checker_poll(checker);
        SDL_Delay(1);
    }
    return !checker->thread;
}

static int updates(void) {
    int order = 0;
    UpdateRelease release;
    CHECK(update_version_compare("v1.10.0", "1.9.9", &order) && order > 0);
    CHECK(update_version_compare("1.0.0-rc.10", "1.0.0-rc.2", &order) && order > 0);
    CHECK(update_version_compare("1.0.0", "1.0.0-rc.10", &order) && order > 0);
    CHECK(update_version_compare("1.0.0+a", "1.0.0+b", &order) && order == 0);
    CHECK(!update_version_compare("1.0.0-01", "1.0.0", &order));
    CHECK(!update_version_compare("1.0", "1.0.0", &order));
    const char *json = "{\"tag_name\":\"v1.2.0\",\"html_url\":\"https://github.com/cupidthecat/cupid-nes/releases/tag/"
                       "v1.2.0\",\"draft\":false,\"prerelease\":false,\"assets\":[{\"id\":123}]}";
    CHECK(update_metadata_parse(json, strlen(json), &release));
    CHECK(!update_metadata_parse("{}", 2, &release));
    const char *malformed = "{\"tag_name\":\"1.2.0\",}";
    CHECK(!update_metadata_parse(malformed, strlen(malformed), &release));

    CHECK(update_version_channel("1.2.0-preview.205.1+abc") == UPDATE_CHANNEL_PREVIEW);
    CHECK(update_version_channel("1.2.0+preview") == UPDATE_CHANNEL_STABLE);
    CHECK(!update_release_url_valid("https://github.com/cupidthecat/cupid-nes/releases/tag/v1.2.0?evil", "v1.2.0", false));
    CHECK(!update_release_url_valid("https://github.com/cupidthecat/cupid-nes/releases/download/v1.2.0/../evil.zip", "v1.2.0", true));
    CHECK(!update_release_url_valid("https://github.com/cupidthecat/cupid-nes/releases/download/v1.2.0/%2e.zip", "v1.2.0", true));
    CHECK(!update_release_url_valid("https://evil.example/releases/download/v1.2.0/cupid-windows-x64.zip", "v1.2.0", true));
    CHECK(update_release_url_valid("https://github.com/cupidthecat/cupid-nes/releases/download/v1.2.0/cupid-windows-x64.zip", "v1.2.0", true));
    const char *preview = "{\"tag_name\":\"v1.3.0-preview.10\",\"html_url\":\"https://github.com/cupidthecat/cupid-nes/releases/tag/v1.3.0-preview.10\",\"draft\":false,\"prerelease\":true,\"assets\":[{\"name\":\"cupid-windows-x64.zip\",\"browser_download_url\":\"https://github.com/cupidthecat/cupid-nes/releases/download/v1.3.0-preview.10/cupid-windows-x64.zip\"}]}";
    char list[4096];
    snprintf(list, sizeof(list), "[%s,%s]", json, preview);
    CHECK(update_metadata_select(list, strlen(list), UPDATE_CHANNEL_PREVIEW, &release) && !strcmp(release.version, "v1.3.0-preview.10"));
#if defined(_WIN64) && !defined(_M_ARM64) && !defined(__aarch64__)
    CHECK(strstr(release.package_url, "/v1.3.0-preview.10/cupid-windows-x64.zip"));
#endif
    CHECK(update_metadata_select(list, strlen(list), UPDATE_CHANNEL_STABLE, &release) && !strcmp(release.version, "v1.2.0"));
    snprintf(list, sizeof(list), "[%s,%s]", preview, json);
    CHECK(update_metadata_select(list, strlen(list), UPDATE_CHANNEL_PREVIEW, &release) && !strcmp(release.version, "v1.3.0-preview.10"));
    CHECK(!update_metadata_parse(preview, strlen(preview), &release));
    CHECK(update_metadata_select("[]", 2, UPDATE_CHANNEL_PREVIEW, &release) && !release.version[0]);
    CHECK(!update_metadata_select("[] trailing", 11, UPDATE_CHANNEL_PREVIEW, &release));
    const char *duplicate = "{\"tag_name\":\"1.2.0\",\"draft\":false,\"draft\":true}";
    CHECK(!update_metadata_select(duplicate, strlen(duplicate), UPDATE_CHANNEL_PREVIEW, &release));

    char changed[4096];
    snprintf(changed, sizeof(changed), "%s", preview);
    char *flag = strstr(changed, "\"draft\":false");
    CHECK(flag);
    /* Same width substitution keeps the JSON valid while changing eligibility. */
    memcpy(flag + 8, "true ", 5);
    CHECK(update_metadata_select(changed, strlen(changed), UPDATE_CHANNEL_PREVIEW, &release) && !release.version[0]);
    snprintf(changed, sizeof(changed), "%s", preview);
    char *asset_tag = strstr(changed, "/download/v1.3.0-preview.10/");
    CHECK(asset_tag);
    asset_tag[11] = '9';
#if defined(_WIN32)
    CHECK(!update_metadata_select(changed, strlen(changed), UPDATE_CHANNEL_PREVIEW, &release));
#endif
    CHECK(!update_metadata_select(list, strlen(list) - 1, UPDATE_CHANNEL_PREVIEW, &release));
    CHECK(!update_metadata_select("[null]", 6, UPDATE_CHANNEL_PREVIEW, &release));
    char path[256], error[256];
    snprintf(path, sizeof(path), "build/update-check-%llu.ini", (unsigned long long)SDL_GetPerformanceCounter());
    UpdateChecker checker;
    CHECK(update_checker_init(&checker, "1.0.0", path, error, sizeof(error)));
    CHECK(update_checker_register_ui(&checker));
    FrontendPanelControl update_controls[10];
    FrontendPanelModel update_model = {update_controls, 10, 0, NULL};
    CHECK(frontend_panel_snapshot(UPDATE_PANEL, &update_model, error, sizeof(error)) && update_model.count == 10);
    strcpy(checker.release.version, "v1.2.0");
    strcpy(checker.release.package_url, "https://evil.example/payload.zip");
    CHECK(!frontend_panel_action(UPDATE_PANEL, 9, NULL, 0, error, sizeof(error)));
    Response response = {json, 200, 0, false};
    checker.fetch = fetch;
    checker.fetch_context = &response;
    CHECK(update_checker_start(&checker, true, error, sizeof(error)) && !checker.thread && response.calls == 0);
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker));
    CHECK(checker.status == UPDATE_AVAILABLE && checker.notify && response.calls == 1);
    CHECK(update_checker_acknowledge(&checker, error, sizeof(error)) && !checker.notify);
    CHECK(update_checker_set_automatic(&checker, true, error, sizeof(error)));
    update_checker_shutdown(&checker);
    CHECK(update_checker_init(&checker, "1.0.0", path, error, sizeof(error)) && checker.automatic &&
          !strcmp(checker.acknowledged, "v1.2.0"));
    checker.fetch = fetch;
    checker.fetch_context = &response;
    CHECK(update_checker_start(&checker, true, error, sizeof(error)) && completed(&checker));
    CHECK(checker.status == UPDATE_AVAILABLE && !checker.notify);
    unsigned calls = response.calls;
    CHECK(update_checker_start(&checker, true, error, sizeof(error)) && !checker.thread && response.calls == calls);
    strcpy(checker.current, "1.2.0");
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) &&
          checker.status == UPDATE_CURRENT);
    response.offline = true;
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) &&
          checker.status == UPDATE_ERROR);
    response.offline = false;
    response.status = 429;
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) &&
          checker.status == UPDATE_ERROR && strstr(checker.message, "rate limit"));
    response.status = 200;
    response.body = "{malformed}";
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) &&
          checker.status == UPDATE_ERROR);

    response.body = list;
    checker.channel = UPDATE_CHANNEL_PREVIEW;
    strcpy(checker.current, "1.3.0-preview.2");
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) && checker.status == UPDATE_AVAILABLE);
    strcpy(checker.current, "1.3.0-preview.10");
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) && checker.status == UPDATE_CURRENT);
    strcpy(checker.current, "0.0.0-dev");
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) && checker.status == UPDATE_UNKNOWN);
    response.body = "[]";
    CHECK(update_checker_start(&checker, false, error, sizeof(error)) && completed(&checker) && checker.status == UPDATE_UNKNOWN && !checker.release.url[0]);
    update_checker_shutdown(&checker);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

#ifndef CUPID_UPDATE_TEST_MAIN
static int options(void) {
    frontend_panels_reset();
    FrontendCliHelp help;
    CHECK(frontend_cli_help_register(&help));
    CHECK(help.count == frontend_cli_count());
    for (size_t i = 0; i < frontend_cli_count(); ++i) {
        const FrontendCliOption *o = frontend_cli_at(i);
        CHECK(frontend_cli_find(o->name) == o->id && o->description[0] && o->default_value[0]);
        if (o->alias) {
            CHECK(frontend_cli_find(o->alias) == o->id);
        }
        if (o->takes_value) {
            CHECK(!frontend_cli_accepts(o, NULL));
        } else {
            CHECK(frontend_cli_accepts(o, NULL));
        }
    }
    CHECK(frontend_cli_help_search(&help, "REGION") && help.count > 0);
    CHECK(frontend_panel_action(CLI_HELP_PANEL, 1, "zz-no-switch", 0, NULL, 0) && help.count == 0);
    CHECK(frontend_cli_help_search(&help, "") && frontend_panel_action(CLI_HELP_PANEL, 9, NULL, 0, NULL, 0) &&
          help.selected == 1);
    CHECK(frontend_panel_action(CLI_HELP_PANEL, 8, NULL, 0, NULL, 0) && help.selected == 0);
    FrontendPanelControl controls[12];
    FrontendPanelModel model = {controls, 12, 0, NULL};
    frontend_panel_set_session_active(false);
    CHECK(frontend_panel_snapshot(CLI_HELP_PANEL, &model, NULL, 0) && model.count == 9);
    char *argv[] = {"cupid", "--help"};
    FrontendCliOptions parsed;
    char error[256];
    CHECK(frontend_cli_parse(2, argv, &parsed, error, sizeof(error)) && parsed.help);
    char *invalid[] = {"cupid", "--region", "wrong"};
    CHECK(!frontend_cli_parse(3, invalid, &parsed, error, sizeof(error)));
    char *missing[] = {"cupid", "--movie"};
    CHECK(!frontend_cli_parse(2, missing, &parsed, error, sizeof(error)));
    NesRegionMode previous_region = nes_region_mode();
    bool previous_decay = ppu_oam_decay_enabled();
    char *valid[] = {"cupid", "--region", "ntsc", "--ppu-oam-decay", "test.nes"};
    bool parsed_valid = frontend_cli_parse(5, valid, &parsed, error, sizeof(error));
    (void)nes_set_region_mode(previous_region);
    ppu_set_oam_decay(previous_decay);
    CHECK(parsed_valid && parsed.rom_path && !strcmp(parsed.rom_path, "test.nes"));
    frontend_cli_help_unregister();
    return 0;
}

int test_update_cli_accuracy(void) {
    int result = updates() + options();
    printf("Update worker/CLI reference: %s\n", result ? "FAIL" : "PASS");
    return result;
}
#endif
#ifdef CUPID_LIFECYCLE_TEST_MAIN
#include "../../include/globals.h"
#include "../joypad/joypad.h"
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
Joypad pad1 = {0}, pad2 = {0};
int test_lifecycle_accuracy(void);

int main(void) {
    int result = test_update_cli_accuracy() + test_lifecycle_accuracy();
    SDL_Quit();
    return result;
}
#endif

#ifdef CUPID_UPDATE_TEST_MAIN
int main(void) {
    int result = updates();
    printf("Update regressions: %s\n", result ? "FAIL" : "PASS");
    SDL_Quit();
    return result;
}
#endif
