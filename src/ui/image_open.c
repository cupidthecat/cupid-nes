/*
 * image_open.c - Desktop image-source selection and activation
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "image_open.h"
#include "../media/image_source.h"
#include "../util/sha1.h"
#include "../apu/epsm.h"
#include "../cheats/cheats.h"
#include "app_paths.h"
#include <stdlib.h>
#include <SDL2/SDL.h>
#include <stdio.h>
#include <string.h>

enum { ARCHIVE_PAGE_SIZE = 8, ARCHIVE_PREVIOUS = 0x7000, ARCHIVE_NEXT = 0x7001, ARCHIVE_CANCEL = 0x7002 };

static bool apply_firmware(const FrontendSettings *settings) {
    return (settings->epsm_adpcm_path[0] ? epsm_load_adpcm_file(settings->epsm_adpcm_path)
                                         : epsm_set_adpcm_rom(NULL, 0)) &&
           rom_set_fcns_kanji_firmware(settings->fcns_kanji_path[0] ? settings->fcns_kanji_path : NULL);
}

static bool copy_text(char *dst, size_t capacity, const char *source) {
    if (!dst || !capacity || !source || strlen(source) >= capacity) {
        return false;
    }
    memcpy(dst, source, strlen(source) + 1);
    return true;
}

static const char *display_name(const NesImageSource *source) {
    const char *name = source->member ? source->member : source->path;
    const char *slash = strrchr(name, '/');
    const char *backslash = strrchr(name, '\\');
    const char *base = slash && backslash ? (slash > backslash ? slash : backslash) : slash ? slash : backslash;
    return base ? base + 1 : name;
}

static bool choose_archive_member(const NesArchiveList *list, char *member, size_t member_size, char *error,
                                  size_t error_size) {
    if (!list || !list->count || !member || !member_size) {
        return false;
    }
    size_t page = 0;
    const size_t page_count = (list->count + ARCHIVE_PAGE_SIZE - 1) / ARCHIVE_PAGE_SIZE;
    for (;;) {
        SDL_MessageBoxButtonData buttons[ARCHIVE_PAGE_SIZE + 3];
        char labels[ARCHIVE_PAGE_SIZE][192];
        size_t button_count = 0;
        size_t first = page * ARCHIVE_PAGE_SIZE;
        size_t last = first + ARCHIVE_PAGE_SIZE;
        if (last > list->count) {
            last = list->count;
        }
        for (size_t index = first; index < last; ++index) {
            snprintf(labels[index - first], sizeof(labels[0]), "%.180s", list->entries[index].name);
            buttons[button_count++] =
                (SDL_MessageBoxButtonData){index == first ? SDL_MESSAGEBOX_BUTTON_RETURNKEY_DEFAULT : 0,
                                           (int)(index - first), labels[index - first]};
        }
        if (page > 0) {
            buttons[button_count++] = (SDL_MessageBoxButtonData){0, ARCHIVE_PREVIOUS, "Previous"};
        }
        if (page + 1 < page_count) {
            buttons[button_count++] = (SDL_MessageBoxButtonData){0, ARCHIVE_NEXT, "Next"};
        }
        buttons[button_count++] =
            (SDL_MessageBoxButtonData){SDL_MESSAGEBOX_BUTTON_ESCAPEKEY_DEFAULT, ARCHIVE_CANCEL, "Cancel"};

        char message[160];
        snprintf(message, sizeof(message), "Choose an image from the archive (page %zu of %zu).", page + 1, page_count);
        SDL_MessageBoxData box = {.flags = SDL_MESSAGEBOX_INFORMATION,
                                  .window = NULL,
                                  .title = "Choose Archive Image",
                                  .message = message,
                                  .numbuttons = (int)button_count,
                                  .buttons = buttons,
                                  .colorScheme = NULL};
        int selected = ARCHIVE_CANCEL;
        if (SDL_ShowMessageBox(&box, &selected) != 0) {
            if (error && error_size) {
                snprintf(error, error_size, "Could not show archive selection: %s", SDL_GetError());
            }
            return false;
        }
        if (selected == ARCHIVE_CANCEL) {
            if (error && error_size) {
                error[0] = '\0';
            }
            return false;
        }
        if (selected == ARCHIVE_PREVIOUS) {
            --page;
            continue;
        }
        if (selected == ARCHIVE_NEXT) {
            ++page;
            continue;
        }
        if (selected < 0 || (size_t)selected >= last - first) {
            if (error && error_size) {
                snprintf(error, error_size, "Invalid archive selection");
            }
            return false;
        }
        if (!copy_text(member, member_size, list->entries[first + (size_t)selected].name)) {
            if (error && error_size) {
                snprintf(error, error_size, "The selected archive member name is too long");
            }
            return false;
        }
        return true;
    }
}

bool frontend_image_open(void *userdata, const FrontendImageRequest *request, FrontendImageResult *result, char *error,
                         size_t error_size) {
    FrontendImageEnvironment *environment = userdata;
    if (environment) {
        environment->machine_changed = false;
    }
    if (!request || !request->path[0] || !result) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    NesImageRequest media_request = {.path = request->path,
                                     .member = request->archive_member[0] ? request->archive_member : NULL,
                                     .patch_path = request->patch_path[0] ? request->patch_path : NULL};
    NesImageSource source;
    NesMediaResult media_result = nes_image_prepare(&media_request, &source, error, error_size);
    char selected_member[FRONTEND_IMAGE_MEMBER_MAX];
    if (media_result == NES_MEDIA_NEEDS_SELECTION && !media_request.member) {
        NesArchiveList list;
        media_result = nes_image_list(request->path, &list, error, error_size);
        if (media_result != NES_MEDIA_OK) {
            return false;
        }
        bool selected = choose_archive_member(&list, selected_member, sizeof(selected_member), error, error_size);
        nes_archive_list_free(&list);
        if (!selected) {
            return false;
        }
        media_request.member = selected_member;
        media_result = nes_image_prepare(&media_request, &source, error, error_size);
    }
    if (media_result != NES_MEDIA_OK) {
        return false;
    }

    FrontendImageResult prepared;
    memset(&prepared, 0, sizeof(prepared));
    nes_sha1(source.data, source.size, prepared.sha1);
    bool valid = copy_text(prepared.save_identity, sizeof(prepared.save_identity), source.save_path) &&
                 copy_text(prepared.title, sizeof(prepared.title), display_name(&source)) &&
                 (!source.member || copy_text(prepared.archive_member, sizeof(prepared.archive_member), source.member));
    if (!valid) {
        nes_image_source_free(&source);
        if (error && error_size) {
            snprintf(error, error_size, "Loaded image identity is too long");
        }
        return false;
    }

    FrontendImageRequest resolved = *request;
    FrontendSettings previous, effective;
    char cheat_path[1024] = {0};
    if (environment && environment->configuration && environment->effective) {
        previous = *environment->effective;
        if (environment->rollback_settings) {
            effective = *environment->rollback_settings;
        } else if (!game_config_prepare(environment->configuration, &prepared, &effective, cheat_path,
                                        sizeof(cheat_path), error, error_size)) {
            nes_image_source_free(&source);
            return false;
        }
        if (!frontend_settings_validate_firmware(&effective, error, error_size) ||
            !frontend_settings_apply_core(&effective, error, error_size)) {
            nes_image_source_free(&source);
            return false;
        }
        if (!apply_firmware(&effective)) {
            (void)apply_firmware(&previous);
            (void)frontend_settings_apply_core(&previous, NULL, 0);
            nes_image_source_free(&source);
            if (error && error_size) {
                snprintf(error, error_size, "Could not apply game firmware");
            }
            return false;
        }
        uint64_t overrides = environment->configuration->config.present | environment->configuration->cli_fields;
        if (!resolved.fds_bios_path[0] || (overrides & (UINT64_C(1) << game_config_find_field("fds_bios")))) {
            strcpy(resolved.fds_bios_path, effective.fds_bios_path);
        }
        if (!resolved.studybox_bios_path[0] || (overrides & (UINT64_C(1) << game_config_find_field("studybox_bios")))) {
            strcpy(resolved.studybox_bios_path, effective.studybox_bios_path);
        }
        if (!resolved.fds_overlay_path[0] || (overrides & (UINT64_C(1) << game_config_find_field("disk_overlay")))) {
            strcpy(resolved.fds_overlay_path, effective.disk_overlay_path);
        }
        resolved.fds_save_mode = effective.disk_save_mode;
        resolved.fds_write_protected = effective.fds_write_protected;
        *environment->effective = effective;
    }
    request = &resolved;
    NesImageLoadOptions options = {.fds_bios = request->fds_bios_path[0] ? request->fds_bios_path : NULL,
                                   .studybox_bios = request->studybox_bios_path[0] ? request->studybox_bios_path : NULL,
                                   .disk_write_protected = request->fds_write_protected,
                                   .disk_save_mode = request->fds_save_mode,
                                   .disk_overlay_path =
                                       request->fds_overlay_path[0] ? request->fds_overlay_path : NULL};
    media_result = nes_image_load_with_options(&source, &options, error, error_size);
    if (media_result != NES_MEDIA_OK) {
        if (environment && environment->configuration && environment->effective) {
            (void)apply_firmware(&previous);
            *environment->effective = previous;
            (void)frontend_settings_apply_core(&previous, NULL, 0);
        }
        nes_image_source_free(&source);
        return false;
    }
    if (environment && environment->configuration) {
        environment->machine_changed = true;
        environment->cheat_override =
            (environment->configuration->config.present & (UINT64_C(1) << game_config_find_field("cheat_file"))) != 0;
        if (environment->rollback_settings && environment->cheat_override) {
            strcpy(cheat_path, environment->configuration->config.values[game_config_find_field("cheat_file")]);
        }
        strcpy(environment->cheat_path, cheat_path);
        if (!environment->cheat_override) {
            char relative[64];
            snprintf(relative, sizeof(relative), "cheats-%08X.txt", rom_file_crc32());
            if (frontend_paths_join(cheat_path, sizeof(cheat_path), relative)) {
                FILE *file = nes_file_open(cheat_path, "rb");
                if (file) {
                    fclose(file);
                } else {
                    cheat_path[0] = 0;
                }
            }
        }
        if (cheat_path[0]) {
            CheatResult loaded = cheats_load_file(cheat_path);
            if (loaded != CHEAT_OK) {
                nes_image_source_free(&source);
                if (error && error_size) {
                    snprintf(error, error_size, "%s", cheats_result_message(loaded));
                }
                return false;
            }
        } else {
            (void)cheats_clear();
        }
    }
    prepared.request_valid = true;
    prepared.opened_request = resolved;
    *result = prepared;
    nes_image_source_free(&source);
    if (error && error_size) {
        error[0] = '\0';
    }
    return true;
}
