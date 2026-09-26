/*
 * tas_navigation_input.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Native bookmark gestures, session guards and scale checks.
 * Included by tas_editor_input_accuracy.c. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_TESTS_TAS_NAVIGATION_INPUT_H
#define CUPID_TESTS_TAS_NAVIGATION_INPUT_H

#include "../system/vs_system.h"
#include <stdlib.h>

static bool navigation_snapshot(FrontendDesktopUi *ui, unsigned percent, unsigned height, const char *view) {
    const char *directory = getenv("CUPID_TAS_SCREENSHOTS");
    int width, pixels;
    if (SDL_GetRendererOutputSize(ui->renderer, &width, &pixels) < 0) {
        return false;
    }
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, width, pixels, 32, SDL_PIXELFORMAT_ARGB8888);
    if (!surface) {
        return false;
    }
    bool saved =
        SDL_RenderReadPixels(ui->renderer, NULL, surface->format->format, surface->pixels, surface->pitch) == 0;
    SDL_FRect tab;
    int window_width, window_height;
    SDL_GetWindowSize(ui->window, &window_width, &window_height);
    saved = saved && window_width > 0 && window_height > 0 &&
            desktop_clay_bounds(ui->clay, HIT_TAS_ACTION, TAS_ACTION_SIDEBAR_NAVIGATION, 0, &tab);
    if (saved) {
        float sx = ui->ui_scale * width / window_width, sy = ui->ui_scale * pixels / window_height;
        int left = (int)((tab.x + 2) * sx), right = (int)((tab.x + tab.w - 2) * sx);
        int top = (int)((tab.y + 2) * sy), bottom = (int)((tab.y + tab.h - 2) * sy);
        unsigned ink = 0;
        saved = left >= 0 && top >= 0 && right <= width && bottom <= pixels;
        for (int y = top; saved && y < bottom; ++y) {
            const uint32_t *row = (const uint32_t *)((const uint8_t *)surface->pixels + y * surface->pitch);
            for (int x = left; x < right; ++x) {
                Uint8 r, g, b;
                SDL_GetRGB(row[x], surface->format, &r, &g, &b);
                ink += r > 120 && g > 120 && b > 120;
            }
        }
        /* Check rasterized text inside the sidebar tab, not just submitted
         * Clay bounds. A stale renderer transform can put the sidebar offscreen. */
        saved = saved && ink >= 8;
    }
    if (saved && directory && *directory) {
        char path[1024];
        int length = snprintf(path, sizeof(path), "%s/tas-bookmarks-%u-%u-%s.bmp", directory, percent, height, view);
        saved = length > 0 && (size_t)length < sizeof(path) && SDL_SaveBMP(surface, path) == 0;
    }
    SDL_FreeSurface(surface);
    return saved;
}

static bool navigation_text_input(FrontendDesktopUi *ui, const char *text) {
    if (!ui->edit_text_active || !send_key(ui, SDL_SCANCODE_A, KMOD_LCTRL)) {
        return false;
    }
    /* SDL text events carry a short UTF-8 chunk, not a complete dialog value. */
    for (size_t offset = 0; text[offset];) {
        SDL_Event event = {.type = SDL_TEXTINPUT};
        event.text.windowID = SDL_GetWindowID(ui->window);
        size_t count = strlen(text + offset);
        if (count >= sizeof(event.text.text)) {
            count = sizeof(event.text.text) - 1;
            while (count && ((unsigned char)text[offset + count] & 0xC0) == 0x80) {
                --count;
            }
        }
        memcpy(event.text.text, text + offset, count);
        event.text.text[count] = '\0';
        if (!frontend_desktop_handle_event(ui->parent ? ui->parent : ui, &event)) {
            return false;
        }
        offset += count;
    }
    return send_key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
}

static bool navigation_finish_seek(NesTasSession *session, size_t target) {
    for (unsigned frames = 0; nes_tas_session_seeking(session) && frames < 1000; ++frames) {
        if (nes_tas_session_frame_boundary(session) != NES_MOVIE_OK) {
            return false;
        }
        vs_start_frame();
        unsigned cycles = 100000;
        while (!ppu.frame_complete && cycles--) {
            if (vs_cpu_step() <= 0 || cpu.halted) {
                return false;
            }
        }
        if (!ppu.frame_complete || nes_tas_session_frame_complete(session, true) != NES_MOVIE_OK) {
            return false;
        }
    }
    NesTasProgress progress;
    nes_tas_session_progress(session, &progress);
    return progress.frame == target && !progress.seeking;
}

static int navigation_detail_visibility(FrontendDesktopUi *ui, const char *head, const char *tail,
                                        const char *token_format, unsigned tokens, unsigned percent, unsigned height) {
    int failures = 0;
    DesktopTasEditor *editor = ui->tas_editor;
    uint64_t revision = nes_tas_project_revision(desktop_tas_project_const(ui));
    size_t selection = editor->navigation_selection;
    uint64_t seen = 0;
    bool saw_head = false, saw_tail = false;
    CHECK(tokens && tokens <= 64);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_DETAILS));
    render_editor(ui);
    CHECK(editor->navigation_details && editor->navigation_detail_pages > 1);
    for (size_t page = 0; page < editor->navigation_detail_pages; ++page) {
        CHECK(editor->navigation_detail_page == page);
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_LIST, 0));
        char label[80];
        snprintf(label, sizeof(label), "Details page %zu / %zu", page + 1, editor->navigation_detail_pages);
        CHECK(desktop_clay_text_visible(ui->clay, label));
        saw_head |= desktop_clay_text_visible(ui->clay, head);
        saw_tail |= desktop_clay_text_visible(ui->clay, tail);
        for (unsigned token = 0; token < tokens; ++token) {
            char value[32];
            snprintf(value, sizeof(value), token_format, token);
            if (desktop_clay_text_visible(ui->clay, value)) {
                seen |= UINT64_C(1) << token;
            }
        }
        if (percent) {
            snprintf(label, sizeof(label), "details-%zu", page + 1);
            CHECK(navigation_snapshot(ui, percent, height, label));
        }
        if (page + 1 < editor->navigation_detail_pages) {
            CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_DETAIL_NEXT, 0));
            CHECK(click_action(ui, TAS_ACTION_NAVIGATION_DETAIL_NEXT));
            render_editor(ui);
        }
    }
    CHECK(seen == (tokens == 64 ? UINT64_MAX : (UINT64_C(1) << tokens) - 1));
    CHECK(saw_head && saw_tail);
    CHECK(!desktop_clay_bounds(ui->clay, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_DETAIL_NEXT, 0, &(SDL_FRect){0}));
    for (size_t remaining = editor->navigation_detail_page; remaining; --remaining) {
        CHECK(click_action(ui, TAS_ACTION_NAVIGATION_DETAIL_PREV));
        render_editor(ui);
        CHECK(editor->navigation_detail_page == remaining - 1);
    }
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_LIST));
    render_editor(ui);
    CHECK(!editor->navigation_details && editor->navigation_selection == selection);
    CHECK(nes_tas_project_revision(desktop_tas_project_const(ui)) == revision && !ui->edit_text_active);
cleanup:
    return failures;
}

static int navigation_gestures(FrontendDesktopUi *ui) {
    int failures = 0;
    FrontendExecutionRuntime *runtime = ui->execution;
    NesTasSession *session = desktop_tas_session(ui);
    NesTasProject *project = nes_tas_session_project(session);
    DesktopTasEditor *editor = ui->tas_editor;
    NesStateBlob before = {0}, after = {0};
    char error[256] = {0};
    const char *replacement_path = "build/tas-navigation-ui-replacement.ctas";
    CHECK(project && nes_tas_project_frame_count(project) >= 12);
    CHECK(tas_frontend_seek(runtime, 0, error, sizeof(error)) && navigation_finish_seek(session, 0));
    size_t length = nes_tas_project_frame_count(project);
    int branch = nes_tas_current_branch(project);
    NesTasProgress progress_before, progress;
    NesTasCacheInfo cache_before, cache_after;
    nes_tas_session_progress(session, &progress_before);
    nes_tas_session_cache_info(session, &cache_before);
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    uint64_t revision = nes_tas_project_revision(project);
    editor->cursor = 6;
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SIDEBAR_NAVIGATION));
    render_editor(ui);
    CHECK(editor->sidebar_tab == 5);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_CURSOR));
    CHECK(navigation_text_input(ui, "Route A") && !ui->edit_text_active);
    NesTasNavigationView view;
    CHECK(nes_tas_navigation(project, 0, &view) && view.frame == 6 && !strcmp(view.name, "Route A"));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NAME));
    CHECK(navigation_text_input(ui, "Route B") && !ui->edit_text_active);
    char note[NES_TAS_NAVIGATION_NOTE_MAX + 1];
    for (unsigned i = 0; i < 32; ++i) {
        snprintf(note + 8 * i, sizeof(note) - 8 * i, "note%03u ", i);
    }
    note[sizeof(note) - 1] = '\0';
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NOTE));
    CHECK(navigation_text_input(ui, note) && !ui->edit_text_active);
    CHECK(nes_tas_navigation(project, 0, &view) && !strcmp(view.name, "Route B") && !strcmp(view.note, note));
    CHECK(nes_tas_project_change_since(project, revision).first_changed_frame == SIZE_MAX);
    CHECK(nes_tas_current_branch(project) == branch && nes_tas_project_frame_count(project) == length);
    nes_tas_session_progress(session, &progress);
    nes_tas_session_cache_info(session, &cache_after);
    CHECK(progress.frame == progress_before.frame && progress.rerecord_count == progress_before.rerecord_count);
    CHECK(cache_before.checkpoint_count == cache_after.checkpoint_count &&
          cache_before.checkpoint_bytes == cache_after.checkpoint_bytes &&
          cache_before.initial_bytes == cache_after.initial_bytes);
    CHECK(nes_state_capture(&after) == NES_STATE_OK);
    CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);

    /* Oversized names remain editable; a valid retry commits once. */
    char name[NES_TAS_BOOKMARK_NAME_MAX + 2];
    memset(name, 'x', sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NAME));
    revision = nes_tas_project_revision(project);
    CHECK(navigation_text_input(ui, name) && ui->edit_text_active);
    CHECK(nes_tas_project_revision(project) == revision);
    snprintf(name, sizeof(name), "%s", "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!");
    CHECK(navigation_text_input(ui, name) && !ui->edit_text_active);

    /* A changed model invalidates a dialog and the list it was opened from. */
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NOTE));
    CHECK(nes_tas_navigation_add(project, 0, "Start", "", NULL) == NES_TAS_OK);
    revision = nes_tas_project_revision(project);
    CHECK(navigation_text_input(ui, "stale") && ui->edit_text_active);
    CHECK(nes_tas_project_revision(project) == revision);
    CHECK(send_key(ui, SDL_SCANCODE_ESCAPE, KMOD_NONE));
    desktop_tas_action(ui, TAS_ACTION_NAVIGATION_REMOVE);
    CHECK(nes_tas_project_revision(project) == revision && nes_tas_navigation_count(project) == 2);
    render_editor(ui);
    CHECK(editor->navigation_selection == SIZE_MAX);

    /* Every add goes through the native text dialog, including later pages. */
    for (size_t frame = 7; frame < 15; ++frame) {
        editor->cursor = frame;
        render_editor(ui);
        CHECK(click_action(ui, TAS_ACTION_NAVIGATION_CURSOR));
        char label[48];
        snprintf(label, sizeof(label), "Position %zu", frame);
        CHECK(navigation_text_input(ui, label) && !ui->edit_text_active);
    }
    CHECK(nes_tas_navigation_count(project) == 10);
    const float scales[] = {1, 1.5f, 2};
    for (size_t i = 0; i < 2 * sizeof(scales) / sizeof(scales[0]); ++i) {
        float scale = scales[i % 3];
        unsigned height = i < 3 ? 760 : 640;
        CHECK(resize_editor(ui, (int)(900 * scale), (int)(height * scale), scale));
        render_editor(ui);
        CHECK(click_action(ui, TAS_ACTION_NAVIGATION_OLDER));
        render_editor(ui);
        CHECK(editor->navigation_scroll == 0);
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_SELECT_BASE + 7, 0));
        CHECK(click_action(ui, TAS_ACTION_NAVIGATION_SELECT_BASE + 1));
        render_editor(ui);
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_NAME, 0));
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_NOTE, 0));
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_GOTO, 0));
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_DETAILS, 0));
        CHECK(navigation_snapshot(ui, (unsigned)(100 * scale), height, "list"));
        CHECK(nes_tas_session_set_read_only(session, true) == NES_MOVIE_OK);
        render_editor(ui);
        CHECK(navigation_detail_visibility(ui, "ABCDEFGHIJK", "0123456789!", "note%03u", 32, (unsigned)(100 * scale),
                                           height) == 0);
        CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
        render_editor(ui);
        CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NEWER));
        render_editor(ui);
        CHECK(editor->navigation_scroll == 8);
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_SELECT_BASE + 1, 0));
        CHECK(click_action(ui, TAS_ACTION_NAVIGATION_SELECT_BASE + 1));
    }
    CHECK(resize_editor(ui, 900, 760, 1));
    /* Newlines can produce many more lines than a full note of ordinary words.
     * UTF-8 names must wrap without turning a split codepoint into replacement glyphs. */
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_OLDER));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_SELECT_BASE + 1));
    render_editor(ui);
    char unicode_name[NES_TAS_BOOKMARK_NAME_MAX + 1];
    for (unsigned i = 0; i < 30; ++i) {
        unicode_name[2 * i] = (char)0xC3;
        unicode_name[2 * i + 1] = (char)0xA9;
    }
    memcpy(unicode_name + 60, "FIN", 4);
    for (unsigned width = 1; width <= 230; ++width) {
        size_t fit = desktop_clay_text_fit(ui->clay, unicode_name, 60, 11, (float)width);
        CHECK(fit <= 60 && fit % 2 == 0);
    }
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NAME));
    CHECK(navigation_text_input(ui, unicode_name) && !ui->edit_text_active);
    char multiline[NES_TAS_NAVIGATION_NOTE_MAX + 1];
    for (unsigned i = 0; i < 64; ++i) {
        snprintf(multiline + 4 * i, sizeof(multiline) - 4 * i, "N%02u\n", i);
    }
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NOTE));
    CHECK(navigation_text_input(ui, multiline) && !ui->edit_text_active);
    render_editor(ui);
    CHECK(navigation_detail_visibility(ui, "\xC3\xA9\xC3\xA9", "FIN", "N%02u", 64, 0, 0) == 0);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NAME));
    CHECK(navigation_text_input(ui, name) && !ui->edit_text_active);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NOTE));
    CHECK(navigation_text_input(ui, note) && !ui->edit_text_active);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NEWER));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_SELECT_BASE + 1));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_REMOVE));
    CHECK(nes_tas_navigation_count(project) == 9);
    CHECK(send_key(ui, SDL_SCANCODE_Z, KMOD_LCTRL) && nes_tas_navigation_count(project) == 10);
    CHECK(send_key(ui, SDL_SCANCODE_Y, KMOD_RCTRL) && nes_tas_navigation_count(project) == 9);

    /* End bookmarks seek the pre-input end boundary, not the last input row. */
    CHECK(tas_frontend_seek(runtime, length, error, sizeof(error)) && navigation_finish_seek(session, length));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_PLAYBACK));
    CHECK(navigation_text_input(ui, "End") && !ui->edit_text_active);
    CHECK(nes_tas_navigation(project, 9, &view) && view.frame == length);
    CHECK(tas_frontend_seek(runtime, 0, error, sizeof(error)) && navigation_finish_seek(session, 0));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NEXT));
    CHECK(navigation_finish_seek(session, 6));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_PREV));
    CHECK(navigation_finish_seek(session, 0));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_NEWER));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_SELECT_BASE + 1));
    revision = nes_tas_project_revision(project);
    CHECK(nes_tas_session_set_read_only(session, true) == NES_MOVIE_OK);
    render_editor(ui);
    CHECK(!desktop_clay_bounds(ui->clay, HIT_TAS_ACTION, TAS_ACTION_NAVIGATION_REMOVE, 0, &(SDL_FRect){0}));
    desktop_tas_action(ui, TAS_ACTION_NAVIGATION_REMOVE);
    CHECK(nes_tas_project_revision(project) == revision);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_GOTO));
    CHECK(navigation_finish_seek(session, length));
    CHECK(editor->cursor == length - 1 && nes_tas_current_branch(project) == branch);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    render_editor(ui);
    /* Recording and an active grouped edit disable both mutating and seek actions. */
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    render_editor(ui);
    desktop_tas_action(ui, TAS_ACTION_NAVIGATION_CURSOR);
    desktop_tas_action(ui, TAS_ACTION_NAVIGATION_PREV);
    CHECK(!ui->edit_text_active);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == length && !progress.seeking);
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    render_editor(ui);
    desktop_tas_action(ui, TAS_ACTION_NAVIGATION_CURSOR);
    desktop_tas_action(ui, TAS_ACTION_NAVIGATION_PREV);
    CHECK(!ui->edit_text_active && nes_tas_edit_active(project));
    nes_tas_session_progress(session, &progress);
    CHECK(progress.frame == length && !progress.seeking);
    nes_tas_edit_cancel(project);

    /* Reopening the same file resets project revisions and can reuse addresses.
     * An activation identity must still invalidate the old text dialog. */
    CHECK(tas_frontend_save(runtime, replacement_path, error, sizeof(error)));
    CHECK(tas_frontend_discard(runtime, error, sizeof(error)));
    CHECK(tas_frontend_open(runtime, replacement_path, false, error, sizeof(error)));
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    project = nes_tas_session_project(session);
    nes_tas_session_progress(session, &progress_before);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_CURSOR) && ui->edit_text_active);
    CHECK(tas_frontend_discard(runtime, error, sizeof(error)));
    CHECK(tas_frontend_open(runtime, replacement_path, false, error, sizeof(error)));
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    project = nes_tas_session_project(session);
    nes_tas_session_progress(session, &progress);
    CHECK(progress.generation != progress_before.generation);
    /* Force the allocator-address coincidence while retaining the real old
     * revision and generation. This also covers allocators that quarantine frees. */
    editor->navigation_edit_project = project;
    CHECK(editor->navigation_edit_revision == nes_tas_project_revision(project));
    revision = nes_tas_project_revision(project);
    CHECK(navigation_text_input(ui, "wrong project") && ui->edit_text_active);
    CHECK(nes_tas_project_revision(project) == revision && nes_tas_navigation_count(project) == 10);
    CHECK(send_key(ui, SDL_SCANCODE_ESCAPE, KMOD_NONE));
    render_editor(ui);
    CHECK(editor->navigation_selection == SIZE_MAX);
    editor->cursor = 1;
    CHECK(click_action(ui, TAS_ACTION_NAVIGATION_CURSOR));
    CHECK(navigation_text_input(ui, "Unsaved bookmark") && !ui->edit_text_active);
cleanup:
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);
    (void)nes_file_remove(replacement_path);
    return failures;
}

#endif
