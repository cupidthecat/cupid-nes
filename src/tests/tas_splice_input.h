/*
 * tas_splice_input.h - Native splicer controls and stale-edit protection
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_TESTS_TAS_SPLICE_INPUT_H
#define CUPID_TESTS_TAS_SPLICE_INPUT_H

static int splice_gestures(FrontendDesktopUi *ui) {
    int failures = 0;
    NesTasSession *session = desktop_tas_session(ui);
    NesTasProject *project = nes_tas_session_project(session);
    DesktopTasEditor *editor = ui->tas_editor;
    CHECK(project && nes_tas_project_frame_count(project) >= 12);
    size_t length = nes_tas_project_frame_count(project);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    execution_control_set_paused(&ui->execution->execution, true);
    editor->cursor = 3;
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_OPEN));
    render_editor(ui);
    CHECK(editor->sidebar_tab == 6 && editor->splice.preview_valid);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_SOURCE_FIRST));
    CHECK(navigation_text_input(ui, "2") && !ui->edit_text_active);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_SOURCE_COUNT));
    CHECK(navigation_text_input(ui, "3") && !ui->edit_text_active);
    render_editor(ui);
    CHECK(editor->splice.preview.resulting_frames == length + 3);
    NesFm2Frame expected = *nes_tas_project_frame(project, 2);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_APPLY));
    CHECK(nes_tas_project_frame_count(project) == length + 3);
    CHECK(!memcmp(nes_tas_project_frame(project, length), &expected, sizeof(expected)));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && nes_tas_project_frame_count(project) == length);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_REPLACE));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_DESTINATION_FIRST));
    CHECK(navigation_text_input(ui, "4") && !ui->edit_text_active);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_DESTINATION_COUNT));
    CHECK(navigation_text_input(ui, "2") && !ui->edit_text_active);
    render_editor(ui);
    CHECK(editor->splice.preview.resulting_frames == length + 1);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_APPLY));
    CHECK(nes_tas_project_frame_count(project) == length + 1);
    CHECK(!memcmp(nes_tas_project_frame(project, 4), &expected, sizeof(expected)));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && nes_tas_project_frame_count(project) == length);
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_SOURCE_COUNT));
    CHECK(navigation_text_input(ui, "0") && ui->edit_text_active);
    CHECK(editor->splice.options.source_count == 3);
    CHECK(send_key(ui, SDL_SCANCODE_ESCAPE, KMOD_NONE));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_SOURCE_FIRST));
    CHECK(nes_tas_marker_set(project, 1, "Timeline changed during range entry") == NES_TAS_OK);
    CHECK(navigation_text_input(ui, "5") && ui->edit_text_active);
    CHECK(editor->splice.options.source_first == 2);
    CHECK(send_key(ui, SDL_SCANCODE_ESCAPE, KMOD_NONE));
    render_editor(ui);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_EXTRACT));
    render_editor(ui);
    CHECK(editor->splice.preview.resulting_frames == 3 && editor->splice.preview.removed == length);
    CHECK(click_action(ui, TAS_ACTION_SPLICE_APPLY));
    CHECK(nes_tas_project_frame_count(project) == 3);
    CHECK(!memcmp(nes_tas_project_frame(project, 0), &expected, sizeof(expected)));
    CHECK(nes_tas_project_undo(project) == NES_TAS_OK && nes_tas_project_frame_count(project) == length);
    render_editor(ui);
    CHECK(nes_tas_session_set_read_only(session, true) == NES_MOVIE_OK);
    render_editor(ui);
    SDL_FRect bounds;
    CHECK(!desktop_clay_bounds(ui->clay, HIT_TAS_ACTION, TAS_ACTION_SPLICE_APPLY, 0, &bounds));
    CHECK(nes_tas_project_frame_count(project) == length);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    const float scales[] = {1, 1.5f, 2};
    for (size_t i = 0; i < sizeof(scales) / sizeof(scales[0]); ++i) {
        CHECK(resize_editor(ui, (int)(900 * scales[i]), (int)(640 * scales[i]), scales[i]));
        render_editor(ui);
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_SPLICE_LOAD, 0));
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_SPLICE_SOURCE_FIRST, 0));
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_SPLICE_EXTRACT, 0));
        CHECK(hit_inside_window(ui, HIT_TAS_ACTION, TAS_ACTION_SPLICE_APPLY, 0));
    }
    CHECK(resize_editor(ui, 900, 760, 1));
cleanup:
    return failures;
}

#endif
