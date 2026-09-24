/*
 * tas_script_accuracy.c - One-shot TAS Lua editing regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../replay/tas_script.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

static NesTasProject *make_project(size_t frame_count) {
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    movie.version = 3;
    movie.ports[0] = NES_FM2_PORT_GAMEPAD;
    movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    if (frame_count) {
        movie.frames = (NesFm2Frame *)calloc(frame_count, sizeof(*movie.frames));
        if (!movie.frames) {
            return NULL;
        }
    }
    movie.frame_count = frame_count;
    for (size_t i = 0; i < frame_count; ++i) {
        movie.frames[i].pads[0] = (uint8_t)(0x10u + i);
        movie.frames[i].pads[1] = (uint8_t)(0x20u + i);
    }
    NesTasProject *project = nes_tas_project_create(&movie);
    nes_fm2_movie_free(&movie);
    return project;
}

static bool marker_note(const NesTasProject *project, size_t frame, const char *note) {
    for (size_t i = 0; i < nes_tas_marker_count(project); ++i) {
        NesTasMarkerView marker;
        if (!nes_tas_marker(project, i, &marker)) {
            return false;
        }
        if (marker.frame == frame) {
            return strcmp(marker.note ? marker.note : "", note ? note : "") == 0;
        }
        if (marker.frame > frame) {
            break;
        }
    }
    return false;
}

static int successful_grouped_edit(void) {
    NesTasProject *project = make_project(4);
    CHECK(project != NULL);
    CHECK(nes_tas_selection_set(project, 1, true) == NES_TAS_OK);
    CHECK(nes_tas_selection_set(project, 3, true) == NES_TAS_OK);

    static const char script[] = "local s=taseditor.getselection()\n"
                                 "assert(#s==2 and s[1]==1 and s[2]==3)\n"
                                 "assert(taseditor.getmarker(1)==nil)\n"
                                 "taseditor.setmarker(1,'alpha')\n"
                                 "assert(taseditor.getmarker(1)=='alpha')\n"
                                 "taseditor.submitinputchange(0,1,0xAA)\n"
                                 "assert(taseditor.applyinputchanges()==0)\n"
                                 "taseditor.submitinputchange(2,0,5)\n"
                                 "local n=taseditor.framecount()\n"
                                 "taseditor.submitinsertframes(n,2)\n"
                                 "assert(taseditor.applyinputchanges()==2)\n"
                                 "assert(taseditor.framecount()==n+2)\n"
                                 "taseditor.submitinputchange(5,2,0x55)\n"
                                 "taseditor.clearinputchanges()\n"
                                 "assert(taseditor.applyinputchanges()==-1)\n"
                                 "assert(os==nil and io==nil and package==nil and debug==nil)\n"
                                 "assert(load==nil and pcall==nil and xpcall==nil and dofile==nil and loadfile==nil)\n"
                                 "assert(type(math)=='table' and type(string)=='table' and type(table)=='table')\n";
    char error[256];
    CHECK(nes_tas_run_script(project, script, sizeof(script) - 1, error, sizeof(error)));
    CHECK(error[0] == '\0');
    CHECK(nes_tas_project_frame_count(project) == 6);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0xAA);
    CHECK(nes_tas_project_frame(project, 2)->commands == 5);
    CHECK(marker_note(project, 1, "alpha"));
    CHECK(nes_tas_selection_count(project) == 2);

    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 4);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    CHECK(nes_tas_project_frame(project, 2)->commands == 0);
    CHECK(nes_tas_marker_count(project) == 0);
    CHECK(nes_tas_selection_contains(project, 1));
    CHECK(nes_tas_selection_contains(project, 3));

    CHECK(nes_tas_project_redo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 6);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0xAA);
    CHECK(marker_note(project, 1, "alpha"));
    nes_tas_project_destroy(project);
    return 0;
}

static int insert_boundaries(void) {
    NesTasProject *project = make_project(3);
    CHECK(project != NULL);
    static const char at_eof[] = "local n=taseditor.framecount()\n"
                                 "taseditor.submitinsertframes(n,3)\n"
                                 "assert(taseditor.applyinputchanges()==n)\n"
                                 "assert(taseditor.framecount()==n+3)\n";
    char error[256];
    CHECK(nes_tas_run_script(project, at_eof, sizeof(at_eof) - 1, error, sizeof(error)));
    CHECK(nes_tas_project_frame_count(project) == 6);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    CHECK(nes_tas_project_frame(project, 2)->pads[0] == 0x12);
    CHECK(nes_tas_project_frame(project, 3)->pads[0] == 0);
    CHECK(nes_tas_project_frame(project, 5)->pads[0] == 0);

    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_project_frame_count(project) == 3);
    static const char past_eof[] = "taseditor.submitinsertframes(5,2)\n"
                                   "assert(taseditor.applyinputchanges()==3)\n"
                                   "assert(taseditor.framecount()==7)\n";
    CHECK(nes_tas_run_script(project, past_eof, sizeof(past_eof) - 1, error, sizeof(error)));
    CHECK(nes_tas_project_frame_count(project) == 7);
    CHECK(nes_tas_project_frame(project, 2)->pads[0] == 0x12);
    CHECK(nes_tas_project_frame(project, 3)->pads[0] == 0);
    CHECK(nes_tas_project_frame(project, 6)->pads[0] == 0);
    nes_tas_project_destroy(project);
    return 0;
}

static int runtime_error_rolls_back(void) {
    NesTasProject *project = make_project(3);
    CHECK(project != NULL);
    CHECK(nes_tas_marker_set(project, 1, "kept") == NES_TAS_OK);
    static const char script[] = "taseditor.submitinputchange(0,1,0xEE)\n"
                                 "taseditor.applyinputchanges()\n"
                                 "taseditor.setmarker(1,'changed')\n"
                                 "error('intentional TAS Lua failure')\n";
    char error[256];
    CHECK(!nes_tas_run_script(project, script, sizeof(script) - 1, error, sizeof(error)));
    CHECK(strstr(error, "intentional TAS Lua failure") != NULL);
    CHECK(nes_tas_project_frame_count(project) == 3);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    CHECK(marker_note(project, 1, "kept"));

    CHECK(nes_tas_project_undo(project) == NES_TAS_OK);
    CHECK(nes_tas_marker_count(project) == 0);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    nes_tas_project_destroy(project);
    return 0;
}

static int instruction_limit_rolls_back(void) {
    NesTasProject *project = make_project(2);
    CHECK(project != NULL);
    static const char script[] = "taseditor.submitinputchange(0,1,0xE1)\n"
                                 "taseditor.applyinputchanges()\n"
                                 "local n=0 while true do n=n+1 end\n";
    char error[256];
    CHECK(!nes_tas_run_script(project, script, sizeof(script) - 1, error, sizeof(error)));
    CHECK(strstr(error, "instruction limit exceeded") != NULL);
    CHECK(nes_tas_project_frame_count(project) == 2);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    CHECK(nes_tas_marker_count(project) == 0);
    nes_tas_project_destroy(project);
    return 0;
}

static int memory_limit_rolls_back(void) {
    NesTasProject *project = make_project(2);
    CHECK(project != NULL);
    static const char script[] = "taseditor.submitinputchange(0,1,0xE2)\n"
                                 "taseditor.applyinputchanges()\n"
                                 "taseditor.setmarker(1,'temporary')\n"
                                 "local huge=string.rep('x',20000000)\n";
    char error[256];
    CHECK(!nes_tas_run_script(project, script, sizeof(script) - 1, error, sizeof(error)));
    CHECK(strstr(error, "memory limit exceeded") != NULL);
    CHECK(nes_tas_project_frame_count(project) == 2);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    CHECK(nes_tas_marker_count(project) == 0);
    nes_tas_project_destroy(project);
    return 0;
}

static int frame_growth_limit_rolls_back(void) {
    NesTasProject *project = make_project(2);
    CHECK(project != NULL);
    static const char script[] = "taseditor.submitinputchange(0,1,0xE3)\n"
                                 "taseditor.applyinputchanges()\n"
                                 "taseditor.submitinsertframes(0,262145)\n"
                                 "taseditor.applyinputchanges()\n";
    char error[256];
    CHECK(!nes_tas_run_script(project, script, sizeof(script) - 1, error, sizeof(error)));
    CHECK(strstr(error, "frame growth limit exceeded") != NULL);
    CHECK(nes_tas_project_frame_count(project) == 2);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == 0x10);
    nes_tas_project_destroy(project);
    return 0;
}

static int invalid_context_is_rejected(void) {
    char error[128];
    CHECK(!nes_tas_run_script(NULL, "", 0, error, sizeof(error)));
    CHECK(strstr(error, "requires a project") != NULL);

    NesTasProject *project = make_project(1);
    CHECK(project != NULL);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    CHECK(!nes_tas_run_script(project, "return", 6, error, sizeof(error)));
    CHECK(strstr(error, "another project edit") != NULL);
    CHECK(nes_tas_edit_active(project));
    nes_tas_edit_cancel(project);
    nes_tas_project_destroy(project);
    return 0;
}

int test_tas_script_accuracy(void) {
    int failures = 0;
    failures += successful_grouped_edit();
    failures += insert_boundaries();
    failures += runtime_error_rolls_back();
    failures += instruction_limit_rolls_back();
    failures += memory_limit_rolls_back();
    failures += frame_growth_limit_rolls_back();
    failures += invalid_context_is_rejected();
    if (!failures) {
        printf("TAS script accuracy tests passed\n");
    }
    return failures;
}
