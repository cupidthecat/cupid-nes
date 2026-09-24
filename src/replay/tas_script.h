/*
 * tas_script.h - One-shot Lua automation for editable TAS projects
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_TAS_SCRIPT_H
#define CUPID_TAS_SCRIPT_H

#include <stdbool.h>
#include <stddef.h>

#include "tas_project.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Run one isolated, text-only Lua 5.4 script against project. The script is
 * wrapped in one TAS edit transaction: any Lua error, allocation failure, edit
 * failure, or instruction-limit failure restores the project to its exact
 * pre-script state. error may be NULL; otherwise capacity includes its NUL.
 *
 * Scripts receive a single global table, taseditor, with these functions:
 *   framecount()
 *   getinput(frame, controller)
 *   submitinputchange(frame, controller, input)
 *   submitinsertframes(frame, count)
 *   submitdeleteframes(frame, count)
 *   applyinputchanges()
 *   clearinputchanges()
 *   getselection()
 *   getmarker(frame)
 *   setmarker(frame, note)
 *   removemarker(frame)
 *   getcurrentbranch()
 *
 * Frame numbers are zero-based. Controller 0 addresses the movie command byte;
 * controllers 1 through 4 address NES pad bytes 1 through 4. Input values are
 * bytes (0..255). getselection() returns ascending zero-based frame numbers.
 * getmarker() returns the exact marker note at frame, or nil when none exists.
 * submit* calls are staged until applyinputchanges(); clearinputchanges()
 * discards the staged batch. applyinputchanges() returns the earliest changed
 * frame, or -1 when applying the batch changes no input.
 *
 * Only the base, math, string, and table libraries are present. Host file,
 * package, OS, debugger, and emulated-machine APIs are unavailable. Execution
 * uses fixed memory and instruction limits so a script cannot run unbounded.
 */
bool nes_tas_run_script(NesTasProject *project, const char *source, size_t length, char *error, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
