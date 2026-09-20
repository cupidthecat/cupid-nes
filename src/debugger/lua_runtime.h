/*
 * lua_runtime.h - Lua debugger scripting interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_LUA_RUNTIME_H
#define CUPID_LUA_RUNTIME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { DEBUG_LUA_OVERLAY_WIDTH = 256, DEBUG_LUA_OVERLAY_HEIGHT = 240 };

bool debugger_lua_load(const char *name, const char *source);
void debugger_lua_unload(void);
bool debugger_lua_loaded(void);
bool debugger_lua_faulted(void);
const char *debugger_lua_error(void);
const char *debugger_lua_log(void);
void debugger_lua_set_instruction_budget(uint32_t instructions);
uint32_t debugger_lua_instruction_budget(void);

const uint32_t *debugger_lua_overlay(void);
void debugger_lua_clear_overlay(void);

/* Called by the debugger core around real CPU accesses/instructions. */
void debugger_lua_on_execute(uint16_t address);
void debugger_lua_on_read(uint16_t address, uint8_t *value);
void debugger_lua_on_write(uint16_t address, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
