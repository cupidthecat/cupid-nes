/*
 * debugger_accuracy.c - Interactive debugger and Lua regression tests
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../debugger/debugger.h"
#include "../debugger/lua_runtime.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned debugger_checks;

#define CHECK(condition) do { \
    ++debugger_checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static uint8_t image[16 + 0x8000 + 0x2000];

static void build_debug_nrom(void) {
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1A", 4);
    image[4] = 2;
    image[5] = 1;
    uint8_t *prg = image + 16;
    memset(prg, 0xEA, 0x8000);
    const uint8_t program[] = {
        0xA9, 0x11,       /* $8000 LDA #$11 */
        0x85, 0x10,       /* $8002 STA $10 */
        0x20, 0x10, 0x80, /* $8004 JSR $8010 */
        0xA5, 0x10,       /* $8007 LDA $10 */
        0x4C, 0x09, 0x80  /* $8009 JMP $8009 */
    };
    memcpy(prg, program, sizeof(program));
    prg[0x10] = 0xE6; prg[0x11] = 0x10; /* INC $10 */
    prg[0x12] = 0x60;                     /* RTS */
    prg[0x7FFA] = 0x10; prg[0x7FFB] = 0x80;
    prg[0x7FFC] = 0x00; prg[0x7FFD] = 0x80;
    prg[0x7FFE] = 0x00; prg[0x7FFF] = 0x80;
}

static int start_debug_machine(void) {
    if (!unload_rom()) return -1;
    if (!nes_set_region_mode(NES_REGION_MODE_NTSC)) return -1;
    build_debug_nrom();
    if (load_rom_memory(image, sizeof(image)) != 0) return -1;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_total_cycles = 0;
    if (!cpu_power_on(&cpu)) return -1;
    debugger_init();
    return 0;
}

static int breakpoints_and_steps(void) {
    CHECK(start_debug_machine() == 0);
    uint32_t execute = debugger_add_breakpoint(DEBUG_BREAK_EXECUTE, 0x8004, 0x8004);
    CHECK(execute != 0);
    CHECK(debugger_run_until_break(&cpu, 8));
    DebugStopInfo stop = debugger_last_stop();
    CHECK(debugger_is_paused() && cpu.pc == 0x8004);
    CHECK(stop.reason == DEBUG_STOP_BREAKPOINT && stop.address == 0x8004);

    CHECK(debugger_step_over());
    for (unsigned i = 0; i < 8 && !debugger_is_paused(); ++i) (void)cpu_step(&cpu);
    CHECK(debugger_is_paused() && cpu.pc == 0x8007);
    CHECK(cpu_peek_internal_ram(0x10) == 0x12);
    CHECK(debugger_last_stop().reason == DEBUG_STOP_STEP);

    CHECK(debugger_step_into());
    CHECK(cpu_step(&cpu) > 0);
    CHECK(cpu_step(&cpu) == 0);
    CHECK(debugger_is_paused() && cpu.pc == 0x8009 && cpu.a == 0x12);

    CHECK(start_debug_machine() == 0);
    CHECK(debugger_add_breakpoint(DEBUG_BREAK_EXECUTE, 0x8010, 0x8010) != 0);
    CHECK(debugger_run_until_break(&cpu, 8));
    CHECK(cpu.pc == 0x8010 && debugger_is_paused());
    CHECK(debugger_step_out());
    for (unsigned i = 0; i < 8 && !debugger_is_paused(); ++i) (void)cpu_step(&cpu);
    CHECK(debugger_is_paused() && cpu.pc == 0x8007);

    CHECK(start_debug_machine() == 0);
    CHECK(debugger_add_breakpoint(DEBUG_BREAK_WRITE, 0x0010, 0x0010) != 0);
    CHECK(cpu_step(&cpu) > 0);
    CHECK(cpu_step(&cpu) > 0);
    stop = debugger_last_stop();
    CHECK(debugger_is_paused() && stop.reason == DEBUG_STOP_BREAKPOINT);
    CHECK(stop.access == DEBUG_BREAK_WRITE && stop.address == 0x0010 && stop.value == 0x11);
    CHECK(cpu_peek_internal_ram(0x10) == 0x11);

    CHECK(start_debug_machine() == 0);
    write_mem(0x0010, 0x5A);
    cpu.pc = 0x8007;
    CHECK(debugger_add_breakpoint(DEBUG_BREAK_READ, 0x0010, 0x0010) != 0);
    CHECK(cpu_step(&cpu) > 0);
    stop = debugger_last_stop();
    CHECK(debugger_is_paused() && stop.access == DEBUG_BREAK_READ && stop.value == 0x5A);
    CHECK(cpu.a == 0x5A);

    CHECK(start_debug_machine() == 0);
    CHECK(debugger_add_breakpoint(DEBUG_BREAK_READ, 0x0200, 0x0200) != 0);
    for (unsigned i = 0; i < 256; ++i) write_mem((uint16_t)(0x0200u + i), (uint8_t)i);
    write_mem(0x4014, 0x02);
    CHECK(cpu_step(&cpu) > 0);
    stop = debugger_last_stop();
    CHECK(debugger_is_paused() && stop.reason == DEBUG_STOP_BREAKPOINT);
    CHECK(stop.access == DEBUG_BREAK_READ && stop.address == 0x0200);

    CHECK(start_debug_machine() == 0);
    CHECK(debugger_add_breakpoint(DEBUG_BREAK_EXECUTE, 0x8010, 0x8010) != 0);
    cpu_request_nmi();
    CHECK(cpu_step(&cpu) > 0 && cpu.pc == 0x8010);
    CHECK(cpu_step(&cpu) == 0);
    CHECK(debugger_is_paused() && debugger_last_stop().address == 0x8010);
    return 0;
}

static uint8_t debug_uxrom[16 + 0x10000 + 0x2000];

static int bank_switch_breakpoint(void) {
    CHECK(unload_rom());
    memset(debug_uxrom, 0, sizeof(debug_uxrom));
    memcpy(debug_uxrom, "NES\x1A", 4);
    debug_uxrom[4] = 4; debug_uxrom[5] = 1; debug_uxrom[6] = 0x20;
    uint8_t *prg = debug_uxrom + 16;
    memset(prg + 0x0000, 0x11, 0x4000);
    memset(prg + 0x4000, 0x22, 0x4000);
    memset(prg + 0x8000, 0x33, 0x4000);
    memset(prg + 0xC000, 0x44, 0x4000);
    prg[0xC000] = 0xA9; prg[0xC001] = 0x01;             /* LDA #1 */
    prg[0xC002] = 0x8D; prg[0xC003] = 0x00; prg[0xC004] = 0x80; /* STA $8000 */
    prg[0xC005] = 0xEA;
    prg[0xFFFA] = 0x00; prg[0xFFFB] = 0xC0;
    prg[0xFFFC] = 0x00; prg[0xFFFD] = 0xC0;
    prg[0xFFFE] = 0x00; prg[0xFFFF] = 0xC0;
    CHECK(load_rom_memory(debug_uxrom, sizeof(debug_uxrom)) == 0);
    ppu_power_on(&ppu); apu_power_on(&apu); cpu_total_cycles = 0;
    CHECK(cpu_power_on(&cpu)); debugger_init();
    CHECK(cart_cpu_peek_bus(0x8000, 0xFF) == 0x11);
    CHECK(debugger_add_breakpoint(DEBUG_BREAK_WRITE, 0x8000, 0x8000) != 0);
    CHECK(cpu_step(&cpu) > 0);
    CHECK(cpu_step(&cpu) > 0);
    DebugStopInfo stop = debugger_last_stop();
    CHECK(debugger_is_paused() && stop.access == DEBUG_BREAK_WRITE && stop.address == 0x8000);
    CHECK(cart_cpu_peek_bus(0x8000, 0xFF) == 0x22);
    return 0;
}

static int inspection_trace_and_watches(void) {
    CHECK(start_debug_machine() == 0);
    debugger_trace_enable(true);
    CHECK(cpu_step(&cpu) > 0);
    CHECK(debugger_trace_count() == 1);
    DebugTraceEntry trace;
    CHECK(debugger_trace_at(0, &trace));
    CHECK(trace.pc == 0x8000 && trace.opcode == 0xA9);

    DebugDisassembly disassembly;
    CHECK(debugger_disassemble(0x8004, &disassembly));
    CHECK(disassembly.length == 3 && disassembly.bytes[0] == JSR_OPCODE);
    CHECK(strstr(disassembly.text, "JSR") != NULL);

    write_mem(0x0020, 0x34); write_mem(0x0021, 0x12);
    uint32_t watch = debugger_add_watch(0x0020, 2, "word");
    CHECK(watch != 0 && debugger_watch_count() == 1);
    DebugWatch info; uint32_t value = 0;
    CHECK(debugger_watch_at(0, &info, &value));
    CHECK(info.id == watch && value == 0x1234);

    ppu.status = 0xE0; ppu.w = 1; ppu.open_bus = 0x1F;
    uint8_t status = ppu.status, write_toggle = ppu.w, open_bus = ppu.open_bus;
    (void)debugger_peek_cpu(0x2002);
    CHECK(ppu.status == status && ppu.w == write_toggle && ppu.open_bus == open_bus);

    APU *active = apu_active_state();
    active->frame_irq = true; active->frame_irq_source = true; active->frame_irq_clear_delay = 0;
    (void)debugger_peek_cpu(0x4015);
    CHECK(active->frame_irq && active->frame_irq_source && active->frame_irq_clear_delay == 0);

    CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
    CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    pad1.strobe = 0; pad1.shift = 0x5A;
    uint8_t shift = pad1.shift;
    (void)debugger_peek_cpu(0x4016);
    CHECK(pad1.shift == shift);

    uint8_t nametable[0x400], pattern[0x1000], palette[32], oam[256];
    ppu_vram[0] = 0x6C; ppu_palette[1] = 0x21; ppu.oam[3] = 0x77;
    debugger_copy_nametable(0, nametable);
    debugger_copy_pattern_table(0, pattern);
    debugger_copy_palette(palette);
    debugger_copy_oam(oam);
    CHECK(nametable[0] == 0x6C && palette[1] == 0x21 && oam[3] == 0x77);
    (void)pattern;
    return 0;
}

static int lua_callbacks_errors_and_overlay(void) {
    CHECK(start_debug_machine() == 0);
    static const char script[] =
        "emu.addMemoryCallback('read',0x10,function(a,v) return 0x66 end)\n"
        "emu.addMemoryCallback('exec',0x8007,function(a) emu.log('exec') "
        "emu.drawPixel(1,2,0xFF00FF00) end)\n";
    CHECK(debugger_lua_load("callbacks", script));
    write_mem(0x0010, 0x22);
    cpu.pc = 0x8007;
    CHECK(cpu_step(&cpu) > 0);
    CHECK(cpu.a == 0x66);
    CHECK(strstr(debugger_lua_log(), "exec") != NULL);
    CHECK(debugger_lua_overlay()[2 * DEBUG_LUA_OVERLAY_WIDTH + 1] == 0xFF00FF00u);

    debugger_lua_unload();
    CHECK(!debugger_lua_loaded());
    CHECK(!debugger_lua_load("error", "error('boom')"));
    CHECK(debugger_lua_faulted() && strstr(debugger_lua_error(), "boom") != NULL);

    debugger_lua_set_instruction_budget(2000);
    CHECK(!debugger_lua_load("budget", "while true do end"));
    CHECK(debugger_lua_faulted());
    CHECK(strstr(debugger_lua_error(), "instruction budget") != NULL);

    CHECK(!debugger_lua_load("bounds",
        "emu.drawRectangle(0,0,1000000000,1000000000,0xFFFFFFFF,true)"));
    CHECK(debugger_lua_faulted());
    CHECK(debugger_lua_load("clipped",
        "emu.drawRectangle(-20,-20,40,40,0x11223344,true); "
        "emu.drawLine(-256,120,512,120,0xAABBCCDD)"));
    const uint32_t *overlay = debugger_lua_overlay();
    CHECK(overlay[0] == 0x11223344u);
    CHECK(overlay[120 * DEBUG_LUA_OVERLAY_WIDTH] == 0xAABBCCDDu);
    debugger_lua_unload();
    return 0;
}

static void make_fds_bios(uint8_t bios[0x2000]) {
    memset(bios, 0xEA, 0x2000);
    bios[0x1FFA] = 0x00; bios[0x1FFB] = 0xE0;
    bios[0x1FFC] = 0x00; bios[0x1FFD] = 0xE0;
    bios[0x1FFE] = 0x00; bios[0x1FFF] = 0xE0;
}

static int fds_peek_side_effects(void) {
    CHECK(unload_rom());
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    uint8_t bios[0x2000]; make_fds_bios(bios);
    const size_t disk_size = 16u + 65500u;
    uint8_t *disk = calloc(1, disk_size);
    CHECK(disk != NULL);
    memcpy(disk, "FDS\x1A", 4); disk[4] = 1; disk[16] = 1; disk[17] = 0x2A;
    CHECK(load_fds_memory(disk, disk_size, bios, sizeof(bios), NULL, false) == 0);
    free(disk);
    ppu_power_on(&ppu); apu_power_on(&apu); cpu_total_cycles = 0;
    CHECK(cpu_power_on(&cpu)); debugger_init();

    cart_cpu_write(0x4020, 1); cart_cpu_write(0x4021, 0); cart_cpu_write(0x4022, 3);
    fds_clock_cpu(2);
    CHECK(fds_irq_pending());
    uint8_t first = debugger_peek_cpu(0x4030);
    uint8_t second = debugger_peek_cpu(0x4030);
    CHECK((first & 1u) && first == second && fds_irq_pending());

    FdsAutomationOptions options = {true, false};
    fds_set_automation_options(options);
    CHECK(fds_disk_inserted());
    size_t side = fds_current_side();
    for (unsigned frame = 1; frame < 100; ++frame) {
        ppu.frame_count = frame;
        (void)debugger_peek_cpu(0x4032);
        fds_automation_frame(frame);
    }
    CHECK(fds_disk_inserted() && fds_current_side() == side);
    return 0;
}

int test_debugger_accuracy(void) {
    debugger_checks = 0;
    int failures = 0;
    failures += breakpoints_and_steps();
    failures += bank_switch_breakpoint();
    failures += inspection_trace_and_watches();
    failures += lua_callbacks_errors_and_overlay();
    failures += fds_peek_side_effects();
    debugger_shutdown();
    unload_rom();
    printf("Debugger/Lua: %u checks, %d failures\n", debugger_checks, failures);
    return failures;
}
