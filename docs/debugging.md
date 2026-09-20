# Debugger and Lua

Cupid exposes one debugger service to the desktop tools and Lua scripts. The same service handles instruction control, breakpoints, memory inspection, PPU viewers, disassembly, watches, and tracing. Inspection reads use dedicated peek paths. They do not acknowledge the APU frame IRQ, clear PPU status, advance controller shift registers, change mapper read latches, or trigger FDS status-read automation.

Open Tools > Debugger to inspect the running game in a separate resizable
window. Drag its title bar to place it beside the game or on another display.
Memory and PPU Viewers and Lua Scripting also have their own windows. Opening
or focusing these tools leaves gameplay and audio running; Pause, breakpoints,
and stepping commands control when the machine stops.

## Execution control and breakpoints

The debugger supports execute, CPU-read, and CPU-write breakpoints over an address or inclusive address range. Execute breakpoints stop before the matching instruction. Read and write breakpoints record the access and pause at the instruction boundary after the access completes, so the hardware bus transaction itself remains complete.

Step Into executes one instruction and stops before the next instruction. Step Over treats a `JSR` as one operation and stops at its return address with the original stack depth. Other instructions behave like Step Into. Step Out runs until the current subroutine returns to the caller's stack depth. Debugger pause state is connected to the normal frontend execution control, so a breakpoint also pauses the shared frame runner and audio path.

Frame Advance can continue from a debugger stop to the end of the current frame while leaving emulation paused. A breakpoint on either machine of a dual VS cabinet stops the shared runner. Pausing audio retains its queued samples and reconstruction state.

The disassembler covers all 256 opcode values, including undocumented instructions. Operands show their addressing mode, and relative branches show the destination address. Inspection reads only the bytes belonging to each instruction.

CPU memory inspection covers `$0000-$FFFF`. PPU memory inspection covers `$0000-$3FFF`. The hardware viewers copy current nametable, pattern-table, palette, and primary OAM data without performing normal CPU or PPU register reads. Mapper-backed pattern and nametable reads use the currently selected banks.

## Lua scripts

Lua scripts run on the embedded Lua 5.4 runtime. File, operating-system, package-loading, and network libraries are not opened. Each script load and callback invocation receives a configurable instruction budget. Exceeding it faults the script instead of allowing an unbounded callback to stall emulation.

The `emu` table provides these debugger operations:

- `emu.read(address)` performs a side-effect-free CPU-space read.
- `emu.readPpu(address)` performs a side-effect-free PPU-space read.
- `emu.write(address, value)` performs a normal CPU bus write, including device and mapper side effects.
- `emu.getRegister(name)` and `emu.setRegister(name, value)` access `A`, `X`, `Y`, `PC`, `SP`, and `P`/`status`.
- `emu.getPpuState()` returns the current control, mask, status, VRAM-address, scroll, scanline, dot, frame, and PPU-cycle fields.
- `emu.getCpuCycleCount()` returns the current CPU cycle count.
- `emu.addMemoryCallback(type, first[, last], function)` registers `read`, `write`, or `exec` callbacks and returns an ID. `emu.removeMemoryCallback(id)` removes it.
- `emu.pause()` and `emu.resume()` use the same execution state as the desktop debugger.
- `emu.log(text)` appends to the script log.
- `emu.drawPixel`, `emu.drawLine`, `emu.drawRectangle`, and `emu.clearOverlay` draw to a 256 by 240 ARGB diagnostic overlay. Drawing is clipped and bounded to that surface.

An execute callback runs immediately before the CPU executes the instruction at its address. A read callback runs after the real hardware read and receives `(address, value)`. Returning an integer from 0 through 255 replaces the value delivered to the CPU without repeating the device read. A write callback runs after the real hardware write and receives `(address, value)`; its return value is ignored. Callbacks triggered by writes made from inside another Lua callback are suppressed until the outer callback returns, which prevents recursive script loops around `emu.write`.

Script runtime errors mark the current script faulted. Further callbacks from that script stop until it is reloaded. Unloading a script drops all registered callback references, clears its log state, and clears the diagnostic overlay.

## Source interfaces

The public debugger API is in [`src/debugger/debugger.h`](../src/debugger/debugger.h), and Lua script control is in [`src/debugger/lua_runtime.h`](../src/debugger/lua_runtime.h). The embedded Lua source and license are under [`src/third_party/lua`](../src/third_party/lua/).
