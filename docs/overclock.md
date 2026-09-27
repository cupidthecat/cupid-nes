# CPU overclock

Overclocking gives a game more CPU time per displayed frame by adding blank
scanlines. It is disabled by default and intentionally changes console timing.
Leave it disabled when checking hardware accuracy or diagnosing a game problem.

The CPU Overclock panel has two counts, each from 0 through 1000:

- **Extra post-render scanlines** add time before the vblank flag and NMI.
- **Extra vblank scanlines** add time after NMI, before pre-render begins.

Both counts default to zero. Enabling overclock with both counts at zero keeps
normal hardware timing. Disabling it retains the chosen counts without using
them. **Restore normal timing defaults** disables overclock, clears both counts,
and enables the DMC compatibility option.

Changes take effect at the next frame boundary or hardware reset. A frame already
in progress finishes with its current settings. The panel shows the effective
extra counts for the current frame. Configuration changes are blocked while a
movie, network session, rewind operation, or speculative frame owns execution.
NSF playback and VS systems retain normal timing.

## Sound and sample compatibility

The DMC compatibility option is enabled by default. At a frame boundary, ongoing
DMC playback or a direct `$4011` sample write during the preceding frame disables
overclock for the next frame. The requested counts stay intact. Overclock resumes
when those conditions clear. This option cannot undo extra cycles that have
already run; a sample started during an extended frame can still be affected.

During extra scanlines, the base APU pauses its channel timers, frame sequencer,
and audio sample generation. CPU bus parity and DMC start/stop handshakes continue.
An already requested DMC transfer can finish, including its IRQ. The DMC divider
retains the CPU get/put phase when synthesis resumes. Turning off compatibility
allows these pauses during sample playback, which can change how samples sound.

Cartridge IRQ counters, expansion sound hardware, and other CPU-clocked devices
continue to receive CPU or master clocks. Expansion sound can therefore change
under overclock. OAM DMA and CPU accesses to PPU memory remain available during
both extra intervals. Automatic rendering fetches do not run in either interval.

## Timing and saved games

NTSC, PAL, and Dendy keep their normal CPU/PPU master-clock ratios and host frame
rates. Each extra line contributes 341 PPU clocks. For example, 90 lines before
NMI and 90 after it add 20,460 CPU cycles to an NTSC frame. The usual NTSC odd-frame
rendering skip still applies. Audio receives roughly the normal number of output
samples per host frame.

Save states and rewind retain requested settings, current-frame settings, the
remaining extra interval, and direct-sample-write history. Restoring a state can
therefore restore a different overclock setting. Loading an older state without
these fields selects normal timing. Normal default states keep the original
TIME chunk bytes; extended states require a build that understands this feature.

The PPU keeps its original hardware scanline coordinates during an extra interval.
Debugger views hold the next hardware line at dot zero while the total PPU clock
continues to advance. This keeps existing PPU state coordinates compatible; the
remaining extra clocks are stored in the timing component.

## Regression coverage

`src/tests/overclock_accuracy.c` exercises production CPU, PPU, APU, cartridge,
state, rewind, and panel code with generated cartridge images. It checks default
state bytes, regional NMI/frame lengths, audio output counts, OAM and DMC DMA in
both intervals, mapper and expansion clocks, active setting changes, state replay,
rewind, old timing chunks, rejected input, and session restrictions.
