# States and replay

[Documentation index](README.md)

## Save states

Use File > Quick Save State and Quick Load State, or F5 and F6, for the selected
slot. Tools > Save States selects one of ten slots and opens or saves a named
state file. Ctrl+F5 and Ctrl+F6 use that file path. The panel includes file pickers.
Settings > Files and storage also exposes the slot and state path.

States contain CPU, PPU, APU, cartridge, controller, disk, and supported peripheral
state. They identify the loaded image and validate all sections before restoring
anything. A corrupt, truncated, incompatible, or unsupported state leaves the
running machine intact. Battery saves and disk overlays remain separate files.
State writes use an atomic replacement; failed writes retain the existing file.

## Rewind and run-ahead

Tools > Rewind, Run-Ahead and Movies sets the retained history and run-ahead depth. History can retain
up to 60 seconds, subject to the memory limit. Ctrl+Backspace moves back through
available frames while held, without depending on keyboard repeat. Releasing
the binding resumes forward playback if the game was running. A menu or panel
rewind step pauses on the restored frame; use Resume to continue. Settings > Emulation >
Rewind speed chooses 1 through 30 frames per activation, saved as
`rewind_step_frames`. Rewind stops at the oldest available frame. Resuming from an earlier
frame discards the abandoned future. Rewind is unavailable until frames have
been retained. Reloading or replacing the image clears history.

Run-ahead supports zero through four speculative frames. Ctrl+Shift+A cycles the
setting. The displayed frame can come from speculative execution, while the
machine resumes from its authoritative state. Speculative frames cannot flush
battery data, disk overlays, tape output, movie events, or capture files. The
completed video trace stays paired with the displayed image for overscan, layer
controls, and HD rendering.

## Input movies

Tools > Rewind, Run-Ahead and Movies provides a movie path, file pickers, current-state or power-on
starting mode, Record, Play, and Stop. A movie records emulator input events and
the initial machine state; it is separate from video recording. Controller and
peripheral events use the same production input path during playback.

Playback validates the format, image, and enabled cheat configuration. It owns
emulated input while active. Reset, state loading, image replacement, rewind,
run-ahead changes, and other conflicting session operations are rejected. Stop
returns to the session that existed before playback. Movie recording and
playback isolate persistent storage from the live game's saves. A recording
write failure retains the buffered recording for another save attempt.

Movies use Cupid's versioned format. Other emulators' movie formats are not
accepted. [Netplay](netplay.md) uses the same deterministic session ownership
rules. Neither a movie nor a save state is a substitute for backing up ordinary
battery-save files.
