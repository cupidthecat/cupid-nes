# Saves and media

[Documentation index](README.md)

Cupid persists cartridge nonvolatile memory, writable FDS/QD images, Family BASIC tape recordings, and supported expansion storage. These files preserve the emulated storage device involved; the application does not expose save-state or rewind commands.

## Cartridge save files

The cartridge loader derives save paths from the image path. It removes the final filename extension and appends the suffix for each persistent device:

| Persistent device | Cartridge image | Save path |
| --- | --- | --- |
| PRG NVRAM | `games/game.nes` | `games/game.sav` |
| CHR NVRAM | `games/game.nes` | `games/game.chr.sav` |
| UNROM 512 flash | `games/game.nes` | `games/game.flash.sav` |
| 128-byte serial EEPROM | `games/game.nes` | `games/game.eeprom128` |
| 256-byte serial EEPROM | `games/game.nes` | `games/game.eeprom256` |

Only devices present in the loaded board configuration get persistence paths. Volatile PRG RAM and ordinary CHR RAM are not written to disk just because a game changes them. Bandai/Datach boards can expose serial EEPROM independently of ordinary battery-backed RAM.

Existing save data is read when the cartridge is loaded. Short PRG, CHR, or EEPROM files leave the unread portion of the allocated device zero-filled. The save size follows the board and NES 2.0 metadata, so `.sav` is not always an 8 KiB file.

A short UNROM 512 flash save behaves differently: it replaces only the bytes read from the file, leaving the remaining PRG flash bytes from the loaded image. An incomplete file should not be treated as a verified backup.

Persistent cartridge data is flushed when the cartridge is shut down, including normal application exit. The main loop has no timed autosave. Close the emulator normally after making progress you want to retain.

The derived filename can collide when two image files share the same stem in one directory. For example, `game.nes` and `game.bin` both derive `game.sav`. Renaming a cartridge image also changes the save path Cupid looks for.

## Board-specific cartridge formats

Some mapper save files contain more than one memory area:

- MMC5 writes PRG NVRAM followed by its 1 KiB ExRAM in the same `.sav` file.
- A battery-backed Namco 163 configuration writes PRG save RAM followed by the N163's 128-byte audio RAM in the same `.sav` file.
- UNROM 512 flash persistence stores the complete writable PRG flash image in `.flash.sav`.
- Bandai serial EEPROM chips use `.eeprom128` or `.eeprom256` files rather than the ordinary PRG `.sav` path.

Do not assume another emulator uses the same composite layout. Keep a backup before moving saves between emulator versions or board configurations.

Ordinary PRG/CHR battery memory and EEPROM files are written directly to their destination. Flash uses a temporary file and replacement. Failed cartridge saves print an error; they do not keep the application open for recovery as a failed FDS save does.

## Expansion storage

The ASCII Turbo File uses an 8 KiB `.turbofile.sav` file derived from the ROM stem. For example, `games/game.nes` uses `games/game.turbofile.sav`. The file must be exactly 8 KiB when it already exists.

Turbo File saves are written to a sibling temporary file and replace the destination only after the complete image is written and closed. If replacement fails during a window-close request, the modified device remains in memory and the emulator stays open so the save can be retried.

## Writable FDS and QD images

Disk-system writes replace the image path passed to Cupid. There is no separate disk `.sav` overlay.

For an image you want to preserve unchanged, launch a copy or start it write-protected:

```sh
./cupid-nes --fds-bios "disksys.rom" --fds-write-protect "game.fds"
```

F10 toggles write protection at runtime. F8 ejects or reinserts the selected side, and F9 selects the next side. Ejecting a side does not save the image.

When dirty media is flushed, Cupid rebuilds every side at the image's original 65,500-byte FDS or 65,536-byte QD size and preserves whether the image had a 16-byte FDS header. It writes a sibling temporary file named with the `.cupid-fds.tmp` suffix and replaces the original image only after the temporary write closes successfully.

If that replacement fails, the dirty disk remains in memory. A normal window-close request reports the FDS save error and leaves the emulator open so you can correct the destination problem and try closing again. The loader also refuses to replace or unload dirty FDS media when it cannot flush it.

If the image became dirty before write protection was enabled, F10 must turn write protection off again before a retry can succeed. Check directory permissions and free space as well. A stale `*.cupid-fds.tmp` file left by an interrupted process can block creation of the next temporary file because Cupid creates it exclusively; preserve or move the stale file aside after confirming no other running instance is using the image.

## Family BASIC tape files

Tape paths are explicit command-line arguments:

```sh
./cupid-nes --console famicom --expansion family-basic --tape-record "program.tap" "basic.nes"
```

F10 starts a fresh recording after BASIC is ready to save. Before starting another recording, Cupid attempts to save any pending capture; a failed save prevents the new recording from starting. A new capture does not append a second program to the previous file. F11 stops the tape and writes the capture to the configured path. Normal exit also attempts to save a pending recording.

The raw tape format stores one digital sample per 88 emulated CPU cycles. Samples are packed least-significant bit first. Only complete bytes are written, so an incomplete final byte is omitted. The file has no WAV header and is not a BASIC source-text file.

Recording saves use a sibling `.cupid-tape.tmp` file and then replace the requested destination. If an F11 save fails, the completed capture remains in memory and another F11 can retry after the path problem is fixed. A tape save failure during application shutdown is reported and the process exits with failure, so use F11 to confirm an important recording before closing the window.

If the recording buffer cannot grow, recording stops and shutdown reports `Tape recording stopped because the capture buffer could not grow`.

## Backups and compatibility

Copy persistent files after Cupid has closed successfully. Keep important saves with the cartridge or disk image and hardware configuration that produced them. Use separate filenames or directories when testing multiple copies of a game.

`load_rom_memory()` copies an image without assigning a persistence path. `load_fds_memory()` accepts an optional disk destination; modified memory-only disk fixtures need an explicit persistence strategy. Tests should use their own temporary media and cleanup helpers. See [development](development.md) for test setup and [hardware](hardware.md) for board-specific memory layouts.
