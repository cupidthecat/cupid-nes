# Saves and media

[Documentation index](README.md)

Cartridge saves, writable disk images, and Family BASIC tape recordings have different storage paths. They preserve emulated persistent media, not the complete running machine. The application has no save-state or rewind command.

## Cartridge save files

The loader configures persistence from the cartridge's metadata and image path. Save files sit beside the image and replace its final extension with the relevant suffix:

| Memory | Image | Save path |
| --- | --- | --- |
| PRG NVRAM | `games/game.nes` | `games/game.sav` |
| CHR NVRAM | `games/game.nes` | `games/game.chr.sav` |
| UNROM 512 flash | `games/game.nes` | `games/game.flash.sav` |
| 128-byte serial EEPROM | `games/game.nes` | `games/game.eeprom128` |
| 256-byte serial EEPROM | `games/game.nes` | `games/game.eeprom256` |

Only the memory devices present in the supported board configuration produce these files. Datach variants can have more than one EEPROM. Volatile work RAM and ordinary CHR RAM do not become save files merely because a game writes to them.

Existing saves load when the image is opened. Dirty persistent memory is flushed during cartridge shutdown or replacement, including normal application exit. There is no timed autosave in the main loop. Close the window normally and retain the log when a save operation reports an error.

The suffix is shared by images with the same basename in one directory. For example, `game.nes` and another supported image named `game.bin` would both derive `game.sav`. Keep distinct games and test copies under distinct names or folders. Renaming a cartridge image also changes where Cupid looks for its saves.

## Save size and board details

NES 2.0 describes volatile and nonvolatile RAM separately. Legacy headers use the board's defaults. A save's size follows the allocated device, so not every `.sav` file is 8 KiB.

An MMC5 save appends 1 KiB of ExRAM after its PRG NVRAM. A battery-backed N163 configuration appends its 128-byte audio RAM after PRG save memory. UNROM 512 flash saves contain the flash image. These formats are implemented in [mapper.c](../src/rom/mapper.c).

Short PRG/CHR save files leave the unread portion of allocated memory zero-filled. This allows a shorter existing save to load, but does not establish that files from other emulators or different board configurations have a compatible layout. Keep a copy before migrating or resizing saves.

Ordinary RAM and EEPROM saves are written directly to their destinations. Flash uses a temporary file and replacement. A failure does not give every cartridge save path the disk loader's refusal-to-close behavior. Check that the game directory is writable and keep backups of progress you need to retain.

## Writable FDS and QD images

**Disk saves replace the image passed to the application. There is no separate disk `.sav` overlay.** Run a working copy when the game needs to write, or start with writes blocked:

```sh
./cupid-nes --fds-bios "disksys.rom" --fds-write-protect "game.fds"
```

The BIOS is read for execution and is not the disk-save destination. F10 toggles the loaded media's write protection. F8 inserts or ejects the selected side, and F9 changes sides. Ejecting alone does not save the image.

When flushing dirty media, Cupid rebuilds the image at its original side size and header layout. It writes a sibling temporary file such as `game.fds.cupid-fds.tmp`, closes it, and replaces the loaded path. Successfully written dirty sides become the new in-memory saved state.

If replacement fails, the dirty image remains in memory. The loader refuses to replace or unload it, and the application's normal quit path displays an error and remains open. Correct the destination problem and close the window again to retry. When write protection was enabled after a game had already changed the disk, disable it with F10 before retrying the save.

A leftover temporary file can prevent a new save because temporary files are created exclusively. Preserve it for inspection, verify that no other instance is using the same image, and move it aside before retrying. Do not force-close the running instance while unsaved media is the only copy of the changes.

These rules follow [fds_flush()](../src/rom/fds.c), [the loader's unload path](../src/rom/rom.c), and [the application's quit handling](../src/main.c). They apply to normal loader and frontend operations; low-level device teardown is not a recovery API.

## Family BASIC tape recordings

Tape paths are supplied explicitly:

```sh
./cupid-nes --console famicom --expansion family-basic --tape-record "program.tap" "basic.nes"
```

F10 starts recording after BASIC is ready to save. F11 stops and writes the capture to the supplied path. A later recording starts a new capture; it does not append a new program to the previous file. Normal exit also attempts to save a pending recording.

The raw format packs digital samples least-significant bit first at one sample per 88 emulated CPU cycles. Only complete bytes are written, so a partial final byte is omitted. It has no WAV header and is not a BASIC source-text file.

Recording saves use a sibling `.cupid-tape.tmp` file followed by replacement. A failed F11 save keeps the capture in memory so the user can fix the destination and press F11 again. Unlike a failed disk save, a tape failure during application shutdown does not keep the application open; it is reported and the process exits with failure. Save a recording successfully with F11 before closing.

## Backups and test fixtures

Copy saves after the application has closed successfully, and keep each save with the image and hardware configuration that produced it. Use separate directories when running multiple copies of the same game.

`load_rom_memory()` copies an image without assigning a persistence path. `load_fds_memory()` accepts an optional disk destination; modified memory-only disk fixtures need an explicit persistence strategy. Tests should use their own temporary media and the existing cleanup helpers. See [development and testing](development.md).
