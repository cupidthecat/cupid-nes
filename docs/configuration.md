# Configuration

[Documentation index](README.md)

Cupid loads saved application settings and input profiles, then applies explicit command-line choices for the current launch. Use the desktop Settings window for the same supported options and **Controllers and shortcuts** for bindings.

Audio > Output device lists detected playback devices and the system default.
Click Browse beside a BIOS or other file path to choose it in the native file
dialog. Cancel keeps the current path; Clear removes an optional setting.
Changes take effect when you choose Apply or OK.

The application data directory contains `settings.ini`, `recent.ini`, and the optional `NesDB.txt`. By default, SDL chooses the current user's preference directory for `cupidthecat/cupid-nes`. `--data-dir DIR` selects another directory; make sure that directory exists and is writable. Settings and recent-file updates replace their destination only after a complete write succeeds. A missing settings file uses defaults. Malformed input is reported instead of partly applying a configuration.

```sh
./cupid-nes [options] "game.nes"
```

On Windows, replace `./cupid-nes` with `.\build\windows\cupid-nes.exe`. Supply one image path. Option values are separate arguments, so use `--port2 zapper`, not `--port2=zapper`. Names are case sensitive. Unknown options and a second image path are errors. Options are processed from left to right. Repeating a normal selector such as `--console`, `--port2`, or `--vs-dip` leaves the last value in effect. `--startup-phase` and `--startup-seed` are mutually exclusive and cannot be repeated; the tape options follow the same one-choice rule.

## Console and CPU/PPU profiles

| Option | Accepted value | Default | What it changes |
| --- | --- | --- | --- |
| `--region MODE` | `auto`, `ntsc`, `pal`, `dendy` | `auto` | CPU/PPU clocks, frame timing, APU periods, audio rates, and pacing |
| `--console MODEL` | `nes-001`, `nes-101`, `famicom`, `av-famicom` | `nes-001` | Console controller-port wiring |
| `--cpu-revision REVISION` | `early-2a03`, `late-2a03` | `early-2a03` | CPU/APU revision behavior used by the DMC model |
| `--cpu-test-mode` | No value | Off | Enables the 2A03 channel-output diagnostic reads at `$4018-$401A` |
| `--apu-disable-noise-mode` | No value | Off | Uses the long noise sequence even when `$400E` selects short mode |
| `--apu-swap-duty-cycles` | No value | Off | Swaps duty selections 1 and 2 on the two base APU pulse channels |
| `--startup-phase CPU:PPU` | Decimal master-clock offsets within the regional dividers | CPU `0`, PPU divider minus one | Selects a reproducible power-on alignment |
| `--startup-seed SEED` | Decimal integer from `0` through `4294967295` | No randomization | Generates a reproducible sequence of legal power-on alignments |
| `--ram-power-on STATE` | `default`, `zero`, `ones`, `random` | `default` | Selects the initial CPU, PPU, cartridge, and disk-adapter RAM contents |
| `--power-on-seed SEED` | Decimal integer from `0` through `4294967295` | Fixed initial seed | Makes random RAM contents and the optional startup VBL flag reproducible |
| `--random-vblank` | No value | Off | Randomizes the PPU VBL flag at power-on independently of the RAM profile |
| `--ppu-revision REVISION` | `2c02-pre-e`, `2c02e-plus` | `2c02e-plus` | PPU silicon revision used by optional OAM behavior |
| `--ppu-oam-row-corruption` | No value | Off | Enables the deterministic worst-case OAM row-corruption profile |
| `--ppu-startup-restriction` | No value | Off | Enables the protected PPU register-write interval after power-on and soft reset |
| `--ppu-oam-decay` | No value | Off | Enables OAM row refresh and decay tracking |
| `--ppu-disable-oamdata-read` | No value | Off | Leaves `$2004` reads on the PPU I/O bus without driving OAM data |
| `--ppu-disable-palette-readback` | No value | Off | Uses buffered `$2007` reads in palette space |
| `--ppu-sprite-eval-wrap-bug` | No value | Off | Models the early PPU's partial sprite entry after evaluation wraps through OAM |
| `--ppu-reset-suppression` | No value | Off | Preserves PPU registers, scrolling, and rendering state during a CPU soft reset |
| `--video-filter` | `direct`, `ntsc-composite` | `direct` | Selects direct RGB output or NTSC composite reconstruction for ordinary NTSC hardware |
| `--mmc3-revision REVISION` | `standard`, `a` | `standard` | Selects the MMC3 IRQ counter revision for compatible MMC3-family cartridges |
| `--cart-dip VALUE` | Integer from 0 through 255 | `0` | Sets cartridge-board DIP inputs, including mapper 105 competition timing |

`--region auto` follows image metadata and applicable database corrections. Explicit `ntsc`, `pal`, and `dendy` choices take precedence over the detected timing without changing the stored header or database record. `--console famicom` selects controller wiring independently of the timing region and PPU revision.

Startup output reports the effective timing, for example `Timing region: PAL (selection: pal)`. The R-key reset keeps that timing. Relaunch the application with a different `--region` value to change it. VS, FDS, and StudyBox require effective NTSC timing; PAL and Dendy selections are rejected before replacing an active machine. Failed image loads retain the previous cartridge and active timing.

```sh
./cupid-nes --region pal "game.nes"
./cupid-nes --region dendy --console famicom "game.nes"
```

The optional PPU controls are independent. Selecting `--ppu-revision 2c02-pre-e` does not enable disabled register readback or the sprite-evaluation wrap behavior. Their timing and limits are in [accuracy](accuracy.md).

`--apu-disable-noise-mode` models the oldest Famicom noise behavior. It retains the written `$400E` mode bit and uses the long-sequence feedback tap at each noise clock. `--apu-swap-duty-cycles` models clone pulse wiring: a write to `$4000` or `$4004` maps duty fields `0, 1, 2, 3` to `0, 2, 1, 3`. MMC5 pulse channels keep their ordinary mapping. Both controls are independent of `--cpu-revision` and remain selected across reset.

`--ppu-disable-oamdata-read` keeps the existing I/O latch value on `$2004` reads, including its decay behavior. `--ppu-disable-palette-readback` returns the normal delayed read buffer in palette space while keeping external memory fetches and address increments. `--ppu-sprite-eval-wrap-bug` lets a Y-only secondary-OAM entry reach the ordinary sprite pipeline, with tile, attributes, and X left at `$FF`. All three controls remain selected across reset and default to off.

The `default` RAM profile clears CPU and nametable RAM and fills primary and secondary OAM with `$FF`. `zero` fills those areas with `$00`; `ones` fills them with `$FF`. All three retain the fixed boot palette. `random` fills RAM from the controlled random source and limits palette entries to six bits. The C++ cartridge board modules apply the same profile to their work, save, CHR, and nametable RAM before trainer and save data are loaded. Existing C cartridge implementations keep their board-specific initialization.

Loading a disk image applies the same profile to the adapter's 32 KiB work RAM and 8 KiB CHR RAM. `default` and `zero` clear both areas, `ones` fills them with `$FF`, and `random` uses the configured power-on seed. Soft reset and disk-side changes preserve this RAM.

`--power-on-seed` sets a separate random source from `--startup-seed`; it may be supplied once. Repeating the same seed, image, and options reproduces the startup state. RAM initialization occurs on hard power-on or cartridge insertion, depending on the memory's owner. Soft reset preserves RAM. `--random-vblank` can be used with any RAM profile and remains off unless supplied.

`--ppu-reset-suppression` leaves the PPU running across the R-key soft reset. CPU, APU, cartridge reset signals, and VS controls still follow their normal reset paths. The PPU retains its registers, scroll latches, raster position, and rendering state; its standalone clock remainder and OAM decay timestamps are cleared. Both screens follow this policy in a dual VS system. NSF and NSFe playback always reset their clock-only PPU state. Hard power-on still initializes the PPU.

`--video-filter ntsc-composite` reconstructs a 512 by 480 image from the PPU's per-pixel palette, grayscale, emphasis, and frame-phase data. Each decoded line is repeated once vertically. The filter runs after the emulated frame completes, so CPU/PPU timing and the light-gun brightness inputs stay unchanged. PAL, Dendy, and VS hardware retain direct output and report that fallback at startup. The composite path uses the PPU signal rather than an edited RGB palette.

`--cpu-test-mode` enables the read-only channel-output diagnostics at `$4018-$401A`. It does not add writable CPU test registers. The selection, CPU revision, PPU revision, and optional PPU profiles stay selected across the R-key soft reset.

Choose either `--startup-phase` or `--startup-seed`. The CPU offset delays reset release in master clocks; the PPU phase selects the initial divider remainder. NTSC accepts CPU `0..11` and PPU `0..3`, PAL accepts `0..15` and `0..4`, and Dendy accepts `0..14` and `0..4`. The startup log records the applied pair and any supplied seed. For example, `--startup-phase 0:3` selects the default NTSC alignment. Soft reset retains the running phase. A seeded dual VS cabinet draws an alignment for each CPU in main-then-secondary order.

`--mmc3-revision a` selects the earlier IRQ qualification rule. MMC6 and MC-ACC keep their own board-specific IRQ behavior.

`--cart-dip` accepts decimal and C-style base prefixes such as `0x`. Mapper 105 uses the low four bits to select its competition timer interval. The FamicomBox menu board exposes all eight bits at `$5002` and its register aliases.

## NSF and NSFe playback

Supply an NSF or NSFe file as the image path. No BIOS or music-specific command-line option is required. The file selects NTSC or PAL timing and its initial track; [Page Up and Page Down](controls.md#keyboard-controls) select other tracks. The music environment does not render a game screen.

## EPSM sound

NES 2.0 console selector 3 with extended subtype 4 enables EPSM sound. The device runs a YMF288 at 8 MHz and uses stereo output. Ordinary NES and VS images do not enable it.

`--epsm-adpcm FILE` supplies the chip's 8 KiB percussion ROM. The file must contain exactly 8,192 bytes; it is separate from the game image. Cupid does not include this firmware. Without the option, FM and SSG sound remain available, but percussion uses zero-filled data and will be incorrect. The startup log reports this condition.

```sh
./cupid-nes --epsm-adpcm "ymf288_adpcm_rom.bin" "epsm-game.nes"
```

## Famicom Network System firmware

NES 2.0 extended console subtype `0x0C` selects the Famicom Network System cartridge hardware. `--fcns-kanji FILE` supplies its 256 KiB character ROM. The file must contain exactly 262,144 bytes. If the option is omitted, the character-ROM window reads zero-filled data.

```sh
./cupid-nes --fcns-kanji "lh5323m1.bin" "fcns.nes"
```

The emulator models the local cartridge, character-ROM interface, RAM banking, and controller. It does not connect to or reproduce the original network service.

## Game database and headerless cartridges

Cupid looks for `NesDB.txt` in its application data folder before loading an image. `--data-dir DIR` selects another data folder, and `--game-db FILE` takes precedence over that default. Startup output reports the selected path and whether the database loaded or the optional default was absent. A missing default allows normal image loading. An unreadable or malformed database, or a missing file explicitly selected with `--game-db`, stops startup with an error. Failed database replacements preserve the previous entries and the running machine.

Database files are limited to 64 MiB and use the 18-field CSV layout for CRC, system, board, PCB, chip, mapper, PRG ROM, CHR ROM, CHR RAM, work RAM, save RAM, battery, mirroring, input, bus conflicts, submapper, VS hardware type, and VS PPU model. ROM and RAM sizes are KiB unless prefixed with `b`, which records an exact byte count. Blank optional RAM fields on unvalidated legacy entries retain the cartridge header's normal RAM defaults.

For legacy iNES images, Cupid hashes the PRG+CHR payload after the header and optional trainer. A matching entry can correct the mapper, submapper, ROM sizes, supported RAM sizes, battery state, mirroring, regional or VS metadata, input type, board/chip information, and bus-conflict setting. NES 2.0 metadata keeps precedence over ordinary database corrections. `--no-game-db-overrides` disables corrections for headered legacy images.

Headerless images use a separate whole-file CRC lookup. A matching database entry supplies the complete cartridge description needed to validate and load the payload. Headerless lookup remains available with `--no-game-db-overrides` because there is no image header to fall back to. Unknown headerless payloads and invalid database records are rejected without replacing an active cartridge. Startup output reports the selected metadata source together with whole-file, PRG, and PRG+CHR CRCs for cartridge images.

```sh
./cupid-nes --data-dir "cupid-data" "game.nes"
./cupid-nes --game-db "NesDB.txt" "game.nes"
./cupid-nes --game-db "NesDB.txt" --no-game-db-overrides "headerless.bin"
```

## UNIF cartridges

UNIF images are detected from their `UNIF` signature. The loader validates the 32-byte header and chunk lengths, assembles `PRG0` through `PRGF` and `CHR0` through `CHRF` in numeric order, and reads `MAPR`, `TVCI`, `BATR`, and `MIRR` metadata. Chunk order does not affect ROM bank order; the last copy of a repeated chunk wins. Unknown chunk types are ignored. Missing PRG data, a missing board name, an unresolved board after database lookup, invalid chunk indexes, and truncated chunks reject the load without replacing the active cartridge.

`MAPR` accepts the board names and `NES-`, `HVC-`, `UNL-`, `BTL-`, and `BMC-` aliases used by the supported board table. Named boards that do not have a numeric mapper use their dedicated cartridge implementations. UNIF does not declare ordinary work-RAM sizes, so supported boards retain their cartridge defaults unless a matching game-database entry supplies an explicit override.

A database match can identify an otherwise unknown board name. Its board, RAM, timing, and input corrections are applied before validating the resulting machine. The assembled UNIF chunks determine ROM sizes. A missing mirroring override keeps the image's mirroring setting, subject to the board's own fixed wiring and startup registers.

## Controllers and expansion devices

| Option | Accepted value | Default |
| --- | --- | --- |
| `--adapter TYPE` | `none`, `four-score`, `famicom-2`, `famicom-4` | `none` |
| `--port1 DEVICE` | `pad`, `none`, `arkanoid`, `power-pad-a`, `power-pad-b`, `zapper`, `snes-pad`, `snes-mouse`, `ntt-keypad`, `virtual-boy` | `pad` |
| `--port2 DEVICE` | `pad`, `none`, `arkanoid`, `power-pad-a`, `power-pad-b`, `zapper`, `subor-mouse`, `snes-pad`, `snes-mouse`, `ntt-keypad`, `virtual-boy` | `pad` |
| `--expansion DEVICE` | `none`, `arkanoid`, `family-trainer-a`, `family-trainer-b`, `zapper`, `family-basic`, `turbo-file`, `battle-box`, `subor-keyboard`, `hori-track`, `konami-hyper-shot`, `bandai-hyper-shot`, `party-tap`, `pachinko`, `exciting-boxing`, `jissen-mahjong`, `barcode-battler`, `oeka-kids-tablet`, `fcns` | `none` |
| `--zapper-radius PIXELS` | Decimal integer from `0` through `255` | `0` |

A Four Score requires both normal ports to stay set to `pad`. The `famicom-2` and `famicom-4` adapters use the expansion connector, so they cannot be combined with another `--expansion` device. An invalid combination exits before loading the image with `An adapter and another device cannot share the same connector`. Four Score validation only reserves the two normal controller ports; it does not reserve the expansion-device setting.

The supported Subor keyboard and mouse combination uses `--expansion subor-keyboard --port2 subor-mouse`. The mouse is accepted only on port 2, but the parser does not require the keyboard and mouse to be selected together.

The SNES controller, SNES mouse, NTT Data keypad, and Virtual Boy controller are accepted on either normal controller port. `snes-pad`, `snes-mouse`, `ntt-keypad`, and `virtual-boy` select them explicitly. NES 2.0 default-input value `0x2B` also selects two SNES controllers when the corresponding command-line fields have not been overridden. Default-input value `0x3B` selects the FCNS controller on the expansion connector. The NES 2.0 SNES-mouse value `0x29` is not auto-connected; use `--port1 snes-mouse` or `--port2 snes-mouse` for that device.

`--console` and the input-device options are independent. Selecting an expansion device does not switch the console model to `famicom`, and selecting `famicom` does not choose an expansion device. Supported ordinary NES 2.0 default-input metadata supplies automatic controller defaults, while an explicit `--adapter`, `--port1`, `--port2`, or `--expansion` value keeps control of that field. The selected console model also determines whether ordinary Zapper metadata uses controller port 2 or the Famicom expansion connector; Dendy timing does not replace that wiring choice. Unsupported ordinary input values leave the current controller configuration in place and print a diagnostic. Supported VS headers use their separate cabinet input decoder for standard, swapped, swapped-A/B, or serial light-gun wiring. A zero VS input code falls back to standard wiring.

Examples:

```sh
./cupid-nes --adapter four-score "four-player.nes"
./cupid-nes --console famicom --adapter famicom-2 "multiplayer.nes"
./cupid-nes --port2 zapper "light-gun.nes"
./cupid-nes --port1 snes-pad "snes-controller-game.nes"
./cupid-nes --port2 snes-mouse "mouse-game.nes"
./cupid-nes --console famicom --expansion arkanoid "paddle.nes"
```

`--zapper-radius` controls the light-sampling radius around the mouse aim point. It does not change the trigger mapping. [Controls](controls.md) explains controller slots, mouse input, and mat keys.

## Datach barcode input

| Option | Accepted value | Default |
| --- | --- | --- |
| `--barcode DIGITS` | Exactly 8 or 13 decimal digits | No barcode scan |

```sh
./cupid-nes --barcode 4901234567894 "datach.nes"
```

`--barcode DIGITS` accepts exactly 8 or 13 decimal digits and requires a supported Datach cartridge. The barcode is scanned once after loading. F8 starts the configured scan again while the game is running.

Barcode Battler uses a separate input option and device state:

| Option | Accepted value | Default |
| --- | --- | --- |
| `--barcode-battler DIGITS` | Exactly 8 or 13 decimal digits | No Barcode Battler scan |

Use it with `--console famicom --expansion barcode-battler`. The configured stream is queued at CPU cycle zero before the initial power-on reset sequence. F8 restarts it later while the game is running. This reader is independent of cartridge barcode hardware.

## Family BASIC tape

Choose one tape option:

| Option | Value | Behavior |
| --- | --- | --- |
| `--tape-play FILE` | Existing raw tape file | Loads the file; F10 starts playback |
| `--tape-record FILE` | Recording destination | F10 starts a fresh recording; F11 stops and saves it |

Both options require `--expansion family-basic`, and the parser rejects using them together.

```sh
./cupid-nes --console famicom --expansion family-basic --tape-play "program.tap" "basic.nes"
./cupid-nes --console famicom --expansion family-basic --tape-record "recording.tap" "basic.nes"
```

Start the matching load or save operation in BASIC before pressing F10. The tape file is a packed digital signal capture, not WAV audio or BASIC source text. Its persistence rules are in [saves and media](saves.md).

## FDS media

| Option | Value | Default and behavior |
| --- | --- | --- |
| `--fds-bios BIOS` | Path to an 8 KiB FDS BIOS | Uses the disk-system loader for the positional image |
| `--fds-side N` | Positive decimal side number starting at `1` | Side 1; inserts the requested side after loading |
| `--fds-eject` | No value | Off; starts with the selected side ejected |
| `--fds-write-protect` | No value | Off; blocks writes to the loaded disk image |

`--fds-side`, `--fds-eject`, and `--fds-write-protect` require `--fds-bios`. A requested side outside the loaded image is rejected.

```sh
./cupid-nes --fds-bios "disksys.rom" --fds-side 2 --fds-eject "game.fds"
```

Runtime disk keys and disk replacement behavior are documented in [controls](controls.md) and [saves and media](saves.md).

## StudyBox media

Use `--studybox-bios BIOS` to load an STBX tape image with the StudyBox hardware. The BIOS must be exactly 256 KiB. The positional image must begin with the `STBX` signature and contain supported version-`0x100` PAGE chunks; an optional type-0 AUDI chunk carries mono 16-bit PCM WAV data used by the tape audio path.

```sh
./cupid-nes --studybox-bios "StudyBox.bin" "lesson.stbx"
```

StudyBox media and FDS firmware selections are mutually exclusive. Supplying an STBX file through the ordinary cartridge loader reports that the StudyBox BIOS is required rather than treating the tape image as an iNES cartridge.

## VS DIP switches

| Option | Accepted value | Default |
| --- | --- | --- |
| `--vs-dip VALUE` | Integer from 0 through 65535 | `0` |

`--vs-dip VALUE` accepts an integer from `0` through `65535`. The parser accepts decimal and C-style base prefixes, including `0x` for hexadecimal. Use plain decimal without a leading zero when you do not intend octal interpretation.

The default DIP value is zero. In a dual system, the low byte belongs to the main cabinet and the high byte belongs to the secondary cabinet. Each game assigns meaning to its own switches.

`--vs-dip` requires a supported VS System image. The cartridge metadata selects the VS PPU, protection behavior, controller wiring, and mapper; the DIP option does not override those fields.

## Application options and test-runner options

The application parser and `accuracy-tests` parser are separate. Options such as `--trace`, `--rom`, and `--accuracycoin` belong to the test runner and are not accepted by `cupid-nes`. See [development](development.md) for the diagnostic runner and [hardware](hardware.md) for supported cartridge hardware.

## Desktop storage controls

Tools > Storage locations shows effective paths and provides Browse and Open Folder
actions. The database path and correction switch are saved as `game_database_path`
and `disable_database_corrections`; explicit `--game-db` and correction options
retain precedence. The selected movie path is saved as `movie_file_path`. See the
[desktop guide](desktop.md) for settings categories and when changes take effect.
