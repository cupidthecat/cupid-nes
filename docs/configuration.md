# Configuration

[Documentation index](README.md)

Cupid reads application settings from command-line arguments at startup. There is no persistent application configuration file or remapping UI.

```sh
./cupid-nes [options] "game.nes"
```

On Windows, replace `./cupid-nes` with `.\build\windows\cupid-nes.exe`. Supply one image path. Option values are separate arguments, so use `--port2 zapper`, not `--port2=zapper`. Names are case sensitive. Unknown options and a second image path are errors. Options are processed from left to right. Repeating a normal selector such as `--console`, `--port2`, or `--vs-dip` leaves the last value in effect. `--startup-phase` and `--startup-seed` are mutually exclusive and cannot be repeated; the tape options follow the same one-choice rule.

## Console and CPU/PPU profiles

| Option | Accepted value | Default | What it changes |
| --- | --- | --- | --- |
| `--console MODEL` | `nes-001`, `nes-101`, `famicom`, `av-famicom` | `nes-001` | Console controller-port wiring |
| `--cpu-revision REVISION` | `early-2a03`, `late-2a03` | `early-2a03` | CPU/APU revision behavior used by the DMC model |
| `--cpu-test-mode` | No value | Off | Enables the 2A03 channel-output diagnostic reads at `$4018-$401A` |
| `--startup-phase CPU:PPU` | Decimal master-clock offsets within the regional dividers | CPU `0`, PPU divider minus one | Selects a reproducible power-on alignment |
| `--startup-seed SEED` | Decimal integer from `0` through `4294967295` | No randomization | Generates a reproducible sequence of legal power-on alignments |
| `--ram-power-on STATE` | `default`, `zero`, `ones`, `random` | `default` | Selects the initial CPU RAM, PPU RAM, and cartridge board RAM contents |
| `--power-on-seed SEED` | Decimal integer from `0` through `4294967295` | Fixed initial seed | Makes random RAM contents and the optional startup VBL flag reproducible |
| `--random-vblank` | No value | Off | Randomizes the PPU VBL flag at power-on independently of the RAM profile |
| `--ppu-revision REVISION` | `2c02-pre-e`, `2c02e-plus` | `2c02e-plus` | PPU silicon revision used by optional OAM behavior |
| `--ppu-oam-row-corruption` | No value | Off | Enables the deterministic worst-case OAM row-corruption profile |
| `--ppu-startup-restriction` | No value | Off | Enables the protected PPU register-write interval after power-on and soft reset |
| `--ppu-oam-decay` | No value | Off | Enables OAM row refresh and decay tracking |
| `--ppu-reset-suppression` | No value | Off | Preserves PPU registers, scrolling, and rendering state during a CPU soft reset |
| `--video-filter` | `direct`, `ntsc-composite` | `direct` | Selects direct RGB output or NTSC composite reconstruction for ordinary NTSC hardware |
| `--mmc3-revision REVISION` | `standard`, `a` | `standard` | Selects the MMC3 IRQ counter revision for compatible MMC3-family cartridges |
| `--cart-dip VALUE` | Integer from 0 through 255 | `0` | Sets cartridge-board DIP inputs, including mapper 105 competition timing |

The ROM header selects the timing region. `--console famicom` changes console wiring and does not force NTSC, PAL, or Dendy timing. There is no application `--region` option.

The three optional PPU profiles are compatibility models with documented assumptions. Their timing and limits are in [accuracy](accuracy.md).

The `default` RAM profile clears CPU and nametable RAM and fills primary and secondary OAM with `$FF`. `zero` fills those areas with `$00`; `ones` fills them with `$FF`. All three retain the fixed boot palette. `random` fills RAM from the controlled random source and limits palette entries to six bits. The C++ cartridge board modules apply the same profile to their work, save, CHR, and nametable RAM before trainer and save data are loaded. Existing C cartridge implementations keep their board-specific initialization.

`--power-on-seed` sets a separate random source from `--startup-seed`; it may be supplied once. Repeating the same seed, image, and options reproduces the startup state. RAM initialization occurs on hard power-on or cartridge insertion, depending on the memory's owner. Soft reset preserves RAM. `--random-vblank` can be used with any RAM profile and remains off unless supplied.

`--ppu-reset-suppression` leaves the PPU running across the R-key soft reset. CPU, APU, cartridge reset signals, and VS controls still follow their normal reset paths. The PPU retains its registers, scroll latches, raster position, and rendering state; its standalone clock remainder and OAM decay timestamps are cleared. Both screens follow this policy in a dual VS system. Hard power-on still initializes the PPU.

`--video-filter ntsc-composite` reconstructs a 512 by 480 image from the PPU's per-pixel palette, grayscale, emphasis, and frame-phase data. Each decoded line is repeated once vertically. The filter runs after the emulated frame completes, so CPU/PPU timing and the light-gun brightness inputs stay unchanged. PAL, Dendy, and VS hardware retain direct output and report that fallback at startup. The composite path uses the PPU signal rather than an edited RGB palette.

`--cpu-test-mode` enables the read-only channel-output diagnostics at `$4018-$401A`. It does not add writable CPU test registers. The selection, CPU revision, PPU revision, and optional PPU profiles stay selected across the R-key soft reset.

Choose either `--startup-phase` or `--startup-seed`. The CPU offset delays reset release in master clocks; the PPU phase selects the initial divider remainder. NTSC accepts CPU `0..11` and PPU `0..3`, PAL accepts `0..15` and `0..4`, and Dendy accepts `0..14` and `0..4`. The startup log records the applied pair and any supplied seed. For example, `--startup-phase 0:3` selects the default NTSC alignment. Soft reset retains the running phase. A seeded dual VS cabinet draws an alignment for each CPU in main-then-secondary order.

`--mmc3-revision a` selects the earlier IRQ qualification rule. MMC6 and MC-ACC keep their own board-specific IRQ behavior.

`--cart-dip` accepts decimal and C-style base prefixes such as `0x`. Mapper 105 uses the low four bits to select its competition timer interval.

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

## VS DIP switches

| Option | Accepted value | Default |
| --- | --- | --- |
| `--vs-dip VALUE` | Integer from 0 through 65535 | `0` |

`--vs-dip VALUE` accepts an integer from `0` through `65535`. The parser accepts decimal and C-style base prefixes, including `0x` for hexadecimal. Use plain decimal without a leading zero when you do not intend octal interpretation.

The default DIP value is zero. In a dual system, the low byte belongs to the main cabinet and the high byte belongs to the secondary cabinet. Each game assigns meaning to its own switches.

`--vs-dip` requires a supported VS System image. The cartridge metadata selects the VS PPU, protection behavior, controller wiring, and mapper; the DIP option does not override those fields.

## Application options and test-runner options

The application parser and `accuracy-tests` parser are separate. Options such as `--trace`, `--rom`, and `--accuracycoin` belong to the test runner and are not accepted by `cupid-nes`. See [development](development.md) for the diagnostic runner and [hardware](hardware.md) for supported cartridge hardware.
