# Configuration

[Documentation index](README.md)

Cupid reads application settings from command-line arguments at startup. There is no persistent application configuration file or remapping UI.

```sh
./cupid-nes [options] "game.nes"
```

On Windows, replace `./cupid-nes` with `.\build\windows\cupid-nes.exe`. Supply one image path. Option values are separate arguments, so use `--port2 zapper`, not `--port2=zapper`. Names are case sensitive. Unknown options and a second image path are errors.

## Console and CPU/PPU profiles

| Option | Accepted value | Default | What it changes |
| --- | --- | --- | --- |
| `--console MODEL` | `nes-001`, `nes-101`, `famicom`, `av-famicom` | `nes-001` | Console controller-port wiring |
| `--cpu-revision REVISION` | `early-2a03`, `late-2a03` | `early-2a03` | CPU/APU revision behavior used by the DMC model |
| `--cpu-test-mode` | No value | Off | Enables the 2A03 channel-output diagnostic reads at `$4018-$401A` |
| `--startup-phase CPU:PPU` | Decimal master-clock offsets within the regional dividers | CPU `0`, PPU divider minus one | Selects a reproducible power-on alignment |
| `--startup-seed SEED` | Decimal integer from `0` through `4294967295` | No randomization | Generates a reproducible sequence of legal power-on alignments |
| `--ppu-revision REVISION` | `2c02-pre-e`, `2c02e-plus` | `2c02e-plus` | PPU silicon revision used by optional OAM behavior |
| `--ppu-oam-row-corruption` | No value | Off | Enables the deterministic worst-case OAM row-corruption profile |
| `--ppu-startup-restriction` | No value | Off | Enables the protected PPU register-write interval after power-on and soft reset |
| `--ppu-oam-decay` | No value | Off | Enables OAM row refresh and decay tracking |

The ROM header selects the timing region. `--console famicom` changes console wiring and does not force NTSC, PAL, or Dendy timing. There is no application `--region` option.

The three optional PPU profiles are compatibility models with documented assumptions. Their timing and limits are in [accuracy](accuracy.md).

Choose either `--startup-phase` or `--startup-seed`. The CPU offset delays reset release in master clocks; the PPU phase selects the initial divider remainder. NTSC accepts CPU `0..11` and PPU `0..3`, PAL accepts `0..15` and `0..4`, and Dendy accepts `0..14` and `0..4`. The startup log records the applied pair and any supplied seed. For example, `--startup-phase 0:3` selects the default NTSC alignment. Soft reset retains the running phase. A seeded dual VS cabinet draws an alignment for each CPU in main-then-secondary order.

## Controllers and expansion devices

| Option | Accepted value | Default |
| --- | --- | --- |
| `--adapter TYPE` | `none`, `four-score`, `famicom-2`, `famicom-4` | `none` |
| `--port1 DEVICE` | `pad`, `none`, `arkanoid`, `power-pad-a`, `power-pad-b`, `zapper` | `pad` |
| `--port2 DEVICE` | `pad`, `none`, `arkanoid`, `power-pad-a`, `power-pad-b`, `zapper` | `pad` |
| `--expansion DEVICE` | `none`, `arkanoid`, `family-trainer-a`, `family-trainer-b`, `zapper`, `family-basic` | `none` |
| `--zapper-radius PIXELS` | Decimal integer from `0` through `255` | `0` |

A Four Score requires both normal ports to stay set to `pad`. The `famicom-2` and `famicom-4` adapters use the expansion connector, so they cannot be combined with another `--expansion` device. An invalid combination exits before loading the image with `An adapter and another device cannot share the same connector`.

Examples:

```sh
./cupid-nes --adapter four-score "four-player.nes"
./cupid-nes --console famicom --adapter famicom-2 "multiplayer.nes"
./cupid-nes --port2 zapper "light-gun.nes"
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
