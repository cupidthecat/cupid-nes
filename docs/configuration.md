# Configuration

[Documentation index](README.md)

The application reads its options at startup in [src/main.c](../src/main.c). It has no persistent settings file. All examples below use the Linux executable; on Windows replace `./cupid-nes` with `.\build\windows\cupid-nes.exe`.

```sh
./cupid-nes [options] "game.nes"
```

Supply one image path and write option values as separate arguments. For example, use `--port2 zapper`, not `--port2=zapper`. Names are case sensitive. Quote paths containing spaces. Unknown options and extra image paths are errors.

## Console and revision selection

| Option | Accepted values | Default and effect |
| --- | --- | --- |
| `--console MODEL` | `nes-001`, `nes-101`, `famicom`, `av-famicom` | `nes-001`; selects controller-port wiring |
| `--cpu-revision REVISION` | `early-2a03`, `late-2a03` | `early-2a03`; selects DMC reload-collision behavior |
| `--ppu-revision REVISION` | `2c02-pre-e`, `2c02e-plus` | `2c02e-plus`; selects the revision used by the optional OAM row-corruption model |
| `--ppu-oam-row-corruption` | No value | Disabled; enables the deterministic worst-case row-copy model |
| `--ppu-oam-decay` | No value | Disabled; enables OAM row refresh and decay tracking |
| `--ppu-startup-restriction` | No value | Disabled; enables the initial PPU protected-register interval |

Timing comes from the ROM header. `--console famicom` changes input wiring; it does not force a region. The application has no `--region` flag. The diagnostic runner's legacy PAL mode is a separate test facility.

The OAM profiles are optional approximations with specific timing and revision assumptions. Their defaults, reset behavior, and known limits are in the [accuracy notes](accuracy.md). Enabling every profile does not establish compatibility with every physical console.

## Controllers and expansion hardware

| Option | Accepted values | Default |
| --- | --- | --- |
| `--adapter TYPE` | `none`, `four-score`, `famicom-2`, `famicom-4` | `none` |
| `--port1 DEVICE` | `pad`, `none`, `arkanoid`, `power-pad-a`, `power-pad-b`, `zapper` | `pad` |
| `--port2 DEVICE` | Same values as port 1 | `pad` |
| `--expansion DEVICE` | `none`, `arkanoid`, `family-trainer-a`, `family-trainer-b`, `zapper`, `family-basic` | `none` |
| `--zapper-radius PIXELS` | Decimal integer from 0 through 255 | `0`; radius around the aim point used for light detection |
| `--barcode DIGITS` | 8 or 13 decimal digits | No scan; requires a Datach cartridge |

A Four Score requires both normal ports to remain `pad`. A Famicom multiplayer adapter cannot share the expansion connector with another expansion device. Use `--console famicom` for the Famicom examples below so the controller bus uses the corresponding wiring.

```sh
./cupid-nes --adapter four-score "four-player.nes"
./cupid-nes --console famicom --adapter famicom-2 "multiplayer.nes"
./cupid-nes --port2 zapper "light-gun.nes"
./cupid-nes --console famicom --expansion arkanoid "paddle.nes"
```

The application maps host input to each selected device. The [controls guide](controls.md) explains mouse aiming, mat keys, and controller assignment. A barcode supplied at startup is scanned after loading; F8 scans the configured value again.

## Family BASIC tape

| Option | Value | Effect |
| --- | --- | --- |
| `--tape-play FILE` | Existing raw tape path | Loads a tape; F10 starts playback |
| `--tape-record FILE` | Recording destination | F10 starts recording; F11 stops and saves |

Choose one tape option. Both require `--expansion family-basic`.

```sh
./cupid-nes --console famicom --expansion family-basic --tape-play "program.tap" "basic.nes"
./cupid-nes --console famicom --expansion family-basic --tape-record "recording.tap" "basic.nes"
```

Issue the appropriate load or save operation in BASIC, then press F10 when it is ready. The file contains packed digital signal samples. It is not a WAV file or a text export of the program. See [saves and media](saves.md) for the format and recording behavior.

## Disk-system options

| Option | Value | Default and effect |
| --- | --- | --- |
| `--fds-bios BIOS` | Path to an 8 KiB BIOS | Selects the disk loader for the positional image path |
| `--fds-side N` | Positive decimal side number, starting at 1 | First side; selects and inserts the requested side |
| `--fds-eject` | No value | Disabled; starts with the selected side ejected |
| `--fds-write-protect` | No value | Disabled; blocks writes to disk media |

The last three options require `--fds-bios`. A side number greater than the image's side count is rejected.

```sh
./cupid-nes --fds-bios "disksys.rom" --fds-side 2 --fds-eject "game.fds"
```

F8 inserts the selected side when the game requests it. Runtime disk keys and disk-file replacement are covered in [controls](controls.md) and [saves](saves.md).

## VS DIP switches

`--vs-dip VALUE` accepts a value from 0 through 65535. Use decimal or a `0x` hexadecimal prefix. Avoid leading zeroes in decimal values because the parser uses base detection. The default is zero.

The low byte belongs to the main cabinet and the high byte to the secondary cabinet. The meaning of individual switches depends on the game. This option requires a VS image with supported metadata. It does not select its PPU model, protection device, or mapper.

## Application options and diagnostic modes

The application and test runner have different parsers. Flags such as `--trace`, `--rom`, and `--accuracycoin` belong to `accuracy-tests`, while the options on this page belong to `cupid-nes`. Running either program without a game argument is not a general help request: the application prints usage, and the test runner executes its hardware suite. See [development and testing](development.md) for the runner's complete mode table.
