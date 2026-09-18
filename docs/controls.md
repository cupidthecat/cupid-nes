# Controls and peripherals

[Documentation index](README.md)

Select emulated input hardware with the options in [configuration](configuration.md) before launching a game. The application has fixed host mappings; it does not expose an interactive controller-remapping screen.

## Keyboard controls

| Key | Action |
| --- | --- |
| Z | Player 1 A |
| X | Player 1 B |
| Right Shift | Player 1 Select |
| Enter | Player 1 Start |
| Arrow keys | Player 1 D-pad |
| R | Soft reset |
| M, held | Original Famicom controller 2 microphone signal |
| F6 | Restore the built-in palette |
| F7 | Toggle the palette editor |
| Ctrl+V | Paste palette text |

Close the window for normal shutdown. The M key supplies the emulated microphone line only; Cupid does not capture a host microphone.

Family BASIC and mat devices consume their matching keyboard events before the normal shortcuts. Their exceptions are listed below.

## Game controllers and player slots

Cupid opens devices that SDL recognizes through its GameController interface. At startup, recognized controllers fill the first free host-player slots in discovery order. Hot-plugged controllers also take the first free slot. Removing a controller clears the buttons held in that slot.

The first controller is player 1, the second is player 2, the third is player 3, and the fourth is player 4. The input layer has six player slots so the Famicom four-player adapter can expose players 5 and 6 as well.

| Host controller button | Emulated button |
| --- | --- |
| A | A |
| B | B |
| Back | Select |
| Start | Start |
| D-pad | Up, Down, Left, Right |

Analog sticks are not mapped by the frontend. The keyboard writes player 1's button state, so player 1 can be driven by both the keyboard and the first controller. They update the same button state; avoid using both devices for the same button at once.

Which player slots reach the game depends on the emulated wiring:

| Configuration | Player routing |
| --- | --- |
| Normal controller ports | Players 1 and 2 |
| `--adapter four-score` | Port 1 reports players 1 then 3; port 2 reports players 2 then 4 |
| `--console famicom --adapter famicom-2` | Built-in players 1/2 plus players 3/4 on expansion data lines |
| `--console famicom --adapter famicom-4` | Built-in players 1/2 plus players 3 through 6 in the expansion adapter reports |
| Dual VS System | Players 1/2 on the main side and players 3/4 on the secondary side |

Each cartridge still decides which reports it reads. More detail about the emulated adapters is in [hardware](hardware.md).

## Arkanoid paddle

Use `--port1 arkanoid`, `--port2 arkanoid`, or `--console famicom --expansion arkanoid` according to the game. Horizontal mouse position controls the paddle. The left mouse button is the paddle fire button.

The frontend sends the same mouse position and fire state to every configured paddle slot. A gamepad analog stick does not control the paddle.

## Zapper

For a standard NES light gun configuration:

```sh
./cupid-nes --port2 zapper "game.nes"
```

Move the mouse over the game window to aim. Left click fires on screen. Right click holds the trigger while treating the aim position as off screen, which supports games that use off-screen shots for reload behavior.

The default light-sampling radius is zero. `--zapper-radius N` expands the sampled area up to 255 pixels. The Famicom expansion version uses `--console famicom --expansion zapper`. VS images whose NES 2.0 metadata selects the VS Zapper use the same mouse aiming and trigger controls.

## Power Pad and Family Trainer

Use `--port1 power-pad-a`, `--port2 power-pad-a`, or the corresponding `power-pad-b` setting for an NES mat. Family Trainer uses `--console famicom --expansion family-trainer-a` or `family-trainer-b`.

The host keys form three rows of four positions:

```text
1  2  3  4
Q  W  E  R
A  S  D  F
```

Those positions are left to right for side A. Side B reverses each row in the device mapping. The same key state is supplied to every active mat slot.

Mat input is handled before the normal keyboard switch. For example, R is a mat position while a selected mat uses that key, so pressing R does not soft-reset the emulator in that configuration.

## Family BASIC keyboard and tape

`--console famicom --expansion family-basic` routes keyboard events to the Family BASIC matrix. Letters, digits, arrows, modifier keys, punctuation, and F1 through F8 map to BASIC keyboard positions.

Some US keyboard mappings are less obvious:

| Host key | Family BASIC key |
| --- | --- |
| Grave | `@` |
| Apostrophe | `:` |
| Equals | `^` |
| Backslash | Yen |
| F9 | Underscore |
| Left Alt | GRPH |
| Right Alt | Kana |
| Home | CLEAR/HOME |
| F12 | STOP |
| Backspace or Delete | DELETE |

While Family BASIC is selected, keyboard events are consumed by the keyboard/tape handler. R is the BASIC letter R, F6 and F7 are BASIC function keys, and the usual palette shortcuts do not run from those keys.

With `--tape-play FILE`, F10 starts playback. With `--tape-record FILE`, F10 starts a new recording. F11 stops the tape and writes a pending recording. [Saves and media](saves.md) describes the tape file format and retry behavior.

## Subor keyboard and mouse

`--console famicom --expansion subor-keyboard --port2 subor-mouse` connects the supported keyboard-and-mouse arrangement. The keyboard routes letters, number row keys, F1 through F12, navigation keys, punctuation, modifiers, and numeric keypad keys into the 13-row matrix. Keyboard input is consumed before normal application shortcuts while this device is selected.

Mouse motion uses SDL relative movement. Left and right mouse buttons map to the two Subor mouse buttons. Small movement uses the one-byte report; larger movement is split across the three-byte report and each axis is bounded to 31 units per packet.

## Hori Track

`--console famicom --expansion hori-track` connects Hori Track. Player-one controller buttons supply its controller byte, while relative mouse movement supplies the trackball axes. Each axis is bounded to -8 through 7 when a report is latched.

## Konami Hyper Shot

`--console famicom --expansion konami-hyper-shot` connects the two-player Hyper Shot controller. Player one and player two use each controller's A button for jump and B button for run. Games control the two active-low player-enable lines through `$4016`.

## Bandai Hyper Shot

`--console famicom --expansion bandai-hyper-shot` connects the combined controller and light gun. Player-one buttons are serialized on expansion `$4016 D1`. Mouse aiming and trigger input use the same beam-aware light detection as the normal Zapper and appear on `$4017 D3-D4`.

## Party Tap

`--console famicom --expansion party-tap` connects the six-button Party Tap. Host keys 1 through 6 map to its six buttons. The first two reads return three buttons each on `$4017 D2-D4`; later reads return the device-detection value.

## Pachinko controller

`--console famicom --expansion pachinko` connects the Pachinko controller. Player-one buttons form the first eight serialized bits on expansion `$4016 D1`. Hold the left mouse button to increase the plunger position and the right mouse button to release it. The position changes only when the game latches a report and is bounded from 0 through 99.

## Exciting Boxing

`--console famicom --expansion exciting-boxing` connects the punching-bag sensors. Host keys 1 through 4 map to the first sensor bank and 5 through 8 map to the second. The game selects a bank with `$4016 D1`; inactive sensors read high on `$4017 D1-D4`.

## Jissen Mahjong

`--console famicom --expansion jissen-mahjong` connects the mahjong panel. Letter keys A through N map to the tile keys, Right Shift is Select, Enter is Start, and number keys 1 through 5 map to Kan, Pon, Chii, Riichi, and Ron. The game selects one of four rows through `$4016 D1-D2` and reads the latched row serially on `$4017 D1`.

## ASCII Turbo File

`--console famicom --expansion turbo-file` connects the 8 KiB serial storage device. Games control its reset, clock, data, and read lines through the Famicom expansion connector. The contents are loaded and saved automatically next to the ROM; there is no host key for manually advancing the device.

`--console famicom --expansion battle-box` connects the two-chip BattleBox serial storage device. Its command, chip-select, read, program, write-enable, and erase signals run entirely through the expansion connector.

## FDS keys

| Key | Disk-system action |
| --- | --- |
| F8 | Eject the inserted side, or reinsert the selected side |
| F9 | Select and insert the next side, wrapping after the last side |
| F10 | Toggle write protection |

Changing or ejecting a side does not flush the disk image. Follow the game's disk prompts and close the emulator normally when the game has written data. See [saves and media](saves.md) for disk persistence.

## VS cabinet controls

| Key | VS input |
| --- | --- |
| 5 | Main cabinet coin slot 1 |
| 6 | Main cabinet coin slot 2 |
| 7 | Secondary cabinet coin slot 1 in dual mode |
| 8 | Secondary cabinet coin slot 2 in dual mode |
| F1 | Main cabinet service input |
| F2 | Secondary cabinet service input in dual mode |

`--vs-dip` sets the cabinet switches. Supported VS metadata can also select controller swaps or A/B wiring changes inside the emulated cabinet. In dual mode, player 3 and player 4 input comes from the third and fourth connected game controllers.

## Datach barcode

`--barcode DIGITS` performs the configured Datach scan after a compatible cartridge loads. F8 starts that same scan again. The value must contain exactly 8 or 13 decimal digits.

## Palette controls

F7 opens or closes the runtime palette overlay. Click one of the 64 swatches to open its color picker, then use the saturation/value area or hue strip to edit that color. F6 restores Cupid's built-in 64-color palette.

Ctrl+V accepts either 64 six-digit RGB tokens or raw hexadecimal bytes for a 192-byte or 1536-byte palette. Token prefixes may be `#`, `0x`, or `$`. Raw input ignores non-hex separators.

Dropping a file on the application tries to load it as palette data. A 192-byte file contains 64 RGB triplets. A 1536-byte file contains eight 64-color emphasis tables. Other sizes display a palette-load error.

Palette edits affect the normal PPU color lookup. VS rendering uses the palette mapping selected by its emulated VS PPU model.
