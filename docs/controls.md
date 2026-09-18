# Controls and peripherals

[Documentation index](README.md)

The SDL application updates host input in [src/main.c](../src/main.c). [The controller layer](../src/joypad/joypad.c) turns those states into the serial data and signal bits that the emulated machine reads. Select devices with the [command-line options](configuration.md) before opening a game.

## Standard keyboard controls

| Key | Action |
| --- | --- |
| Z | Player 1 A |
| X | Player 1 B |
| Right Shift | Player 1 Select |
| Enter | Player 1 Start |
| Arrow keys | Player 1 D-pad |
| R | Soft reset |
| M, held | Microphone signal for the original Famicom wiring |
| F7 | Toggle the palette editor |
| F6 | Restore the default palette |
| Ctrl+V | Paste palette text |

Close the window to exit normally. The microphone key supplies an input signal; the frontend does not capture a host microphone. Family BASIC and mat devices consume some keyboard events before the normal shortcuts, as described below.

## Game controllers and player assignment

SDL-recognized game controllers fill the first available player slots as they are opened. Controllers connected at startup are opened before the main loop; later connection and removal events update the slots. Removing a controller clears its held buttons.

| Host controller button | Emulated button |
| --- | --- |
| A / B | A / B |
| Back / Start | Select / Start |
| D-pad | Up, Down, Left, Right |

The frontend handles controller buttons and the D-pad. It has no analog-stick mapping or interactive remapping screen. The keyboard drives player 1; another player needs another connected controller. Player 1's keyboard and controller both update the same button state, so avoid using them simultaneously for the same button.

The input layer has six host player slots. Their visibility to software depends on the selected hardware:

| Configuration | Routing |
| --- | --- |
| Normal ports | Players 1 and 2 |
| `--adapter four-score` | Players 1 and 3 on the first serial report, players 2 and 4 on the second |
| `--console famicom --adapter famicom-2` | Built-in players 1/2, with players 3/4 on expansion data lines |
| `--console famicom --adapter famicom-4` | Built-in players 1/2, with players 3 through 6 in the expansion adapter's reports |
| Dual VS | Players 1/2 on the main cabinet and players 3/4 on the secondary cabinet |

The Famicom adapter slot count describes the wiring exposed by the emulator. Each game decides which reports to use.

## Arkanoid paddle

Use `--port1 arkanoid`, `--port2 arkanoid`, or `--console famicom --expansion arkanoid`, according to the cartridge. Horizontal mouse position controls the paddle and the left mouse button is fire. The frontend supplies the same mouse state to all paddle slots.

The paddle reports a latched position through its device protocol. Moving a gamepad stick does not move it.

## Light gun

For an NES light gun, start with:

```sh
./cupid-nes --port2 zapper "game.nes"
```

Move the mouse over the game image to aim. Left click fires at that position. Right click fires with the aim treated as off screen, which provides the off-screen trigger behavior some games use to reload.

The sensor reads recently rendered bright pixels around the aim point. Its default radius is zero; `--zapper-radius N` changes that radius. The Famicom version uses `--console famicom --expansion zapper`. VS Zapper wiring is a different protocol and remains unsupported.

## Power Pad and Family Trainer

Choose `--port2 power-pad-a` or `power-pad-b` for the Power Pad, or `--console famicom --expansion family-trainer-a` or `family-trainer-b` for Family Trainer. Host keys represent three rows of four positions:

```text
1  2  3  4
Q  W  E  R
A  S  D  F
```

These are the left-to-right positions viewed from side A. Side B reverses each row in the device mapping. The same keys update every active mat slot. While a mat is selected, R is a pad position and does not trigger the normal reset shortcut.

## Family BASIC keyboard and tape

`--console famicom --expansion family-basic` connects the keyboard matrix. Ordinary letters, digits, arrows, shifts, punctuation, and F1 through F8 map to BASIC keys. The less direct mappings on a US keyboard are:

| Host key | BASIC key |
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

Keyboard events in the game window go to BASIC while this device is selected. R is a letter, F6/F7 are BASIC keys, and Escape is a BASIC key rather than an exit shortcut.

For a configured tape path, F10 starts playback or recording and F11 stops the tape and saves a recording. Start the corresponding operation in BASIC first. [Saves and media](saves.md) explains the raw tape format and how to retry a recording save before closing.

## Disk-system keys

| Key | Action when a disk image is loaded |
| --- | --- |
| F8 | Eject the inserted side, or insert the selected side |
| F9 | Select and insert the next side, wrapping at the last side |
| F10 | Toggle write protection |

Use the game's disk prompts to decide when to change sides. Ejection itself does not flush the image or acknowledge a pending disk IRQ. Quit normally to save modified media. The [disk guide](saves.md) explains what happens when a save fails.

## VS cabinet keys

| Key | Cabinet input |
| --- | --- |
| 5 / 6 | Main cabinet coin slots 1 / 2 |
| 7 / 8 | Secondary cabinet coin slots 3 / 4, in dual mode |
| F1 | Main service input |
| F2 | Secondary service input, in dual mode |

`--vs-dip` sets the cabinet switches. Each VS image selects its controller wiring and protection behavior from its header. Player 3/4 controls come from the third and fourth connected game controllers, not extra keyboard bindings.

## Datach and palette input

For a Datach cartridge, `--barcode DIGITS` performs the configured scan after loading. F8 scans it again. Accepted input is 8 or 13 decimal digits.

Palette files contain 192 bytes for 64 RGB colors or 1536 bytes for the emphasis palettes. Drop one onto the application, or use Ctrl+V with 64 `RRGGBB` tokens or raw palette hex bytes. The palette tool is a display facility; it is not a ROM loader or a way to change a cartridge's mapper. VS rendering uses its selected hardware palette.
