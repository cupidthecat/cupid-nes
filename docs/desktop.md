# Desktop interface

[Documentation index](README.md)

Launch Cupid without an image to open the startup window. Use Open, press
Ctrl+O, drop a game image, or select a recent entry. Recent entries retain the
archive member, patch, firmware selection, and save identity needed to reopen
that image. A failed replacement leaves the current machine available.

The toolbar provides Open, Pause/Resume, Reset, and Settings. Menus group the
shared emulator commands and feature panels. Alt+F opens the menu bar; arrow
keys select a menu or command, Enter activates it, and Escape closes it. The
status bar shows the title, effective region, execution state, active capture
or movie, and disk activity where applicable. Netplay supplies its connection
status while listening or connected.

The startup screen, menus, settings, feature panels, palette editor, and text
entry use Clay layouts with cached TrueType text. Controls follow the layout
when the window or display scale changes. No separate font installation is
needed.

Settings, game information, the palette editor, and feature panels open in separate resizable windows.
Drag their title bars to move them beside the game or onto another display.
Opening a debugger or inspection tool leaves emulation running. A breakpoint,
Pause, or a stepping command can still stop it. Moving focus between Cupid
windows does not trigger the pause-on-focus-loss preference.

## Settings

Ctrl+Comma opens Settings. The eight categories cover General, Emulation,
Video, Audio, Controllers, Media and firmware, Files and storage, and Advanced
hardware. Arrow keys select and change a row. Ctrl+Tab changes categories;
Tab moves among rows, buttons, and the category list. Enter starts text or
binding entry and commits entered text. Click the minus/plus buttons to adjust
a value, or click its value field. Text fields start with their contents selected;
typing replaces them. Ctrl+A selects all, Ctrl+C copies the selection, Ctrl+V
pastes, and End lets you append. Escape cancels an edit or closes the
window. F4 opens a picker for supported file-path rows. Lists scroll to keep
the selected row visible.

Apply validates and saves changes while leaving the window open. OK applies
and closes it. Cancel discards unapplied edits. Defaults resets the current
category; Reset all requires a second activation before replacing the staged
settings. Explicit launch options retain precedence over saved preferences.
The settings window indicates when those overrides are present.

Display, mixing, speed, and binding changes apply to the running session.
Rewind speed selects 1 through 30 retained frames per step. Holding the rewind
binding repeats steps at the emulated frame rate. A menu step pauses on the
restored frame; Resume continues from there. Applying unrelated settings keeps
the retained history.
Timing changes take effect on reload; startup alignment, RAM initialization,
and VS DIP changes take effect on power cycle. Firmware changes require a
restart. The application reports these boundaries after Apply. Invalid firmware,
conflicting connectors, failed output paths, or a failed settings write keep
the preceding configuration. Stop a deterministic session before applying
settings, and stop recording before changing its output configuration.

Text entry and dialog navigation are captured by the interface. They do not
reach the game. Leaving the game window releases held keyboard input. Controller removal
releases that player's held buttons. Mouse aiming uses the actual game rectangle
after menus, aspect scaling, overscan, HD rendering, and dual-display layout.

![Settings in its own window](images/desktop-settings-window.png)

## Feature panels

| Task | Location |
| --- | --- |
| Save slots and state files | File commands and Tools > Save States |
| Rewind, run-ahead, input movies | Tools > Rewind, Run-Ahead and Movies |
| Direct network sessions | Tools > Netplay |
| Breakpoints, registers, memory, hardware inspection, Lua | Tools debugger panels |
| Cheat list and code entry | Tools > Cheats |
| Track selection, playback, repeat, shuffle | Music player panel |
| Disk sides, insertion, write protection, tape, barcodes, cabinet input | Media device panels |
| PNG screenshots, WAV audio, AVI video | Capture panel |
| Replacement graphics/audio, pack install/export/capture | Tools > HD Packs |
| Effective paths and database selection | Tools > Storage locations |
| Loaded image and effective hardware | Help > Game information |
| Recent application errors and notices | Help > Recent messages |

Panels use Up/Down or Tab to select a row and Enter to activate it. Text rows
accept paths or values, choice rows cycle through their entries, and long panels
scroll. A disabled action belongs to hardware or a session that is unavailable;
clicking an unavailable menu item explains the requirement. Disk commands need
a loaded FDS image; tape commands need the Family BASIC keyboard selected as
the expansion device. Click an action row or its Edit, Toggle, or Run button;
minus and plus buttons move through choices. Long values stay within their
columns, with the selected value shown below the rows.

View > Palette editor (F7) opens a separate window with all 64 colors. Select a swatch, drag an RGB
slider, or use its minus/plus buttons. Load palette opens a `.pal` file; Reset
palette restores the default colors. Ctrl+V accepts palette text while the
editor is open. Escape or Close returns to the game.

![Debugger in its own window](images/desktop-debugger-window.png)

## Frame and audio performance

Settings and menus follow the pause-on-UI preference. This automatic pause is
separate from manual pause, so closing a window or returning from another app
does not undo a manual pause. Returning to the game resets its pacing deadline
and audio output buffer. Selecting a speed ends held or toggled fast-forward;
VSync is suspended at speeds other than 100%. Tool windows refresh at most
30 times per second and do not wait for VSync.

Rewind snapshots lock audio state briefly without pausing and restarting the
output device every frame. Snapshot checksums process eight bytes at a time;
the saved-state format and checksum value are unchanged. The desktop caches
glyphs in a texture atlas instead of drawing each character as individual
pixel rectangles. The FPS counter measures completed emulated frames over
elapsed wall time, including pacing delays.

To measure uncapped frontend work with a local ROM:

```powershell
.\build\windows\accuracy-tests.exe --benchmark-frontend 600 "C:\Games\game.nes"
```

This diagnostic measures 600 frames with rewind disabled and with ten seconds
of history configured, then times the desktop renderer separately. It uses
SDL's dummy audio/video drivers and a software renderer. It does not write
cartridge saves, measure display latency, or establish how a physical audio
device sounds. Run it without other heavy work for a useful comparison.

Storage locations shows the configuration directory, cartridge save identity,
disk overlay, state and movie paths, three capture paths, current patch, HD root,
and database. Browse changes supported output or database paths; Open Folder
opens the effective directory. Cartridge identities and active disk overlays are
read-only here. Use `--data-dir` at launch to select a different application data
root. Existing saves are not relocated or deleted by an output-path change.

## Rendered examples and validation

These images come from the production SDL renderer driven by the desktop
regression harness. They use synthetic session labels and contain no game ROM
artwork. They are automated render checks, not records of manual desktop use.

![Startup window](images/desktop-startup.png)

![Main window](images/desktop-main.png)

[Palette editor](images/desktop-palette.png) · [Text entry](images/desktop-text-entry.png)

| Category | Render |
| --- | --- |
| General | [View](images/desktop-category-0.png) |
| Emulation | [View](images/desktop-category-1.png) |
| Video | [View](images/desktop-category-2.png) |
| Audio | [View](images/desktop-category-3.png) |
| Controllers | [View](images/desktop-category-4.png) |
| Media and firmware | [View](images/desktop-category-5.png) |
| Files and storage | [View](images/desktop-category-6.png) |
| Advanced hardware | [View](images/desktop-category-7.png) |

The event tests cover keyboard navigation, category defaults and cancellation,
scrolling, mouse targets at three scales, palette changes, text replacement,
window-close events during dialogs, uninterrupted audio-device state during
snapshot locks, settings-write rollback, storage selection, tape
transport, and barcode input. Render checks exercise [100%](images/desktop-settings.png),
[150%](images/desktop-settings-150.png), and [200%](images/desktop-settings-2x.png) layout
and game rectangles. Run just the desktop checks with
`accuracy-tests --desktop`. The broader hardware suite covers regional and dual-system
execution. Manual Windows/Linux checks with physical controllers, native file
dialogs, display-DPI changes, and representative games remain separate acceptance
work; automated rendering does not establish those results.
