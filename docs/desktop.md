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

## Settings

Ctrl+Comma opens Settings. The eight categories cover General, Emulation,
Video, Audio, Controllers, Media and firmware, Files and storage, and Advanced
hardware. Arrow keys select and change a row. Ctrl+Tab changes categories;
Tab moves among rows, buttons, and the category list. Enter starts text or
binding entry and commits entered text. Escape cancels an edit or closes the
window. F4 opens a picker for supported file-path rows. Lists scroll to keep
the selected row visible.

Apply validates and saves changes while leaving the window open. OK applies
and closes it. Cancel discards unapplied edits. Defaults resets the current
category; Reset all requires a second activation before replacing the staged
settings. Explicit launch options retain precedence over saved preferences.
The settings window indicates when those overrides are present.

Display, mixing, speed, and binding changes apply to the running session.
Timing changes take effect on reload; startup alignment, RAM initialization,
and VS DIP changes take effect on power cycle. Firmware changes require a
restart. The application reports these boundaries after Apply. Invalid firmware,
conflicting connectors, failed output paths, or a failed settings write keep
the preceding configuration. Stop a deterministic session before applying
settings, and stop recording before changing its output configuration.

Text entry and dialog navigation are captured by the interface. They do not
reach the game. Closing a panel releases captured host input. Controller removal
releases that player's held buttons. Mouse aiming uses the actual game rectangle
after menus, aspect scaling, overscan, HD rendering, and dual-display layout.

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
its panel status explains relevant conflicts.

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

![Main window](images/desktop-main.png)

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
scrolling, nested audio locks, settings-write rollback, storage selection, tape
transport, and barcode input. Render checks exercise [100%](images/desktop-settings.png),
[150%](images/desktop-settings-150.png), and [200%](images/desktop-settings-2x.png) layout
and game rectangles. The broader hardware suite covers regional and dual-system
execution. Manual Windows/Linux checks with physical controllers, native file
dialogs, display-DPI changes, and representative games remain separate acceptance
work; automated rendering does not establish those results.
