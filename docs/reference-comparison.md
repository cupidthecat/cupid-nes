# NES reference comparison

[Documentation index](README.md)

This review compares the local reference sources with PR #206 at
`20a76b184335f6093df619c19ddaa25e6c1778e5`, plus the fixes below. It covers
the listed repositories, their main desktop workflows, and representative NES
hardware paths. Runtime checks exercise Cupid. The reference emulators were
inspected as source; they were not all built or played through. This is not a
claim of complete feature parity or an exhaustive bug inventory.

## Reference snapshots

Paths are relative to each reference repository. Revisions identify the local
snapshots used, rather than a moving upstream release.

| Repository | Revision | Source inspected and its role |
| --- | --- | --- |
| [Ares](https://github.com/ares-emulator/ares/tree/234c99911) | `234c99911` | `desktop-ui/presentation/presentation.cpp`, `desktop-ui/input/hotkeys.cpp`, `ares/fc/ppu/scroll.cpp`: window sizing, aspect choices, slots, held actions, and PPU scroll transfers. |
| [FCEUX](https://github.com/TASVideos/fceux/tree/8cc753da) | `8cc753da` | `src/drivers/Qt/ConsoleWindow.cpp`, `src/drivers/sdl/input.cpp`: desktop commands, TAS, Lua, memory tools, recording, fullscreen controls, and joystick axes. |
| [Mesen2](https://github.com/SourMesen/Mesen2/tree/b9fa69dd) | `b9fa69dd` | `UI/ViewModels/MainMenuViewModel.cs`, `UI/Utilities/LoadRomHelper.cs`, `UI/Utilities/ShortcutHandler.cs`, `Core/Shared/KeyManager.cpp`, `Core/Shared/SaveStateManager.cpp`: states, archives, patches, history, input, and save/load lifecycle. |
| [MesenCE](https://github.com/nesdev-org/MesenCE/tree/4b8a196e) | `4b8a196e` | Its menu and ROM-loading sources were inspected separately: numbered states, history, debugger/viewers, HD-pack prompts, and netplay controller selection. Similar source is not independent runtime evidence. |
| [Nestopia JG](https://gitlab.com/jgemu/nestopia/-/tree/b3415a2) | `b3415a2` | `jg.cpp` and the API organization under `src/`: audio/video information, crop/aspect conversion, settings, states, and cheats. Its host supplies the desktop UI. |
| [Nestopia UE](https://github.com/0ldsk00l/nestopia/tree/7b5c87d) | `7b5c87d` | `source/win32/NstManagerInput.cpp`, `NstManagerRecentFiles.cpp`, `NstManagerSaveStates.cpp`: peripheral choices, held actions, recent files, and numbered states in the Win32 frontend. |
| [Nintendulator](https://github.com/quietust/nintendulator/tree/1409c6a) | `1409c6a` | `src/Nintendulator.rc`, `src/Controllers.cpp`, `src/PPU.cpp`: command availability, reset/power, stepping, axes, CPU/PPU tools, and sprite evaluation. |
| [Nintendulator mappers](https://github.com/quietust/nintendulator-mappers/tree/c6e48bb) | `c6e48bb` | `src/iNES/mapper004.cpp` and the `Hardware`, `UNIF`, `FDS`, `NSF`, `Vs` organization: state, MMC3/MMC6 separation, and PPU-cycle callbacks. There is no standalone desktop UI. |
| [puNES](https://github.com/punesemu/puNES/tree/b1758ec2) | `b1758ec2` | `src/gui/mainWindow.cpp`, `wdgScreen.cpp`, `recentFiles.cpp`: focus cleanup, keyboard/mouse routing, numbered slots, rewind transport, fullscreen chrome, recent files, and tape formats. |

## Features and workflows

"Present" means a production implementation and UI or CLI entry point were
found in Cupid. It does not mean every mode was tested against every reference.
Reference names are examples used for comparison, not exclusive support lists.

| Area | Reference behavior | Cupid implementation and result |
| --- | --- | --- |
| Media lifecycle | Mesen2/CE, Nestopia UE, Nintendulator separate open, reload, unload. | Present in `session_actions.c` and `lifecycle_frontend.c`, including replacement rollback and persistent-data checks. |
| Recent games | Ares, Mesen2/CE, Nestopia UE, puNES retain recent choices. | Present, including archive members and patches. Native menus track history revisions. Startup Open now honors its configured binding. |
| Archives and patches | Mesen2/CE offers member selection and patch association. | ZIP/7z and IPS/UPS/BPS are present. Multi-image archives use paged SDL buttons. |
| Quick states | Mesen2/CE and Nestopia UE expose numbered slots; Nintendulator has previous/next slot. | Slots and explicit file dialogs are present. File > Save states > Quick state slot now selects any of ten slots and marks the selected slot. |
| Recovery and history | Mesen2/CE has history viewing; FCEUX has a recorder and TAS tools. | Session recovery, automatic snapshots, rewind playback, bookmarks, previews, and range exports are present, separately from quick slots. |
| Execution controls | Nintendulator and Mesen distinguish pause, reset, power, stepping. | Present. Automatic suspension is separate from manual pause. Peripheral keys now precede application shortcuts. |
| Speed and rewind | Ares and Nestopia UE track held actions; FCEUX supports axes. | Keyboard/controller holds now have separate ownership. Left-stick movement now contributes D-pad directions alongside digital bindings. |
| Controller assignment | References expose controller configuration and peripherals. | SDL GameController, GUID assignment, six host-player slots, and profiles are present. Release follows the press-time mapping; unplug preserves keyboard input. Arbitrary axis rebinding remains a gap. |
| Specialty inputs | Nestopia UE/JG and the NES cores support devices beyond a pad. | Family BASIC, Subor, mats, Party Tap, boxing, Mahjong, FCNS, mouse and light-gun paths are present. Focus cleanup now releases peripheral keys, keypad/mouse buttons, and arcade holds. |
| Picture controls | Ares has aspect/integer sizing; puNES has fullscreen/interpolation controls. | Aspect, overscan, integer scaling, palettes, filters, shaders, and fullscreen are present. Desktop bars stay visible in fullscreen. |
| Sound | Reference frontends provide volume/mute and output configuration. | Mixing, panning, muting, host playback buffering, and WAV capture are present. Dummy audio checks do not establish audible device quality. |
| Cheats | Mesen2/CE, FCEUX, Nestopia UE/JG expose cheat management. | Raw/Game Genie entry, finder, catalog loading, and management are present. |
| Debugging | Mesen2/CE and FCEUX provide execution, viewers, watches, search, scripts, tracing. | Debugger, assembler, PPU viewers, hex editor, search/watch, profiler, trace, and Lua paths are present. Rendering checks do not qualify arbitrary scripts. |
| Movies/TAS | FCEUX emphasizes editing; Mesen2/CE has recording/replay. | Native movies, FM2/FM3, TAS projects, editing, and FCM conversion are present, with deterministic ownership restrictions. |
| Capture/HD/netplay | Mesen2/CE and FCEUX offer recording; Mesen2/CE also offers HD packs and netplay. | These have production owners and panels. Network/capture checks are separate from the window-rendering suite. |

## Defects corrected in this pass

- Ordinary NES controllers ignored the left analog stick. Both left axes now
  contribute D-pad directions with a fixed 16,000-unit dead zone on SDL's
  signed axis range. Digital D-pad input remains available.
- Keyboard and gamepad releases cancelled each other's buttons. Inputs now
  combine by source. Disconnecting a controller preserves held keyboard input.
- Remapping a held physical button could strand its old emulated button.
  Keyboard and controller releases now use the press-time action, including
  releases after modifier changes.
- Releasing gamepad fast-forward or rewind cancelled a keyboard hold. Each
  source now retains ownership until release. Release still works after a
  change in deterministic execution policy.
- Focus cleanup omitted peripheral keys and Subor modifier-side bookkeeping.
  Cleanup now clears these inputs without resetting device read protocols.
  Peripheral state snapshots verify return to neutral.
- Application shortcuts preceded emulated keyboard input. Ctrl+R could reset
  instead of reaching Family BASIC or Subor. The game now routes peripheral
  keys first while it owns focus.
- The startup desktop skipped configurable shortcuts because it had no
  execution runtime. Open now uses the configured command path before and
  after loading a game. The hardcoded startup Ctrl+O fallback was removed.
- The game view consumed unbound Tab/Shift+Tab presses, preventing normal GTK
  keyboard focus navigation. These now reach GTK unless a profile binding or
  an emulated keyboard owns the key.
- Rebinding Quick Load exposed a leftover F6 palette-reset handler. The handler
  is removed; palette reset remains available from View > Palette. The controls
  guide now describes the current F6/F7 actions.

The numbered quick-slot menu is a usability addition. Selecting a slot changes
the quick-save/load target without saving or loading a state.

## Hardware and validation boundaries

Representative integration checks included Ares PPU scroll transfers,
Nintendulator sprite evaluation and MMC3/MMC6 callbacks, Nestopia JG crop/aspect
and audio information, and Mesen input/state ownership. Cupid's corresponding
owners are `ppu.c`, `rom/boards`, `joypad.c`, `state_runtime.c`, and the audio/video
frontends. No independently reproduced CPU, PPU, APU, or mapper defect was
established by this comparison. This pass changes host input and desktop behavior.

The focused input regression failed eleven assertions on the original PR
revision and passes with the fixes. Additional coverage checks peripheral focus
cleanup, physical GTK key signals, startup remapping, peripheral shortcut
priority, numbered-slot actions, and their checkmarks.

Run `build/accuracy-tests --host-input` for focused input checks and
`build/gtk-ui-smoke --session-ui-check` for native shortcuts, menus, state dialogs,
and status. The normal hardware suite includes the input checks. The full GTK
suite also renders tool windows at two sizes. The final PR update records local
results and the exact revision tested by CI.

Core diagnostics, sanitizers, startup, packaging, and gameplay are distinct
checks. These results do not establish full commercial-game compatibility,
physical-controller coverage, monitor scanout cadence, audible output quality,
or parity with every reference option.

## Remaining differences

- Mesen2/CE has visual recent-game and state selection. Cupid's idle screen is
  a prompt with a recent-games menu. Bookmark previews are not a quick-state grid.
- Cupid has no searchable archive-member list; Mesen2/CE has a dedicated
  selection window.
- puNES and FCEUX expose fullscreen menu hiding. Cupid retains the menu,
  toolbar, and status bar.
- Cupid profiles bind keys and controller buttons. Per-axis binding,
  adjustable dead zones, and axis-based shortcut capture are not implemented.
- Mapper/device presence is not a complete game-by-game compatibility
  comparison. References differ on configurable hardware behavior; matching
  one default alone is insufficient evidence to change the core.

These remain follow-up work. Rendering the existing panels does not close them.
