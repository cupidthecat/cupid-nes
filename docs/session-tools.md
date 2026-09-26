# Sessions and recovery

Cupid opens its desktop without a game. Use File to open an image or choose a recent game from the startup view. Help tools remain available while the machine is off.

## Game configuration

Open **Tools > Game Configuration** while a game is loaded. Select a setting to see its inherited global value and whether it comes from global preferences, a saved override, or the command line. Enter a value using the format shown beside the field, then choose **Save override for next load**. **Reset to inherited value** removes that override. Reopen the image to apply either change.

Overrides cover selected region and hardware settings, controllers, firmware and disk options, listening volume, video presentation, shader selection, audio backend, and the cheat file. An empty cheat-file override disables automatic cheats for that image. Other settings keep their global values. Command-line choices take precedence for that launch. Cupid disables override editing during deterministic playback or recording.

Cupid identifies configurations by the loaded image's content hash and archive member. Patches affect the content hash. Renaming an ordinary image preserves its overrides; a different archive member or patched image has its own configuration. Saving global preferences does not copy game overrides into them.

## Resume a session

On a clean exit with a game loaded, Cupid saves an automatic state and its image-opening information. **File > Load Last Session** reopens that image and restores the state. In **Tools > Session and State Recorder**, enable **Resume last session on startup** to do this automatically. The default is off. An explicit image or movie on the command line takes priority.

Missing images, changed media, incompatible states, and corrupt recovery files produce an error. Failed startup recovery leaves the desktop available. A failed restore while another game is running attempts to restore that previous session. Recovery files use the normal core state compatibility checks.

Session files are replaced atomically. If a required capture, media, peripheral, or session save fails during exit, Cupid keeps the application open so you can correct the destination and retry. Exiting from the startup view preserves the last saved session.

## Power off and unload

**File > Power Off / Unload Game** returns to the startup view without closing Cupid. It finalizes captures, saves writable cartridge, disk, and peripheral data, and releases the active machine and game presentation resources. Recent games remain available. If a required save fails, the game stays loaded and Cupid reports the failure. Stop deterministic play before unloading.

## State recorder

Open **Tools > Session and State Recorder**. Choose an interval in frames or emulated milliseconds, set how many snapshots to retain, and enable **Recorder running**. **Start recorder when a game loads** controls automatic startup. Defaults are automatic startup off, an interval of 3,600 frames, and 10 snapshots. Retention accepts 1 through 64 snapshots.

Only completed live emulation frames advance the interval. Pause time, rewind, and wall-clock time do not count. Reset, power cycle, and state restoration restart the interval. Each game has a separate history. Select an entry in the oldest-first list and choose **Restore selected snapshot**, or use **Record snapshot now** for an immediate recovery point.

Recorder files are separate from numbered save slots and rewind. A capture writes a spare file, commits the history index, and then removes unreferenced recorder files. An interrupted write keeps the committed history. A failed snapshot or index write stops recording and reports the error. Cleanup retries when that game's history is loaded again.

## Updates and command-line help

**Help > Check for Updates** shows the running version and latest release. Checks run in a worker while the desktop remains responsive. Offline use, rate limits, and invalid responses appear as non-fatal results. **Open release page** opens the release in your browser; installing it is manual.

Automatic checking is off by default. When enabled, it checks once at startup. **Acknowledge this release**, or opening its release page, saves the acknowledgement so automatic checks do not keep announcing the same version. Development builds without a release version show `0.0.0-dev` and report that version ordering is unavailable.

**Help > Command Line Help** lists application switches, accepted values, defaults, aliases, and descriptions from the same definitions used by the parser. Search matches those fields without regard to case. Use Tab to select controls, Enter to edit search, and the list's navigation or Previous/Next buttons to browse. The reference works without a game. `cupid-nes --help` prints it in a terminal.

## Files

These files live in Cupid's configured data directory, which can be selected with `--data-dir`:

| File | Contents |
| --- | --- |
| `<identity>.game.ini` | Per-game overrides |
| `<identity>.session` | Automatic state and image-opening metadata |
| `last-session` | Identity of the last saved session |
| `session-tools.ini` | Resume and recorder preferences |
| `<identity>.recorder-index` | Committed recorder history |
| `<identity>.recorder-NN.cstate` | Reserved recorder snapshots |
| `updates.ini` | Automatic checking preference and acknowledged release |

The identity is a 40-character hash. Keep the automatic state and its referenced image, patch, and firmware files together when backing up a session. Recovery does not relocate missing files automatically.
