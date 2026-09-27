# Movie preferences

**Settings > Movie preferences** contains the defaults shared by movie playback,
TAS editing, and capture. The status line shows the active session's effective
read-only state and recording mode. Settings are saved with the application
configuration. The panel distinguishes recording defaults from current session
status: choosing Insert does not start recording.

| Setting | Default | Effect |
| --- | --- | --- |
| At movie end | Pause at final frame | Pause at the end, or stop and restore the preceding live game |
| Read-only playback default | On | Protect an opened TAS movie from edits; changing it during playback updates the active TAS session |
| Insert recorded TAS frames | Off | Use overwrite or insert for subsequent recording takes |
| Start recordings from power-on | Off | Select current-state or power-on startup for supported recording workflows |
| Show movie subtitles | On | Draw scheduled subtitle text over gameplay |
| Subtitle duration | 180 movie frames | End a subtitle at this age, or when the next subtitle starts |
| Game input/frame/status displays | Off | Show selected overlays in one of four corners |
| Capture overlays | All off | Independently include input, subtitles, frame number, and status in AVI or GIF |
| Automatic movie and TAS backups | On | Preserve the previous file before replacement |
| Retained backups | 5 | Keep the most recent 1 through 100 versions |
| Backup directory | Empty | Save beside the movie; a custom directory must already exist |

Read-only and recording-mode changes are rejected while a TAS frame or seek is
in progress. Finish that operation and apply the setting again. Enabling
read-only ends an active recording take. New recording sessions retain their
recording access even when the playback default is read-only.

## Subtitle timing

FM2 subtitle records have the form `subtitle FRAME TEXT`, where FRAME is a
nonnegative, zero-based movie frame. Cupid displays the latest record whose
start frame has been reached. An empty TEXT clears the display. If records have
the same start frame, the last record in the file wins.

The display follows the completed input row. It does not use host elapsed time:
pause retains the text, frame advance moves it once, and seeking recalculates
it from the destination frame. The display duration is measured in movie frames,
so 180 frames have different durations in NTSC and PAL playback.

Subtitle compilation accepts up to 100000 records, each with fewer than 512 bytes
of UTF-8 text. Overflowing frame numbers, invalid UTF-8, control characters, and
malformed records disable subtitle rendering for that track and report an error.
The original movie metadata remains intact. Editing and exporting preserve the
records. The display wraps text into the three lines reserved near the bottom of
the game image; text that exceeds that space is clipped. Glyphs absent from the bundled font use a question-mark replacement.

## Recovering a previous movie

For a movie named `run.fm2`, the newest local backup is `run.fm2.bak.001`, followed
by `.bak.002`, and so on. The backup's filesystem modification time records when
the recovery copy was written. A custom backup directory uses the original
basename plus a stable hash of the destination path, followed by the same numbered
suffix. Movies with the same basename in different directories therefore use
separate backup sets. Changing the spelling of a destination path can create a
separate set in a custom directory.

Cupid stages the new file before it reads and preserves the old one. It rotates
recovery copies from oldest to newest, replaces each copy atomically, and only
then atomically replaces the active movie. A write or backup failure leaves the
active file unchanged. Interrupted replacement leaves either the prior active
file or its recovery copy available. Partially staged temporary files can remain
after forced termination. Old backups above a lowered retention limit are pruned
after a successful save; deletion failures may leave extra recovery copies.

To recover, copy the desired backup to a new `.fm2`, `.cmv`, or `.ctas` path and
open that copy. The numbered backups contain the original file bytes, including
movie metadata and native TAS project data. Keep a copy before experimenting
with an older version.

The same backup policy applies to native movies, FM2 export, and TAS project saves.
Disabling it still uses atomic replacement but no longer creates recovery copies.
Existing files larger than 512 MiB are rejected by the bounded backup reader;
disable backups only if replacing such a file without a recovery copy is intended.

[Capture controls and encoders](capture.md) describes recording formats and the
AVI/RIFF inspector.
