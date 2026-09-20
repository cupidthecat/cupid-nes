# Screenshots and recordings

Cupid saves screenshots as PNG images, audio as stereo PCM WAV files, and video
as AVI files containing uncompressed video and stereo PCM audio. These files
contain the emulator's output. They do not contain the controller input needed
to replay a game.

## Capture controls

| Shortcut | Action |
| --- | --- |
| F12 | Save a screenshot |
| Ctrl+F12 | Start an audio recording |
| Shift+F12 | Start a video recording |
| Ctrl+Shift+F12 | Stop and finalize the recording |

The first capture action opens a file chooser. Subsequent actions use the
selected path until it is changed in the capture panel. The panel has separate
paths for screenshots, audio, and video, along with Browse buttons, output
selection, sample rate, and a file-size limit. One audio or video recording can
run at a time. Screenshots remain available during recording and while paused.

On Windows, the chooser supports Unicode paths. Linux uses `zenity` or
`kdialog`; when neither is installed, enter the output path in the panel. A
cancelled chooser leaves the existing path unchanged. Cupid rejects output paths
that identify the active image, its firmware, or its protected save files,
including another spelling of the same path.

## What is captured

**Displayed output** uses the game image after the selected presentation filter.
Raw output uses the original game framebuffer. A single raw screen is 256 by
240 pixels; dual VS output places both screens side by side at 512 by 240.
NTSC composite output uses the filter's wider output image. Window resizing
does not change capture resolution. Window borders, menus, the palette editor,
and other desktop controls are excluded.

PNG preserves the framebuffer's RGBA pixels. AVI stores bottom-up 24-bit RGB
frames without alpha. Its video stream uses the active region's rational frame
rate. NTSC cartridge recordings use the average frame duration for alternating
odd-frame skips; NSF and VS recordings use their unshortened frame duration.

Audio is 16-bit little-endian stereo PCM. The default sample rate is 44,100 Hz;
the panel also offers 48,000 and 96,000 Hz. Base and cartridge sound use the
production output mix, and EPSM retains separate left and right channels. Dual
VS recordings average both machines into each output channel, matching their
ordinary combined output. Capturing audio reads an emulation-thread observer
and does not remove samples from the host device's audio buffer.

## Timing and interruption

Capture writes one video frame and its corresponding audio interval for each
completed emulated frame. Pausing adds no frames or silence. Frame advance adds
one frame. Fast-forward and slow motion affect how quickly recording progresses
on the host; playback of the saved file uses the normal emulated rate. Music
track changes retain the recording timeline while restarting the track timer.

Reset, power cycle, image replacement, and state changes finalize the current
recording before changing the machine. Rewind and speculative run-ahead frames
are excluded. A change in video dimensions, region, or machine identity also
stops recording. A partially executed frame is discarded when recording stops;
previous complete frames remain in the file.

## File limits and failures

Recordings are written to a temporary file beside the destination. Stop and
normal shutdown finalize the headers and AVI index, then replace the destination
atomically. An existing destination remains intact until that succeeds. A crash
or forced termination can leave a temporary file without replacing the previous
recording.

The default limit is 4,294,967,295 bytes, within the implemented RIFF container
limit. Lower limits can be selected in the panel. Uncompressed video can reach
that limit quickly. When the next complete frame would exceed the limit, Cupid
finalizes the frames already written and reports why recording stopped. A file
write or replacement failure reports an error and preserves the running game
and any previous destination file.

The focused capture tests decode PNG chunks and pixels, inspect WAV sample
metadata and channel order, and verify AVI frame data and index offsets. They
also exercise regional sample counts, pauses and speed changes, music track
changes, EPSM stereo, dual VS output, protected paths, and failed writes through
the production capture and frontend APIs.
