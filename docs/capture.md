# Screenshots and recordings

Cupid saves screenshots as PNG images, audio as stereo PCM WAV files, and video
as AVI files containing raw or lossless ZMBV video and stereo PCM audio. Animated
GIF recording is also available. These files
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
and other desktop controls are excluded. Game overlays are optional, as described
below.

PNG preserves the framebuffer's RGBA pixels. Raw AVI stores bottom-up 24-bit RGB
frames without alpha. ZMBV stores lossless RGB in independently decodable
keyframes. Its video stream uses the active region's rational frame
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
that limit quickly. ZMBV usually reduces the size, especially for flat colors. When the next complete frame would exceed the limit, Cupid
finalizes the frames already written and reports why recording stopped. A file
write or replacement failure reports an error and preserves the running game
and any previous destination file.

The focused capture tests decode PNG chunks and pixels, inspect WAV sample
metadata and channel order, and verify AVI frame data and index offsets. They
also exercise regional sample counts, pauses and speed changes, music track
changes, EPSM stereo, dual VS output, protected paths, and failed writes through
the production capture and frontend APIs.

## Encoding and animated GIF

Open **Tools > Capture encoding** to select AVI or animated GIF. AVI offers
**Uncompressed RGB24** and **ZMBV lossless RGB32**. Compression levels run from
0 through 9; the default is 6. Level 0 uses uncompressed deflate blocks within
ZMBV. Every encoded frame is a keyframe. This keeps seeking and recovery simple,
but does not compress repeated frames as much as an encoder with interframes.
Both AVI codecs use the same PCM audio stream and frame clock.

For GIF, enter **GIF output path**, choose 1x, 2x, 3x, or 4x nearest-neighbor
scaling, and select **Record animated GIF**. The ordinary Stop Recording command
finalizes either format. The output path must identify a writable file location;
GIF path entry currently uses the text field in the encoding panel.

GIF has no audio. Cupid converts RGB to a fixed 256-color RGB332 palette and
compresses each frame with LZW. A fractional accumulator converts the region's
frame rate to GIF's hundredths-of-a-second delays without accumulating rounding
drift. No gameplay frames are dropped. Some GIF viewers clamp very short delays,
so their playback can be slower than the timing stored in the file. Use AVI when
exact timing in a video player matters. GIF dimensions cannot exceed 4096 pixels
in either direction after scaling.

## Game overlays and capture overlays

Open **Settings > Movie preferences** to show controller inputs, a frame counter,
or movie/TAS status over gameplay. Choose any of the four screen corners for
these displays. Subtitle display has its own checkbox. Window scaling and
fullscreen scale the composed game image together with the overlay.

The controller display snapshots the emulated buttons after authoritative frames,
including movie and TAS injection. Pausing retains the preceding frame's input;
frame advance updates it once. Rejected host input during playback does not
replace recorded input in the overlay.

The four **Capture** overlay checkboxes separately select controller inputs,
movie subtitles, frame counter, and movie/TAS status for AVI and GIF. They are
all off by default. On-screen visibility does not implicitly enable capture
content. Overlays are composed into a separate pixel buffer before encoding;
GIF scaling happens afterward. Screenshots continue to use the clean source.

See [Movie preferences](movie-preferences.md) for subtitle timing, movie defaults,
and recovery copies.

## AVI / RIFF Inspector

Open **Tools > AVI / RIFF Inspector**, enter a path, and choose **Inspect file**.
The panel shows nested RIFF/LIST chunks, file offsets, declared sizes, video and
audio stream formats, frame rates, stream lengths, and legacy `idx1` entries.
Index entries are checked against their target chunk headers. The inspector
opens files read-only and does not repair them.

A truncated header, an oversized chunk, missing padding, a short stream header,
or a mismatched index entry produces a warning beside the affected row. Unknown
chunks remain visible. Inspection stops at 8192 rows or 16 nested containers and
reports that limit. Stream formats it cannot interpret are labeled as unknown;
OpenDML index chunks are shown without decoding their entries.
