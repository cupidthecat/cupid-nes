# Presentation tools

These tools change host video and audio output. They do not change CPU/PPU timing, controller input, light-gun sensing, or deterministic machine state.

## History Viewer

Open **Tools → Rewind and movies → History Viewer** with a game loaded and rewind enabled. The native GTK window shows the retained history, starting at its oldest frame. Drag the timeline or enter a frame number to seek. Left/Right step one frame; Home/End jump to the boundaries; Space or a click on the video toggles playback. Playback uses the game's regional frame rate, pauses at the end, and starts again at the beginning when Play is pressed. Volume controls historical audio separately. The scale selector resizes the preview while preserving its aspect ratio.

**File → Create save state** saves the selected boundary. **Resume gameplay** continues from it and discards that boundary and all later rewind entries, matching ordinary rewind. Closing the viewer returns to the unchanged live position and preserves the previous pause state.

Use **Set start** and **Set end** to select an inclusive range, then **File → Export movie range** to write a Cupid state-start input movie. The export embeds the first boundary and the accepted controller/peripheral input events for subsequent frames. The first frame's actions are already part of that boundary and are not applied twice. Incomplete frames or input logs disable export for ranges containing them.

The **Bookmarks** sidebar stores ten temporary positions with preview images. Ctrl+0–9 stores a position; 0–9 jumps to one. Bookmarks belong to the open history and are cleared when it is reopened.

Browsing pauses live execution. Each preview temporarily restores an authoritative rewind snapshot, copies its video, then restores the live machine and its completed display frame. The viewer plays retained final stereo samples through a separate output device. It never runs the core, saves battery data, or injects preview inputs. Snapshots, input events and audio share the configured rewind memory limit; shorter retention is possible when that limit is reached. A missing audio device leaves video navigation available.

Movie/netplay ownership and unreplayable host state prevent browsing. Reset, game replacement and rewind reconfiguration invalidate the viewer. Cupid freezes its live timeline while browsing. Movie exports use Cupid's `.movie` format; `.mmo` files are not supported. The SDL fallback retains its separate preview window and shared panel controls.

## Shader Presets

Use **Load GLSL preset (.glslp)** to choose a preset, or **Discover preset directory** to list presets in one directory. **Reload preset** rereads its sources and textures. Select a parameter and enter a value within its declared range. Turn off **Enable GPU preset** for ordinary output.

The GPU path supports up to eight GLSL passes, PNG lookup textures, source/viewport/absolute pass dimensions, nearest or linear input sampling, named parameters, pass aliases, and prior-pass textures. Shader files can include relative files below the preset directory. Parameter values are saved by name, so changing declaration order does not change their meaning.

This implementation requires an OpenGL 3.3 compatibility context. Slang and Cg presets, temporal feedback, mipmaps, floating-point/sRGB targets, and wrapping other than `clamp_to_edge` are rejected with an error. Sources, includes, textures, output dimensions, and parameter counts have explicit limits. A failed load or render disables GPU processing in the video runtime and keeps ordinary output available.

Shaders run after HD rendering and pixel filters. GPU output is read back for both the SDL game texture and displayed-output capture; this readback has a host performance cost. Raw captures and light-gun sensing still use the original machine pixels. Shader effects can move visible pixels, so a heavily distorted shader may make aiming visually awkward even though the normal screen-to-console mapping remains unchanged.

## Native Audio Output

Choose `default`, `wasapi`, or `directsound`, then select an output device. The choices use the corresponding native drivers included in the installed SDL build. The panel also shows the actual active driver.

**Buffer samples** accepts 64–8192. The existing sample-rate setting accepts 8,000–192,000 Hz. Output remains stereo float audio from Cupid's final mix, with the same volume, channel balance, pause, mute, rewind, and session rules. Only shared output is exposed; there is no exclusive-mode control.

A backend change reopens SDL audio. If it fails, Cupid tries the previous driver/device, then the default output. Device removal triggers a reopen on a default device. If recovery fails, emulation continues without an output device until another output is selected. Selecting the system-default endpoint lets the native driver handle default-endpoint changes.

## HD Pack Builder

The builder edits a format-109 definition inside Cupid and imports the assets it references. It is a definition editor with live preview; image drawing still happens in an image editor.

1. Choose **New 2x draft** or open an existing pack ZIP.
2. Select a definition line, edit it, and choose **Apply line**. Use the line number to reach any line; the list shows the current 128-line page. A line number equal to the current count plus one appends a line.
3. Enter an **Asset member name**, such as `tiles.png`, then import its PNG, WAV, OGG, or palette file. Add or edit the definition that references it.
4. Choose **Validate draft**, then **Open / close live preview**. The preview uses the loaded game's completed tile observations and the same renderer as ordinary HD packs.
5. Choose **Export validated pack ZIP**. Export validates again and writes a self-contained archive atomically. Invalid edits leave an existing export untouched.

The editor supports the runtime's tile, background, condition, addition, fallback, overscan, layer-priority, supported-ROM, option, and replacement-audio declarations. Validation uses the production loader, including its path, asset, decoding, and memory limits. Hardware-changing pack options are metadata only, and ROM patch declarations are rejected. Preview never installs the draft or changes the loaded cartridge. Invalid drafts keep the last validated preview until repaired. Imported assets unused by the definition are omitted from the validated export.

## Frame Timing Statistics

The statistics panel retains 240 completed host-frame samples. It shows recent, average, minimum, maximum, and nearest-rank 95th-percentile milliseconds for emulation, rendering, presentation/VSync, pacing delay, and the full frame interval. Emulation FPS is 1,000 divided by the average interval in milliseconds. Jitter is the standard deviation of those intervals: a lower value means more even spacing.

The Display draws row measures when the game viewport draws a newly submitted image.
It has its own FPS, jitter, p95, and maximum interval. Repainting the same image
does not count twice. On Windows the accelerated viewport records completed SDL
presentation calls. The GTK widget path can combine multiple submissions into one draw, so
this rate can be lower than the emulation rate. These measurements do not report
when the monitor displays the image. The SDL frontend marks draw timing unavailable.

The desktop prepares each image, then waits for its deadline before submitting
it to GTK. Variations in emulation and filter time therefore use the available
frame budget instead of moving each image's submission time. The wait services
pending GTK work and uses the high-resolution clock for its final fraction of a
millisecond. Both this wait and any wait after presentation count as pacing time.
GTK drawing during a wait also counts as pacing time.

The main status-bar FPS counter uses the last 60 completed frame intervals,
about one second at normal NTSC speed. It updates four times per second and
starts a new measurement after a pause, session change, or speed change.
It reports the measured rate, including sustained slowdowns and fast-forward;
it does not clamp the number to the regional target.

The native desktop spreads periodic tool updates across game presentations,
rotating through visible windows. Each tool waits at least 100 ms between
these refreshes; with many tools open, updates can be less frequent. Live video
previews continue to update per frame. Buttons and edits still act immediately.
Settings categories retain their controls after
the first visit. Unchanged labels, menu checks, and theme preferences do not
request another layout or redraw. GTK control animations are suspended during
gameplay and restored while paused or idle, following the original animation
preference when the desktop closes.

These are host wall-clock measurements, not emulated CPU or PPU cycles. Idle/pause transitions reset the measurements. The audio row shows the host playback queue and the number of underrun callbacks since the output device was opened. Startup prefill is excluded. Playback retains a short reservoir and adjusts host resampling by at most 0.5% to follow the device clock; emulated clocks, recorded samples, and movie input are unchanged. **Reset measurements** clears both sets of retained samples.

## Existing video controls

The existing filter selection includes None, LCD Grid, xBRZ 2x–6x, HQ2x/3x/4x, Scale2x/3x/4x, 2xSaI, Super2xSaI, SuperEagle, and prescale 2x/3x/4x/6x/8x/10x. LCD Grid expands each source pixel to four cells with separate brightness percentages; its defaults are 100, 85, 85, and 85. Dual-screen filtering processes each screen separately so neighboring pixels do not cross the screen boundary.

Bilinear interpolation controls the final game texture for direct, composite, HD, and filtered output. Debugger inspection textures keep their own sampling rules. NTSC picture settings expose hue, brightness, contrast, saturation, luma/I/Q filter widths, gamma, scanline strength, and field handling, with Default composite, Sharp composite, Soft television, and Monochrome presets. These controls operate on presentation data and preserve the original PPU signal.
