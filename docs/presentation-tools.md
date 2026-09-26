# Presentation tools

These tools change host video and audio output. They do not change CPU/PPU timing, controller input, light-gun sensing, or deterministic machine state.

## Rewind History

Open **Rewind History** with a game loaded and rewind enabled, then choose **Open video timeline**. The window shows retained frame boundaries. Click or drag its bottom bar to scrub, use Left/Right to step, and press Space to play or pause the retained history. The position starts at 1 for the oldest retained frame.

Browsing pauses live execution. Each preview restores the selected authoritative rewind snapshot temporarily, copies its video, and restores the live machine. **Save selected state** writes that snapshot as a normal save state. **Resume from selection** restores it and discards the selected boundary and every later rewind entry, matching ordinary rewind. Closing the window leaves the live position unchanged.

Movie export is unavailable because the rewind ring stores snapshots, not the controller input stream needed to reproduce a range. Movie/netplay ownership and unreplayable host state prevent browsing. Reset, game replacement, and rewind reconfiguration close the viewer.

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

The statistics panel retains 240 completed host-frame samples. It shows recent, average, minimum, maximum, and nearest-rank 95th-percentile milliseconds for emulation, rendering, presentation/VSync, pacing delay, and the full frame interval. Effective FPS is 1,000 divided by the average interval in milliseconds.

These are host wall-clock measurements, not emulated CPU or PPU cycles. Idle/pause transitions reset the measurements. Callback audio does not expose trustworthy queue depth or underrun counters, so those fields are marked unavailable. **Reset measurements** clears the retained samples.

## Existing video controls

The existing filter selection includes None, LCD Grid, xBRZ 2x–6x, HQ2x/3x/4x, Scale2x/3x/4x, 2xSaI, Super2xSaI, SuperEagle, and prescale 2x/3x/4x/6x/8x/10x. LCD Grid expands each source pixel to four cells with separate brightness percentages; its defaults are 100, 85, 85, and 85. Dual-screen filtering processes each screen separately so neighboring pixels do not cross the screen boundary.

Bilinear interpolation controls the final game texture for direct, composite, HD, and filtered output. Debugger inspection textures keep their own sampling rules. NTSC picture settings expose hue, brightness, contrast, saturation, luma/I/Q filter widths, gamma, scanline strength, and field handling, with Default composite, Sharp composite, Soft television, and Monochrome presets. These controls operate on presentation data and preserve the original PPU signal.
