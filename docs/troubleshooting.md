# Troubleshooting

[Documentation index](README.md)

Run Cupid from a terminal so its startup and error messages remain visible. Keep the exact command, commit, image SHA-256, operating system, and compiler version when reporting a failure. The [contribution guide](../CONTRIBUTING.md#reporting-a-bug) lists the useful report fields.

## Build errors

| Message or symptom | Check |
| --- | --- |
| `SDL2/SDL.h` cannot be found on Linux | Install the SDL2 development package, such as `libsdl2-dev` on Ubuntu |
| `Missing SDL2 VC SDK file` on Windows | Pass the extracted VC development directory containing `include/SDL.h` and `lib/x64/SDL2.lib` and `SDL2.dll` |
| `clang` cannot be found | Make the Clang executable available in the build shell, or pass its path with `-Compiler` |
| Windows linker or SDK libraries are missing | Use the environment supplied by the installed Windows SDK/MSVC build tools |
| `SDL2.dll` is missing when launching | Keep the copied DLL beside the executable in the script's output directory |
| A build behaves differently after changing compiler flags | Rebuild Linux objects with `make clean` before changing compilers or `CFLAGS` |
| Sanitized Windows executable lacks its runtime | Use `-Sanitize` with an installed Clang runtime and keep the copied runtime DLL beside the binary |

The Windows build script uses the x64 VC SDL library, not a MinGW import library. The [setup guide](getting-started.md) gives the expected directory layout. If a strict compiler warning fails the build, include that warning in the report instead of removing `-Werror` from the acceptance checks.

## Image loading

| Error | Meaning and next check |
| --- | --- |
| `open` or `Failed to read ROM file` | Check the path, file access, and current directory; quote paths with spaces |
| `Invalid iNES signature` | The file is not recognized as an unpacked iNES/NES 2.0 image; check whether it is an archive or a disk image |
| `Truncated iNES header`, trainer, or payload | The header and actual file length do not agree |
| `Unsupported mapper/submapper` | Compare both numbers with the supported board variants |
| `Unsupported ROM/RAM size` or `Unsupported RAM layout` | The selected board cannot represent the declared geometry |
| `Nonvolatile RAM declared without the battery flag` | The image metadata is inconsistent |
| `Unsupported VS System configuration` | Check the console, PPU, wiring, timing, and mapper fields in the image header |
| `Invalid FDS disk image or BIOS` | Check BIOS length, image layout, side size, and declared side count |

Do not repeatedly edit header bits until a ROM loads. Capture the original header and compare it with the board's known metadata. The [hardware guide](hardware.md) explains why a listed mapper can still reject a particular layout.

The positional image path selects one game at startup. Dropping a ROM onto the window invokes the palette loader and can produce a palette error. Relaunch with the image path. `--fds-bios` selects disk loading; simply naming a disk file on the ordinary cartridge command does not do so.

## Input does not respond

The keyboard controls player 1. Other players need connected SDL game controllers and an appropriate multiplayer or dual-VS configuration. Controller axes are not mapped; use the D-pad. Connect controllers in the order needed for the available slots, and check whether SDL recognizes each device as a game controller.

A Four Score requires ordinary pads on both ports. A Famicom adapter and another expansion device cannot occupy the same connector. Use the [configuration table](configuration.md) to check the combination.

When Family BASIC is selected, typing in the game window goes to the emulated keyboard, including R and F6/F7. With a floor mat selected, R becomes a pad position. These keys will not perform the ordinary shortcuts in those configurations.

For paddles, use horizontal mouse position and the left button. For a light gun, aim over the game image and use left click; right click supplies an off-screen trigger. VS Zapper images remain unsupported. See [controls and peripherals](controls.md) for complete mappings.

## Sound or display problems

`Warning: audio disabled` means SDL could not open the requested audio device. The application continues without sound. Keep SDL's accompanying error text and check the host device before investigating emulated channel state.

A window or renderer creation error is reported separately. The frontend requests an accelerated SDL renderer, so a headless session is not equivalent to running the desktop application. Use `accuracy-tests` for hardware checks that do not require an interactive window.

To distinguish a palette change from a rendering defect, restore the default palette with F6 when ordinary shortcuts are active. Palette files and pasted text have their own size and parsing requirements. VS games use their hardware-selected palettes.

For a visual regression, record when the first wrong frame appears and include a screenshot or diagnostic PPM where available. A blank frame by itself does not identify whether the fault is CPU execution, mapper banking, PPU timing, or a display problem.

## Saves and disks

Cartridge save files are derived from the image basename and live beside it. Check the directory's write access and whether the image has been renamed. Some boards append extra device memory to a `.sav`, and several use a different extension.

FDS writes replace the loaded image. A failed disk save keeps the application open so the dirty media can be retried. Check write protection, directory access, and an existing `.cupid-fds.tmp` file. Preserve a leftover temporary file and move it aside only after checking that no second instance is using that media.

Tape recording saves can be retried with F11 while the capture remains in memory. A failed tape save at shutdown does not keep the process open. Save with F11 before closing. The [save guide](saves.md) distinguishes the guarantees and limits of each persistence path.

## A test does not report success

The ordinary diagnostic runner expects a known result protocol. Some older tests report only on screen or assume writable cartridge RAM without enabling it. Use the documented mode for that collection; an absent signature is not a pass.

Verify the test ROM revision and AccuracyCoin hash before comparing results. A screenshot of the result screen alone does not replace the runner's checks of all 144 descriptors and the final tally. A timeout or skipped result still needs investigation.

When CI fails, first identify the failing job, commit, command, and assertion. A Linux sanitizer failure can reveal a problem that a normal Windows build does not exercise. Reproduce the same compiler flags and data pins from [development and testing](development.md), and include the failure log rather than rerunning until one attempt happens to pass.
