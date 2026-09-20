# HD packs

[Documentation index](README.md)

Tools > HD Packs discovers packs under `hd-packs` in the application data folder.
A pack is a directory containing `hires.txt` or a self-contained ZIP. A directory
or ZIP named after the loaded image is a discovery candidate; packs can also
identify compatible images with a SHA-1 declaration. Archive members and patched
images use the identity of the materialized game image.

Select a pack and enable it in the panel. Use Browse beside Pack ZIP file or
Pack source folder to select the installation source. The two controls share
the same source path. Installed ZIP name is the filename to use below the
game's pack folder; Validate and install pack checks and writes that ZIP. Rescan refreshes the list. The selected pack and enabled state are saved
per image in the application data folder and restored on the next launch.
A missing or invalid saved pack reports an error and leaves ordinary video
available. Failed pack switches preserve the currently loaded pack.

The renderer supports format 109, PNG replacement tiles and backgrounds,
conditions, additions, fallback tiles, layer priority, alpha blending, and scales
from 1 through 10. WAV and Ogg Vorbis assets provide replacement music and sound
effects through the pack audio registers. The alternate register range is
supported. Restoring a state or replay timeline stops enhanced-audio voices
from the previous timeline. Memory conditions use side-effect-free reads. Rendering observes the
completed PPU frame and does not change sprite evaluation, mapper clocks, or
light-gun sensing.

Pack overscan, the selected regional crop, layer visibility, dual-screen output,
and mouse aiming use the resulting display dimensions. Screenshots and video
capture can include the displayed HD output. Missing replacement tiles retain
the ordinary image unless the pack explicitly disables original tiles.

Export current pack creates a self-contained ZIP. To create a starting pack from
the game, enable Arm HD pack capture, let a complete frame run, select a capture
ZIP path, and choose Capture completed frame as pack. The resulting definition
and tile sheet can be edited outside the emulator. Capture does not change the
active pack.

Only format 109 is accepted. Unsupported tags and malformed assets produce
errors rather than partial loads. Pack-provided ROM patches and the
`disableSpriteLimit` option are rejected because they change emulated behavior;
apply supported IPS, UPS, or BPS patches through the image-opening path instead.
Asset paths, ZIP entries, decoded sizes, image dimensions, and audio buffers have
bounds. The bundled fixtures exercise loading, malformed assets, rendering,
audio mixing, switching, installation, export, capture, and preference restoration.
