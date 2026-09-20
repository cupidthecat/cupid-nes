# Desktop components

The desktop uses [Clay](https://github.com/nicbarker/clay) for layout and SDL2 for
rendering and input. `clay/clay.h` is pinned to
`e6cc36941ab2af5d81107617039d6f527a1c660b`. Its
[zlib/libpng license](clay/LICENSE.md) is retained. `clay.c` enables the
implementation and suppresses warnings in the upstream implementation only.
The vendored header is unchanged.

[stb_truetype](https://github.com/nothings/stb) rasterizes glyphs into a cached
texture atlas. `stb/stb_truetype.h` is pinned to
`2c980bb59875b0d32144a71867fbdebb2f77cd20`; its public-domain/MIT license choices
remain in the header. The wrapper only enables the implementation.

The embedded font is unmodified DejaVu Sans 2.37, distributed by Ubuntu's
`fonts-dejavu-core` package. Its [license and notices](fonts/LICENSE.txt) are
included. The font's SHA-256 is `ae7b7855e115a5966d8b1b3f80f254ccc117ec86f9965e202ee2940453837280`.
`fonts/font_data.h` stores those original bytes compressed with zlib, so the
executable does not depend on an installed font or a working-directory asset.
The renderer displays a replacement glyph when the font lacks a character;
UTF-8 paths and field values retain their original bytes.

Include these license files when distributing binaries. Cupid's layout, input,
and renderer code lives in `src/ui`; it does not modify the layout library.
