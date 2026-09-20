# Archive codecs

The image loader builds the ZIP reader from the miniz 3.1.2 amalgamation in
[`miniz`](miniz). Its copyright and permission notice are in
[`miniz/LICENSE`](miniz/LICENSE).

The 7z reader uses the C decoder files from 7-Zip 26.03 in [`lzma`](lzma).
The selected source files retain their individual public-domain notices. The
distribution's full [license text](lzma/LICENSE.txt) and [C decoder notes](lzma/7zC.txt)
are also included. This subset supports LZMA, LZMA2, PPMd, and the included branch
and delta filters. It does not include password handling or every codec in the
full archive application.

`lzma/CpuArch.h` has one local portability change: native integer loads and stores
use fixed-size `memcpy` calls. Archive byte offsets need not have integer
alignment, even on processors that support unaligned machine instructions. The
change also avoids accessing byte storage through an incompatible integer type.
The endian-conversion macros keep their original behavior. Both regular and
sanitizer builds use the same accessors.

Trailing whitespace and extra blank lines at the end of imported files were
removed. The original notices and license text are retained.

The application reads archives directly from bounded memory buffers. It neither
launches an external decompressor nor extracts member names onto the filesystem.
The host wrappers in [`src/media`](../media) enforce image, entry, name, and
allocation limits before passing selected bytes to the production image loaders.
