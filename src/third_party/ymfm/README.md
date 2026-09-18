# ymfm

This directory contains the OPN, SSG, and ADPCM portions of Aaron Giles's ymfm sound-chip engine. Cupid NES uses its YMF288 model for EPSM expansion sound. The original copyright and BSD 3-Clause license notices remain in each source file and in [LICENSE](LICENSE).

The C interface, CPU bus timing, firmware loading, and application audio integration live in [`src/apu/epsm.cpp`](../../apu/epsm.cpp). The chip engine is compiled as C++17; the NES core remains C11.

Unused parameter names are omitted where the arguments are not used, so the engine builds with the project's warning-as-error checks. These edits do not change the chip behavior.
