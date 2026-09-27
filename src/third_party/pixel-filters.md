# Pixel scaling components

The Video pixel filters use these bundled components:

- [xBRZ](xbrz/xbrz.cpp), by Zenju, under GPL-3.0 with the retained linking exception.
- [HQx](hqx/hqx.h), by Maxim Stepin, Cameron Zemek, and Francois Gannaz, under LGPL-2.1-or-later.
- [Scale2x and Scale3x](scale2x/scale2x.h), by Andrea Mazzoleni, under GPL-2.0-or-later. This distribution uses GPL-3.0.
- [2xSaI, Super2xSaI, and SuperEagle](sai/SaiEagle.h), with the retained RetroArch notices for Hans-Kristian Arntzen and Daniel De Matteis, under GPL-3.0-or-later.

Copyright notices remain in the source files, and each directory contains its
license text. The imports use standard integer headers instead of a shared
precompiled header. HQx initializes the white lookup-table entry and converts
negative chroma through signed integers before applying its offset.
xBRZ leaves two unused parameter names commented out for strict warning builds.

Cupid filters each VS screen separately. Scale4x uses two Scale2x passes with
owned scratch storage so allocation failures reach the frontend. The row
wrapper repeats edge rows and handles a one-column crop directly. LCD cell
brightness and integer prescaling are implemented in Cupid's presentation
layer. None of these transforms writes to emulated video memory.
