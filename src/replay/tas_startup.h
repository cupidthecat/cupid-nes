/* Movie startup profile. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_TAS_STARTUP_H
#define CUPID_TAS_STARTUP_H

#include "fm2.h"
#include "movie.h"

bool nes_tas_rom_md5(uint8_t digest[16]);
bool nes_tas_pin_cheats(NesFm2Movie *movie);
NesMovieResult nes_tas_startup_validate(const NesFm2Movie *movie, char *error, size_t capacity);
bool nes_tas_startup_configure(const NesFm2Movie *movie);
bool nes_tas_startup_power(const NesFm2Movie *movie);

#endif
