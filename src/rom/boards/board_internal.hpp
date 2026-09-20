/*
 * board_internal.hpp - C++ cartridge board ownership details
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef CUPID_BOARD_INTERNAL_HPP
#define CUPID_BOARD_INTERNAL_HPP

#include "runtime.hpp"

struct CartridgeBoard {
    std::vector<uint8_t> ownedPrg;
    std::unique_ptr<cupid::boards::Board> instance;
};

#endif
