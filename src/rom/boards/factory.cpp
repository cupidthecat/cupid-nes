/*
 * factory.cpp - Cartridge board selection
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "runtime.hpp"
#include "bandai.hpp"

namespace cupid::boards {

std::unique_ptr<Board> CreateBoard(unsigned mapper) {
    switch (mapper) {
        case 70: return std::make_unique<Bandai74161>(false);
        case 152: return std::make_unique<Bandai74161>(true);
        default: return nullptr;
    }
}

} // namespace cupid::boards

bool board_handles_mapper(unsigned mapper) {
    switch (mapper) {
        case 70: case 152: return true;
        default: return false;
    }
}
