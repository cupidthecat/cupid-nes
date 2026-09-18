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
#include "codemasters.hpp"
#include "homebrew.hpp"
#include "jaleco.hpp"
#include "ffe.hpp"
#include "farid.hpp"
#include "ntdec.hpp"

namespace cupid::boards {

std::unique_ptr<Board> CreateBoard(unsigned mapper) {
    switch (mapper) {
        case 29: return std::make_unique<SealieComputing>();
        case 41: return std::make_unique<Caltron41>();
        case 63: return std::make_unique<Ntdec63>();
        case 112: return std::make_unique<Ntdec112>();
        case 174: return std::make_unique<Ntdec174>();
        case 193: return std::make_unique<NtdecTc112>();
        case 221: return std::make_unique<Ntdec221>();
        case 290: return std::make_unique<NtdecNtd03>();
        case 298: return std::make_unique<Tf1201>();
        case 6: case 8: case 17: return std::make_unique<FrontFareast>();
        case 31: return std::make_unique<NsfCartridge>();
        case 70: return std::make_unique<Bandai74161>(false);
        case 86: return std::make_unique<JalecoJf13>();
        case 104: return std::make_unique<GoldenFive>();
        case 152: return std::make_unique<Bandai74161>(true);
        case 218: return std::make_unique<MagicFloor>();
        case 323: return std::make_unique<FaridSlrom>();
        case 324: return std::make_unique<FaridUnrom>();
        default: return nullptr;
    }
}

} // namespace cupid::boards

bool board_handles_mapper(unsigned mapper) {
    switch (mapper) {
        case 29: return true;
        case 41: case 63: case 112: case 174: case 193: case 221: case 290: case 298: return true;
        case 6: case 8: case 17: return true;
        case 323: case 324: return true;
        case 31: case 70: case 86: case 104: case 152: case 218: return true;
        default: return false;
    }
}
