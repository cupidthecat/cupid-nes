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
#include "mmc3_95.hpp"
#include "ntdec.hpp"
#include "racermate.hpp"
#include "taito.hpp"
#include "sachen.hpp"
#include "kaiser.hpp"

namespace cupid::boards {

std::unique_ptr<Board> CreateBoard(unsigned mapper) {
    switch (mapper) {
        case 29: return std::make_unique<SealieComputing>();
        case 41: return std::make_unique<Caltron41>();
        case 63: return std::make_unique<Ntdec63>();
        case 112: return std::make_unique<Ntdec112>();
        case 174: return std::make_unique<Ntdec174>();
        case 168: return std::make_unique<Racermate>();
        case 193: return std::make_unique<NtdecTc112>();
        case 221: return std::make_unique<Ntdec221>();
        case 290: return std::make_unique<NtdecNtd03>();
        case 298: return std::make_unique<Tf1201>();
        case 6: case 8: case 17: return std::make_unique<FrontFareast>();
        case 31: return std::make_unique<NsfCartridge>();
        case 12: return std::make_unique<Mmc3_12>();
        case 14: return std::make_unique<Mmc3_14>();
        case 37: return std::make_unique<Mmc3_37>();
        case 44: return std::make_unique<Mmc3_44>();
        case 45: return std::make_unique<Mmc3_45>();
        case 47: return std::make_unique<Mmc3_47>();
        case 49: return std::make_unique<Mmc3_49>();
        case 52: return std::make_unique<Mmc3_52>();
        case 70: return std::make_unique<Bandai74161>(false);
        case 86: return std::make_unique<JalecoJf13>();
        case 104: return std::make_unique<GoldenFive>();
        case 114: return std::make_unique<Mmc3_114>();
        case 115: return std::make_unique<Mmc3_115>();
        case 121: return std::make_unique<Mmc3_121>();
        case 123: return std::make_unique<Mmc3_123>();
        case 152: return std::make_unique<Bandai74161>(true);
        case 188: return std::make_unique<BandaiKaraoke>();
        case 218: return std::make_unique<MagicFloor>();
        case 323: return std::make_unique<FaridSlrom>();
        case 324: return std::make_unique<FaridUnrom>();
        case 552: return std::make_unique<TaitoX1017>();
        case 133: case 143: case 145: case 148: case 149: return std::make_unique<SachenDiscrete>();
        case 136: return std::make_unique<SachenJv001>(false);
        case 147: return std::make_unique<SachenJv001>(true);
        case 137: return std::make_unique<Sachen8259>(Sachen8259Variant::D);
        case 138: return std::make_unique<Sachen8259>(Sachen8259Variant::B);
        case 139: return std::make_unique<Sachen8259>(Sachen8259Variant::C);
        case 141: return std::make_unique<Sachen8259>(Sachen8259Variant::A);
        case 150: case 243: return std::make_unique<Sachen74LS374>();
        case 513: return std::make_unique<Sachen9602>();
        case 56: case 142: return std::make_unique<Kaiser202>();
        case 171: return std::make_unique<Kaiser7058>();
        case 175: return std::make_unique<Kaiser7022>();
        case 302: return std::make_unique<Kaiser7057>();
        case 303: return std::make_unique<Kaiser7017>();
        case 305: return std::make_unique<Kaiser7031>();
        case 306: return std::make_unique<Kaiser7016>();
        case 307: return std::make_unique<Kaiser7037>();
        case 312: return std::make_unique<Kaiser7013B>();
        case 346: return std::make_unique<Kaiser7012>();
        default: return nullptr;
    }
}

} // namespace cupid::boards

bool board_handles_mapper(unsigned mapper) {
    switch (mapper) {
        case 29: return true;
        case 168: return true;
        case 552: return true;
        case 133: case 136: case 137: case 138: case 139: case 141:
        case 143: case 145: case 147: case 148: case 149: case 150: return true;
        case 243: case 513: return true;
        case 56: case 142: case 171: case 175: case 302: case 303:
        case 305: case 306: case 307: case 312: case 346: return true;
        case 41: case 63: case 112: case 174: case 193: case 221: case 290: case 298: return true;
        case 6: case 8: case 17: return true;
        case 323: case 324: return true;
        case 31: case 70: case 86: case 104: case 152: case 188: case 218: return true;
        case 12: case 14: case 37: case 44: case 45: case 47: case 49: case 52:
        case 114: case 115: case 121: case 123: return true;
        default: return false;
    }
}
