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
#include "mmc3_96.hpp"
#include "mmc3_97.hpp"
#include "mmc3_98.hpp"
#include "ntdec.hpp"
#include "racermate.hpp"
#include "taito.hpp"
#include "sachen.hpp"
#include "kaiser.hpp"
#include "jy_small.hpp"
#include "nintendo.hpp"
#include "drip_game.hpp"
#include "rainbow.hpp"
#include "unlicensed_109.hpp"

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
        case 38: return std::make_unique<UnlPci556>();
        case 39: return std::make_unique<Mapper39>();
        case 42: return std::make_unique<Mapper42>();
        case 43: return std::make_unique<Mapper43>();
        case 46: return std::make_unique<ColorDreams46>();
        case 50: return std::make_unique<Mapper50>();
        case 51: return std::make_unique<Bmc51>();
        case 53: return std::make_unique<Supervision>();
        case 54: return std::make_unique<NovelDiamond>();
        case 57: return std::make_unique<Mapper57>();
        case 58: return std::make_unique<Mapper58>();
        case 59: return std::make_unique<UnlD1038>();
        case 70: return std::make_unique<Bandai74161>(false);
        case 86: return std::make_unique<JalecoJf13>();
        case 104: return std::make_unique<GoldenFive>();
        case 114: return std::make_unique<Mmc3_114>();
        case 115: return std::make_unique<Mmc3_115>();
        case 121: return std::make_unique<Mmc3_121>();
        case 123: return std::make_unique<Mmc3_123>();
        case 126: return std::make_unique<Mmc3_126>();
        case 134: return std::make_unique<Mmc3_134>();
        case 165: return std::make_unique<Mmc3_165>();
        case 182: return std::make_unique<Mmc3_182>();
        case 187: return std::make_unique<Mmc3_187>();
        case 196: return std::make_unique<Mmc3_196>();
        case 197: return std::make_unique<Mmc3_197>();
        case 198: return std::make_unique<Mmc3_198>();
        case 199: return std::make_unique<Mmc3_199>();
        case 205: return std::make_unique<Mmc3_205>();
        case 208: return std::make_unique<Mmc3_208>();
        case 215: return std::make_unique<Mmc3_215>();
        case 217: return std::make_unique<Mmc3_217>();
        case 219: return std::make_unique<Mmc3_219>();
        case 224: return std::make_unique<Mmc3_224>();
        case 238: return std::make_unique<Mmc3_238>();
        case 245: return std::make_unique<Mmc3_245>();
        case 249: return std::make_unique<Mmc3_249>();
        case 250: return std::make_unique<Mmc3_250>();
        case 254: return std::make_unique<Mmc3_254>();
        case 258: return std::make_unique<Unl158B>();
        case 259: return std::make_unique<Mmc3_BmcF15>();
        case 260: return std::make_unique<BmcHpxx>();
        case 262: return std::make_unique<Mmc3_StreetHeroes>();
        case 263: return std::make_unique<Mmc3_263>();
        case 268: return std::make_unique<Mmc3_268>();
        case 287: return std::make_unique<Mmc3_287>();
        case 292: return std::make_unique<Mmc3_292>();
        case 313: return std::make_unique<Mmc3_313>();
        case 325: return std::make_unique<Mmc3_325>();
        case 333: return std::make_unique<Mmc3_333>();
        case 348: return std::make_unique<Mmc3_348>();
        case 366: return std::make_unique<Mmc3_366>();
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
        case 35: return std::make_unique<Jy35>();
        case 91: return std::make_unique<Jy91>();
        case 284: return std::make_unique<DripGame>();
        case 682: return std::make_unique<Rainbow>();
        default: return nullptr;
    }
}

std::unique_ptr<Board> CreateFcnsBoard() { return std::make_unique<FnsMmc1>(); }

} // namespace cupid::boards

bool board_handles_mapper(unsigned mapper) {
    switch (mapper) {
        case 29: return true;
        case 168: return true;
        case 552: return true;
        case 133: case 136: case 137: case 138: case 139: case 141:
        case 143: case 145: case 147: case 148: case 149: case 150: return true;
        case 243: case 513: return true;
        case 35: case 91: return true;
        case 284: return true;
        case 682: return true;
        case 56: case 142: case 171: case 175: case 302: case 303:
        case 305: case 306: case 307: case 312: case 346: return true;
        case 41: case 63: case 112: case 174: case 193: case 221: case 290: case 298: return true;
        case 6: case 8: case 17: return true;
        case 323: case 324: return true;
        case 31: case 70: case 86: case 104: case 152: case 188: case 218: return true;
        case 12: case 14: case 37: case 44: case 45: case 47: case 49: case 52:
        case 114: case 115: case 121: case 123: return true;
        case 126: case 134: case 165: case 182: case 187: case 196:
        case 197: case 198: case 199: case 205: case 208: case 215: return true;
        case 217: case 219: case 224: case 238: case 245: case 249:
        case 250: case 254: case 258: case 259: case 260: case 262: return true;
        case 263: case 268: case 287: case 292: case 313: case 325:
        case 333: case 348: case 366: return true;
        case 38: case 39: case 42: case 43: case 46: case 50: case 51:
        case 53: case 54: case 57: case 58: case 59: return true;
        default: return false;
    }
}

bool board_is_fcns_header(const iNESHeader *header) {
    if (!header || (header->flags7 & 0x0C) != 0x08) return false;
    return (header->flags7 & 3u) == 3u && (header->zero[2] & 0x0Fu) == 0x0Cu;
}

bool board_handles_header(const iNESHeader *header) {
    return board_is_fcns_header(header)
        || (header && board_handles_mapper((unsigned)rom_mapper_number(header)));
}
