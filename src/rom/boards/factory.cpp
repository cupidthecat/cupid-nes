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
#include "irem.hpp"
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
#include "unlicensed_111.hpp"
#include "unlicensed_112.hpp"
#include "txc_107.hpp"
#include "unlicensed_113.hpp"
#include "unlicensed_114.hpp"
#include "unlicensed_110.hpp"
#include "unlicensed_115.hpp"
#include "unif.hpp"
#include "../unif.h"
#include "waixing_116.hpp"
#include "whirlwind_117.hpp"

namespace cupid::boards {

std::unique_ptr<Board> CreateBoard(unsigned mapper) {
    switch (mapper) {
        case 29: return std::make_unique<SealieComputing>();
        case 36: return std::make_unique<Txc22000>();
        case 40: return std::make_unique<Whirlwind40>();
        case 60: return std::make_unique<Unl60>();
        case 41: return std::make_unique<Caltron41>();
        case 61: return std::make_unique<TxcMapper61>();
        case 62: return std::make_unique<Unl62>();
        case 63: return std::make_unique<Ntdec63>();
        case 83: return std::make_unique<Unl83>();
        case 103: return std::make_unique<Unl103>();
        case 106: return std::make_unique<Unl106>();
        case 107: return std::make_unique<Unl107>();
        case 108: return std::make_unique<Unl108>();
        case 112: return std::make_unique<Ntdec112>();
        case 116: return std::make_unique<Unl116>();
        case 117: return std::make_unique<Unl117>();
        case 120: return std::make_unique<Unl120>();
        case 125: return std::make_unique<Lh32>();
        case 132: return std::make_unique<Txc22211A>();
        case 156: return std::make_unique<Unl156>();
        case 162: return std::make_unique<Waixing162>();
        case 163: return std::make_unique<Unl163>();
        case 164: return std::make_unique<Waixing164>();
        case 174: return std::make_unique<Ntdec174>();
        case 176: return std::make_unique<Fk23C>();
        case 178: return std::make_unique<Waixing178>();
        case 168: return std::make_unique<Racermate>();
        case 172: return std::make_unique<Txc22211B>();
        case 173: return std::make_unique<Txc22211C>();
        case 189: return std::make_unique<Mmc3_189>();
        case 193: return std::make_unique<NtdecTc112>();
        case 221: return std::make_unique<Ntdec221>();
        case 242: return std::make_unique<Waixing242>();
        case 252: return std::make_unique<Waixing252>();
        case 253: return std::make_unique<Waixing253>();
        case 286: return std::make_unique<WaixingBs5>();
        case 290: return std::make_unique<NtdecNtd03>();
        case 298: return std::make_unique<Tf1201>();
        case 299: return std::make_unique<Bmc11160>();
        case 304: return std::make_unique<Smb2j>();
        case 309: return std::make_unique<Lh51>();
        case 522: return std::make_unique<Lh10>();
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
        case 166: case 167: return std::make_unique<Subor166>(mapper == 166);
        case 170: return std::make_unique<Mapper170>();
        case 177: return std::make_unique<Henggedianzi177>();
        case 179: return std::make_unique<Henggedianzi179>();
        case 190: return std::make_unique<MagicKidGooGoo>();
        case 200: return std::make_unique<Mapper200>();
        case 201: return std::make_unique<NovelDiamond>();
        case 202: return std::make_unique<Mapper202>();
        case 203: return std::make_unique<Mapper203>();
        case 204: return std::make_unique<Mapper204>();
        case 212: return std::make_unique<Mapper212>();
        case 213: return std::make_unique<Mapper213>();
        case 214: return std::make_unique<Mapper214>();
        case 216: return std::make_unique<Mapper216>();
        case 222: return std::make_unique<Mapper222>();
        case 225: return std::make_unique<Mapper225>();
        case 226: return std::make_unique<Mapper226>();
        case 227: return std::make_unique<Mapper227>();
        case 228: return std::make_unique<ActionEnterprises>();
        case 229: return std::make_unique<Mapper229>();
        case 230: return std::make_unique<Mapper230>();
        case 231: return std::make_unique<Mapper231>();
        case 233: return std::make_unique<Mapper233>();
        case 234: return std::make_unique<Mapper234>();
        case 235: return std::make_unique<Bmc235>();
        case 236: return std::make_unique<Bmc70in1>();
        case 240: return std::make_unique<Mapper240>();
        case 241: return std::make_unique<Mapper241>();
        case 244: return std::make_unique<Mapper244>();
        case 246: return std::make_unique<Mapper246>();
        case 255: return std::make_unique<Bmc255>();
        case 261: return std::make_unique<Bmc810544CA1>();
        case 264: return std::make_unique<Yoko>();
        case 265: return std::make_unique<T262>();
        case 266: return std::make_unique<CityFighter>();
        case 274: return std::make_unique<Bmc80013B>();
        case 283: return std::make_unique<Gs2004>();
        case 285: return std::make_unique<A65AS>();
        case 288: return std::make_unique<Gkcx1>();
        case 289: return std::make_unique<Bmc60311C>();
        case 300: return std::make_unique<Bmc190in1>();
        case 301: return std::make_unique<Bmc8157>();
        case 314: return std::make_unique<Bmc64in1NoRepeat>();
        case 319: return std::make_unique<Hp898f>();
        case 320: return std::make_unique<Bmc830425C4391T>();
        case 328: return std::make_unique<Rt01>();
        case 329: return std::make_unique<Edu2000>();
        case 331: return std::make_unique<Bmc12in1>();
        case 332: return std::make_unique<Super40in1Ws>();
        case 336: return std::make_unique<BmcK3046>();
        case 349: return std::make_unique<BmcG146>();
        case 487: return std::make_unique<Mapper487>();
        case 518: return std::make_unique<Dance2000>();
        case 519: return std::make_unique<Eh8813A>();
        case 521: return std::make_unique<DreamTech01>();
        case 529: return std::make_unique<T230>();
        case 530: return std::make_unique<Ax5705>();
        case 70: return std::make_unique<Bandai74161>(false);
        case 77: return std::make_unique<IremLrog017>();
        case 80: return std::make_unique<TaitoX1005>(false);
        case 82: return std::make_unique<TaitoX1017>();
        case 86: return std::make_unique<JalecoJf13>();
        case 104: return std::make_unique<GoldenFive>();
        case 74: return std::make_unique<Mmc3ChrRam>(0x08, 0x09, 2);
        case 119: return std::make_unique<Mmc3ChrRam>(0x40, 0x7F, 8);
        case 191: return std::make_unique<Mmc3ChrRam>(0x80, 0xFF, 2);
        case 192: return std::make_unique<Mmc3ChrRam>(0x08, 0x0B, 4);
        case 194: return std::make_unique<Mmc3ChrRam>(0x00, 0x01, 2);
        case 195: return std::make_unique<Mmc3ChrRam>(0x00, 0x03, 4);
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
        case 207: return std::make_unique<TaitoX1005>(true);
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
        case UNIF_BOARD_MALEE: return std::make_unique<UnifMalee>();
        case UNIF_BOARD_GS2013: return std::make_unique<UnifGs2013>();
        case UNIF_BOARD_GHOSTBUSTERS_63IN1: return std::make_unique<UnifGhostbusters63in1>();
        case UNIF_BOARD_CC21: return std::make_unique<UnifCc21>();
        case UNIF_BOARD_AC08: return std::make_unique<UnifAc08>();
        case UNIF_BOARD_PUZZLE: return std::make_unique<UnifPuzzle>();
        case UNIF_BOARD_255IN1: return std::make_unique<Unif255in1>();
        case UNIF_BOARD_8237A: return std::make_unique<Unif8237A>();
        case UNIF_BOARD_SSS_NROM_256: return std::make_unique<UnifFamicomBox>();
        default: return nullptr;
    }
}

std::unique_ptr<Board> CreateFcnsBoard() { return std::make_unique<FnsMmc1>(); }

} // namespace cupid::boards

bool board_handles_mapper(unsigned mapper) {
    switch (mapper) {
        case 29: return true;
        case 36: case 61: case 132: case 172: case 173: case 189: case 299: return true;
        case 40: case 125: case 304: case 309: case 522: return true;
        case 60: case 62: case 83: case 103: case 106: case 107: case 108:
        case 116: case 117: case 120: case 156: case 163: return true;
        case 162: case 164: case 176: case 178: case 242: case 252: case 253: case 286: return true;
        case 80: case 82: case 168: case 207: return true;
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
        case 31: case 70: case 74: case 77: case 86: case 104: case 119:
        case 152: case 188: case 191: case 192: case 194: case 195: case 218: return true;
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
        case 166: case 167: case 170: case 177: case 179: case 190:
        case 200: case 201: case 202: case 203: case 204: case 212: return true;
        case 213: case 214: case 216: case 222: case 225: case 226: case 227:
        case 228: case 229: case 230: case 231: case 233: return true;
        case 234: case 235: case 236: case 240: case 241: case 244:
        case 246: case 255: case 261: case 264: case 265: case 266: return true;
        case 274: case 283: case 285: case 288: case 289: case 300:
        case 301: case 314: case 319: case 320: case 328: case 329: return true;
        case 331: case 332: case 336: case 349: case 487:
        case 518: case 519: case 521: case 529: case 530: return true;
        case UNIF_BOARD_MALEE: case UNIF_BOARD_GS2013: case UNIF_BOARD_GHOSTBUSTERS_63IN1:
        case UNIF_BOARD_CC21: case UNIF_BOARD_AC08: case UNIF_BOARD_PUZZLE:
        case UNIF_BOARD_255IN1: case UNIF_BOARD_8237A: case UNIF_BOARD_SSS_NROM_256: return true;
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
