/*
 * codemasters.hpp - Golden Five cartridge banking
 *
 * Copyright (C) 2014-2026 Sour and contributors
 * Copyright (C) 2026 Francis Hagan
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_BOARDS_CODEMASTERS_HPP
#define CUPID_BOARDS_CODEMASTERS_HPP
#include "runtime.hpp"

namespace cupid::boards {

class GoldenFive final : public Board {
    uint8_t _prgRegister = 0;

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        _prgRegister = 0;
        SelectPrgPage(1, 15);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0xC000) {
            _prgRegister = (_prgRegister & 0xF0) | (value & 15);
            SelectPrgPage(0, _prgRegister);
        } else if (address <= 0x9FFF && (value & 8)) {
            _prgRegister = (_prgRegister & 15) | ((value << 4) & 0x70);
            SelectPrgPage(0, _prgRegister);
            SelectPrgPage(1, ((value << 4) & 0x70) | 15);
        }
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("golden_five.prg_register", _prgRegister);
    }
};

} // namespace cupid::boards
#endif
