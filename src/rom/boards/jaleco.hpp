/*
 * jaleco.hpp - Jaleco cartridge register wiring
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
#ifndef CUPID_BOARDS_JALECO_HPP
#define CUPID_BOARDS_JALECO_HPP
#include "runtime.hpp"

namespace cupid::boards {

class JalecoJf13 final : public Board {
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0x7FFF; }
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override { SelectPrgPage(0, 0); }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0x7000) == 0x6000) {
            SelectPrgPage(0, (value & 0x30) >> 4);
            SelectChrPage(0, (value & 3) | ((value >> 4) & 4));
        }
        // The speech device at $7000-$7FFF is not emulated.
    }
};

} // namespace cupid::boards
#endif
