/*
 * nina.hpp - NINA-001 and BNROM cartridge wiring
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
#ifndef CUPID_BOARDS_NINA_HPP
#define CUPID_BOARDS_NINA_HPP
#include "runtime.hpp"

namespace cupid::boards {

class NinaBnrom final : public Board {
    bool IsNina() const {
        return _romInfo.SubMapperID == 1 || (_romInfo.SubMapperID == 0 && HasChrRom());
    }

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return IsNina() ? 0x1000 : 0x2000; }
    uint16_t RegisterStartAddress() override { return IsNina() ? 0x7FFD : 0x8000; }
    uint16_t RegisterEndAddress() override { return IsNina() ? 0x7FFF : 0xFFFF; }
    bool HasBusConflicts() override { return !IsNina(); }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        if (!IsNina()) SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (!IsNina()) {
            SelectPrgPage(0, value);
            return;
        }

        switch (address) {
            case 0x7FFD: SelectPrgPage(0, value); break;
            case 0x7FFE: SelectChrPage(0, value); break;
            case 0x7FFF: SelectChrPage(1, value); break;
        }
        WritePrgRam(address, value);
    }
};

} // namespace cupid::boards
#endif
