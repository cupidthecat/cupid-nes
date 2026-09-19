/*
 * irem.hpp - Irem cartridge boards implemented on the shared runtime
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
#ifndef CUPID_BOARDS_IREM_HPP
#define CUPID_BOARDS_IREM_HPP
#include "runtime.hpp"

namespace cupid::boards {

class IremLrog017 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x0800; }
    uint32_t GetChrRamSize() override { return 0x1800; }
    uint16_t GetChrRamPageSize() override { return 0x0800; }
    uint32_t GetNametableCount() override { return 4; }
    bool HasBusConflicts() override { return true; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
        SetMirroringType(MirroringType::FourScreens);
        SelectChrPage(1, 0, ChrMemoryType::ChrRam);
        SelectChrPage(2, 1, ChrMemoryType::ChrRam);
        SelectChrPage(3, 2, ChrMemoryType::ChrRam);
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, value & 0x0F);
        SelectChrPage(0, (value >> 4) & 0x0F);
    }
};

} // namespace cupid::boards
#endif
