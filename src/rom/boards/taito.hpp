/*
 * taito.hpp - Taito X1-017 cartridge register and RAM wiring
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
#ifndef CUPID_BOARDS_TAITO_HPP
#define CUPID_BOARDS_TAITO_HPP
#include "runtime.hpp"

namespace cupid::boards {

class TaitoX1017 final : public Board {
    uint8_t _chrMode = 0;
    uint8_t _chrRegs[6]{}, _ramPermission[3]{};
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    uint16_t RegisterStartAddress() override { return 0x7EF0; }
    uint16_t RegisterEndAddress() override { return 0x7EFF; }
    uint32_t GetSaveRamSize() override { return 0x1400; }
    uint32_t GetSaveRamPageSize() override { return 0x400; }

    void UpdateRamAccess() {
        int8_t first = _ramPermission[0] == 0xCA ? ReadWrite : NoAccess;
        int8_t second = _ramPermission[1] == 0x69 ? ReadWrite : NoAccess;
        int8_t third = _ramPermission[2] == 0x84 ? ReadWrite : NoAccess;
        SetCpuMemoryMapping(0x6000, 0x63FF, 0, PrgMemoryType::SaveRam, first);
        SetCpuMemoryMapping(0x6400, 0x67FF, 1, PrgMemoryType::SaveRam, first);
        SetCpuMemoryMapping(0x6800, 0x6BFF, 2, PrgMemoryType::SaveRam, second);
        SetCpuMemoryMapping(0x6C00, 0x6FFF, 3, PrgMemoryType::SaveRam, second);
        SetCpuMemoryMapping(0x7000, 0x73FF, 4, PrgMemoryType::SaveRam, third);
    }

    void UpdateChrBanking() {
        uint16_t paired = _chrMode ? 2 : 0;
        uint16_t single = _chrMode ? 0 : 4;
        SelectChrPage2x(paired, _chrRegs[0] & 0xFE);
        SelectChrPage2x(paired + 1, _chrRegs[1] & 0xFE);
        for (uint16_t reg = 0; reg < 4; ++reg)
            SelectChrPage(single + reg, _chrRegs[reg + 2]);
    }

    void InitMapper() override {
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        UpdateRamAccess();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0x7EF0 && address <= 0x7EF5) {
            _chrRegs[address & 15] = value;
            UpdateChrBanking();
        } else if (address == 0x7EF6) {
            SetMirroringType(value & 1 ? MirroringType::Vertical : MirroringType::Horizontal);
            _chrMode = (value >> 1) & 1;
            UpdateChrBanking();
        } else if (address >= 0x7EF7 && address <= 0x7EF9) {
            _ramPermission[address - 0x7EF7] = value;
            UpdateRamAccess();
        } else if (address >= 0x7EFA && address <= 0x7EFC) {
            uint8_t page = ((value & 0x20) >> 5) | ((value & 0x10) >> 3)
                | ((value & 8) >> 1) | ((value & 4) << 1)
                | ((value & 2) << 3) | ((value & 1) << 5);
            SelectPrgPage(address - 0x7EFA, page);
        }
    }
};

} // namespace cupid::boards
#endif
