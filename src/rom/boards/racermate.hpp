/*
 * racermate.hpp - Racermate cartridge memory and periodic interrupt
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
#ifndef CUPID_BOARDS_RACERMATE_HPP
#define CUPID_BOARDS_RACERMATE_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Racermate final : public Board {
    uint16_t _irqCounter = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x1000; }
    uint32_t GetChrRamSize() override { return 0x10000; }
    uint32_t GetSaveRamSize() override { return 0; }
    bool ForceChrBattery() override { return !IsNes20(); }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        _irqCounter = 0;
        if (!IsNes20()) _saveChrRamSize = _chrRamSize / 2;
        SelectPrgPage(1, static_cast<uint16_t>(-1));
        SelectChrPage(0, 0);
    }

    void ProcessCpuClock() override {
        if (--_irqCounter == 0) {
            _irqCounter = 1024;
            SetIrq(true);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0xC000) == 0x8000) {
            SelectPrgPage(0, (value >> 6) & 3);
            SelectChrPage(1, value & 15);
        } else {
            _irqCounter = 1024;
            SetIrq(false);
        }
    }
};

} // namespace cupid::boards
#endif
