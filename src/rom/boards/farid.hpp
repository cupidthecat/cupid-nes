/*
 * farid.hpp - Farid multicart outer banks and reset latches
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
#ifndef CUPID_BOARDS_FARID_HPP
#define CUPID_BOARDS_FARID_HPP
#include "mmc1.hpp"

namespace cupid::boards {

class FaridSlrom final : public Mmc1Board {
    uint8_t _outerBank = 0;
    bool _locked = false;

    void InitMapper() override {
        AddRegisterRange(0x6000, 0x7FFF, MemoryOperation::Write);
        Mmc1Board::InitMapper();
    }

    void Reset(bool softReset) override {
        Mmc1Board::Reset(softReset);
        _outerBank = 0;
        _locked = false;
        UpdateState();
    }

    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type) override {
        Mmc1Board::SelectPrgPage(slot, _outerBank | (page & 7), type);
    }

    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type) override {
        Mmc1Board::SelectChrPage(slot, (_outerBank << 2) | (page & 31), type);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0x8000) {
            Mmc1Board::WriteRegister(address, value);
        } else if (!_wramDisable && !_locked) {
            _outerBank = (value & 0x70) >> 1;
            _locked = (value & 8) != 0;
            UpdateState();
        }
    }
};

class FaridUnrom final : public Board {
    uint8_t _reg = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool HasBusConflicts() override { return true; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 7);
        SelectChrPage(0, 0);
    }

    void Reset(bool softReset) override {
        _reg = softReset ? (_reg & 0x87) : 0;
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        if (!(_reg & 8) && !(_reg & 0x80) && (value & 0x80))
            _reg = (_reg & 0x87) | (value & 0x78);
        _reg = (_reg & 0x78) | (value & 0x87);
        uint8_t outer = (_reg & 0x70) >> 1;
        SelectPrgPage(0, (_reg & 7) | outer);
        SelectPrgPage(1, 7 | outer);
    }
};

} // namespace cupid::boards
#endif
