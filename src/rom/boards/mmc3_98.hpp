/*
 * mmc3_98.hpp - MMC3-derived multicart and protection boards
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
#ifndef CUPID_BOARDS_MMC3_98_HPP
#define CUPID_BOARDS_MMC3_98_HPP

#include "mmc3.hpp"
extern "C" {
#include "../../cpu/cpu.h"
}

namespace cupid::boards {

class Mmc3_263 final : public Mmc3 {
    void WriteRegister(uint16_t address, uint8_t value) override {
        value = static_cast<uint8_t>((value & 0xD8)
              | ((value & 0x20) >> 4)
              | ((value & 0x04) << 3)
              | ((value & 0x02) >> 1)
              | ((value & 0x01) << 2));
        if (address == 0x9000) address = 0x8001;
        else if (address == 0xD000) address = 0xC001;
        else if (address == 0xF000) address = 0xE001;
        Mmc3::WriteRegister(address, value);
    }
};

class Mmc3_268 final : public Mmc3 {
    uint8_t _exRegs[4]{};

    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint32_t GetChrRamSize() override { return 0x40000; }

    void Reset(bool softReset) override {
        std::memset(_exRegs, 0, sizeof(_exRegs));
        Board::Reset(softReset);
        ResetMmc3();
        UpdateState();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        (void)type;
        uint16_t address = static_cast<uint16_t>(slot * 0x400);
        uint32_t mask = 0xFFu ^ (_exRegs[0] & 0x80u);
        int cbase = _chrMode ? 0x1000 : 0;
        if (_exRegs[3] & 0x10) {
            if (_exRegs[3] & 0x40) {
                switch (cbase ^ address) {
                    case 0x0400:
                    case 0x0C00: page &= 0x7F; break;
                    default: break;
                }
            }
            page = static_cast<uint16_t>((page & 0x80u & mask)
                 | (((_exRegs[0] & 0x08u) << 4) & ~mask)
                 | ((_exRegs[2] & 0x0Fu) << 3)
                 | slot);
            Mmc3::SelectChrPage(slot, page);
        } else {
            if (_exRegs[3] & 0x40) {
                switch (cbase ^ address) {
                    case 0x0000: page = _registers[0]; break;
                    case 0x0800: page = _registers[1]; break;
                    case 0x0400:
                    case 0x0C00: page = 0; break;
                    default: break;
                }
            }
            Mmc3::SelectChrPage(slot, static_cast<uint16_t>((page & mask)
                | (((_exRegs[0] & 0x08u) << 4) & ~mask)));
        }
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        uint16_t address = static_cast<uint16_t>(0x8000 + slot * 0x2000);
        uint32_t mask = ((0x3Fu | (_exRegs[1] & 0x40u)
                       | ((_exRegs[1] & 0x20u) << 2))
                       ^ ((_exRegs[0] & 0x40u) >> 2))
                       ^ ((_exRegs[1] & 0x80u) >> 2);
        uint32_t base = (_exRegs[0] & 0x07u)
                      | ((_exRegs[1] & 0x10u) >> 1)
                      | ((_exRegs[1] & 0x0Cu) << 2)
                      | ((_exRegs[0] & 0x30u) << 2);

        if ((_exRegs[3] & 0x40) && page >= 0xFE && _prgMode) {
            switch (slot) {
                case 1: if (_prgMode) page = 0; break;
                case 2: if (!_prgMode) page = 0; break;
                case 3: page = 0; break;
                default: break;
            }
        }

        if (!(_exRegs[3] & 0x10)) {
            Mmc3::SelectPrgPage(slot, static_cast<uint16_t>(((base << 4) & ~mask)
                                 | (page & mask)), type);
        } else {
            mask &= 0xF0;
            uint8_t emask = (_exRegs[1] & 0x02)
                          ? static_cast<uint8_t>((_exRegs[3] & 0x0C)
                            | ((address & 0x4000) >> 13))
                          : static_cast<uint8_t>(_exRegs[3] & 0x0E);
            Mmc3::SelectPrgPage(slot, static_cast<uint16_t>(((base << 4) & ~mask)
                                 | (page & mask) | emask | (slot & 0x01)), type);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (GetState().regA001 & 0x80) WritePrgRam(address, value);
            if ((_exRegs[3] & 0x90) != 0x80) {
                _exRegs[address & 0x03] = value;
                UpdateState();
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_268.exRegs", _exRegs);
    }
};

class Mmc3_287 final : public Mmc3 {
    uint8_t _exReg = 0;

    uint32_t GetDipSwitchCount() override { return 1; }

    void InitMapper() override {
        AddRegisterRange(0x6000, 0xFFFF, MemoryOperation::Write);
        _exReg = 0;
        Mmc3::InitMapper();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, static_cast<uint16_t>(page | ((_exReg & 0x03) << 7)), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_exReg & (0x08 | (GetDipSwitches() << 2))) {
            Mmc3::SelectPrgPage4x(0, static_cast<uint16_t>((((_exReg >> 4) & 0x03) | 0x0C) << 2));
        } else {
            Mmc3::SelectPrgPage(slot, static_cast<uint16_t>((page & 0x0F)
                                 | ((_exReg & 0x03) << 4)), type);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            (void)value;
            _exReg = static_cast<uint8_t>(address);
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_287.exReg", _exReg);
    }
};

class Mmc3_292 final : public Mmc3 {
    uint8_t _exRegs[3]{};

    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        std::memset(_exRegs, 0, sizeof(_exRegs));
        Mmc3::InitMapper();
        AddRegisterRange(0x6000, 0x6FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        (void)type;
        if (slot == 0) {
            SelectChrPage2x(0, static_cast<uint16_t>(((page >> 1) ^ _exRegs[1]) << 1));
        } else if (slot == 2) {
            SelectChrPage2x(1, static_cast<uint16_t>(((page >> 1)
                | ((_exRegs[2] & 0x40) << 1)) << 1));
        } else if (slot == 4) {
            SelectChrPage4x(1, static_cast<uint16_t>((_exRegs[2] & 0x3F) << 2));
        }
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (slot == 0) Mmc3::SelectPrgPage(slot, _exRegs[0] & 0x1F, type);
        else Mmc3::SelectPrgPage(slot, page, type);
    }

    uint8_t ReadRegister(uint16_t address) override {
        if (!(address & 0x01)) {
            if ((_exRegs[0] & 0xE0) == 0xC0) _exRegs[1] = cpu_peek_internal_ram(0x006A);
            else _exRegs[2] = cpu_peek_internal_ram(0x00FF);
            UpdateState();
        }
        return 0;
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (!(address & 0x01)) {
                _exRegs[0] = value;
                UpdateState();
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_292.exRegs", _exRegs);
    }
};

class Mmc3_313 final : public Mmc3 {
    uint8_t _resetCounter = 0;

    void Reset(bool softReset) override {
        Mmc3::Reset(softReset);
        if (softReset) {
            _resetCounter = static_cast<uint8_t>((_resetCounter + 1) & 0x03);
            UpdateState();
        } else {
            _resetCounter = 0;
        }
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, static_cast<uint16_t>((_resetCounter << 7)
                             | (page & 0x7F)), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        Mmc3::SelectPrgPage(slot, static_cast<uint16_t>((_resetCounter << 4)
                             | (page & 0x0F)), type);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_313.resetCounter", _resetCounter);
    }
};

class Mmc3_325 final : public Mmc3 {
    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, static_cast<uint16_t>((page & 0xDD)
                             | ((page & 0x20) >> 4) | ((page & 0x02) << 4)), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        Mmc3::SelectPrgPage(slot, static_cast<uint16_t>((page & 0x03)
                             | ((page & 0x08) >> 1) | ((page & 0x04) << 1)), type);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0xC000) {
            address = static_cast<uint16_t>((address & 0xFFFE)
                    | ((address >> 2) & 0x01) | ((address >> 3) & 0x01));
        } else {
            address = static_cast<uint16_t>((address & 0xFFFE) | ((address >> 3) & 0x01));
        }
        Mmc3::WriteRegister(address, value);
    }
};

class Mmc3_333 final : public Mmc3 {
    uint8_t _reg = 0;

    void InitMapper() override {
        _reg = 0;
        Mmc3::InitMapper();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, static_cast<uint16_t>(((_reg & 0x0C) << 5)
                             | (page & 0x7F)), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_reg & 0x10) {
            Mmc3::SelectPrgPage(slot, static_cast<uint16_t>(((_reg & 0x0C) << 2)
                                 | (page & 0x0F)), type);
        } else {
            SelectPrgPage4x(0, static_cast<uint16_t>((_reg & 0x0F) << 2));
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address & 0x1000) {
            _reg = value;
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_333.reg", _reg);
    }
};

class Mmc3_348 final : public Mmc3 {
    uint8_t _reg = 0;

    void InitMapper() override {
        _reg = 0;
        Mmc3::InitMapper();
        AddRegisterRange(0x6800, 0x68FF, MemoryOperation::Write);
    }

    void Reset(bool softReset) override {
        _reg = 0;
        Mmc3::Reset(softReset);
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, static_cast<uint16_t>(((_reg & 0x0C) << 5)
                             | (page & 0x7F)), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if ((_reg & 0x0C) == 0x0C) {
            if (slot == 0) {
                Mmc3::SelectPrgPage(0, static_cast<uint16_t>(((_reg & 0x0C) << 2)
                                     | (page & 0x0F)), type);
                Mmc3::SelectPrgPage(2, static_cast<uint16_t>(0x32 | (page & 0x0F)), type);
            } else if (slot == 1) {
                Mmc3::SelectPrgPage(1, static_cast<uint16_t>(((_reg & 0x0C) << 2)
                                     | (page & 0x0F)), type);
                Mmc3::SelectPrgPage(3, static_cast<uint16_t>(0x32 | (page & 0x0F)), type);
            }
        } else {
            Mmc3::SelectPrgPage(slot, static_cast<uint16_t>(((_reg & 0x0C) << 2)
                                 | (page & 0x0F)), type);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            _reg = value;
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_348.reg", _reg);
    }
};

class Mmc3_366 final : public Mmc3 {
    uint8_t _selectedBlock = 0;
    bool _wramEnabled = false;

    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }

    void Reset(bool softReset) override {
        Mmc3::Reset(softReset);
        if (softReset) {
            _selectedBlock = 0;
            _wramEnabled = false;
            ResetMmc3();
            UpdateState();
        }
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, static_cast<uint16_t>((page & 0x7F)
                             | (_selectedBlock << 3)), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        Mmc3::SelectPrgPage(slot, static_cast<uint16_t>((page & 0x0F)
                             | _selectedBlock), type);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x7000) {
            if (!_wramEnabled) {
                _selectedBlock = static_cast<uint8_t>(address & 0x30);
                _wramEnabled = (address & 0x80) != 0;
                UpdateState();
            } else {
                WritePrgRam(address, value);
            }
        } else if (address < 0x8000) {
            if (!_wramEnabled) {
                _selectedBlock = value & 0x30;
                UpdateState();
            } else {
                WritePrgRam(address, value);
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_366.selectedBlock", _selectedBlock)
            && state.Field("mmc3_366.wramEnabled", _wramEnabled);
    }
};

} // namespace cupid::boards
#endif
