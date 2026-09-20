/*
 * mmc3_95.hpp - MMC3-derived boards for issue 95
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
#ifndef CUPID_BOARDS_MMC3_95_HPP
#define CUPID_BOARDS_MMC3_95_HPP

#include "mmc3.hpp"
#include <cstring>

namespace cupid::boards {

class Mmc3_12 final : public Mmc3 {
    uint8_t _chrSelection = 0;
    bool ForceMmc3RevAIrqs() override { return true; }
    void InitMapper() override {
        AddRegisterRange(0x4020, 0x5FFF);
        Mmc3::InitMapper();
    }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        if ((slot < 4 && (_chrSelection & 0x01)) || (slot >= 4 && (_chrSelection & 0x10)))
            page |= 0x100;
        Mmc3::SelectChrPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0x5FFF) {
            _chrSelection = value;
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_12.chrSelection", _chrSelection);
    }
};

class Mmc3_14 final : public Mmc3 {
    uint8_t _vrcChrRegs[8]{};
    uint8_t _vrcPrgRegs[2]{};
    uint8_t _vrcMirroring = 0;
    uint8_t _mode = 0;

    void InitMapper() override {
        _mode = 0;
        _vrcMirroring = 0;
        std::memset(_vrcPrgRegs, 0, sizeof(_vrcPrgRegs));
        std::memset(_vrcChrRegs, 0, sizeof(_vrcChrRegs));
        Mmc3::InitMapper();
    }
    void UpdateChrMapping() override {
        int swap = (GetState().reg8000 & 0x80) ? 4 : 0;
        int outer0 = (_mode & 0x08) ? 0x100 : 0;
        int outer1 = (_mode & 0x20) ? 0x100 : 0;
        int outer2 = (_mode & 0x80) ? 0x100 : 0;
        SelectChrPage(0 ^ swap, static_cast<uint16_t>(outer0 | (_registers[0] & ~1)));
        SelectChrPage(1 ^ swap, static_cast<uint16_t>(outer0 | _registers[0] | 1));
        SelectChrPage(2 ^ swap, static_cast<uint16_t>(outer0 | (_registers[1] & ~1)));
        SelectChrPage(3 ^ swap, static_cast<uint16_t>(outer0 | _registers[1] | 1));
        SelectChrPage(4 ^ swap, static_cast<uint16_t>(outer1 | _registers[2]));
        SelectChrPage(5 ^ swap, static_cast<uint16_t>(outer1 | _registers[3]));
        SelectChrPage(6 ^ swap, static_cast<uint16_t>(outer2 | _registers[4]));
        SelectChrPage(7 ^ swap, static_cast<uint16_t>(outer2 | _registers[5]));
    }
    void UpdateVrcState() {
        SelectPrgPage(0, _vrcPrgRegs[0]);
        SelectPrgPage(1, _vrcPrgRegs[1]);
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        for (unsigned i = 0; i < 8; ++i) SelectChrPage(static_cast<uint16_t>(i), _vrcChrRegs[i]);
        SetMirroringType((_vrcMirroring & 1) ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0xA131) _mode = value;
        if (_mode & 0x02) {
            Mmc3::UpdateState();
            Mmc3::WriteRegister(address, value);
            return;
        }
        if (address >= 0xB000 && address <= 0xEFFF) {
            uint8_t reg = static_cast<uint8_t>(((((address >> 12) & 7) - 3) << 1) + ((address >> 1) & 1));
            if (!(address & 1)) _vrcChrRegs[reg] = (_vrcChrRegs[reg] & 0xF0) | (value & 0x0F);
            else _vrcChrRegs[reg] = (_vrcChrRegs[reg] & 0x0F) | ((value & 0x0F) << 4);
        } else {
            switch (address & 0xF003) {
                case 0x8000: _vrcPrgRegs[0] = value; break;
                case 0x9000: _vrcMirroring = value; break;
                case 0xA000: _vrcPrgRegs[1] = value; break;
            }
        }
        UpdateVrcState();
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_14.vrcChrRegs", _vrcChrRegs)
            && state.Field("mmc3_14.vrcPrgRegs", _vrcPrgRegs)
            && state.Field("mmc3_14.vrcMirroring", _vrcMirroring)
            && state.Field("mmc3_14.mode", _mode);
    }
};

class Mmc3_37 final : public Mmc3 {
    uint8_t _selectedBlock = 0;
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void Reset(bool) override { _selectedBlock = 0; UpdateState(); }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        if (_selectedBlock >= 4) page |= 0x80;
        Mmc3::SelectChrPage(slot, page, type);
    }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_selectedBlock <= 2) page &= 0x07;
        else if (_selectedBlock == 3) page = (page & 0x07) | 0x08;
        else if (_selectedBlock == 7) page = (page & 0x07) | 0x20;
        else page = (page & 0x0F) | 0x10;
        Mmc3::SelectPrgPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (CanWriteToWorkRam()) { _selectedBlock = value & 7; UpdateState(); }
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_37.selectedBlock", _selectedBlock);
    }
};

class Mmc3_44 final : public Mmc3 {
    uint8_t _selectedBlock = 0;
    void Reset(bool) override { _selectedBlock = 0; UpdateState(); }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        page &= _selectedBlock <= 5 ? 0x7F : 0xFF;
        page |= static_cast<uint16_t>(_selectedBlock) * 0x80;
        Mmc3::SelectChrPage(slot, page, type);
    }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        page &= _selectedBlock <= 5 ? 0x0F : 0x1F;
        page |= static_cast<uint16_t>(_selectedBlock) * 0x10;
        Mmc3::SelectPrgPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0xE001) == 0xA001) {
            _selectedBlock = value & 7;
            if (_selectedBlock == 7) _selectedBlock = 6;
        }
        Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_44.selectedBlock", _selectedBlock);
    }
};

class Mmc3_45 final : public Mmc3 {
    uint8_t _regIndex = 0;
    uint8_t _reg[4]{};
    void InitMapper() override {
        _regIndex = 0;
        std::memset(_reg, 0, sizeof(_reg));
        _reg[2] = 0x0F;
        AddRegisterRange(0x6000, 0x7FFF);
        Mmc3::InitMapper();
        _registers[0] = 0; _registers[1] = 2; _registers[2] = 4;
        _registers[3] = 5; _registers[4] = 6; _registers[5] = 7;
        UpdateChrMapping();
    }
    void Reset(bool) override {
        AddRegisterRange(0x6000, 0x7FFF);
        _regIndex = 0;
        std::memset(_reg, 0, sizeof(_reg));
        _reg[2] = 0x0F;
        UpdateState();
    }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        if (!HasChrRam()) {
            page &= static_cast<uint16_t>(0xFFu >> (0x0F - (_reg[2] & 0x0F)));
            page |= static_cast<uint16_t>(_reg[0] | ((_reg[2] & 0xF0) << 4));
        }
        Mmc3::SelectChrPage(slot, page, type);
    }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        page &= static_cast<uint16_t>(0x3F ^ (_reg[3] & 0x3F));
        page |= _reg[1];
        Mmc3::SelectPrgPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (!(_reg[3] & 0x40)) {
                _reg[_regIndex] = value;
                _regIndex = (_regIndex + 1) & 3;
            }
            if (_reg[3] & 0x40) RemoveRegisterRange(0x6000, 0x7FFF);
            UpdateState();
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_45.regIndex", _regIndex)
            && state.Field("mmc3_45.reg", _reg);
    }
};

class Mmc3_47 final : public Mmc3 {
    uint8_t _selectedBlock = 0;
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void Reset(bool) override { _selectedBlock = 0; UpdateState(); }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        page = (page & 0x7F) | (_selectedBlock ? 0x80 : 0);
        Mmc3::SelectChrPage(slot, page, type);
    }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        page = (page & 0x0F) | (_selectedBlock ? 0x10 : 0);
        Mmc3::SelectPrgPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (CanWriteToWorkRam()) { _selectedBlock = value & 1; UpdateState(); }
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_47.selectedBlock", _selectedBlock);
    }
};

class Mmc3_49 final : public Mmc3 {
    uint8_t _selectedBlock = 0, _prgReg = 0, _prgMode = 0;
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void Reset(bool) override { _selectedBlock = _prgReg = _prgMode = 0; UpdateState(); }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        page = (page & 0x7F) | static_cast<uint16_t>(0x80 * _selectedBlock);
        Mmc3::SelectChrPage(slot, page, type);
    }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_prgMode) page = (page & 0x0F) | static_cast<uint16_t>(0x10 * _selectedBlock);
        else page = static_cast<uint16_t>(_prgReg * 4 + slot);
        Mmc3::SelectPrgPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (CanWriteToWorkRam()) {
                _selectedBlock = (value >> 6) & 3;
                _prgReg = (value >> 4) & 3;
                _prgMode = value & 1;
                UpdateState();
            }
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_49.selectedBlock", _selectedBlock)
            && state.Field("mmc3_49.prgReg", _prgReg)
            && state.Field("mmc3_49.prgMode", _prgMode);
    }
};

class Mmc3_52 final : public Mmc3 {
    uint8_t _extraReg = 0;
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void Reset(bool) override { _extraReg = 0; UpdateState(); }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        if (_extraReg & 0x40) {
            page &= 0x7F;
            page |= static_cast<uint16_t>(((_extraReg & 0x04) | ((_extraReg >> 4) & 3)) << 7);
        } else {
            page &= 0xFF;
            page |= static_cast<uint16_t>(((_extraReg & 0x04) | ((_extraReg >> 4) & 2)) << 7);
        }
        Mmc3::SelectChrPage(slot, page, type);
    }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_extraReg & 0x08) page = (page & 0x0F) | static_cast<uint16_t>((_extraReg & 7) << 4);
        else page = (page & 0x1F) | static_cast<uint16_t>((_extraReg & 6) << 4);
        Mmc3::SelectPrgPage(slot, page, type);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (CanWriteToWorkRam()) {
                if (!(_extraReg & 0x80)) { _extraReg = value; UpdateState(); }
                else WritePrgRam(address, value);
            }
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_52.extraReg", _extraReg);
    }
};

class Mmc3_114 final : public Mmc3 {
    inline static constexpr uint8_t Security[8] = {0, 3, 1, 5, 6, 7, 2, 4};
    uint8_t _exRegs[2]{};
    uint16_t RegisterStartAddress() override { return 0x5000; }
    bool ForceMmc3RevAIrqs() override { return true; }
    void InitMapper() override { Mmc3::InitMapper(); _exRegs[0] = _exRegs[1] = 0; }
    void UpdatePrgMapping() override {
        if (_exRegs[0] & 0x80) {
            uint16_t page = static_cast<uint16_t>((_exRegs[0] & 0x0F) << 1);
            SelectPrgPage2x(0, page); SelectPrgPage2x(1, page);
        } else Mmc3::UpdatePrgMapping();
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) { _exRegs[0] = value; UpdatePrgMapping(); return; }
        switch (address & 0xE001) {
            case 0x8001: Mmc3::WriteRegister(0xA000, value); break;
            case 0xA000:
                Mmc3::WriteRegister(0x8000, static_cast<uint8_t>((value & 0xC0) | Security[value & 7]));
                _exRegs[1] = 1;
                break;
            case 0xA001: _irqReloadValue = value; break;
            case 0xC000:
                if (_exRegs[1]) { _exRegs[1] = 0; Mmc3::WriteRegister(0x8001, value); }
                break;
            case 0xC001: _irqReload = true; break;
            case 0xE000: SetIrq(false); _irqEnabled = false; break;
            case 0xE001: _irqEnabled = true; break;
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_114.exRegs", _exRegs);
    }
};

class Mmc3_115 final : public Mmc3 {
    uint8_t _prgReg = 0, _chrReg = 0, _protectionReg = 0;
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        AddRegisterRange(0x4100, 0x7FFF, MemoryOperation::Write);
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Read);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        Mmc3::InitMapper();
    }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        Board::SelectChrPage(slot, page | (static_cast<uint16_t>(_chrReg) << 8), type);
    }
    void UpdateState() override {
        Mmc3::UpdateState();
        if (_prgReg & 0x80) {
            if (_prgReg & 0x20) SelectPrgPage4x(0, static_cast<uint16_t>(((_prgReg & 0x0F) >> 1) << 2));
            else {
                uint16_t page = static_cast<uint16_t>((_prgReg & 0x0F) << 1);
                SelectPrgPage2x(0, page); SelectPrgPage2x(1, page);
            }
        }
    }
    uint8_t ReadRegister(uint16_t) override { return _protectionReg; }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (address == 0x5080) _protectionReg = value;
            else {
                if (address & 1) _chrReg = value & 1;
                else _prgReg = value;
                UpdateState();
            }
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_115.prgReg", _prgReg)
            && state.Field("mmc3_115.chrReg", _chrReg)
            && state.Field("mmc3_115.protectionReg", _protectionReg);
    }
};

class Mmc3_121 final : public Mmc3 {
    uint8_t _exRegs[8]{};
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        std::memset(_exRegs, 0, sizeof(_exRegs));
        Mmc3::InitMapper();
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }
    void Reset(bool) override { std::memset(_exRegs, 0, sizeof(_exRegs)); _exRegs[3] = 0x80; }
    uint8_t ReadRegister(uint16_t) override { return _exRegs[4]; }
    void SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type = PrgMemoryType::PrgRom) override {
        uint8_t outer = (_exRegs[3] & 0x80) >> 2;
        Board::SelectPrgPage(slot, (page & 0x1F) | outer, type);
        if (_exRegs[5] & 0x3F) {
            Board::SelectPrgPage(1, _exRegs[2] | outer, type);
            Board::SelectPrgPage(2, _exRegs[1] | outer, type);
            Board::SelectPrgPage(3, _exRegs[0] | outer, type);
        }
    }
    void SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type = ChrMemoryType::Default) override {
        if (_prgSize == _chrRomSize) page |= static_cast<uint16_t>((_exRegs[3] & 0x80) << 1);
        else if (slot >= 4) page |= 0x100;
        Board::SelectChrPage(slot, page, type);
    }
    void UpdateExRegs() {
        switch (_exRegs[5] & 0x3F) {
            case 0x20: case 0x29: case 0x2B: case 0x3C: case 0x3F:
                _exRegs[7] = 1; _exRegs[0] = _exRegs[6]; break;
            case 0x26: _exRegs[7] = 0; _exRegs[0] = _exRegs[6]; break;
            case 0x2C: _exRegs[7] = 1; if (_exRegs[6]) _exRegs[0] = _exRegs[6]; break;
            case 0x28: _exRegs[7] = 0; _exRegs[1] = _exRegs[6]; break;
            case 0x2A: _exRegs[7] = 0; _exRegs[2] = _exRegs[6]; break;
            case 0x2F: break;
            default: _exRegs[5] = 0; break;
        }
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            static constexpr uint8_t lookup[4] = {0x83, 0x83, 0x42, 0x00};
            _exRegs[4] = lookup[value & 3];
            if ((address & 0x5180) == 0x5180) { _exRegs[3] = value; UpdateState(); }
        } else if (address < 0xA000) {
            if ((address & 3) == 3) {
                _exRegs[5] = value; UpdateExRegs(); Mmc3::WriteRegister(0x8000, value);
            } else if (address & 1) {
                _exRegs[6] = static_cast<uint8_t>(((value & 1) << 5) | ((value & 2) << 3)
                           | ((value & 4) << 1) | ((value & 8) >> 1)
                           | ((value & 0x10) >> 3) | ((value & 0x20) >> 5));
                if (!_exRegs[7]) UpdateExRegs();
                Mmc3::WriteRegister(0x8001, value);
            } else Mmc3::WriteRegister(0x8000, value);
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_121.exRegs", _exRegs);
    }
};

class Mmc3_123 final : public Mmc3 {
    inline static constexpr uint8_t Security[8] = {0, 3, 1, 5, 6, 7, 2, 4};
    uint8_t _exReg[2]{};
    void UpdatePrgMapping() override {
        if (_exReg[0] & 0x40) {
            uint8_t bank = static_cast<uint8_t>((_exReg[0] & 0x05) | ((_exReg[0] & 0x08) >> 2)
                                                 | ((_exReg[0] & 0x20) >> 2));
            if (_exReg[0] & 2) SelectPrgPage4x(0, static_cast<uint16_t>((bank & 0xFE) << 1));
            else {
                uint16_t page = static_cast<uint16_t>(bank << 1);
                SelectPrgPage2x(0, page); SelectPrgPage2x(1, page);
            }
        } else Mmc3::UpdatePrgMapping();
    }
    void InitMapper() override {
        Mmc3::InitMapper();
        _exReg[0] = _exReg[1] = 0;
        AddRegisterRange(0x5001, 0x5FFF, MemoryOperation::Write);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000 && (address & 0x0800)) {
            _exReg[address & 1] = value;
            UpdatePrgMapping();
        } else if (address < 0xA000) {
            if ((address & 0x8001) == 0x8000)
                Mmc3::WriteRegister(0x8000, static_cast<uint8_t>((value & 0xC0) | Security[value & 7]));
            else Mmc3::WriteRegister(0x8001, value);
        } else Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_123.exReg", _exReg);
    }
};

} // namespace cupid::boards
#endif
