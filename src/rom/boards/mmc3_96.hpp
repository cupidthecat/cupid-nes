/*
 * mmc3_96.hpp - MMC3-derived multicart and protection boards
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
#ifndef CUPID_BOARDS_MMC3_96_HPP
#define CUPID_BOARDS_MMC3_96_HPP

#include "mmc3.hpp"

namespace cupid::boards {

class Mmc3_126 final : public Mmc3 {
    uint8_t _exRegs[4]{};

    uint16_t GetChrOuterBank() const {
        uint16_t reg = _exRegs[0];
        return static_cast<uint16_t>(((~reg) & 0x0080 & _exRegs[2])
            | ((reg << 4) & 0x0080 & reg)
            | ((reg << 3) & 0x0100)
            | ((reg << 5) & 0x0200));
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        uint16_t reg = _exRegs[0];
        page &= static_cast<uint16_t>(((~reg >> 2) & 0x10) | 0x0F);
        page |= static_cast<uint16_t>((reg & (0x06 | ((reg & 0x40) >> 6))) << 4
                                   | (reg & 0x10) << 3);

        if (!(_exRegs[3] & 0x03)) {
            Mmc3::SelectPrgPage(slot, page, type);
        } else if ((_prgMode << 1) == slot) {
            if ((_exRegs[3] & 0x03) == 0x03) {
                Mmc3::SelectPrgPage(0, page, type);
                Mmc3::SelectPrgPage(1, page + 1, type);
                Mmc3::SelectPrgPage(2, page + 2, type);
                Mmc3::SelectPrgPage(3, page + 3, type);
            } else {
                Mmc3::SelectPrgPage(0, page, type);
                Mmc3::SelectPrgPage(1, page + 1, type);
                Mmc3::SelectPrgPage(2, page, type);
                Mmc3::SelectPrgPage(3, page + 1, type);
            }
        }
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType = ChrMemoryType::Default) override {
        if (!(_exRegs[3] & 0x10))
            Mmc3::SelectChrPage(slot, GetChrOuterBank() | (page & ((_exRegs[0] & 0x80) - 1)));
    }

    void InitMapper() override {
        Mmc3::InitMapper();
        std::memset(_exRegs, 0, sizeof(_exRegs));
        AddRegisterRange(0x6000, 0x8000, MemoryOperation::Write);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            address &= 3;
            if (address == 1 || address == 2
                || ((address == 0 || address == 3) && !(_exRegs[3] & 0x80))) {
                if (_exRegs[address] != value) {
                    _exRegs[address] = value;
                    if (_exRegs[3] & 0x10) {
                        uint16_t page = GetChrOuterBank() | ((_exRegs[2] & 0x0F) << 3);
                        for (uint16_t slot = 0; slot < 8; ++slot)
                            Mmc3::SelectChrPage(slot, page + slot);
                    } else {
                        Mmc3::UpdateChrMapping();
                    }
                    Mmc3::UpdatePrgMapping();
                }
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_126.exRegs", _exRegs);
    }
};

class Mmc3_134 final : public Mmc3 {
    uint8_t _exReg = 0;

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        Board::SelectChrPage(slot, (page & 0xFF) | ((_exReg & 0x20) << 3), type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        Board::SelectPrgPage(slot, (page & 0x1F) | ((_exReg & 0x02) << 4), type);
    }

    void InitMapper() override {
        Mmc3::InitMapper();
        AddRegisterRange(0x6001, 0x6001, MemoryOperation::Write);
    }

    void Reset(bool softReset) override {
        if (softReset) {
            _exReg = 0;
            Mmc3::UpdateState();
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x6001) {
            _exReg = value;
            Mmc3::UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_134.exReg", _exReg);
    }
};

class Mmc3_165 final : public Mmc3 {
    bool _chrLatch[2]{};
    bool _needUpdate = false;

    uint16_t GetChrPageSize() override { return 0x1000; }
    uint32_t GetChrRamSize() override { return 0x1000; }
    uint16_t GetChrRamPageSize() override { return 0x1000; }

    void UpdateChrMapping() override {
        for (uint16_t slot = 0; slot < 2; ++slot) {
            uint8_t reg = slot == 0 ? (_chrLatch[0] ? 1 : 0) : (_chrLatch[1] ? 4 : 2);
            uint16_t page = _registers[reg];
            if (page == 0)
                SelectChrPage(slot, 0, ChrMemoryType::ChrRam);
            else
                SelectChrPage(slot, page >> 2, ChrMemoryType::ChrRom);
        }
        _needUpdate = false;
    }

public:
    void NotifyVramAddressChange(uint16_t address) override {
        if (_needUpdate) UpdateChrMapping();
        switch (address & 0x2FF8) {
            case 0x0FD0:
            case 0x0FE8:
                _chrLatch[(address >> 12) & 1] = (address & 8) != 0;
                _needUpdate = true;
                break;
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_165.chrLatch", _chrLatch)
            && state.Field("mmc3_165.needUpdate", _needUpdate);
    }
};

class Mmc3_182 final : public Mmc3 {
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE001) {
            case 0x8001:
                Mmc3::WriteRegister(0xA000, value);
                break;
            case 0xA000: {
                static constexpr uint8_t security[8] = {0, 3, 1, 5, 6, 7, 2, 4};
                Mmc3::WriteRegister(0x8000, (value & 0xF8) | security[value & 7]);
                break;
            }
            case 0xC000:
                Mmc3::WriteRegister(0x8001, value);
                break;
            case 0xC001:
                Mmc3::WriteRegister(0xC000, value);
                Mmc3::WriteRegister(0xC001, value);
                break;
            case 0xE000:
                Mmc3::WriteRegister(0xE000, value);
                break;
            case 0xE001:
                Mmc3::WriteRegister(0xE001, value);
                break;
        }
    }
};

class Mmc3_187 final : public Mmc3 {
    uint8_t _exRegs[2]{};

    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        Mmc3::InitMapper();
        std::memset(_exRegs, 0, sizeof(_exRegs));
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Any);
        AddRegisterRange(0x6000, 0x6FFF, MemoryOperation::Write);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType = ChrMemoryType::Default) override {
        if (slot >= 4) page |= 0x100;
        Board::SelectChrPage(slot, page);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (!(_exRegs[0] & 0x80)) {
            Board::SelectPrgPage(slot, page & 0x3F, type);
            return;
        }

        uint16_t exPage = _exRegs[0] & 0x1F;
        if (_exRegs[0] & 0x20) {
            if (_exRegs[0] & 0x40) {
                exPage &= 0xFC;
                Board::SelectPrgPage(0, exPage);
                Board::SelectPrgPage(1, exPage + 1);
                Board::SelectPrgPage(2, exPage + 2);
                Board::SelectPrgPage(3, exPage + 3);
            } else {
                exPage &= 0xFE;
                Board::SelectPrgPage(0, exPage << 1);
                Board::SelectPrgPage(1, (exPage << 1) + 1);
                Board::SelectPrgPage(2, (exPage << 1) + 2);
                Board::SelectPrgPage(3, (exPage << 1) + 3);
            }
        } else {
            Board::SelectPrgPage(0, exPage << 1);
            Board::SelectPrgPage(1, (exPage << 1) + 1);
            Board::SelectPrgPage(2, exPage << 1);
            Board::SelectPrgPage(3, (exPage << 1) + 1);
        }
    }

    uint8_t ReadRegister(uint16_t) override {
        static constexpr uint8_t security[4] = {0x83, 0x83, 0x42, 0x00};
        return security[_exRegs[1] & 3];
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (address == 0x5000 || address == 0x6000) {
                _exRegs[0] = value;
                Mmc3::UpdatePrgMapping();
            }
        } else if (address == 0x8000) {
            _exRegs[1] = 1;
            Mmc3::WriteRegister(address, value);
        } else if (address == 0x8001) {
            if (_exRegs[1] == 1) Mmc3::WriteRegister(address, value);
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_187.exRegs", _exRegs);
    }
};

class Mmc3_196 final : public Mmc3 {
    uint8_t _exRegs[2]{};

    void InitMapper() override {
        Mmc3::InitMapper();
        _exRegs[0] = _exRegs[1] = 0;
        AddRegisterRange(0x6000, 0x6FFF, MemoryOperation::Write);
    }

    void UpdatePrgMapping() override {
        if (_exRegs[0])
            SelectPrgPage4x(0, static_cast<uint16_t>(_exRegs[1] << 2));
        else
            Mmc3::UpdatePrgMapping();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            _exRegs[0] = 1;
            _exRegs[1] = (value & 0x0F) | (value >> 4);
            UpdatePrgMapping();
            return;
        }
        if (address >= 0xC000)
            address = (address & 0xFFFE) | ((address >> 2) & 1) | ((address >> 3) & 1);
        else
            address = (address & 0xFFFE) | ((address >> 2) & 1)
                    | ((address >> 3) & 1) | ((address >> 1) & 1);
        Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_196.exRegs", _exRegs);
    }
};

class Mmc3_197 final : public Mmc3 {
    void UpdateChrMapping() override {
        if (_chrMode == 0) {
            SelectChrPage4x(0, static_cast<uint16_t>(_registers[0] << 1));
            SelectChrPage2x(2, static_cast<uint16_t>(_registers[2] << 1));
            SelectChrPage2x(3, static_cast<uint16_t>(_registers[3] << 1));
        } else {
            SelectChrPage4x(0, static_cast<uint16_t>(_registers[2] << 1));
            SelectChrPage2x(2, static_cast<uint16_t>(_registers[0] << 1));
            SelectChrPage2x(3, static_cast<uint16_t>(_registers[0] << 1));
        }
    }
};

class Mmc3_198 final : public Mmc3 {
    uint8_t _exRegs[4]{};

    uint32_t GetWorkRamSize() override { return 0x1000; }
    uint32_t GetWorkRamPageSize() override { return 0x1000; }
    uint16_t GetChrRamPageSize() override { return 0x0400; }
    bool ForceWorkRamSize() override { return true; }

    void InitMapper() override {
        _exRegs[0] = 0;
        _exRegs[1] = 1;
        _exRegs[2] = static_cast<uint8_t>(GetPrgPageCount() - 2);
        _exRegs[3] = static_cast<uint8_t>(GetPrgPageCount() - 1);
        SetCpuMemoryMapping(0x5000, 0x7FFF, 0, PrgMemoryType::WorkRam);
        Mmc3::InitMapper();
        // With a save chip, MMC3 owns $6000-$7FFF and its protection bits.
        // Without one, the board mirrors its forced 4 KiB work RAM through
        // the complete $5000-$7FFF window.
        if (_saveRamSize == 0)
            SetCpuMemoryMapping(0x5000, 0x7FFF, 0, PrgMemoryType::WorkRam);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x8001 && (GetState().reg8000 & 7) >= 6)
            _exRegs[(GetState().reg8000 & 7) - 6] = value & (value >= 0x40 ? 0x4F : 0x3F);
        Mmc3::WriteRegister(address, value);
    }

    void SelectPrgPage(uint16_t slot, uint16_t,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        Mmc3::SelectPrgPage(slot, _exRegs[slot], type);
    }

    void UpdateChrMapping() override {
        if (_chrRamSize > 0 && _registers[0] == 0 && _registers[1] == 0
            && _registers[2] == 0 && _registers[3] == 0
            && _registers[4] == 0 && _registers[5] == 0) {
            SelectChrPage8x(0, 0, ChrMemoryType::ChrRam);
        } else {
            Mmc3::UpdateChrMapping();
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_198.exRegs", _exRegs);
    }
};

class Mmc3_199 final : public Mmc3 {
    uint8_t _exRegs[4]{};

    uint32_t GetChrRamSize() override { return 0x2000; }
    uint16_t GetChrRamPageSize() override { return 0x0400; }

    void InitMapper() override {
        _exRegs[0] = 0xFE;
        _exRegs[1] = 0xFF;
        _exRegs[2] = 1;
        _exRegs[3] = 3;
        Mmc3::InitMapper();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x8001 && (GetState().reg8000 & 0x08)) {
            _exRegs[GetState().reg8000 & 3] = value;
            UpdatePrgMapping();
            UpdateChrMapping();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    void UpdateMirroring() override {
        switch (GetState().regA000 & 3) {
            case 0: SetMirroringType(MirroringType::Vertical); break;
            case 1: SetMirroringType(MirroringType::Horizontal); break;
            case 2: SetMirroringType(MirroringType::ScreenAOnly); break;
            case 3: SetMirroringType(MirroringType::ScreenBOnly); break;
        }
    }

    void UpdatePrgMapping() override {
        Mmc3::UpdatePrgMapping();
        SelectPrgPage(2, _exRegs[0]);
        SelectPrgPage(3, _exRegs[1]);
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType = ChrMemoryType::Default) override {
        Mmc3::SelectChrPage(slot, page,
                            page < 8 ? ChrMemoryType::ChrRam : ChrMemoryType::ChrRom);
        Mmc3::SelectChrPage(0, _registers[0],
                            _registers[0] < 8 ? ChrMemoryType::ChrRam : ChrMemoryType::ChrRom);
        Mmc3::SelectChrPage(1, _exRegs[2],
                            _exRegs[2] < 8 ? ChrMemoryType::ChrRam : ChrMemoryType::ChrRom);
        Mmc3::SelectChrPage(2, _registers[1],
                            _registers[1] < 8 ? ChrMemoryType::ChrRam : ChrMemoryType::ChrRom);
        Mmc3::SelectChrPage(3, _exRegs[3],
                            _exRegs[3] < 8 ? ChrMemoryType::ChrRam : ChrMemoryType::ChrRom);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_199.exRegs", _exRegs);
    }
};

class Mmc3_205 final : public Mmc3 {
    uint8_t _selectedBlock = 0;

    uint16_t RegisterStartAddress() override { return 0x6000; }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (_selectedBlock >= 2) {
            page &= 0x7F;
            page |= 0x100;
        }
        if (_selectedBlock == 1 || _selectedBlock == 3) page |= 0x80;
        Mmc3::SelectChrPage(slot, page, type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        page &= _selectedBlock <= 1 ? 0x1F : 0x0F;
        page |= static_cast<uint16_t>(_selectedBlock * 0x10);
        Mmc3::SelectPrgPage(slot, page, type);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            _selectedBlock = value & 3;
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_205.selectedBlock", _selectedBlock);
    }
};

class Mmc3_208 final : public Mmc3 {
    inline static constexpr uint8_t ProtectionLut[256] = {
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x49,0x19,0x09,0x59,0x49,0x19,0x09,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x51,0x41,0x11,0x01,0x51,0x41,0x11,0x01,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x49,0x19,0x09,0x59,0x49,0x19,0x09,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x51,0x41,0x11,0x01,0x51,0x41,0x11,0x01,
        0x00,0x10,0x40,0x50,0x00,0x10,0x40,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x08,0x18,0x48,0x58,0x08,0x18,0x48,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x10,0x40,0x50,0x00,0x10,0x40,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x08,0x18,0x48,0x58,0x08,0x18,0x48,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x58,0x48,0x18,0x08,0x58,0x48,0x18,0x08,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x50,0x40,0x10,0x00,0x50,0x40,0x10,0x00,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x58,0x48,0x18,0x08,0x58,0x48,0x18,0x08,
        0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x50,0x40,0x10,0x00,0x50,0x40,0x10,0x00,
        0x01,0x11,0x41,0x51,0x01,0x11,0x41,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x09,0x19,0x49,0x59,0x09,0x19,0x49,0x59,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x01,0x11,0x41,0x51,0x01,0x11,0x41,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x09,0x19,0x49,0x59,0x09,0x19,0x49,0x59,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    uint8_t _exRegs[6]{};

    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        _exRegs[5] = 3;
        Mmc3::InitMapper();
        AddRegisterRange(0x4800, 0x4FFF, MemoryOperation::Write);
        AddRegisterRange(0x6800, 0x6FFF, MemoryOperation::Write);
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Write);
        AddRegisterRange(0x5800, 0x5FFF, MemoryOperation::Read);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    void UpdatePrgMapping() override {
        SelectPrgPage4x(0, static_cast<uint16_t>(_exRegs[5] << 2));
    }

    uint8_t ReadRegister(uint16_t address) override {
        return _exRegs[address & 3];
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0x5000 && address <= 0x5FFF) {
            if (address <= 0x57FF)
                _exRegs[4] = value;
            else
                _exRegs[address & 3] = value ^ ProtectionLut[_exRegs[4]];
        } else if (address < 0x8000) {
            _exRegs[5] = (value & 1) | ((value >> 3) & 2);
            UpdatePrgMapping();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_208.exRegs", _exRegs);
    }
};

class Mmc3_215 : public Mmc3 {
protected:
    inline static constexpr uint8_t LutReg[8][8] = {
        {0,1,2,3,4,5,6,7}, {0,2,6,1,7,3,4,5},
        {0,5,4,1,7,2,6,3}, {0,6,3,7,5,2,4,1},
        {0,2,5,3,6,1,7,4}, {0,1,2,3,4,5,6,7},
        {0,1,2,3,4,5,6,7}, {0,1,2,3,4,5,6,7},
    };
    inline static constexpr uint8_t LutAddr[8][8] = {
        {0,1,2,3,4,5,6,7}, {3,2,0,4,1,5,6,7},
        {0,1,2,3,4,5,6,7}, {5,0,1,2,3,7,6,4},
        {3,1,0,5,2,4,6,7}, {0,1,2,3,4,5,6,7},
        {0,1,2,3,4,5,6,7}, {0,1,2,3,4,5,6,7},
    };
    uint8_t _exRegs[3]{};

    uint16_t RegisterStartAddress() override { return 0x5000; }

    void InitMapper() override {
        _exRegs[0] = 0;
        _exRegs[1] = 3;
        _exRegs[2] = 0;
        Mmc3::InitMapper();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (_exRegs[0] & 0x40)
            Mmc3::SelectChrPage(slot, ((_exRegs[1] & 0x0C) << 6)
                                    | (page & 0x7F) | ((_exRegs[1] & 0x20) << 2), type);
        else
            Mmc3::SelectChrPage(slot, ((_exRegs[1] & 0x0C) << 6) | page, type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        uint8_t subBank = 0;
        uint8_t bank = 0;
        uint8_t mask;
        if (_exRegs[0] & 0x40) {
            mask = 0x0F;
            subBank = _exRegs[1] & 0x10;
            if (_exRegs[0] & 0x80)
                bank = ((_exRegs[1] & 3) << 4) | (_exRegs[0] & 7) | (subBank >> 1);
        } else {
            mask = 0x1F;
            if (_exRegs[0] & 0x80)
                bank = ((_exRegs[1] & 3) << 4) | (_exRegs[0] & 0x0F);
        }

        if (_exRegs[0] & 0x80) {
            bank <<= 1;
            if (_exRegs[0] & 0x20) {
                Mmc3::SelectPrgPage(0, bank);
                Mmc3::SelectPrgPage(1, bank + 1);
                Mmc3::SelectPrgPage(2, bank + 2);
                Mmc3::SelectPrgPage(3, bank + 3);
            } else {
                Mmc3::SelectPrgPage(0, bank);
                Mmc3::SelectPrgPage(1, bank + 1);
                Mmc3::SelectPrgPage(2, bank);
                Mmc3::SelectPrgPage(3, bank + 1);
            }
        } else {
            Mmc3::SelectPrgPage(slot, ((_exRegs[1] & 3) << 5) | (page & mask) | subBank, type);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            switch (address) {
                case 0x5000:
                    _exRegs[0] = value;
                    UpdateState();
                    break;
                case 0x5001:
                    _exRegs[1] = value;
                    UpdateState();
                    break;
                case 0x5007:
                    _exRegs[2] = value & 7;
                    break;
            }
            return;
        }
        uint8_t lutValue = LutAddr[_exRegs[2]][((address >> 12) & 6) | (address & 1)];
        address = static_cast<uint16_t>((lutValue & 1) | ((lutValue & 6) << 12) | 0x8000);
        if (lutValue == 0)
            value = (value & 0xC0) | LutReg[_exRegs[2]][value & 7];
        Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("mmc3_215.exRegs", _exRegs);
    }
};

} // namespace cupid::boards
#endif
