/*
 * unlicensed_114.hpp - Multicart address decoders and banked cartridge RAM
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
#ifndef CUPID_BOARDS_UNLICENSED_114_HPP
#define CUPID_BOARDS_UNLICENSED_114_HPP
#include "runtime.hpp"
#include <cstdlib>

namespace cupid::boards {

class Bmc80013B final : public Board {
    std::array<uint8_t, 2> _regs{};
    uint8_t _mode = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void UpdateState() {
        SelectPrgPage(0, _mode & 2 ? (_regs[0] & 15) | (_regs[1] & 0x70) : _regs[0] & 3);
        SelectPrgPage(1, _regs[1] & 0x7F);
        SetMirroringType(_regs[0] & 0x10 ? MirroringType::Vertical : MirroringType::Horizontal);
    }
    void InitMapper() override { SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        unsigned reg = (address >> 13) & 3;
        if (!reg) _regs[0] = value;
        else { _regs[1] = value; _mode = static_cast<uint8_t>(reg); }
        UpdateState();
    }
public:
    void Reset(bool) override { _regs.fill(0); _mode = 0; UpdateState(); }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bmc80013b.regs", _regs)
            && state.Field("bmc80013b.mode", _mode);
    }
};

class Gs2004 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override { SelectPrgPage4x(0, (value & 7) << 2); }
public:
    void Reset(bool) override {
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0x20, PrgMemoryType::PrgRom);
        SelectPrgPage4x(0, 0x1C);
    }
};

class A65AS final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectChrPage(0, 0); WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        if (value & 0x40) SelectPrgPage2x(0, value & 0x1E);
        else {
            SelectPrgPage(0, ((value & 0x30) >> 1) | (value & 7));
            SelectPrgPage(1, ((value & 0x30) >> 1) | 7);
        }
        SetMirroringType(value & 0x80
            ? (value & 0x20 ? MirroringType::ScreenBOnly : MirroringType::ScreenAOnly)
            : (value & 8 ? MirroringType::Horizontal : MirroringType::Vertical));
    }
};

class Gkcx1 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectPrgPage(0, (address >> 3) & 3);
        SelectChrPage(0, address & 7);
    }
};

class Bmc60311C final : public Board {
    uint8_t _inner = 0, _outer = 0, _mode = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void UpdateState() {
        uint8_t page = _outer | (_mode & 4 ? 0 : _inner);
        switch (_mode & 3) {
            case 0: SelectPrgPage(0, page); SelectPrgPage(1, page); break;
            case 1: SelectPrgPage2x(0, page & 0xFE); break;
            case 2: SelectPrgPage(0, page); SelectPrgPage(1, _outer | 7); break;
            case 3: break;
        }
        SetMirroringType(_mode & 8 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override { UpdateState(); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0x8000) _inner = value & 7;
        else if (address & 1) _outer = value;
        else _mode = value & 15;
        UpdateState();
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bmc60311c.inner", _inner)
            && state.Field("bmc60311c.outer", _outer)
            && state.Field("bmc60311c.mode", _mode);
    }
};

class Bmc190in1 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        unsigned bank = (value >> 2) & 7;
        SelectPrgPage(0, bank);
        SelectPrgPage(1, bank);
        SelectChrPage(0, bank);
        SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Bmc8157 final : public Board {
    uint16_t _address = 0;
    uint32_t GetDipSwitchCount() override { return 1; }
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void UpdateState() {
        unsigned inner = (_address >> 2) & 7;
        unsigned mode = ((_address >> 7) & 1) | ((_address >> 8) & 2);
        unsigned outer = ((_address >> 5) & 3) << 3;
        bool upperChip = (_address & 0x100) != 0;
        if (upperChip && _prgSize <= 0x80000 && GetDipSwitches()) {
            RemoveCpuMemoryMapping(0x8000, 0xFFFF);
            return;
        }
        outer |= upperChip ? 0x40 : 0;
        SelectPrgPage(0, outer | inner);
        SelectPrgPage(1, outer | (mode == 0 ? 0 : mode == 1 ? inner : 7));
        SetMirroringType(_address & 2 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override { UpdateState(); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t) override { _address = address; UpdateState(); }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bmc8157.address", _address);
    }
};

class Bmc64in1NoRepeat final : public Board {
    std::array<uint8_t, 4> _regs{};
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void UpdateState() {
        unsigned bank = ((_regs[1] & 0x1F) << 1) | ((_regs[1] >> 6) & 1);
        if (_regs[0] & 0x80) {
            if (_regs[1] & 0x80) SelectPrgPage2x(0, (_regs[1] & 0x1F) << 1);
            else { SelectPrgPage(0, bank); SelectPrgPage(1, bank); }
        } else SelectPrgPage(1, bank);
        SetMirroringType(_regs[0] & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
        SelectChrPage(0, (_regs[2] << 2) | ((_regs[0] >> 1) & 3));
    }
    void InitMapper() override { AddRegisterRange(0x5000, 0x5003, MemoryOperation::Write); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[address < 0x8000 ? address & 3 : 3] = value;
        UpdateState();
    }
public:
    void Reset(bool) override { _regs = {0x80, 0x43, 0, 0}; UpdateState(); }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bmc64in1norepeat.regs", _regs);
    }
};

class Hp898f final : public Board {
    std::array<uint8_t, 2> _regs{};
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void UpdateState() {
        unsigned bank = (_regs[1] >> 3) & 7;
        unsigned mask = (_regs[1] >> 4) & 4;
        unsigned chrMask = ((_regs[0] & 1) << 2) | (_regs[0] & 2);
        SelectPrgPage(0, bank & ~mask);
        SelectPrgPage(1, bank | mask);
        SelectChrPage(0, ((_regs[0] >> 4) & 7) & ~chrMask);
        SetMirroringType(_regs[1] & 0x80 ? MirroringType::Vertical : MirroringType::Horizontal);
    }
    void InitMapper() override { UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0x6000) == 0x6000) { _regs[(address >> 2) & 1] = value; UpdateState(); }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("hp898f.regs", _regs);
    }
};

class Bmc830425C4391T final : public Board {
    uint8_t _inner = 0, _outer = 0;
    bool _mode = false;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void UpdateState() {
        unsigned mask = _mode ? 7 : 15;
        SelectPrgPage(0, (_inner & mask) | (_outer << 3));
        SelectPrgPage(1, mask | (_outer << 3));
    }
    void InitMapper() override { SelectChrPage(0, 0); UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        _inner = value & 15;
        if ((address & 0xFFE0) == 0xF0E0) { _outer = address & 15; _mode = (address & 0x10) != 0; }
        UpdateState();
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bmc830425c4391t.inner", _inner)
            && state.Field("bmc830425c4391t.outer", _outer)
            && state.Field("bmc830425c4391t.mode", _mode);
    }
};

class Rt01 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x800; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 0);
        for (unsigned slot = 0; slot < 4; ++slot) SelectChrPage(slot, 0);
    }
    uint8_t ReadRegister(uint16_t address) override {
        if ((address >= 0xCE80 && address < 0xCF00) || (address >= 0xFE80 && address < 0xFF00))
            return static_cast<uint8_t>(0xF2 | (std::rand() & 0x0D));
        return InternalReadRam(address);
    }
};

class Edu2000 final : public Board {
    uint8_t _reg = 0;
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetWorkRamSize() override { return 0x8000; }
    uint32_t GetWorkRamPageSize() override { return 0x2000; }
    void UpdateState() {
        SelectPrgPage(0, _reg & 0x1F);
        SetCpuMemoryMapping(0x6000, 0x7FFF, (_reg >> 6) & 3, PrgMemoryType::WorkRam);
    }
    void InitMapper() override { UpdateState(); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override { _reg = value; UpdateState(); }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("edu2000.reg", _reg);
    }
};

} // namespace cupid::boards
#endif
