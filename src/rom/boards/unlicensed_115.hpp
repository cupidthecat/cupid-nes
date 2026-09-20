/*
 * unlicensed_115.hpp - Multicart protection, bank latches, and T230 interrupts
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
#ifndef CUPID_BOARDS_UNLICENSED_115_HPP
#define CUPID_BOARDS_UNLICENSED_115_HPP
#include "runtime.hpp"
#include "vrc_irq.hpp"

namespace cupid::boards {

class Bmc12in1 final : public Board {
    std::array<uint8_t, 2> _regs{};
    uint8_t _mode = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x1000; }
    void UpdateState() {
        unsigned outer = (_mode & 3) << 3;
        SelectChrPage(0, (_regs[0] >> 3) | (outer << 2));
        SelectChrPage(1, (_regs[1] >> 3) | (outer << 2));
        if (_mode & 8) SelectPrgPage2x(0, outer | (_regs[0] & 6));
        else { SelectPrgPage(0, outer | (_regs[0] & 7)); SelectPrgPage(1, outer | 7); }
        SetMirroringType(_mode & 4 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override { UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE000) {
            case 0xA000: _regs[0] = value; break;
            case 0xC000: _regs[1] = value; break;
            case 0xE000: _mode = value & 15; break;
            default: return;
        }
        UpdateState();
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bmc12in1.regs", _regs)
            && state.Field("bmc12in1.mode", _mode);
    }
};

class Super40in1Ws final : public Board {
    bool _locked = false;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0x6FFF; }
    void InitMapper() override { WriteRegister(0x6000, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (_locked) return;
        if (address & 1) SelectChrPage(0, value);
        else {
            _locked = (value & 0x20) != 0;
            unsigned paired = (value & 8) ? 0 : 1;
            SelectPrgPage(0, value & ~paired);
            SelectPrgPage(1, value | paired);
            SetMirroringType(value & 0x10 ? MirroringType::Horizontal : MirroringType::Vertical);
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("super40in1ws.locked", _locked);
    }
};

class BmcK3046 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectPrgPage(1, 7); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, (value & 0x38) | (value & 7));
        SelectPrgPage(1, (value & 0x38) | 7);
    }
};

class BmcG146 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {}
    void WriteRegister(uint16_t address, uint8_t) override {
        if (address & 0x800) {
            SelectPrgPage(0, (address & 0x1F) | (address & ((address & 0x40) >> 6)));
            SelectPrgPage(1, (address & 0x18) | 7);
        } else if (address & 0x40) {
            SelectPrgPage(0, address & 0x1F);
            SelectPrgPage(1, address & 0x1F);
        } else SelectPrgPage2x(0, address & 0x1E);
        SetMirroringType(address & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
public:
    void Reset(bool) override { WriteRegister(0x8000, 0); SelectChrPage(0, 0); }
};

class Mapper487 final : public Board {
    std::array<uint8_t, 2> _regs{};
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4100; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    void UpdateState() {
        uint8_t prg = _regs[1] & 0x1E;
        uint8_t chr = ((_regs[1] & 0x1E) << 2) | (_regs[0] & 3);
        if (_regs[1] & 0x40) { prg |= (_regs[0] & 8) >> 3; chr |= _regs[0] & 4; }
        else { prg |= _regs[1] & 1; chr |= (_regs[1] & 1) << 2; }
        if (_regs[1] & 0x20) { prg += 0x10; chr += 0x40; }
        SelectPrgPage(0, prg);
        SelectChrPage(0, chr);
        SetMirroringType(_regs[1] & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override { AddRegisterRange(0x8000, 0xFFFF, MemoryOperation::Write); UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x6000) {
            if (!(address & 0x100)) return;
            if (address & 0x80) _regs[1] = value;
            else if (!(_regs[1] & 0x20)) _regs[0] = value & 15;
        } else {
            if (!(_regs[1] & 0x20)) return;
            _regs[0] = ((value & 1) << 3) | ((value & 0x70) >> 4);
        }
        UpdateState();
    }
public:
    void Reset(bool) override { _regs.fill(0); UpdateState(); }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("mapper487.regs", _regs);
    }
};

class Dance2000 final : public Board {
    uint8_t _prg = 0, _mode = 0, _lastNt = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x1000; }
    bool AllowRegisterRead() override { return true; }
    bool EnableVramAddressHook() override { return true; }
    void UpdateState() {
        SelectChrPage(0, _lastNt);
        SelectChrPage(1, 1);
        if (_mode & 4) SelectPrgPage2x(0, (_prg & 7) << 1);
        else { SelectPrgPage(0, _prg & 15); SelectPrgPage(1, 0); }
        SetMirroringType(_mode & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override {
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Write);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Write);
        UpdateState();
    }
    uint8_t ReadRegister(uint16_t address) override {
        return _prg & 0x40 ? GetOpenBus() : InternalReadRam(address);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x5000) { _prg = value; UpdateState(); }
        else if (address == 0x5200) { _mode = value; if (_mode & 4) UpdateState(); }
    }
public:
    void NotifyVramAddressChange(uint16_t address) override {
        if (_mode & 2) {
            if ((address & 0x3000) != 0x2000) return;
            uint8_t nextNt = (address >> 11) & 1;
            if (nextNt == _lastNt) return;
            _lastNt = nextNt;
        } else {
            if (!_lastNt) return;
            _lastNt = 0;
        }
        SelectChrPage(0, _lastNt);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("dance2000.prg", _prg)
            && state.Field("dance2000.mode", _mode)
            && state.Field("dance2000.lastNt", _lastNt);
    }
};

class Eh8813A final : public Board {
    bool _alterRead = false;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetDipSwitchCount() override { return 4; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override { SetMirroringType(MirroringType::Vertical); }
    uint8_t ReadRegister(uint16_t address) override {
        return InternalReadRam(_alterRead ? (address & 0xFFF0) | GetDipSwitches() : address);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address & 0x100) return;
        _alterRead = (address & 0x40) != 0;
        if (address & 0x80) { SelectPrgPage(0, address & 7); SelectPrgPage(1, address & 7); }
        else SelectPrgPage2x(0, address & 6);
        SelectChrPage(0, value & 15);
    }
public:
    void Reset(bool) override { WriteRegister(0x8000, 0); _alterRead = false; }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("eh8813a.alterRead", _alterRead);
    }
};

class DreamTech01 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x5020; }
    uint16_t RegisterEndAddress() override { return 0x5020; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectPrgPage(1, 8); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override { SelectPrgPage(0, value & 7); }
};

class T230 final : public Board {
    VrcIrq _irq;
    uint8_t _prg0 = 0, _prg1 = 0, _mode = 0;
    uint16_t _outer = 0;
    std::array<uint8_t, 8> _chrHigh{}, _chrLow{};
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    bool EnableCpuClockHook() override { return true; }
    void UpdateState() {
        if (_chrRamSize) SelectChrPage8x(0, 0);
        else for (unsigned slot = 0; slot < 8; ++slot) SelectChrPage(slot, _chrLow[slot] | (_chrHigh[slot] << 4));
        SelectPrgPage(0, (_mode ? 0x1E : _prg0) | _outer);
        SelectPrgPage(2, (_mode ? _prg0 : 0x1E) | _outer);
        SelectPrgPage(1, _prg1);
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }
    void InitMapper() override { UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        address = (address & 0xF000) | (address & 0x2A ? 2 : 0) | (address & 0x15 ? 1 : 0);
        if (address >= 0x9000 && address <= 0x9001) {
            static constexpr MirroringType modes[] = {
                MirroringType::Vertical, MirroringType::Horizontal,
                MirroringType::ScreenAOnly, MirroringType::ScreenBOnly
            };
            if (value < 4) SetMirroringType(modes[value]);
        } else if (address >= 0x9002 && address <= 0x9003) _mode = (value >> 1) & 1;
        else if (address >= 0xA000 && address <= 0xA003) {
            _prg0 = (value & 0x1F) << 1;
            _prg1 = _prg0 | 1;
        } else if (address >= 0xB000 && address <= 0xE003) {
            if (_chrRamSize) _outer = (value & 8) << 2;
            else {
                unsigned slot = ((((address >> 12) & 7) - 3) << 1) | ((address >> 1) & 1);
                if (address & 1) _chrHigh[slot] = value & 0x1F;
                else _chrLow[slot] = value & 15;
            }
        } else if (address == 0xF000) _irq.SetReloadNibble(value, false);
        else if (address == 0xF001) _irq.SetReloadNibble(value, true);
        else if (address == 0xF002) { _irq.SetControl(value); SetIrq(false); }
        else if (address == 0xF003) { _irq.Acknowledge(); SetIrq(false); }
        UpdateState();
    }
public:
    void ProcessCpuClock() override { if (_irq.Clock()) SetIrq(true); }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && _irq.VisitState(state)
            && state.Field("t230.prg0", _prg0)
            && state.Field("t230.prg1", _prg1)
            && state.Field("t230.mode", _mode)
            && state.Field("t230.outer", _outer)
            && state.Field("t230.chrHigh", _chrHigh)
            && state.Field("t230.chrLow", _chrLow);
    }
};

class Ax5705 final : public Board {
    std::array<uint8_t, 8> _chr{};
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    void InitMapper() override {
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        for (unsigned slot = 0; slot < 8; ++slot) SelectChrPage(slot, 0);
    }
    void UpdateChr(unsigned slot, uint8_t value, bool low) {
        if (low) _chr[slot] = (_chr[slot] & 0xF0) | (value & 15);
        else _chr[slot] = (_chr[slot] & 15) | ((((value & 4) >> 1) | ((value & 2) << 1) | (value & 9)) << 4);
        SelectChrPage(slot, _chr[slot]);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0xA008) {
            bool low = !(address & 1);
            switch (address & 0xF00E) {
                case 0xA008: UpdateChr(0, value, low); break;
                case 0xA00A: UpdateChr(1, value, low); break;
                case 0xC000: UpdateChr(2, value, low); break;
                case 0xC002: UpdateChr(3, value, low); break;
                case 0xC008: UpdateChr(4, value, low); break;
                case 0xC00A: UpdateChr(5, value, low); break;
                case 0xE000: UpdateChr(6, value, low); break;
                case 0xE002: UpdateChr(7, value, low); break;
            }
        } else {
            uint8_t bank = ((value & 2) << 2) | ((value & 8) >> 2) | (value & 5);
            switch (address & 0xF00F) {
                case 0x8000: SelectPrgPage(0, bank); break;
                case 0xA000: SelectPrgPage(1, bank); break;
                case 0x8008: SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical); break;
            }
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("ax5705.chr", _chr);
    }
};

} // namespace cupid::boards
#endif
