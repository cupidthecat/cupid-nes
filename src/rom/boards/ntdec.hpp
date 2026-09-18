/*
 * ntdec.hpp - NTDEC cartridge banking and IRQ hardware
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
#ifndef CUPID_BOARDS_NTDEC_HPP
#define CUPID_BOARDS_NTDEC_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Caltron41 final : public Board {
    uint8_t _prgBank = 0, _chrBank = 0;
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { AddRegisterRange(0x6000, 0x67FF, MemoryOperation::Write); }
    void Reset(bool) override {
        _chrBank = _prgBank = 0;
        WriteRegister(0x6000, 0);
        WriteRegister(0x8000, 0);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0x67FF) {
            _prgBank = address & 7;
            _chrBank = (_chrBank & 3) | ((address >> 1) & 0x0C);
            SelectPrgPage(0, _prgBank);
            SelectChrPage(0, _chrBank);
            SetMirroringType(address & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
        } else if (_prgBank >= 4) {
            _chrBank = (_chrBank & 0x0C) | (value & 3);
            SelectChrPage(0, _chrBank);
        }
    }
};

class Ntdec63 final : public Board {
    bool _openBus = false;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void Reset(bool) override { _openBus = false; }
    void WriteRegister(uint16_t address, uint8_t) override {
        _openBus = (address & 0x300) == 0x300;
        uint16_t outer = (address >> 1) & 0x1FC;
        uint16_t lower = address & 2 ? 0 : ((address >> 1) & 2);
        if (_openBus) {
            RemoveCpuMemoryMapping(0x8000, 0xBFFF);
        } else {
            SelectPrgPage(0, outer | lower);
            SelectPrgPage(1, outer | (lower + 1));
        }
        SelectPrgPage(2, outer | (address & 2 ? 2 : lower));
        SelectPrgPage(3, address & 0x800
            ? (address & 0x7C) | (address & 6 ? 3 : 1)
            : outer | (address & 2 ? 3 : (lower | 1)));
        SetMirroringType(address & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Ntdec112 final : public Board {
    uint8_t _currentReg = 0, _outerChrBank = 0;
    uint8_t _registers[8]{};
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    void UpdateState() {
        SelectPrgPage(0, _registers[0]);
        SelectPrgPage(1, _registers[1]);
        SelectChrPage2x(0, _registers[2]);
        SelectChrPage2x(1, _registers[3]);
        SelectChrPage(4, _registers[4] | ((_outerChrBank & 0x10) << 4));
        SelectChrPage(5, _registers[5] | ((_outerChrBank & 0x20) << 3));
        SelectChrPage(6, _registers[6] | ((_outerChrBank & 0x40) << 2));
        SelectChrPage(7, _registers[7] | ((_outerChrBank & 0x80) << 1));
    }
    void InitMapper() override {
        SetMirroringType(MirroringType::Vertical);
        AddRegisterRange(0x4020, 0x5FFF, MemoryOperation::Write);
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        UpdateState();
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE001) {
            case 0x8000: _currentReg = value & 7; break;
            case 0xA000: _registers[_currentReg] = value; break;
            case 0xC000: _outerChrBank = value; break;
            case 0xE000:
                SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
                break;
        }
        UpdateState();
    }
};

// This board follows the documented wiring; its hardware remains unverified.
class Ntdec174 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        uint8_t prg = (address >> 4) & 7;
        if (address & 0x80) {
            SelectPrgPage2x(0, prg & 0xFE);
        } else {
            SelectPrgPage(0, prg);
            SelectPrgPage(1, prg);
        }
        SelectChrPage(0, (address >> 1) & 7);
        SetMirroringType(address & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class NtdecTc112 final : public Board {
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0x7FFF; }
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x800; }
    void InitMapper() override {
        SelectPrgPage(1, static_cast<uint16_t>(-3));
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 3) {
            case 0:
                SelectChrPage(0, value >> 1);
                SelectChrPage(1, (value >> 1) + 1);
                break;
            case 1: SelectChrPage(2, value >> 1); break;
            case 2: SelectChrPage(3, value >> 1); break;
            case 3: SelectPrgPage(0, value); break;
        }
    }
};

class Ntdec221 final : public Board {
    uint16_t _mode = 0;
    uint8_t _prgReg = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void UpdateState() {
        uint16_t outer = (_mode & 0xFC) >> 2;
        if (_mode & 2) {
            if (_mode & 0x100) {
                SelectPrgPage(0, outer | _prgReg);
                SelectPrgPage(1, outer | 7);
            } else {
                SelectPrgPage2x(0, outer | (_prgReg & 6));
            }
        } else {
            SelectPrgPage(0, outer | _prgReg);
            SelectPrgPage(1, outer | _prgReg);
        }
        SetMirroringType(_mode & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override { SelectChrPage(0, 0); UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t) override {
        if ((address & 0xC000) == 0x8000) _mode = address;
        else if ((address & 0xC000) == 0xC000) _prgReg = address & 7;
        UpdateState();
    }
};

class NtdecNtd03 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {}
    void Reset(bool) override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        uint8_t prg = (address >> 10) & 0x1E;
        uint8_t chr = ((address & 0x300) >> 5) | (address & 7);
        if (address & 0x80) {
            SelectPrgPage(0, prg | ((address >> 6) & 1));
            SelectPrgPage(1, prg | ((address >> 6) & 1));
        } else {
            SelectPrgPage2x(0, prg & 0xFE);
        }
        SelectChrPage(0, chr);
        SetMirroringType(address & 0x400 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Tf1201 final : public Board {
    uint8_t _chrRegs[8]{}, _prgRegs[2]{};
    bool _swapPrg = false;
    uint8_t _irqCounter = 0, _irqReloadValue = 0;
    int16_t _irqScaler = 0;
    bool _irqEnabled = false;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    bool EnableCpuClockHook() override { return true; }
    void UpdateChr() {
        for (uint16_t slot = 0; slot < 8; ++slot) SelectChrPage(slot, _chrRegs[slot]);
    }
    void UpdatePrg() {
        SelectPrgPage(0, _swapPrg ? static_cast<uint16_t>(-2) : _prgRegs[0]);
        SelectPrgPage(2, _swapPrg ? _prgRegs[0] : static_cast<uint16_t>(-2));
        SelectPrgPage(1, _prgRegs[1]);
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }
    void InitMapper() override { UpdateChr(); UpdatePrg(); }
    void ProcessCpuClock() override {
        if (_irqEnabled) {
            _irqScaler -= 3;
            if (_irqScaler <= 0) {
                _irqScaler += 341;
                if (++_irqCounter == 0) SetIrq(true);
            }
        }
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        address = (address & 0xF003) | ((address & 0x0C) >> 2);
        if (address >= 0xB000 && address <= 0xE003) {
            uint8_t slot = (((address >> 11) - 6) | (address & 1)) & 7;
            uint8_t shift = (address & 2) << 1;
            _chrRegs[slot] = (_chrRegs[slot] & (0xF0 >> shift)) | ((value & 15) << shift);
            UpdateChr();
        } else {
            switch (address & 0xF003) {
                case 0x8000: _prgRegs[0] = value; UpdatePrg(); break;
                case 0xA000: _prgRegs[1] = value; UpdatePrg(); break;
                case 0x9000:
                    SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
                    break;
                case 0x9001: _swapPrg = (value & 3) != 0; UpdatePrg(); break;
                case 0xF000: _irqReloadValue = (_irqReloadValue & 0xF0) | (value & 15); break;
                case 0xF002: _irqReloadValue = (_irqReloadValue & 15) | (value << 4); break;
                case 0xF001:
                    _irqEnabled = (value & 2) != 0;
                    if (_irqEnabled) { _irqScaler = 341; _irqCounter = _irqReloadValue; }
                    SetIrq(false);
                    break;
                case 0xF003: SetIrq(false); break;
            }
        }
    }
};

} // namespace cupid::boards
#endif
