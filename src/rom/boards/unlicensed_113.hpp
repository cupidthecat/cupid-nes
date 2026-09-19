/*
 * unlicensed_113.hpp - Multicart read latches, bank wiring, and CPU timers
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
#ifndef CUPID_BOARDS_UNLICENSED_113_HPP
#define CUPID_BOARDS_UNLICENSED_113_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Mapper234 final : public Board {
    std::array<uint8_t, 2> _regs{};
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0xFF80; }
    uint16_t RegisterEndAddress() override { return 0xFF9F; }
    bool AllowRegisterRead() override { return true; }
    bool HasBusConflicts() override { return true; }
    void UpdateState() {
        if (_regs[0] & 0x40) {
            SelectPrgPage(0, (_regs[0] & 0x0E) | (_regs[1] & 1));
            SelectChrPage(0, ((_regs[0] << 2) & 0x38) | ((_regs[1] >> 4) & 7));
        } else {
            SelectPrgPage(0, _regs[0] & 0x0F);
            SelectChrPage(0, ((_regs[0] << 2) & 0x3C) | ((_regs[1] >> 4) & 3));
        }
        SetMirroringType(_regs[0] & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
    void InitMapper() override { AddRegisterRange(0xFFE8, 0xFFF8); UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0xFF9F) {
            if (_regs[0] & 0x3F) return;
            _regs[0] = value;
        } else _regs[1] = value & 0x71;
        UpdateState();
    }
    uint8_t ReadRegister(uint16_t address) override {
        uint8_t value = InternalReadRam(address);
        WriteRegister(address, value);
        return value;
    }
};

class Bmc235 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage2x(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        static constexpr uint8_t config[4][4][2] = {
            {{0, 0}, {0, 1}, {0, 1}, {0, 1}},
            {{0, 0}, {0, 1}, {0x20, 0}, {0, 1}},
            {{0, 0}, {0, 1}, {0x20, 0}, {0x40, 0}},
            {{0, 0}, {0x20, 0}, {0x40, 0}, {0x60, 0}}
        };
        SetMirroringType(address & 0x400 ? MirroringType::ScreenAOnly
                         : address & 0x2000 ? MirroringType::Horizontal : MirroringType::Vertical);
        uint32_t pages = GetPrgPageCount();
        unsigned mode = pages == 64 ? 0 : pages == 128 ? 1 : pages == 256 ? 2 : 3;
        const uint8_t *chip = config[mode][(address >> 8) & 3];
        uint8_t bank = chip[0] | (address & 0x1F);
        if (chip[1]) RemoveCpuMemoryMapping(0x8000, 0xFFFF);
        else if (address & 0x800) {
            bank = static_cast<uint8_t>((bank << 1) | ((address >> 12) & 1));
            SelectPrgPage(0, bank);
            SelectPrgPage(1, bank);
        } else SelectPrgPage2x(0, bank << 1);
    }
public:
    void Reset(bool) override { SelectPrgPage2x(0, 0); }
};

class Bmc70in1 final : public Board {
    uint8_t _mode = 0, _outer = 0, _prg = 0, _chr = 0;
    bool _useOuter = false;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetDipSwitchCount() override { return 4; }
    bool AllowRegisterRead() override { return true; }
    void UpdateState() {
        unsigned bank = _outer | _prg;
        if (_mode == 0x20) SelectPrgPage2x(0, bank & 0xFE);
        else {
            SelectPrgPage(0, bank);
            SelectPrgPage(1, _mode == 0x30 ? bank : _outer | 7);
        }
        if (!_useOuter) SelectChrPage(0, _chr);
    }
    void InitMapper() override { _useOuter = !HasChrRom(); SelectChrPage(0, 0); UpdateState(); }
    uint8_t ReadRegister(uint16_t address) override {
        return InternalReadRam(_mode == 0x10 ? (address & 0xFFF0) | GetDipSwitches() : address);
    }
    void WriteRegister(uint16_t address, uint8_t) override {
        if (address & 0x4000) { _mode = address & 0x30; _prg = address & 7; }
        else {
            SetMirroringType(address & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
            if (_useOuter) _outer = (address & 3) << 3;
            else _chr = address & 7;
        }
        UpdateState();
    }
public:
    void Reset(bool) override { _mode = 0; _outer = 0; }
};

class Mapper240 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4020; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, value >> 4);
        SelectChrPage(0, value & 15);
    }
};

class Mapper241 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override { SelectPrgPage(0, value); }
};

class Mapper244 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        static constexpr uint8_t prg[4][4] = {
            {0, 1, 2, 3}, {3, 2, 1, 0}, {0, 2, 1, 3}, {3, 1, 2, 0}
        };
        static constexpr uint8_t chr[8][8] = {
            {0, 1, 2, 3, 4, 5, 6, 7}, {0, 2, 1, 3, 4, 6, 5, 7},
            {0, 1, 4, 5, 2, 3, 6, 7}, {0, 4, 1, 5, 2, 6, 3, 7},
            {0, 4, 2, 6, 1, 5, 3, 7}, {0, 2, 4, 6, 1, 3, 5, 7},
            {7, 6, 5, 4, 3, 2, 1, 0}, {7, 6, 5, 4, 3, 2, 1, 0}
        };
        if (value & 8) SelectChrPage(0, chr[(value >> 4) & 7][value & 7]);
        else SelectPrgPage(0, prg[(value >> 4) & 3][value & 3]);
    }
};

class Mapper246 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x800; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0x67FF; }
    void InitMapper() override { SelectPrgPage(3, 0xFF); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 7) <= 3) SelectPrgPage(address & 3, value);
        else SelectChrPage(address & 3, value);
    }
public:
    void Reset(bool) override { SelectPrgPage(3, 0xFF); }
};

class Bmc255 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned bit = address & 0x1000 ? 0 : 1;
        unsigned bank = ((address >> 8) & 0x40) | ((address >> 6) & 0x3F);
        SelectPrgPage(0, bank & ~bit);
        SelectPrgPage(1, bank | bit);
        SelectChrPage(0, ((address >> 8) & 0x40) | (address & 0x3F));
        SetMirroringType(address & 0x2000 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Bmc810544CA1 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {}
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned bank = (address >> 6) & 0xFFFE;
        if (address & 0x40) SelectPrgPage2x(0, bank);
        else {
            bank |= (address >> 5) & 1;
            SelectPrgPage(0, bank);
            SelectPrgPage(1, bank);
        }
        SelectChrPage(0, address & 15);
        SetMirroringType(address & 0x10 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
public:
    void Reset(bool) override { WriteRegister(0x8000, 0); }
};

class Yoko final : public Board {
    std::array<uint8_t, 7> _regs{};
    std::array<uint8_t, 4> _extra{};
    uint8_t _mode = 0, _bank = 0;
    uint16_t _counter = 0;
    bool _enabled = false;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x800; }
    uint16_t RegisterStartAddress() override { return 0x5000; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    uint32_t GetDipSwitchCount() override { return 2; }
    bool AllowRegisterRead() override { return true; }
    bool EnableCpuClockHook() override { return true; }
    void UpdateState() {
        SetMirroringType(_mode & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
        for (unsigned slot = 0; slot < 4; ++slot) SelectChrPage(slot, _regs[slot + 3]);
        if (_mode & 0x10) {
            unsigned outer = (_bank & 8) << 1;
            for (unsigned slot = 0; slot < 3; ++slot) SelectPrgPage(slot, outer | (_regs[slot] & 15));
            SelectPrgPage(3, outer | 15);
        } else if (_mode & 8) SelectPrgPage4x(0, (_bank & 0xFE) << 1);
        else { SelectPrgPage2x(0, _bank << 1); SelectPrgPage2x(1, static_cast<uint16_t>(-2)); }
    }
    void InitMapper() override {
        RemoveRegisterRange(0x5000, 0x53FF, MemoryOperation::Write);
        AddRegisterRange(0x8000, 0xFFFF, MemoryOperation::Write);
        UpdateState();
    }
    uint8_t ReadRegister(uint16_t address) override {
        return address <= 0x53FF ? GetOpenBus(0xFC) | GetDipSwitches() : _extra[address & 3];
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) { _extra[address & 3] = value; return; }
        switch (address & 0x8C17) {
            case 0x8000: _bank = value; break;
            case 0x8400: _mode = value; break;
            case 0x8800: _counter = (_counter & 0xFF00) | value; SetIrq(false); return;
            case 0x8801: _counter = (_counter & 0xFF) | (value << 8); _enabled = (_mode & 0x80) != 0; return;
            case 0x8C00: case 0x8C01: case 0x8C02: _regs[address & 3] = value; break;
            case 0x8C10: case 0x8C11: _regs[3 + (address & 1)] = value; break;
            case 0x8C16: case 0x8C17: _regs[5 + (address & 1)] = value; break;
            default: return;
        }
        UpdateState();
    }
public:
    void Reset(bool soft) override { if (soft) { _mode = 0; _bank = 0; } }
    void ProcessCpuClock() override {
        if (_enabled && --_counter == 0) { _enabled = false; _counter = 0xFFFF; SetIrq(true); }
    }
};

class T262 final : public Board {
    bool _locked = false, _mode = false;
    uint8_t _base = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectPrgPage(1, 7); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (!_locked) {
            _base = ((address & 0x60) >> 2) | ((address & 0x100) >> 3);
            _mode = (address & 0x80) != 0;
            _locked = (address & 0x2000) != 0;
            SetMirroringType(address & 2 ? MirroringType::Horizontal : MirroringType::Vertical);
        }
        SelectPrgPage(0, _base | (value & 7));
        SelectPrgPage(1, _base | (_mode ? value & 7 : 7));
    }
};

class CityFighter final : public Board {
    uint8_t _prg = 0, _mode = 0, _mirroring = 0;
    std::array<uint8_t, 8> _chr{};
    uint16_t _counter = 0;
    bool _enabled = false;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    bool EnableCpuClockHook() override { return true; }
    void UpdateState() {
        SelectPrgPage4x(0, _prg);
        if (!_mode) SelectPrgPage(2, _prg);
        for (unsigned slot = 0; slot < 8; ++slot) SelectChrPage(slot, _chr[slot]);
        static constexpr MirroringType modes[] = {
            MirroringType::Vertical, MirroringType::Horizontal,
            MirroringType::ScreenAOnly, MirroringType::ScreenBOnly
        };
        SetMirroringType(modes[_mirroring]);
    }
    void InitMapper() override { UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xF00C) {
            case 0x9000: _prg = value & 12; _mirroring = value & 3; break;
            case 0x9004: case 0x9008: case 0x900C:
                if (address & 0x800) WriteCpuBus(0x4011, (value & 15) << 3);
                else _prg = value & 12;
                break;
            case 0xC000: case 0xC004: case 0xC008: case 0xC00C: _mode = value & 1; break;
            case 0xA000: case 0xA004: case 0xA008: case 0xA00C:
            case 0xB000: case 0xB004: case 0xB008: case 0xB00C:
            case 0xD000: case 0xD004: case 0xD008: case 0xD00C:
            case 0xE000: case 0xE004: case 0xE008: case 0xE00C: {
                unsigned high = address >> 12;
                unsigned slot = (high == 0xD ? 0 : high == 0xA ? 2 : high == 0xB ? 4 : 6) + ((address >> 3) & 1);
                _chr[slot] = static_cast<uint8_t>(address & 4 ? (_chr[slot] & 15) | (value << 4)
                                                                            : (_chr[slot] & 0xF0) | (value & 15));
                break;
            }
            case 0xF000: _counter = (_counter & 0x1E0) | ((value & 15) << 1); break;
            case 0xF004: _counter = (_counter & 0x1E) | ((value & 15) << 5); break;
            case 0xF008: _enabled = (value & 2) != 0; SetIrq(false); break;
        }
        UpdateState();
    }
public:
    void ProcessCpuClock() override { if (_enabled && --_counter == 0) SetIrq(true); }
};

} // namespace cupid::boards
#endif
