/*
 * whirlwind_117.hpp - Whirlwind cartridge boards
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
#ifndef CUPID_BOARDS_WHIRLWIND_117_HPP
#define CUPID_BOARDS_WHIRLWIND_117_HPP

#include "runtime.hpp"
#include <cstring>

namespace cupid::boards {

class Whirlwind40 final : public Board {
    uint16_t _irqCounter = 0;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        _irqCounter = 0;
        SetCpuMemoryMapping(0x6000, 0x7FFF, 6, PrgMemoryType::PrgRom);
        SelectPrgPage(0, 4);
        SelectPrgPage(1, 5);
        SelectPrgPage(3, 7);
        SelectChrPage(0, 0);
    }

    void ProcessCpuClock() override {
        if (_irqCounter && --_irqCounter == 0) SetIrq(true);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE000) {
            case 0x8000:
                _irqCounter = 0;
                SetIrq(false);
                break;
            case 0xA000:
                _irqCounter = 4096;
                break;
            case 0xE000:
                SelectPrgPage(2, value);
                break;
        }
    }
};

class Lh32 final : public Board {
    uint8_t _prgReg = 0;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0x6000; }

    void UpdateState() {
        SetCpuMemoryMapping(0x6000, 0x7FFF, _prgReg, PrgMemoryType::PrgRom);
    }

    void InitMapper() override {
        _prgReg = 0;
        SelectChrPage(0, 0);
        SelectPrgPage(0, static_cast<uint16_t>(-4));
        SelectPrgPage(1, static_cast<uint16_t>(-3));
        SelectPrgPage(2, 0, PrgMemoryType::WorkRam);
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        UpdateState();
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        _prgReg = value;
        UpdateState();
    }
};

class Smb2j final : public Board {
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;

    uint16_t GetPrgPageSize() override { return 0x1000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4122; }
    uint16_t RegisterEndAddress() override { return 0x4122; }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        SelectPrgPage4x(0, 0);
        SelectPrgPage4x(1, 4);
        SelectChrPage(0, 0);
        if (_prgSize >= 0x10000)
            AddRegisterRange(0x4022, 0x4022, MemoryOperation::Write);
        SetCpuMemoryMapping(0x5000, 0x5FFF,
                            static_cast<int16_t>(GetPrgPageCount() - 3), PrgMemoryType::PrgRom);
        SetCpuMemoryMapping(0x6000, 0x6FFF,
                            static_cast<int16_t>(GetPrgPageCount() - 2), PrgMemoryType::PrgRom);
        SetCpuMemoryMapping(0x7000, 0x7FFF,
                            static_cast<int16_t>(GetPrgPageCount() - 1), PrgMemoryType::PrgRom);
        _irqCounter = 0;
        _irqEnabled = false;
    }

    void ProcessCpuClock() override {
        if (!_irqEnabled) return;
        _irqCounter = static_cast<uint16_t>((_irqCounter + 1) & 0x0FFF);
        if (_irqCounter == 0) {
            _irqEnabled = false;
            SetIrq(true);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x4022) {
            SelectPrgPage4x(0, static_cast<uint16_t>((value & 1) << 2));
            SelectPrgPage4x(1, static_cast<uint16_t>(((value & 1) << 2) + 4));
        } else if (address == 0x4122) {
            _irqEnabled = (value & 3) != 0;
            _irqCounter = 0;
            SetIrq(false);
        }
    }
};

class Lh51 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 13);
        SelectPrgPage(2, 14);
        SelectPrgPage(3, 15);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE000) {
            case 0x8000:
                SelectPrgPage(0, value & 0x0F);
                break;
            case 0xE000:
                SetMirroringType(value & 8 ? MirroringType::Horizontal : MirroringType::Vertical);
                break;
        }
    }
};

class Lh10 final : public Board {
    uint8_t _currentRegister = 0;
    uint8_t _regs[8]{};

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void UpdateState() {
        SetCpuMemoryMapping(0x6000, 0x7FFF, -2, PrgMemoryType::PrgRom);
        SelectPrgPage(0, _regs[6]);
        SelectPrgPage(1, _regs[7]);
        SelectPrgPage(2, 0, PrgMemoryType::WorkRam);
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }

    void InitMapper() override {
        std::memset(_regs, 0, sizeof(_regs));
        _currentRegister = 0;
        SelectChrPage(0, 0);
        RemoveRegisterRange(0xC000, 0xDFFF);
        UpdateState();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE001) {
            case 0x8000:
                _currentRegister = value & 7;
                break;
            case 0x8001:
                _regs[_currentRegister] = value;
                UpdateState();
                break;
        }
    }
};

} // namespace cupid::boards
#endif
