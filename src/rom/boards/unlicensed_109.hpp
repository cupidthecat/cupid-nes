/*
 * unlicensed_109.hpp - Unlicensed cartridge boards for mapper issue 109
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
#ifndef CUPID_BOARDS_UNLICENSED_109_HPP
#define CUPID_BOARDS_UNLICENSED_109_HPP
#include "runtime.hpp"

namespace cupid::boards {

class UnlPci556 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x7000; }
    uint16_t RegisterEndAddress() override { return 0x7FFF; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, value & 3);
        SelectChrPage(0, (value >> 2) & 3);
    }
};

class Mapper39 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    void Reset(bool) override { SelectPrgPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override { SelectPrgPage(0, value); }
};

class Mapper42 final : public Board {
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;
    uint8_t _prgReg = 0;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool EnableCpuClockHook() override { return true; }

    void UpdateState() {
        SetCpuMemoryMapping(0x6000, 0x7FFF, _prgReg & 0x0F, PrgMemoryType::PrgRom);
    }

    void InitMapper() override {
        _irqCounter = 0;
        _irqEnabled = false;
        _prgReg = 0;
        SelectPrgPage(0, -4);
        SelectPrgPage(1, -3);
        SelectPrgPage(2, -2);
        SelectPrgPage(3, -1);
        SelectChrPage(0, 0);
        UpdateState();
    }

    void ProcessCpuClock() override {
        if (!_irqEnabled) return;
        ++_irqCounter;
        if (_irqCounter >= 0x8000) _irqCounter -= 0x8000;
        SetIrq(_irqCounter >= 0x6000);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE003) {
            case 0x8000:
                if (_chrRomSize) SelectChrPage(0, value & 0x0F);
                break;
            case 0xE000:
                _prgReg = value & 0x0F;
                UpdateState();
                break;
            case 0xE001:
                SetMirroringType(value & 8 ? MirroringType::Horizontal : MirroringType::Vertical);
                break;
            case 0xE002:
                _irqEnabled = value == 2;
                if (!_irqEnabled) {
                    SetIrq(false);
                    _irqCounter = 0;
                }
                break;
        }
    }
};

class Mapper43 final : public Board {
    uint8_t _reg = 0;
    bool _swap = false;
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4020; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }
    bool EnableCpuClockHook() override { return true; }

    void UpdateState() {
        SetCpuMemoryMapping(0x6000, 0x7FFF, _swap ? 0 : 2, PrgMemoryType::PrgRom);
        SelectPrgPage(2, _reg);
        SelectPrgPage(3, _swap ? 8 : 9);
    }

    void InitMapper() override {
        _reg = 0;
        _swap = false;
        _irqCounter = 0;
        _irqEnabled = false;
        UpdateState();
        SetCpuMemoryMapping(0x5000, 0x5FFF, 8, PrgMemoryType::PrgRom);
        SelectPrgPage(0, 1);
        SelectPrgPage(1, 0);
        SelectChrPage(0, 0);
    }

    void ProcessCpuClock() override {
        if (!_irqEnabled) return;
        if (++_irqCounter >= 4096) {
            _irqEnabled = false;
            SetIrq(true);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        static constexpr uint8_t lut[8] = {4, 3, 5, 3, 6, 3, 7, 3};
        switch (address & 0xF1FF) {
            case 0x4022:
                _reg = lut[value & 7];
                UpdateState();
                break;
            case 0x4120:
                _swap = (value & 1) != 0;
                UpdateState();
                break;
            case 0x4122:
            case 0x8122:
                _irqEnabled = (value & 1) != 0;
                SetIrq(false);
                _irqCounter = 0;
                break;
        }
    }
};

class ColorDreams46 final : public Board {
    uint8_t _regs[2]{};

    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void UpdateState() {
        SelectPrgPage(0, ((_regs[0] & 0x0F) << 1) | (_regs[1] & 1));
        SelectChrPage(0, ((_regs[0] & 0xF0) >> 1) | ((_regs[1] & 0x70) >> 4));
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[address < 0x8000 ? 0 : 1] = value;
        UpdateState();
    }

    void InitMapper() override {
        _regs[0] = _regs[1] = 0;
        UpdateState();
    }

    void Reset(bool) override {
        _regs[0] = _regs[1] = 0;
        UpdateState();
    }
};

class Mapper50 final : public Board {
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;

    uint16_t RegisterStartAddress() override { return 0x4020; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        _irqCounter = 0;
        _irqEnabled = false;
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0x0F, PrgMemoryType::PrgRom);
        SelectPrgPage(0, 8);
        SelectPrgPage(1, 9);
        SelectPrgPage(3, 0x0B);
        SelectChrPage(0, 0);
    }

    void ProcessCpuClock() override {
        if (!_irqEnabled) return;
        if (++_irqCounter == 0x1000) {
            SetIrq(true);
            _irqEnabled = false;
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0x4120) {
            case 0x4020:
                SelectPrgPage(2, (value & 8) | ((value & 1) << 2) | ((value & 6) >> 1));
                break;
            case 0x4120:
                if (value & 1) {
                    _irqEnabled = true;
                } else {
                    SetIrq(false);
                    _irqCounter = 0;
                    _irqEnabled = false;
                }
                break;
        }
    }
};

class Bmc51 final : public Board {
    uint8_t _bank = 0;
    uint8_t _mode = 0;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }

    void UpdateState() {
        if (_mode & 1) {
            SelectPrgPage4x(0, _bank << 2);
            SetCpuMemoryMapping(0x6000, 0x7FFF, 0x23 | (_bank << 2), PrgMemoryType::PrgRom);
        } else {
            SelectPrgPage2x(0, (_bank << 2) | _mode);
            SelectPrgPage2x(1, (_bank << 2) | 0x0E);
            SetCpuMemoryMapping(0x6000, 0x7FFF, 0x2F | (_bank << 2), PrgMemoryType::PrgRom);
        }
        SetMirroringType(_mode == 3 ? MirroringType::Horizontal : MirroringType::Vertical);
    }

    void InitMapper() override {
        _bank = 0;
        _mode = 1;
        UpdateState();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0x7FFF) {
            _mode = ((value >> 3) & 2) | ((value >> 1) & 1);
        } else if (address >= 0xC000 && address <= 0xDFFF) {
            _bank = value & 0x0F;
            _mode = ((value >> 3) & 2) | (_mode & 1);
        } else {
            _bank = value & 0x0F;
        }
        UpdateState();
    }
};

class Supervision final : public Board {
    static constexpr uint32_t EpromCrc = 0x63794E25;
    uint8_t _regs[2]{};
    bool _epromFirst = false;

    static uint32_t Crc32(const uint8_t *bytes, uint32_t size) {
        uint32_t crc = 0xFFFFFFFFu;
        for (uint32_t i = 0; i < size; ++i) {
            crc ^= bytes[i];
            for (unsigned bit = 0; bit < 8; ++bit)
                crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
        return crc ^ 0xFFFFFFFFu;
    }

    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void UpdateState() {
        uint16_t r = static_cast<uint16_t>(_regs[0] << 3) & 0x78;
        SetCpuMemoryMapping(0x6000, 0x7FFF, (r << 1 | 0x0F) + (_epromFirst ? 4 : 0),
                            PrgMemoryType::PrgRom);
        if (_regs[0] & 0x10) {
            SelectPrgPage2x(0, ((r | (_regs[1] & 7)) + (_epromFirst ? 2 : 0)) << 1);
            SelectPrgPage2x(1, ((r | 7) + (_epromFirst ? 2 : 0)) << 1);
        } else {
            SelectPrgPage2x(0, (_epromFirst ? 0 : 0x80) << 1);
            SelectPrgPage2x(1, (_epromFirst ? 1 : 0x80) << 1);
        }
        SetMirroringType(_regs[0] & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
    }

    void InitMapper() override {
        _epromFirst = _prgSize >= 0x8000 && Crc32(_prgRom, 0x8000) == EpromCrc;
        _regs[0] = _regs[1] = 0;
        UpdateState();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[address < 0x8000 ? 0 : 1] = value;
        UpdateState();
    }
};

class NovelDiamond final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t) override {
        SelectPrgPage(0, address & 3);
        SelectChrPage(0, address & 7);
    }
};

class Mapper57 final : public Board {
    uint8_t _registers[2]{};

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void UpdateState() {
        SetMirroringType(_registers[1] & 8 ? MirroringType::Horizontal : MirroringType::Vertical);
        SelectChrPage(0, ((_registers[0] & 0x40) >> 3) | ((_registers[0] | _registers[1]) & 7));
        if (_registers[1] & 0x10) {
            SelectPrgPage(0, (_registers[1] >> 5) & 6);
            SelectPrgPage(1, ((_registers[1] >> 5) & 6) + 1);
        } else {
            SelectPrgPage(0, (_registers[1] >> 5) & 7);
            SelectPrgPage(1, (_registers[1] >> 5) & 7);
        }
    }

    void InitMapper() override {
        _registers[0] = _registers[1] = 0;
        UpdateState();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0x8800) {
            case 0x8000: _registers[0] = value; break;
            case 0x8800: _registers[1] = value; break;
        }
        UpdateState();
    }
};

class Mapper58 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 1);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t) override {
        uint8_t prgBank = address & 7;
        if (address & 0x40) {
            SelectPrgPage(0, prgBank);
            SelectPrgPage(1, prgBank);
        } else {
            SelectPrgPage2x(0, prgBank & 6);
        }
        SelectChrPage(0, (address >> 3) & 7);
        SetMirroringType(address & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class UnlD1038 final : public Board {
    bool _returnDipSwitch = false;

    uint32_t GetDipSwitchCount() override { return 2; }
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        _returnDipSwitch = false;
        WriteRegister(0x8000, 0);
    }

    uint8_t ReadRegister(uint16_t address) override {
        return _returnDipSwitch ? static_cast<uint8_t>(GetDipSwitches()) : InternalReadRam(address);
    }

    void WriteRegister(uint16_t address, uint8_t) override {
        if (address & 0x80) {
            SelectPrgPage(0, (address & 0x70) >> 4);
            SelectPrgPage(1, (address & 0x70) >> 4);
        } else {
            SelectPrgPage2x(0, (address & 0x60) >> 4);
        }
        SelectChrPage(0, address & 7);
        SetMirroringType(address & 8 ? MirroringType::Horizontal : MirroringType::Vertical);
        _returnDipSwitch = (address & 0x100) != 0;
    }
};

} // namespace cupid::boards
#endif
