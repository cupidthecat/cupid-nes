/*
 * waixing_116.hpp - Waixing cartridge boards
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
#ifndef CUPID_BOARDS_WAIXING_116_HPP
#define CUPID_BOARDS_WAIXING_116_HPP

#include "runtime.hpp"
#include "a12_watcher.hpp"
#include "vrc_irq.hpp"
#include <cstring>

namespace cupid::boards {

class Waixing162 final : public Board {
    uint8_t _regs[4]{};

    uint16_t RegisterStartAddress() override { return 0x5000; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void UpdateState() {
        switch (_regs[3] & 5) {
            case 0:
                SelectPrgPage(0, static_cast<uint16_t>((_regs[0] & 0x0C)
                            | (_regs[1] & 0x02) | ((_regs[2] & 0x0F) << 4)));
                break;
            case 1:
                SelectPrgPage(0, static_cast<uint16_t>((_regs[0] & 0x0C)
                            | ((_regs[2] & 0x0F) << 4)));
                break;
            case 4:
                SelectPrgPage(0, static_cast<uint16_t>((_regs[0] & 0x0E)
                            | ((_regs[1] >> 1) & 1) | ((_regs[2] & 0x0F) << 4)));
                break;
            case 5:
                SelectPrgPage(0, static_cast<uint16_t>((_regs[0] & 0x0F)
                            | ((_regs[2] & 0x0F) << 4)));
                break;
        }
    }

    void InitMapper() override {
        _regs[0] = 3;
        _regs[1] = 0;
        _regs[2] = 0;
        _regs[3] = 7;
        SelectChrPage(0, 0);
        UpdateState();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[(address >> 8) & 3] = value;
        UpdateState();
    }
};

class Waixing164 final : public Board {
    uint8_t _prgBank = 0x0F;

    uint16_t RegisterStartAddress() override { return 0x5000; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        _prgBank = 0x0F;
        SelectPrgPage(0, _prgBank);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0x7300) {
            case 0x5000:
                _prgBank = static_cast<uint8_t>((_prgBank & 0xF0) | (value & 0x0F));
                SelectPrgPage(0, _prgBank);
                break;
            case 0x5100:
                _prgBank = static_cast<uint8_t>((_prgBank & 0x0F) | ((value & 0x0F) << 4));
                SelectPrgPage(0, _prgBank);
                break;
        }
    }
};

class Fk23C final : public Board {
    uint8_t _prgBankingMode = 0;
    uint8_t _outerChrBankSize = 0;
    bool _selectChrRam = false;
    bool _mmc3ChrMode = true;
    bool _cnromChrMode = false;
    uint16_t _prgBaseBits = 0;
    uint8_t _chrBaseBits = 0;
    bool _extendedMmc3Mode = false;
    uint8_t _wramBankSelect = 0;
    bool _ramInFirstChrBank = false;
    bool _allowSingleScreenMirroring = false;
    bool _fk23RegistersEnabled = false;
    bool _wramConfigEnabled = false;
    bool _wramEnabled = false;
    bool _wramWriteProtected = false;
    bool _invertPrgA14 = false;
    bool _invertChrA12 = false;
    uint8_t _currentRegister = 0;
    uint8_t _irqReloadValue = 0;
    uint8_t _irqCounter = 0;
    bool _irqReload = false;
    bool _irqEnabled = false;
    uint8_t _mirroringReg = 0;
    uint8_t _cnromChrReg = 0;
    uint8_t _mmc3Registers[12]{};
    uint8_t _irqDelay = 0;
    A12Watcher _a12;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    uint32_t GetChrRamSize() override { return 0x40000; }
    uint16_t GetChrRamPageSize() override { return 0x0400; }
    uint32_t GetWorkRamSize() override { return 0x8000; }
    uint32_t GetWorkRamPageSize() override { return 0x2000; }
    bool EnableCpuClockHook() override { return true; }
    bool EnableVramAddressHook() override { return true; }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType = ChrMemoryType::Default) override {
        bool useRam = !HasChrRom() || (_selectChrRam && _chrRamSize)
                   || (_wramConfigEnabled && _ramInFirstChrBank && page <= 7);
        Board::SelectChrPage(slot, page, useRam ? ChrMemoryType::ChrRam
                                                : ChrMemoryType::ChrRom);
    }

    void UpdatePrg() {
        switch (_prgBankingMode) {
            case 0:
            case 1:
            case 2:
                if (_extendedMmc3Mode) {
                    uint8_t swap = _invertPrgA14 ? 2 : 0;
                    uint16_t outer = static_cast<uint16_t>(_prgBaseBits << 1);
                    SelectPrgPage(0 ^ swap, static_cast<uint16_t>(_mmc3Registers[6] | outer));
                    SelectPrgPage(1, static_cast<uint16_t>(_mmc3Registers[7] | outer));
                    SelectPrgPage(2 ^ swap, static_cast<uint16_t>(_mmc3Registers[8] | outer));
                    SelectPrgPage(3, static_cast<uint16_t>(_mmc3Registers[9] | outer));
                } else {
                    uint8_t swap = _invertPrgA14 ? 2 : 0;
                    uint8_t innerMask = static_cast<uint8_t>(0x3F >> _prgBankingMode);
                    uint16_t outer = static_cast<uint16_t>((_prgBaseBits << 1) & ~innerMask);
                    SelectPrgPage(0 ^ swap, static_cast<uint16_t>((_mmc3Registers[6] & innerMask) | outer));
                    SelectPrgPage(1, static_cast<uint16_t>((_mmc3Registers[7] & innerMask) | outer));
                    SelectPrgPage(2 ^ swap, static_cast<uint16_t>((0xFE & innerMask) | outer));
                    SelectPrgPage(3, static_cast<uint16_t>((0xFF & innerMask) | outer));
                }
                break;
            case 3:
                SelectPrgPage2x(0, static_cast<uint16_t>(_prgBaseBits << 1));
                SelectPrgPage2x(1, static_cast<uint16_t>(_prgBaseBits << 1));
                break;
            case 4:
                SelectPrgPage4x(0, static_cast<uint16_t>((_prgBaseBits & 0xFFE) << 1));
                break;
            default:
                break;
        }
    }

    void UpdateChr() {
        if (!_mmc3ChrMode) {
            uint16_t innerMask = _cnromChrMode ? (_outerChrBankSize ? 1 : 3) : 0;
            for (unsigned i = 0; i < 8; ++i)
                SelectChrPage(i, static_cast<uint16_t>((((_cnromChrReg & innerMask)
                              | _chrBaseBits) << 3) + i));
            return;
        }

        uint8_t swap = _invertChrA12 ? 4 : 0;
        if (_extendedMmc3Mode) {
            uint16_t outer = static_cast<uint16_t>(_chrBaseBits << 3);
            SelectChrPage(0 ^ swap, static_cast<uint16_t>(_mmc3Registers[0] | outer));
            SelectChrPage(1 ^ swap, static_cast<uint16_t>(_mmc3Registers[10] | outer));
            SelectChrPage(2 ^ swap, static_cast<uint16_t>(_mmc3Registers[1] | outer));
            SelectChrPage(3 ^ swap, static_cast<uint16_t>(_mmc3Registers[11] | outer));
            SelectChrPage(4 ^ swap, static_cast<uint16_t>(_mmc3Registers[2] | outer));
            SelectChrPage(5 ^ swap, static_cast<uint16_t>(_mmc3Registers[3] | outer));
            SelectChrPage(6 ^ swap, static_cast<uint16_t>(_mmc3Registers[4] | outer));
            SelectChrPage(7 ^ swap, static_cast<uint16_t>(_mmc3Registers[5] | outer));
        } else {
            uint8_t innerMask = _outerChrBankSize ? 0x7F : 0xFF;
            uint16_t outer = static_cast<uint16_t>((_chrBaseBits << 3) & ~innerMask);
            SelectChrPage(0 ^ swap, static_cast<uint16_t>(((_mmc3Registers[0] & 0xFE) & innerMask) | outer));
            SelectChrPage(1 ^ swap, static_cast<uint16_t>(((_mmc3Registers[0] | 1) & innerMask) | outer));
            SelectChrPage(2 ^ swap, static_cast<uint16_t>(((_mmc3Registers[1] & 0xFE) & innerMask) | outer));
            SelectChrPage(3 ^ swap, static_cast<uint16_t>(((_mmc3Registers[1] | 1) & innerMask) | outer));
            SelectChrPage(4 ^ swap, static_cast<uint16_t>((_mmc3Registers[2] & innerMask) | outer));
            SelectChrPage(5 ^ swap, static_cast<uint16_t>((_mmc3Registers[3] & innerMask) | outer));
            SelectChrPage(6 ^ swap, static_cast<uint16_t>((_mmc3Registers[4] & innerMask) | outer));
            SelectChrPage(7 ^ swap, static_cast<uint16_t>((_mmc3Registers[5] & innerMask) | outer));
        }
    }

    void UpdateState() {
        switch (_mirroringReg & (_allowSingleScreenMirroring ? 3 : 1)) {
            case 0: SetMirroringType(MirroringType::Vertical); break;
            case 1: SetMirroringType(MirroringType::Horizontal); break;
            case 2: SetMirroringType(MirroringType::ScreenAOnly); break;
            case 3: SetMirroringType(MirroringType::ScreenBOnly); break;
        }
        UpdatePrg();
        UpdateChr();

        if (_wramConfigEnabled) {
            uint8_t nextBank = static_cast<uint8_t>((_wramBankSelect + 1) & 3);
            PrgMemoryType type = HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam;
            SetCpuMemoryMapping(0x4000, 0x5FFF, nextBank, type, ReadWrite);
            SetCpuMemoryMapping(0x6000, 0x7FFF, _wramBankSelect, type, ReadWrite);
        } else {
            if (_wramEnabled)
                SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::WorkRam,
                                    _wramWriteProtected ? Read : ReadWrite);
            else
                RemoveCpuMemoryMapping(0x6000, 0x7FFF);
            RemoveCpuMemoryMapping(0x4000, 0x5FFF);
        }
    }

    void InitMapper() override {
        _prgBankingMode = 0;
        _outerChrBankSize = 0;
        _selectChrRam = false;
        _mmc3ChrMode = true;
        _cnromChrMode = false;
        _prgBaseBits = (_prgSize == 0x100000 && _prgSize == _chrRomSize) ? 0x20 : 0;
        _chrBaseBits = 0;
        _extendedMmc3Mode = false;
        _wramBankSelect = 0;
        _ramInFirstChrBank = false;
        _allowSingleScreenMirroring = false;
        _fk23RegistersEnabled = false;
        _wramConfigEnabled = false;
        _wramEnabled = false;
        _wramWriteProtected = false;
        _invertPrgA14 = false;
        _invertChrA12 = false;
        _currentRegister = 0;
        _irqReloadValue = _irqCounter = _irqDelay = 0;
        _irqReload = _irqEnabled = false;
        _mirroringReg = _cnromChrReg = 0;
        _a12 = {};
        const uint8_t initial[12] = {0, 2, 4, 5, 6, 7, 0, 1, 0xFE, 0xFF, 0xFF, 0xFF};
        std::memcpy(_mmc3Registers, initial, sizeof(initial));
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Write);
        UpdateState();
    }

    void Reset(bool softReset) override {
        if (softReset && _wramConfigEnabled && _selectChrRam && HasBattery()) {
            _prgBaseBits = 0;
            UpdateState();
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (_fk23RegistersEnabled || !_wramConfigEnabled) {
                if ((address & 0x5010) != 0x5010) return;
                switch (address & 3) {
                    case 0:
                        _prgBankingMode = value & 7;
                        _outerChrBankSize = static_cast<uint8_t>((value & 0x10) >> 4);
                        _selectChrRam = (value & 0x20) != 0;
                        _mmc3ChrMode = (value & 0x40) == 0;
                        _prgBaseBits = static_cast<uint16_t>((_prgBaseBits & ~0x180)
                                     | ((value & 0x80) << 1) | ((value & 8) << 4));
                        break;
                    case 1:
                        _prgBaseBits = static_cast<uint16_t>((_prgBaseBits & ~0x7F)
                                     | (value & 0x7F));
                        break;
                    case 2:
                        _prgBaseBits = static_cast<uint16_t>((_prgBaseBits & ~0x200)
                                     | ((value & 0x40) << 3));
                        _chrBaseBits = value;
                        _cnromChrReg = 0;
                        break;
                    case 3:
                        _extendedMmc3Mode = (value & 2) != 0;
                        _cnromChrMode = (value & 0x44) != 0;
                        break;
                }
                UpdateState();
            } else {
                WritePrgRam(address, value);
            }
            return;
        }

        if (_cnromChrMode && (address <= 0x9FFF || address >= 0xC000)) {
            _cnromChrReg = value & 3;
            UpdateState();
        }
        switch (address & 0xE001) {
            case 0x8000:
                if (_prgSize == 0x1000000 && (value == 0x46 || value == 0x47)) value ^= 1;
                _invertPrgA14 = (value & 0x40) != 0;
                _invertChrA12 = (value & 0x80) != 0;
                _currentRegister = value & 0x0F;
                UpdateState();
                break;
            case 0x8001: {
                uint8_t reg = _currentRegister & (_extendedMmc3Mode ? 0x0F : 7);
                if (reg < 12) {
                    _mmc3Registers[reg] = value;
                    UpdateState();
                }
                break;
            }
            case 0xA000:
                _mirroringReg = value & 3;
                UpdateState();
                break;
            case 0xA001:
                if (!(value & 0x20)) value &= 0xC0;
                _wramBankSelect = value & 3;
                _ramInFirstChrBank = (value & 4) != 0;
                _allowSingleScreenMirroring = (value & 8) != 0;
                _wramConfigEnabled = (value & 0x20) != 0;
                _fk23RegistersEnabled = (value & 0x40) != 0;
                _wramWriteProtected = (value & 0x40) != 0;
                _wramEnabled = (value & 0x80) != 0;
                UpdateState();
                break;
            case 0xC000:
                _irqReloadValue = value;
                break;
            case 0xC001:
                _irqCounter = 0;
                _irqReload = true;
                break;
            case 0xE000:
                _irqEnabled = false;
                SetIrq(false);
                break;
            case 0xE001:
                _irqEnabled = true;
                break;
        }
    }

public:
    void ProcessCpuClock() override {
        if (_irqDelay && --_irqDelay == 0) SetIrq(true);
    }

    void NotifyVramAddressChange(uint16_t address) override {
        if (!_a12.Rising(address, PpuFrameCycle())) return;
        if (_irqCounter == 0 || _irqReload) _irqCounter = _irqReloadValue;
        else --_irqCounter;
        if (_irqCounter == 0 && _irqEnabled) _irqDelay = 2;
        _irqReload = false;
    }
};

class Waixing178 final : public Board {
    uint8_t _regs[4]{};

    uint16_t RegisterStartAddress() override { return 0x4800; }
    uint16_t RegisterEndAddress() override { return 0x4FFF; }
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetWorkRamSize() override { return 0x8000; }

    void UpdateState() {
        uint16_t small = _regs[1] & 7;
        uint16_t big = _regs[2];
        if (_regs[0] & 2) {
            SelectPrgPage(0, static_cast<uint16_t>((big << 3) | small));
            if (_regs[0] & 4)
                SelectPrgPage(1, static_cast<uint16_t>((big << 3) | 6 | (_regs[1] & 1)));
            else
                SelectPrgPage(1, static_cast<uint16_t>((big << 3) | 7));
        } else {
            uint16_t bank = static_cast<uint16_t>((big << 3) | small);
            if (_regs[0] & 4) {
                SelectPrgPage(0, bank);
                SelectPrgPage(1, bank);
            } else {
                SelectPrgPage2x(0, bank);
            }
        }
        SetCpuMemoryMapping(0x6000, 0x7FFF, _regs[3] & 3,
                            PrgMemoryType::WorkRam, ReadWrite);
        SetMirroringType(_regs[0] & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }

    void InitMapper() override {
        std::memset(_regs, 0, sizeof(_regs));
        UpdateState();
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[address & 3] = value;
        UpdateState();
    }
};

class Waixing242 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        Reset(false);
        SelectChrPage(0, 0);
    }

    void Reset(bool) override {
        SelectPrgPage(0, 0);
        SetMirroringType(MirroringType::Vertical);
    }

    void WriteRegister(uint16_t address, uint8_t) override {
        SetMirroringType(address & 2 ? MirroringType::Horizontal : MirroringType::Vertical);
        SelectPrgPage(0, static_cast<uint16_t>((address >> 3) & 0x0F));
    }
};

class Waixing252 final : public Board {
    uint8_t _chrRegs[8]{};
    VrcIrq _irq;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    bool EnableCpuClockHook() override { return true; }

    void UpdateState() {
        for (unsigned i = 0; i < 8; ++i)
            SetPpuMemoryMapping(static_cast<uint16_t>(i * 0x400),
                                static_cast<uint16_t>(i * 0x400 + 0x3FF),
                                _chrRegs[i], ChrMemoryType::Default, ReadWrite);
    }

    void InitMapper() override {
        std::memset(_chrRegs, 0, sizeof(_chrRegs));
        _irq = VrcIrq{};
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }

    void ProcessCpuClock() override {
        if (_irq.Clock()) SetIrq(true);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0x8FFF) {
            SelectPrgPage(0, value);
        } else if (address >= 0xA000 && address <= 0xAFFF) {
            SelectPrgPage(1, value);
        } else if (address >= 0xB000 && address <= 0xEFFF) {
            uint8_t shift = address & 4;
            uint8_t bank = static_cast<uint8_t>((((address - 0xB000) >> 1 & 0x1800)
                         | (address << 7 & 0x0400)) / 0x400);
            _chrRegs[bank] = static_cast<uint8_t>((_chrRegs[bank] & (0xF0 >> shift))
                           | ((value & 0x0F) << shift));
            UpdateState();
        } else {
            switch (address & 0xF00C) {
                case 0xF000: _irq.SetReloadNibble(value, false); break;
                case 0xF004: _irq.SetReloadNibble(value, true); break;
                case 0xF008: _irq.SetControl(value); SetIrq(false); break;
                case 0xF00C: _irq.Acknowledge(); SetIrq(false); break;
            }
        }
    }
};

class Waixing253 final : public Board {
    uint8_t _chrLow[8]{};
    uint8_t _chrHigh[8]{};
    bool _forceChrRom = false;
    uint8_t _irqReloadValue = 0;
    uint8_t _irqCounter = 0;
    bool _irqEnabled = false;
    uint16_t _irqScaler = 0;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    uint32_t GetChrRamSize() override { return 0x0800; }
    uint16_t GetChrRamPageSize() override { return 0x0400; }
    bool EnableCpuClockHook() override { return true; }

    void UpdateChr() {
        for (uint16_t i = 0; i < 8; ++i) {
            uint16_t page = static_cast<uint16_t>(_chrLow[i] | (_chrHigh[i] << 8));
            if ((_chrLow[i] == 4 || _chrLow[i] == 5) && !_forceChrRom)
                SelectChrPage(i, page & 1, ChrMemoryType::ChrRam);
            else
                SelectChrPage(i, page);
        }
    }

    void InitMapper() override {
        std::memset(_chrLow, 0, sizeof(_chrLow));
        std::memset(_chrHigh, 0, sizeof(_chrHigh));
        _forceChrRom = false;
        _irqReloadValue = _irqCounter = 0;
        _irqEnabled = false;
        _irqScaler = 0;
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }

    void ProcessCpuClock() override {
        if (!_irqEnabled) return;
        if (++_irqScaler >= 114) {
            _irqScaler = 0;
            if (++_irqCounter == 0) {
                _irqCounter = _irqReloadValue;
                SetIrq(true);
            }
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0xB000 && address <= 0xE00C) {
            uint8_t slot = static_cast<uint8_t>(((((address & 8) | (address >> 8)) >> 3) + 2) & 7);
            uint8_t shift = address & 4;
            uint8_t low = static_cast<uint8_t>((_chrLow[slot] & (0xF0 >> shift))
                        | (value << shift));
            _chrLow[slot] = low;
            if (slot == 0) {
                if (low == 0xC8) _forceChrRom = false;
                else if (low == 0x88) _forceChrRom = true;
            }
            if (shift) _chrHigh[slot] = value >> 4;
            UpdateChr();
            return;
        }

        switch (address) {
            case 0x8010: SelectPrgPage(0, value); break;
            case 0xA010: SelectPrgPage(1, value); break;
            case 0x9400:
                switch (value & 3) {
                    case 0: SetMirroringType(MirroringType::Vertical); break;
                    case 1: SetMirroringType(MirroringType::Horizontal); break;
                    case 2: SetMirroringType(MirroringType::ScreenAOnly); break;
                    case 3: SetMirroringType(MirroringType::ScreenBOnly); break;
                }
                break;
            case 0xF000:
                _irqReloadValue = static_cast<uint8_t>((_irqReloadValue & 0xF0) | (value & 0x0F));
                SetIrq(false);
                break;
            case 0xF004:
                _irqReloadValue = static_cast<uint8_t>((_irqReloadValue & 0x0F) | (value << 4));
                SetIrq(false);
                break;
            case 0xF008:
                _irqCounter = _irqReloadValue;
                _irqEnabled = (value & 2) != 0;
                _irqScaler = 0;
                SetIrq(false);
                break;
        }
    }
};

class WaixingBs5 final : public Board {
    uint32_t GetDipSwitchCount() override { return 2; }
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0800; }

    void InitMapper() override {
        for (unsigned i = 0; i < 4; ++i) {
            SelectPrgPage(i, static_cast<uint16_t>(-1));
            SelectChrPage(i, static_cast<uint16_t>(-1));
        }
    }

    void WriteRegister(uint16_t address, uint8_t) override {
        uint16_t bank = static_cast<uint16_t>((address >> 10) & 3);
        switch (address & 0xF000) {
            case 0x8000:
                SelectChrPage(bank, address & 0x1F);
                break;
            case 0xA000:
                if (address & (1 << (GetDipSwitches() + 4)))
                    SelectPrgPage(bank, address & 0x0F);
                break;
        }
    }
};

} // namespace cupid::boards
#endif
