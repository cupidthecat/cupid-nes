/*
 * kaiser.hpp - Kaiser cartridge address decoders and timer hardware
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
#ifndef CUPID_BOARDS_KAISER_HPP
#define CUPID_BOARDS_KAISER_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Kaiser202 final : public Board {
    uint16_t _reload = 0, _counter = 0;
    uint8_t _control = 0, _selected = 0;
    std::array<uint8_t, 4> _prg{};
    bool _useRom = false;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        _reload = _counter = 0;
        _control = _selected = 0;
        _prg.fill(0);
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xF000) {
            case 0x8000: _reload = (_reload & 0xFFF0) | (value & 0x0F); break;
            case 0x9000: _reload = (_reload & 0xFF0F) | ((value & 0x0F) << 4); break;
            case 0xA000: _reload = (_reload & 0xF0FF) | ((value & 0x0F) << 8); break;
            case 0xB000: _reload = (_reload & 0x0FFF) | ((value & 0x0F) << 12); break;
            case 0xC000:
                _control = value;
                if (_control & 2) _counter = _reload;
                SetIrq(false);
                break;
            case 0xD000: SetIrq(false); break;
            case 0xE000: _selected = (value & 7) - 1; break;
            case 0xF000:
                if (_selected < 4) _prg[_selected] = (_prg[_selected] & 0x10) | (value & 0x0F);
                else if (_selected == 4) _useRom = (value & 4) != 0;
                if (_romInfo.MapperID == 56) {
                    switch (address & 0xFC00) {
                        case 0xF000: {
                            unsigned bank = address & 3;
                            _prg[bank] = (value & 0x10) | (_prg[bank] & 0x0F);
                            break;
                        }
                        case 0xF800:
                            SetMirroringType(value & 1 ? MirroringType::Vertical : MirroringType::Horizontal);
                            break;
                        case 0xFC00: SelectChrPage(address & 7, value); break;
                    }
                }
                if (_useRom) SetCpuMemoryMapping(0x6000, 0x7FFF, _prg[3], PrgMemoryType::PrgRom, Read);
                else SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::WorkRam, ReadWrite);
                SelectPrgPage(0, _prg[0]);
                SelectPrgPage(1, _prg[1]);
                SelectPrgPage(2, _prg[2]);
                break;
        }
    }

public:
    void ProcessCpuClock() override {
        if ((_control & 2) && ++_counter == 0xFFFF) {
            _counter = _reload;
            _control &= ~2;
            SetIrq(true);
        }
    }
};

class Kaiser7012 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        SelectPrgPage(0, 1);
        SelectChrPage(0, 0);
    }
    void WriteRegister(uint16_t address, uint8_t) override {
        if (address == 0xE0A0) SelectPrgPage(0, 0);
        else if (address == 0xEE36) SelectPrgPage(0, 1);
    }
};

class Kaiser7013B final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, static_cast<uint16_t>(-1));
        SelectChrPage(0, 0);
        SetMirroringType(MirroringType::Vertical);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) SelectPrgPage(0, value);
        else SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Kaiser7016 final : public Board {
    uint8_t _prg = 8;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        _prg = 8;
        for (unsigned slot = 0; slot < 4; ++slot) SelectPrgPage(slot, 0x0C + slot);
        SelectChrPage(0, 0);
        UpdateState();
    }
    void UpdateState() { SetCpuMemoryMapping(0x6000, 0x7FFF, _prg, PrgMemoryType::PrgRom); }
    void WriteRegister(uint16_t address, uint8_t) override {
        bool mode = (address & 0x30) == 0x30;
        switch (address & 0xD943) {
            case 0xD943:
                _prg = mode ? 0x0B : (address >> 2) & 0x0F;
                UpdateState();
                break;
            case 0xD903:
                _prg = mode ? 0x08 | ((address >> 2) & 3) : 0x0B;
                UpdateState();
                break;
        }
    }
};

class Kaiser7017 final : public Board {
    uint8_t _prg = 0;
    MirroringType _pendingMirroring = MirroringType::Vertical;
    uint16_t _counter = 0;
    bool _enabled = false;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4020; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    bool AllowRegisterRead() override { return true; }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        RemoveRegisterRange(0x4020, 0x5FFF, MemoryOperation::Read);
        AddRegisterRange(0x4030, 0x4030, MemoryOperation::Read);
        SelectChrPage(0, 0);
        _prg = 0;
        _pendingMirroring = MirroringType::Vertical;
        _counter = 0;
        _enabled = false;
        UpdateState();
    }
    void UpdateState() {
        SelectPrgPage(0, _prg);
        SelectPrgPage(1, 2);
        SetMirroringType(_pendingMirroring);
    }
    uint8_t ReadRegister(uint16_t) override {
        uint8_t value = IrqPending() ? 1 : 0;
        SetIrq(false);
        return value;
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0xFF00) == 0x4A00) _prg = ((address >> 2) & 3) | ((address >> 4) & 4);
        else if ((address & 0xFF00) == 0x5100) UpdateState();
        else if (address == 0x4020) {
            SetIrq(false);
            _counter = (_counter & 0xFF00) | value;
        } else if (address == 0x4021) {
            SetIrq(false);
            _counter = (_counter & 0xFF) | (value << 8);
            _enabled = true;
        } else if (address == 0x4025) {
            _pendingMirroring = value & 8 ? MirroringType::Horizontal : MirroringType::Vertical;
        }
    }

public:
    void ProcessCpuClock() override {
        if (_enabled && _counter && --_counter == 0) {
            _enabled = false;
            SetIrq(true);
        }
    }
};

class Kaiser7022 final : public Board {
    uint8_t _reg = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        _reg = 0;
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        AddRegisterRange(0xFFFC, 0xFFFC);
        SelectPrgPage(0, 0);
    }
    uint8_t ReadRegister(uint16_t address) override {
        SelectChrPage(0, _reg);
        SelectPrgPage(0, _reg);
        SelectPrgPage(1, _reg);
        return InternalReadRam(address);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x8000) SetMirroringType(value & 4 ? MirroringType::Horizontal : MirroringType::Vertical);
        else if (address == 0xA000) _reg = value & 0x0F;
    }
public:
    void Reset(bool) override {
        _reg = 0;
        ReadRegister(0xFFFC);
    }
};

class Kaiser7031 final : public Board {
    std::array<uint8_t, 4> _regs{};
    uint16_t GetPrgPageSize() override { return 0x800; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        _regs.fill(0);
        SetMirroringType(MirroringType::Vertical);
        for (unsigned slot = 0; slot < 16; ++slot) SelectPrgPage(slot, 15 - slot);
        SelectChrPage(0, 0);
        UpdateState();
    }
    void UpdateState() {
        for (unsigned slot = 0; slot < 4; ++slot)
            SetCpuMemoryMapping(0x6000 + slot * 0x800, 0x67FF + slot * 0x800, _regs[slot], PrgMemoryType::PrgRom);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[(address >> 11) & 3] = value;
        UpdateState();
    }
};

class Kaiser7037 final : public Board {
    uint8_t _current = 0;
    std::array<uint8_t, 8> _regs{};
    uint32_t GetWorkRamPageSize() override { return 0x1000; }
    uint16_t GetPrgPageSize() override { return 0x1000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        _current = 0;
        _regs.fill(0);
        SelectChrPage(0, 0);
        RemoveRegisterRange(0xA000, 0xBFFF);
        UpdateState();
    }
    void UpdateState() {
        SetCpuMemoryMapping(0x6000, 0x6FFF, 0, PrgMemoryType::WorkRam);
        SetCpuMemoryMapping(0x7000, 0x7FFF, 15, PrgMemoryType::PrgRom);
        SelectPrgPage2x(0, _regs[6] << 1);
        SelectPrgPage(2, static_cast<uint16_t>(-4));
        SelectPrgPage(3, 1, PrgMemoryType::WorkRam);
        SelectPrgPage2x(2, _regs[7] << 1);
        SelectPrgPage2x(3, static_cast<uint16_t>(-2));
        SetNametables(_regs[2] & 1, _regs[4] & 1, _regs[3] & 1, _regs[5] & 1);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE001) {
            case 0x8000: _current = value & 7; break;
            case 0x8001:
                _regs[_current] = value;
                UpdateState();
                break;
        }
    }
};

class Kaiser7057 final : public Board {
    std::array<uint8_t, 8> _regs{};
    uint16_t GetPrgPageSize() override { return 0x800; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        _regs.fill(0);
        SelectChrPage(0, 0);
        UpdateState();
    }
    void UpdateState() {
        for (unsigned slot = 0; slot < 4; ++slot) {
            SetCpuMemoryMapping(0x6000 + slot * 0x800, 0x67FF + slot * 0x800,
                                _regs[slot + 4], PrgMemoryType::PrgRom);
            SelectPrgPage(slot, _regs[slot]);
        }
        SelectPrgPage4x(1, 0x34);
        SelectPrgPage4x(2, 0x38);
        SelectPrgPage4x(3, 0x3C);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        unsigned slot;
        switch (address & 0xF002) {
            case 0x8000: case 0x8002: case 0x9000: case 0x9002:
                SetMirroringType(value & 1 ? MirroringType::Vertical : MirroringType::Horizontal);
                return;
            case 0xB000: slot = 0; break;
            case 0xB002: slot = 1; break;
            case 0xC000: slot = 2; break;
            case 0xC002: slot = 3; break;
            case 0xD000: slot = 4; break;
            case 0xD002: slot = 5; break;
            case 0xE000: slot = 6; break;
            case 0xE002: slot = 7; break;
            default: return;
        }
        _regs[slot] = address & 1 ? (_regs[slot] & 0x0F) | ((value << 4) & 0xF0)
                                  : (_regs[slot] & 0xF0) | (value & 0x0F);
        UpdateState();
    }
};

class Kaiser7058 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x1000; }
    uint16_t RegisterStartAddress() override { return 0xF000; }
    void InitMapper() override { SelectPrgPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        SelectChrPage((address & 0x80) != 0, value);
    }
};

} // namespace cupid::boards
#endif
