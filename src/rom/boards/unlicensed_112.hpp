/*
 * unlicensed_112.hpp - Address-latched multicarts and filtered PPU interrupts
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
#ifndef CUPID_BOARDS_UNLICENSED_112_HPP
#define CUPID_BOARDS_UNLICENSED_112_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Mapper213 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectPrgPage(0, (address >> 1) & 3);
        SelectChrPage(0, (address >> 3) & 7);
    }
};

class Mapper214 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectPrgPage(0, (address >> 2) & 3);
        SelectPrgPage(1, (address >> 2) & 3);
        SelectChrPage(0, address & 3);
    }
};

class Mapper216 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        AddRegisterRange(0x5000, 0x5000);
        WriteRegister(0x8000, 0);
    }
    uint8_t ReadRegister(uint16_t) override { return 0; }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectPrgPage(0, address & 1);
        SelectChrPage(0, (address >> 1) & 7);
    }
};

class Mapper222 final : public Board {
    uint16_t _counter = 0;
    uint32_t _lastFrameCycle = 0, _cyclesDown = 0;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    bool EnableVramAddressHook() override { return true; }
    void InitMapper() override { SelectPrgPage2x(1, static_cast<uint16_t>(-2)); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xF003) {
            case 0x8000: SelectPrgPage(0, value); break;
            case 0x9000: SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical); break;
            case 0xA000: SelectPrgPage(1, value); break;
            case 0xB000: case 0xB002: case 0xC000: case 0xC002:
            case 0xD000: case 0xD002: case 0xE000: case 0xE002:
                SelectChrPage(((address >> 12) - 0xB) * 2 + ((address >> 1) & 1), value);
                break;
            case 0xF000: _counter = value; SetIrq(false); break;
        }
    }
public:
    void NotifyVramAddressChange(uint16_t address) override {
        uint32_t frameCycle = PpuFrameCycle();
        if (_cyclesDown)
            _cyclesDown += _lastFrameCycle > frameCycle ? 89342u - _lastFrameCycle + frameCycle
                                                       : frameCycle - _lastFrameCycle;
        bool rising = false;
        if (!(address & 0x1000)) {
            if (!_cyclesDown) _cyclesDown = 1;
        } else {
            rising = _cyclesDown > 10;
            _cyclesDown = 0;
        }
        _lastFrameCycle = frameCycle;
        if (rising && _counter && ++_counter >= 240) {
            SetIrq(true);
            _counter = 0;
        }
    }
};

class Mapper225 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage2x(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned high = (address >> 8) & 0x40;
        unsigned bank = ((address >> 6) & 0x3F) | high;
        SelectPrgPage(0, address & 0x1000 ? bank : bank & 0xFE);
        SelectPrgPage(1, address & 0x1000 ? bank : (bank & 0xFE) + 1);
        SelectChrPage(0, (address & 0x3F) | high);
        SetMirroringType(address & 0x2000 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Mapper226 : public Board {
protected:
    std::array<uint8_t, 2> _regs{};
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage2x(0, 0); SelectChrPage(0, 0); }
    virtual uint8_t PrgBank() { return (_regs[0] & 0x1F) | ((_regs[0] & 0x80) >> 2) | ((_regs[1] & 1) << 6); }
    void UpdatePrg() {
        unsigned bank = PrgBank();
        SelectPrgPage(0, _regs[0] & 0x20 ? bank : bank & 0xFE);
        SelectPrgPage(1, _regs[0] & 0x20 ? bank : (bank & 0xFE) + 1);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        _regs[address & 1] = value;
        UpdatePrg();
        SetMirroringType(_regs[0] & 0x40 ? MirroringType::Vertical : MirroringType::Horizontal);
    }
public:
    void Reset(bool soft) override {
        if (soft) { _regs.fill(0); SelectPrgPage2x(0, 0); SelectChrPage(0, 0); }
    }
};

class Mapper227 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned bank = ((address >> 2) & 0x1F) | ((address & 0x100) >> 3);
        if (address & 0x80) {
            SelectPrgPage(0, address & 1 ? bank & 0x3E : bank);
            SelectPrgPage(1, address & 1 ? (bank & 0x3E) + 1 : bank);
        } else {
            SelectPrgPage(0, address & 1 ? bank & 0x3E : bank);
            SelectPrgPage(1, address & 0x200 ? bank | 7 : bank & 0x38);
        }
        SetMirroringType(address & 2 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class ActionEnterprises final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        unsigned chip = (address >> 11) & 3;
        if (chip == 3) chip = 2;
        unsigned bank = ((address >> 6) & 0x1F) | (chip << 5);
        SelectPrgPage(0, address & 0x20 ? bank : bank & 0xFE);
        SelectPrgPage(1, address & 0x20 ? bank : (bank & 0xFE) + 1);
        SelectChrPage(0, ((address & 15) << 2) | (value & 3));
        SetMirroringType(address & 0x2000 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
public:
    void Reset(bool) override { WriteRegister(0x8000, 0); }
};

class Mapper229 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectChrPage(0, address & 0xFF);
        if (!(address & 0x1E)) SelectPrgPage2x(0, 0);
        else { SelectPrgPage(0, address & 0x1F); SelectPrgPage(1, address & 0x1F); }
        SetMirroringType(address & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Mapper230 final : public Board {
    bool _contraMode = false;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectChrPage(0, 0); Reset(true); }
    void WriteRegister(uint16_t, uint8_t value) override {
        if (_contraMode) SelectPrgPage(0, value & 7);
        else {
            unsigned bank = value & 0x20 ? (value & 0x1F) + 8 : (value & 0x1E) + 8;
            SelectPrgPage(0, bank);
            SelectPrgPage(1, value & 0x20 ? bank : bank + 1);
            SetMirroringType(value & 0x40 ? MirroringType::Vertical : MirroringType::Horizontal);
        }
    }
public:
    void Reset(bool soft) override {
        if (!soft) return;
        _contraMode = !_contraMode;
        SelectPrgPage(0, _contraMode ? 0 : 8);
        SelectPrgPage(1, _contraMode ? 7 : 9);
        SetMirroringType(_contraMode ? MirroringType::Vertical : MirroringType::Horizontal);
    }
};

class Mapper231 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectPrgPage(1, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned bank = ((address >> 5) & 1) | (address & 0x1E);
        SelectPrgPage(0, bank & 0x1E);
        SelectPrgPage(1, bank);
        SetMirroringType(address & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
public:
    void Reset(bool) override { SelectPrgPage(0, 0); SelectPrgPage(1, 0); }
};

class Mapper233 final : public Mapper226 {
    uint8_t _reset = 0;
    uint8_t PrgBank() override { return (_regs[0] & 0x1F) | (_reset << 5) | ((_regs[1] & 1) << 6); }
public:
    void Reset(bool soft) override {
        Mapper226::Reset(soft);
        if (soft) { _reset ^= 1; UpdatePrg(); }
        else _reset = 0;
    }
};

} // namespace cupid::boards
#endif
