/*
 * unlicensed_111.hpp - Subor and discrete address/data cartridge boards
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
#ifndef CUPID_BOARDS_UNLICENSED_111_HPP
#define CUPID_BOARDS_UNLICENSED_111_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Subor166 final : public Board {
    std::array<uint8_t, 4> _regs{};
    bool _is166;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        static constexpr uint8_t masks[] = {0x10, 0x1C, 0x1F, 0x1F};
        unsigned reg = (address >> 13) & 3;
        _regs[reg] = value & masks[reg];
        unsigned outer = ((_regs[0] ^ _regs[1]) & 0x10) << 1;
        unsigned inner = _regs[2] ^ _regs[3];
        if (_regs[1] & 8) {
            unsigned bank = (outer | inner) & 0xFE;
            SelectPrgPage(0, bank + (_is166 ? 0 : 1));
            SelectPrgPage(1, bank + (_is166 ? 1 : 0));
        } else if (_regs[1] & 4) {
            SelectPrgPage(0, 0x1F);
            SelectPrgPage(1, outer | inner);
        } else {
            SelectPrgPage(0, outer | inner);
            SelectPrgPage(1, _is166 ? 7 : 0x20);
        }
    }
public:
    explicit Subor166(bool is166) : _is166(is166) {}
};

class Mapper170 final : public Board {
    uint8_t _reg = 0;
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x7000; }
    uint16_t RegisterEndAddress() override { return 0x7001; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
        RemoveRegisterRange(0x7000, 0x7000, MemoryOperation::Read);
        RemoveRegisterRange(0x7001, 0x7001, MemoryOperation::Write);
        AddRegisterRange(0x6502, 0x6502, MemoryOperation::Write);
        AddRegisterRange(0x7777, 0x7777, MemoryOperation::Read);
    }
    uint8_t ReadRegister(uint16_t address) override { return _reg | ((address >> 8) & 0x7F); }
    void WriteRegister(uint16_t, uint8_t value) override { _reg = (value << 1) & 0x80; }
public:
    void Reset(bool) override { _reg = 0; }
};

class Henggedianzi177 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, value);
        SetMirroringType(value & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Henggedianzi179 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x5000; }
    void InitMapper() override {
        RemoveRegisterRange(0x6000, 0x7FFF);
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x6000) SelectPrgPage(0, value >> 1);
        else SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class MagicKidGooGoo final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x0800; }
    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 0);
        SelectChrPage4x(0, 0);
        SetMirroringType(MirroringType::Vertical);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0xE000) == 0x8000) SelectPrgPage(0, value & 7);
        else if ((address & 0xE000) == 0xC000) SelectPrgPage(0, (value & 7) | 8);
        else if ((address & 0xA000) == 0xA000) SelectChrPage(address & 3, value);
    }
};

class Mapper200 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        SelectPrgPage(0, 0); SelectPrgPage(1, 0); SelectChrPage(0, 0);
    }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectPrgPage(0, address & 7);
        SelectPrgPage(1, address & 7);
        SelectChrPage(0, address & 7);
        SetMirroringType(address & 8 ? MirroringType::Vertical : MirroringType::Horizontal);
    }
};

class Mapper202 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        SelectPrgPage(0, 0); SelectPrgPage(1, 0); SelectChrPage(0, 0);
    }
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned bank = (address >> 1) & 7;
        SelectPrgPage(0, bank);
        SelectPrgPage(1, bank + ((address & 9) == 9 ? 1 : 0));
        SelectChrPage(0, bank);
        SetMirroringType(address & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Mapper203 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        SelectPrgPage(0, 0); SelectPrgPage(1, 0); SelectChrPage(0, 0);
    }
    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, value >> 2);
        SelectPrgPage(1, value >> 2);
        SelectChrPage(0, value & 3);
    }
};

class Mapper204 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        unsigned pair = address & 6;
        unsigned page = pair + (pair == 6 ? 0 : address & 1);
        SelectPrgPage(0, page);
        SelectPrgPage(1, pair + (pair == 6 ? 1 : address & 1));
        SelectChrPage(0, page);
        SetMirroringType(address & 0x10 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Mapper212 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        AddRegisterRange(0x6000, 0x7FFF, MemoryOperation::Read);
        WriteRegister(0x8000, 0);
    }
    uint8_t ReadRegister(uint16_t address) override {
        uint8_t value = InternalReadRam(address);
        return (address & 0xE010) == 0x6000 ? value | 0x80 : value;
    }
    void WriteRegister(uint16_t address, uint8_t) override {
        if (address & 0x4000) SelectPrgPage2x(0, address & 6);
        else { SelectPrgPage(0, address & 7); SelectPrgPage(1, address & 7); }
        SelectChrPage(0, address & 7);
        SetMirroringType(address & 8 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

} // namespace cupid::boards
#endif
