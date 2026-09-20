/*
 * txc_107.hpp - TXC and related cartridge boards
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
#ifndef CUPID_BOARDS_TXC_107_HPP
#define CUPID_BOARDS_TXC_107_HPP

#include "mmc3.hpp"
#include "txc_chip.hpp"

namespace cupid::boards {

class Txc22000 final : public Board {
    TxcChip _txc{false};
    uint8_t _chrBank = 0;

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x8000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        AddRegisterRange(0x4100, 0x5FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        _chrBank = 0;
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    void UpdateState() {
        SelectPrgPage(0, _txc.GetOutput() & 0x03);
        SelectChrPage(0, _chrBank);
    }

    uint8_t ReadRegister(uint16_t address) override {
        uint8_t openBus = GetOpenBus();
        uint8_t value = openBus;
        if ((address & 0x0103) == 0x0100)
            value = static_cast<uint8_t>((openBus & 0xCF) | ((_txc.Read() << 4) & 0x30));
        UpdateState();
        return value;
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0xF200) == 0x4200) _chrBank = value;
        _txc.Write(address, static_cast<uint8_t>((value >> 4) & 0x03));
        UpdateState();
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && _txc.VisitState(state)
            && state.Field("txc22000.chr_bank", _chrBank);
    }
};

class TxcMapper61 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 1);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t) override {
        uint8_t page = static_cast<uint8_t>(((address & 0x0F) << 1)
                     | ((address >> 5) & 0x01));
        if (address & 0x10) {
            SelectPrgPage(0, page);
            SelectPrgPage(1, page);
        } else {
            SelectPrgPage(0, page & 0xFE);
            SelectPrgPage(1, static_cast<uint8_t>((page & 0xFE) + 1));
        }
        SetMirroringType(address & 0x80 ? MirroringType::Horizontal
                                        : MirroringType::Vertical);
    }
};

class Txc22211A : public Board {
protected:
    TxcChip _txc{false};

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x8000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        AddRegisterRange(0x4020, 0x5FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    virtual void UpdateState() {
        SelectPrgPage(0, (_txc.GetOutput() >> 2) & 0x01);
        SelectChrPage(0, _txc.GetOutput() & 0x03);
    }

    uint8_t ReadRegister(uint16_t address) override {
        uint8_t openBus = GetOpenBus();
        uint8_t value = openBus;
        if ((address & 0x0100) == 0x0100)
            value = static_cast<uint8_t>((openBus & 0xF0) | (_txc.Read() & 0x0F));
        UpdateState();
        return value;
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _txc.Write(address, value & 0x0F);
        UpdateState();
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state) && _txc.VisitState(state);
    }
};

class Txc22211B final : public Board {
    TxcChip _txc{true};

    static uint8_t ConvertValue(uint8_t value) {
        return static_cast<uint8_t>(((value & 0x01) << 5) | ((value & 0x02) << 3)
             | ((value & 0x04) << 1) | ((value & 0x08) >> 1)
             | ((value & 0x10) >> 3) | ((value & 0x20) >> 5));
    }

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x8000; }
    uint16_t RegisterEndAddress() override { return 0xFFFF; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        AddRegisterRange(0x4020, 0x5FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    void UpdateState() {
        SelectChrPage(0, _txc.GetOutput());
        SetMirroringType(_txc.GetInvertFlag() ? MirroringType::Vertical
                                           : MirroringType::Horizontal);
    }

    uint8_t ReadRegister(uint16_t address) override {
        uint8_t openBus = GetOpenBus();
        uint8_t value = openBus;
        if ((address & 0x0103) == 0x0100)
            value = static_cast<uint8_t>((openBus & 0xC0) | ConvertValue(_txc.Read()));
        UpdateState();
        return value;
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _txc.Write(address, ConvertValue(value));
        if (address >= 0x8000) UpdateState();
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state) && _txc.VisitState(state);
    }
};

class Txc22211C final : public Txc22211A {
    void UpdateState() override {
        SelectPrgPage(0, 0);
        if (_chrRomSize > 0x2000) {
            SelectChrPage(0, static_cast<uint16_t>((_txc.GetOutput() & 0x01)
                         | (_txc.GetY() ? 0x02 : 0)
                         | ((_txc.GetOutput() & 0x02) << 1)));
        } else if (_txc.GetY()) {
            SelectChrPage(0, 0);
        } else {
            RemovePpuMemoryMapping(0, 0x1FFF);
        }
    }
};

class Mmc3_189 final : public Mmc3 {
    uint8_t _prgReg = 0;

    uint16_t RegisterStartAddress() override { return 0x4120; }

    void UpdateState() override {
        Mmc3::UpdateState();
        uint8_t page = static_cast<uint8_t>(((_prgReg | (_prgReg >> 4)) & 0x07) * 4);
        SelectPrgPage(0, page);
        SelectPrgPage(1, page + 1);
        SelectPrgPage(2, page + 2);
        SelectPrgPage(3, page + 3);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0x7FFF) {
            _prgReg = value;
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state) && state.Field("mmc3_189.prg_reg", _prgReg);
    }
};

class Bmc11160 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {}

    void Reset(bool softReset) override {
        Board::Reset(softReset);
        WriteRegister(0x8000, 0);
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        uint8_t bank = static_cast<uint8_t>((value >> 4) & 0x07);
        SelectPrgPage(0, bank);
        SelectChrPage(0, static_cast<uint16_t>((bank << 2) | (value & 0x03)));
        SetMirroringType(value & 0x80 ? MirroringType::Vertical
                                      : MirroringType::Horizontal);
    }
};

} // namespace cupid::boards
#endif
