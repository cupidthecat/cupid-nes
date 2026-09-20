/*
 * sachen.hpp - Sachen discrete, JV001, and 8259 cartridge boards
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
#ifndef CUPID_BOARDS_SACHEN_HPP
#define CUPID_BOARDS_SACHEN_HPP
#include "runtime.hpp"
#include "txc_chip.hpp"
#include "mmc3.hpp"

namespace cupid::boards {

class SachenDiscrete final : public Board {
    uint16_t GetPrgPageSize() override { return _romInfo.MapperID == 143 ? 0x4000 : 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override {
        return _romInfo.MapperID == 148 || _romInfo.MapperID == 149 ? 0x8000 : 0x4100;
    }
    uint16_t RegisterEndAddress() override {
        return _romInfo.MapperID == 143 ? 0x5FFF : _romInfo.MapperID == 145 ? 0x7FFF : 0xFFFF;
    }
    bool AllowRegisterRead() override { return _romInfo.MapperID == 143; }
    bool HasBusConflicts() override { return _romInfo.MapperID == 148; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        if (_romInfo.MapperID == 143) SelectPrgPage(1, 1);
        if (_romInfo.MapperID == 133 || _romInfo.MapperID == 143) SelectChrPage(0, 0);
    }

    uint8_t ReadRegister(uint16_t address) override { return (~address & 0x3F) | 0x40; }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (_romInfo.MapperID) {
            case 133:
                if ((address & 0x6100) == 0x4100) {
                    SelectPrgPage(0, (value >> 2) & 1);
                    SelectChrPage(0, value & 3);
                }
                break;
            case 145:
                if ((address & 0x4100) == 0x4100) SelectChrPage(0, (value >> 7) & 1);
                break;
            case 148:
                SelectPrgPage(0, (value >> 3) & 1);
                SelectChrPage(0, value & 7);
                break;
            case 149: SelectChrPage(0, (value >> 7) & 1); break;
        }
    }
};

class SachenJv001 final : public Board {
    TxcChip _chip{true};
    bool _bankPrg;

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        AddRegisterRange(0x4020, 0x5FFF);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        SelectPrgPage(0, 0);
        SelectChrPage(0, 0);
    }

    void UpdateState() {
        uint8_t output = _chip.GetOutput();
        if (_bankPrg) {
            SelectPrgPage(0, ((output & 0x20) >> 4) | (output & 1));
            SelectChrPage(0, (output & 0x1E) >> 1);
        } else {
            SelectChrPage(0, output);
        }
    }

    uint8_t ReadRegister(uint16_t address) override {
        uint8_t value = GetOpenBus();
        if ((address & 0x103) == 0x100) {
            uint8_t chipValue = _chip.Read();
            value = _bankPrg ? ((chipValue & 0x3F) << 2) | ((chipValue & 0xC0) >> 6)
                            : (value & 0xC0) | (chipValue & 0x3F);
        }
        UpdateState();
        return value;
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        _chip.Write(address, _bankPrg ? ((value & 0xFC) >> 2) | ((value & 3) << 6)
                                     : value & 0x3F);
        if (!_bankPrg || address >= 0x8000) UpdateState();
    }

public:
    explicit SachenJv001(bool bankPrg) : _bankPrg(bankPrg) {}

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && _chip.VisitState(state)
            && state.InvariantBool("sachenjv001.bankPrg", _bankPrg);
    }
};

enum class Sachen8259Variant { A, B, C, D };

class Sachen8259 final : public Board {
    Sachen8259Variant _variant;
    uint8_t _current = 0;
    std::array<uint8_t, 8> _regs{};
    uint8_t _shift;
    std::array<uint8_t, 3> _chrOr;

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return _variant == Sachen8259Variant::D ? 0x400 : 0x800; }
    uint16_t RegisterStartAddress() override { return 0x4100; }
    uint16_t RegisterEndAddress() override { return 0x7FFF; }

    void InitMapper() override {
        _current = 0;
        _regs.fill(0);
        SelectPrgPage(0, 0);
    }

    void UpdateState() {
        bool variantD = _variant == Sachen8259Variant::D;
        bool simple = (_regs[7] & 1) != 0;
        switch ((_regs[7] >> 1) & 3) {
            case 0: SetMirroringType(variantD ? MirroringType::Horizontal : MirroringType::Vertical); break;
            case 1: SetMirroringType(variantD ? MirroringType::Vertical : MirroringType::Horizontal); break;
            case 2: SetNametables(0, 1, 1, 1); break;
            case 3: SetMirroringType(MirroringType::ScreenAOnly); break;
        }
        if (simple) SetMirroringType(variantD ? MirroringType::Horizontal : MirroringType::Vertical);
        SelectPrgPage(0, _regs[5]);
        if (variantD) {
            SelectChrPage(0, _regs[0]);
            SelectChrPage(1, ((_regs[4] & 1) << 4) | _regs[simple ? 0 : 1]);
            SelectChrPage(2, ((_regs[4] & 2) << 3) | _regs[simple ? 0 : 2]);
            SelectChrPage(3, ((_regs[4] & 4) << 2) | ((_regs[6] & 1) << 3) | _regs[simple ? 0 : 3]);
            SelectChrPage4x(1, static_cast<uint16_t>(-4));
        } else if (!HasChrRam()) {
            uint8_t high = _regs[4] << 3;
            SelectChrPage(0, (high | _regs[0]) << _shift);
            for (unsigned slot = 1; slot < 4; ++slot)
                SelectChrPage(slot, ((high | _regs[simple ? 0 : slot]) << _shift) | _chrOr[slot - 1]);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xC101) {
            case 0x4100: _current = value & 7; break;
            case 0x4101:
                _regs[_current] = value & 7;
                UpdateState();
                break;
        }
    }

public:
    explicit Sachen8259(Sachen8259Variant variant)
        : _variant(variant), _shift(variant == Sachen8259Variant::A ? 1 : variant == Sachen8259Variant::C ? 2 : 0),
          _chrOr(variant == Sachen8259Variant::A ? std::array<uint8_t, 3>{1, 0, 1}
                 : variant == Sachen8259Variant::C ? std::array<uint8_t, 3>{1, 2, 3}
                                                  : std::array<uint8_t, 3>{0, 0, 0}) {}

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.InvariantU32("sachen8259.variant", static_cast<uint32_t>(_variant))
            && state.Field("sachen8259.current", _current)
            && state.Field("sachen8259.regs", _regs)
            && state.InvariantU8("sachen8259.shift", _shift)
            && state.InvariantBytes("sachen8259.chrOr", _chrOr.data(), _chrOr.size());
    }
};

class Sachen74LS374 final : public Board {
    uint8_t _current = 0;
    std::array<uint8_t, 8> _regs{};

    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4100; }
    uint16_t RegisterEndAddress() override { return 0x7FFF; }
    uint32_t GetDipSwitchCount() override { return _romInfo.MapperID == 150 ? 1 : 0; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        _current = 0;
        _regs.fill(0);
        UpdateState();
    }

    void UpdateState() {
        uint8_t chrPage = _romInfo.MapperID == 150
                        ? ((_regs[4] & 1) << 2) | (_regs[6] & 3)
                        : (_regs[2] & 1) | ((_regs[4] & 1) << 1) | ((_regs[6] & 3) << 2);
        SelectChrPage(0, chrPage);
        SelectPrgPage(0, _regs[5] & 3);
        switch ((_regs[7] >> 1) & 3) {
            case 0: SetNametables(0, 0, 0, 1); break;
            case 1: SetMirroringType(MirroringType::Horizontal); break;
            case 2: SetMirroringType(MirroringType::Vertical); break;
            case 3: SetMirroringType(MirroringType::ScreenAOnly); break;
        }
    }

    uint8_t ReadRegister(uint16_t address) override {
        uint8_t bus = GetOpenBus();
        if ((address & 0xC101) != 0x4101) return bus;
        return GetDipSwitches() & 1 ? (bus & 0xFC) | (_regs[_current] & 3)
                                   : (bus & 0xF8) | (_regs[_current] & 7);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (GetDipSwitches() & 1) value |= 4;
        switch (address & 0xC101) {
            case 0x4100: _current = value & 7; break;
            case 0x4101:
                _regs[_current] = value & 7;
                UpdateState();
                break;
        }
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("sachen74ls374.current", _current)
            && state.Field("sachen74ls374.regs", _regs);
    }
};

class Sachen9602 final : public Mmc3 {
    uint8_t _selected = 0, _outerPrg = 0;
    bool ForceChrBattery() override { return true; }
    uint32_t GetChrRamSize() override { return 0x8000; }

    void InitMapper() override {
        _selected = _outerPrg = 0;
        // The cartridge supplies battery power to the entire CHR RAM chip.
        _saveChrRamSize = _chrRamSize;
        Mmc3::InitMapper();
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType = PrgMemoryType::PrgRom) override {
        Mmc3::SelectPrgPage(slot, (page & 0x3F) | (_outerPrg << 6));
        Mmc3::SelectPrgPage(_prgMode ? 0 : 2, 0x3E);
        Mmc3::SelectPrgPage(3, 0x3F);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE001) {
            case 0x8000: _selected = value; break;
            case 0x8001:
                if ((_selected & 7) < 6) {
                    _outerPrg = value >> 6;
                    value &= 0x1F;
                    UpdatePrgMapping();
                }
                break;
        }
        Mmc3::WriteRegister(address, value);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.Field("sachen9602.selected", _selected)
            && state.Field("sachen9602.outerPrg", _outerPrg);
    }
};

} // namespace cupid::boards
#endif
