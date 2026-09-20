/*
 * mmc3.hpp - Shared MMC3 cartridge hardware
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
#ifndef CUPID_BOARDS_MMC3_HPP
#define CUPID_BOARDS_MMC3_HPP

#include "runtime.hpp"
#include <cstring>
extern "C" {
#include "../mapper.h"
}

namespace cupid::boards {

class Mmc3 : public Board {
protected:
    struct State {
        uint8_t reg8000 = 0;
        uint8_t regA000 = 0;
        uint8_t regA001 = 0;
    } _state;

    uint8_t _irqReloadValue = 0;
    uint8_t _irqCounter = 0;
    bool _irqReload = false;
    bool _irqEnabled = false;
    uint8_t _prgMode = 0;
    uint8_t _chrMode = 0;
    uint8_t _registers[8]{};

private:
    uint8_t _currentRegister = 0;
    bool _wramEnabled = false;
    bool _wramWriteProtected = false;
    uint64_t _a12LowClock = 0;
    bool _forceMmc3RevAIrqs = false;

protected:
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    uint32_t GetSaveRamPageSize() override { return _romInfo.SubMapperID == 1 ? 0x200 : 0x2000; }
    uint32_t GetSaveRamSize() override { return _romInfo.SubMapperID == 1 ? 0x400 : 0x2000; }
    bool EnableVramAddressHook() override { return true; }

    virtual bool ForceMmc3RevAIrqs() { return _forceMmc3RevAIrqs; }

    State GetState() const { return _state; }
    uint8_t GetCurrentRegister() const { return _currentRegister; }
    uint8_t GetChrMode() const { return _chrMode; }

    bool CanWriteToWorkRam() const {
        return _wramEnabled && !_wramWriteProtected;
    }

    void ResetMmc3() {
        _state = {};
        _currentRegister = 0;
        _wramEnabled = false;
        _wramWriteProtected = false;
        _irqReloadValue = 0;
        _irqCounter = 0;
        _irqReload = false;
        _irqEnabled = false;
        _prgMode = 0;
        _chrMode = 0;
        const uint8_t initial[8] = {0, 2, 4, 5, 6, 7, 0, 1};
        std::memcpy(_registers, initial, sizeof(initial));
    }

    virtual void UpdateMirroring() {
        if (GetMirroringType() != MirroringType::FourScreens)
            SetMirroringType((_state.regA000 & 1) ? MirroringType::Horizontal
                                                   : MirroringType::Vertical);
    }

    virtual void UpdatePrgMapping() {
        if (!_prgMode) {
            SelectPrgPage(0, _registers[6]);
            SelectPrgPage(1, _registers[7]);
            SelectPrgPage(2, static_cast<uint16_t>(-2));
            SelectPrgPage(3, static_cast<uint16_t>(-1));
        } else {
            SelectPrgPage(0, static_cast<uint16_t>(-2));
            SelectPrgPage(1, _registers[7]);
            SelectPrgPage(2, _registers[6]);
            SelectPrgPage(3, static_cast<uint16_t>(-1));
        }
    }

    virtual void UpdateChrMapping() {
        if (!_chrMode) {
            SelectChrPage(0, _registers[0] & 0xFE);
            SelectChrPage(1, _registers[0] | 1);
            SelectChrPage(2, _registers[1] & 0xFE);
            SelectChrPage(3, _registers[1] | 1);
            SelectChrPage(4, _registers[2]);
            SelectChrPage(5, _registers[3]);
            SelectChrPage(6, _registers[4]);
            SelectChrPage(7, _registers[5]);
        } else {
            SelectChrPage(0, _registers[2]);
            SelectChrPage(1, _registers[3]);
            SelectChrPage(2, _registers[4]);
            SelectChrPage(3, _registers[5]);
            SelectChrPage(4, _registers[0] & 0xFE);
            SelectChrPage(5, _registers[0] | 1);
            SelectChrPage(6, _registers[1] & 0xFE);
            SelectChrPage(7, _registers[1] | 1);
        }
    }

    virtual void UpdateState() {
        _currentRegister = _state.reg8000 & 7;
        _chrMode = (_state.reg8000 >> 7) & 1;
        _prgMode = (_state.reg8000 >> 6) & 1;
        _wramEnabled = (_state.regA001 & 0x80) != 0;
        _wramWriteProtected = (_state.regA001 & 0x40) != 0;

        if (_romInfo.SubMapperID == 0) {
            int8_t access = NoAccess;
            if (_wramEnabled) access = _wramWriteProtected ? Read : ReadWrite;
            if ((HasBattery() && _saveRamSize) || (!HasBattery() && _workRamSize))
                SetCpuMemoryMapping(0x6000, 0x7FFF, 0,
                                    HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam,
                                    access);
            else
                RemoveCpuMemoryMapping(0x6000, 0x7FFF);
        }

        UpdatePrgMapping();
        UpdateChrMapping();
    }

    void InitMapper() override {
        _forceMmc3RevAIrqs = _romInfo.DatabaseInfo.Chip.rfind("MMC3A", 0) == 0;
        ResetMmc3();
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0,
                            HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam);
        UpdateState();
        UpdateMirroring();
    }

    bool IsA12RisingEdge(uint16_t address) {
        uint64_t clock = CpuClock();
        if (address & 0x1000) {
            bool rising = _a12LowClock > 0 && clock >= _a12LowClock
                       && clock - _a12LowClock >= 3;
            _a12LowClock = 0;
            return rising;
        }
        if (_a12LowClock == 0) _a12LowClock = clock;
        return false;
    }

    virtual void TriggerIrq() { SetIrq(true); }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE001) {
            case 0x8000:
                _state.reg8000 = value;
                UpdateState();
                break;
            case 0x8001:
                if (_currentRegister <= 1) value &= static_cast<uint8_t>(~1u);
                _registers[_currentRegister] = value;
                UpdateState();
                break;
            case 0xA000:
                _state.regA000 = value;
                UpdateMirroring();
                break;
            case 0xA001:
                _state.regA001 = value;
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
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("mmc3.reg8000", _state.reg8000)
            && state.Field("mmc3.regA000", _state.regA000)
            && state.Field("mmc3.regA001", _state.regA001)
            && state.Field("mmc3.irq_reload_value", _irqReloadValue)
            && state.Field("mmc3.irq_counter", _irqCounter)
            && state.Field("mmc3.irq_reload", _irqReload)
            && state.Field("mmc3.irq_enabled", _irqEnabled)
            && state.Field("mmc3.prg_mode", _prgMode, 1)
            && state.Field("mmc3.chr_mode", _chrMode, 1)
            && state.Field("mmc3.registers", _registers)
            && state.Field("mmc3.current_register", _currentRegister, 7)
            && state.Field("mmc3.wram_enabled", _wramEnabled)
            && state.Field("mmc3.wram_write_protected", _wramWriteProtected)
            && state.Field("mmc3.a12_low_clock", _a12LowClock)
            && state.InvariantBool("mmc3.force_rev_a", _forceMmc3RevAIrqs);
    }

    void NotifyVramAddressChange(uint16_t address) override {
        if (!IsA12RisingEdge(address)) return;

        uint8_t previous = _irqCounter;
        bool explicitReload = _irqReload;
        if (_irqCounter == 0 || _irqReload) _irqCounter = _irqReloadValue;
        else --_irqCounter;

        bool revisionA = ForceMmc3RevAIrqs()
                      || std::strcmp(cart_mmc3_revision_name(), "a") == 0;
        bool trigger = _irqCounter == 0 && _irqEnabled;
        if (revisionA) trigger = trigger && (previous != 0 || explicitReload);
        if (trigger) TriggerIrq();
        _irqReload = false;
    }
};

class Mmc3ChrRam final : public Mmc3 {
    uint16_t _firstRamBank;
    uint16_t _lastRamBank;
    uint16_t _defaultRamPages;

protected:
    uint16_t GetChrRamPageSize() override { return 0x0400; }
    uint32_t GetChrRamSize() override {
        return static_cast<uint32_t>(_defaultRamPages) * 0x0400;
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (page >= _firstRamBank && page <= _lastRamBank) {
            type = ChrMemoryType::ChrRam;
            page = static_cast<uint16_t>(page - _firstRamBank);
        }
        Mmc3::SelectChrPage(slot, page, type);
    }

public:
    Mmc3ChrRam(uint16_t firstRamBank, uint16_t lastRamBank, uint16_t defaultRamPages)
        : _firstRamBank(firstRamBank), _lastRamBank(lastRamBank),
          _defaultRamPages(defaultRamPages) {}

    bool VisitState(BoardStateVisitor &state) override {
        return Mmc3::VisitState(state)
            && state.InvariantU16("mmc3_chr_ram.first_bank", _firstRamBank)
            && state.InvariantU16("mmc3_chr_ram.last_bank", _lastRamBank)
            && state.InvariantU16("mmc3_chr_ram.default_pages", _defaultRamPages);
    }
};

} // namespace cupid::boards
#endif
