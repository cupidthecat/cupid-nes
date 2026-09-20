/*
 * mmc1.hpp - Serial MMC1 banking shared by derived cartridge boards
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
#ifndef CUPID_BOARDS_MMC1_HPP
#define CUPID_BOARDS_MMC1_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Mmc1Board : public Board {
protected:
    uint8_t _writeBuffer = 0, _shiftCount = 0;
    bool _wramDisable = false, _chrMode = false, _prgMode = false, _slotSelect = false;
    uint8_t _chrReg0 = 0, _chrReg1 = 0, _prgReg = 0;
    uint64_t _lastWriteCycle = 0;
    bool _forceWramOn = false;
    uint16_t _lastChrReg = 0;

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x1000; }

    void ResetBuffer() { _shiftCount = 0; _writeBuffer = 0; }

    void ProcessRegisterWrite(uint16_t address, uint8_t value) {
        switch (address & 0xE000) {
            case 0x8000:
                switch (value & 3) {
                    case 0: SetMirroringType(MirroringType::ScreenAOnly); break;
                    case 1: SetMirroringType(MirroringType::ScreenBOnly); break;
                    case 2: SetMirroringType(MirroringType::Vertical); break;
                    case 3: SetMirroringType(MirroringType::Horizontal); break;
                }
                _slotSelect = (value & 4) != 0;
                _prgMode = (value & 8) != 0;
                _chrMode = (value & 0x10) != 0;
                break;
            case 0xA000: _lastChrReg = address; _chrReg0 = value & 31; break;
            case 0xC000: _lastChrReg = address; _chrReg1 = value & 31; break;
            case 0xE000: _prgReg = value & 15; _wramDisable = (value & 0x10) != 0; break;
        }
    }

    virtual void UpdateState() {
        uint8_t extraReg = _lastChrReg == 0xC000 && _chrMode ? _chrReg1 : _chrReg0;
        uint8_t prgBankSelect = _prgSize == 0x80000 ? (extraReg & 0x10) : 0;
        int8_t access = _wramDisable && !_forceWramOn ? NoAccess : ReadWrite;
        PrgMemoryType type = HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam;
        uint32_t ramBytes = _saveRamSize + _workRamSize;
        if (ramBytes > 0x4000) {
            SetCpuMemoryMapping(0x6000, 0x7FFF, (extraReg >> 2) & 3, type, access);
        } else if (ramBytes > 0x2000) {
            if (_saveRamSize == 0x2000 && _workRamSize == 0x2000) {
                SetCpuMemoryMapping(0x6000, 0x7FFF, 0, extraReg & 8
                    ? PrgMemoryType::WorkRam : PrgMemoryType::SaveRam, access);
            } else {
                SetCpuMemoryMapping(0x6000, 0x7FFF, (extraReg >> 2) & 1, type, access);
            }
        } else if (ramBytes == 0) {
            RemoveCpuMemoryMapping(0x6000, 0x7FFF);
        } else {
            SetCpuMemoryMapping(0x6000, 0x7FFF, 0, type, access);
        }

        if (_romInfo.SubMapperID == 5) {
            SelectPrgPage2x(0, 0);
        } else if (_prgMode) {
            if (_slotSelect) {
                SelectPrgPage(0, _prgReg | prgBankSelect);
                SelectPrgPage(1, 15 | prgBankSelect);
            } else {
                SelectPrgPage(0, prgBankSelect);
                SelectPrgPage(1, _prgReg | prgBankSelect);
            }
        } else {
            SelectPrgPage2x(0, (_prgReg & 0xFE) | prgBankSelect);
        }
        SelectChrPage(0, _chrMode ? _chrReg0 : (_chrReg0 & 0x1E));
        SelectChrPage(1, _chrMode ? _chrReg1 : ((_chrReg0 & 0x1E) + 1));
    }

    void ProcessBitWrite(uint16_t address, uint8_t value) {
        if (value & 0x80) {
            ResetBuffer();
            _prgMode = true;
            _slotSelect = true;
            UpdateState();
        } else {
            _writeBuffer = (_writeBuffer >> 1) | ((value << 4) & 0x10);
            if (++_shiftCount == 5) {
                ProcessRegisterWrite(address, _writeBuffer);
                UpdateState();
                ResetBuffer();
            }
        }
    }

    void InitMapper() override {
        ProcessRegisterWrite(0x8000, GetPowerOnByte() | 0x0C);
        ProcessRegisterWrite(0xA000, GetPowerOnByte());
        ProcessRegisterWrite(0xC000, GetPowerOnByte());
        ProcessRegisterWrite(0xE000,
            _romInfo.DatabaseInfo.Board.find("MMC1B") != std::string::npos ? 0x10 : 0);
        _forceWramOn = _romInfo.DatabaseInfo.Board == "MMC1A";
        _lastChrReg = 0xA000;
        UpdateState();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        uint64_t cycle = CpuClock();
        if ((value & 0x80) || cycle - _lastWriteCycle >= 2)
            ProcessBitWrite(address, value);
        _lastWriteCycle = cycle;
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("mmc1.write_buffer", _writeBuffer, 0x1F)
            && state.Field("mmc1.shift_count", _shiftCount, 4)
            && state.Field("mmc1.wram_disable", _wramDisable)
            && state.Field("mmc1.chr_mode", _chrMode)
            && state.Field("mmc1.prg_mode", _prgMode)
            && state.Field("mmc1.slot_select", _slotSelect)
            && state.Field("mmc1.chr_reg0", _chrReg0, 31)
            && state.Field("mmc1.chr_reg1", _chrReg1, 31)
            && state.Field("mmc1.prg_reg", _prgReg, 15)
            && state.Field("mmc1.last_write_cycle", _lastWriteCycle)
            && state.InvariantBool("mmc1.force_wram_on", _forceWramOn)
            && state.Field("mmc1.last_chr_reg", _lastChrReg);
    }
};

} // namespace cupid::boards
#endif
