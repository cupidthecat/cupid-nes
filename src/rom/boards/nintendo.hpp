/*
 * nintendo.hpp - Nintendo cartridge boards implemented on the shared runtime
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
#ifndef CUPID_BOARDS_NINTENDO_HPP
#define CUPID_BOARDS_NINTENDO_HPP
#include "runtime.hpp"

namespace cupid::boards {

class FnsMmc1 final : public Board {
    uint8_t _writeBuffer = 0;
    uint8_t _shiftCount = 0;
    bool _wramDisable = false;
    bool _chrMode = false;
    bool _prgMode = true;
    bool _slotSelect = true;
    bool _forceWramOn = false;
    uint8_t _chrReg0 = 0;
    uint8_t _chrReg1 = 0;
    uint8_t _prgReg = 0;
    uint16_t _lastChrReg = 0xA000;
    uint64_t _lastWriteCycle = 0;
    bool _hasWriteCycle = false;

    std::vector<uint8_t> _kanjiRomData;
    MirroringType _mirroringSelect = MirroringType::Vertical;
    uint8_t _kanjiRomPos = 0;
    uint8_t _kanjiRomBank = 0;
    uint8_t _chrRamBank = 0;
    bool _workRamEnable1 = true;
    bool _workRamEnable2 = false;

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x1000; }
    bool AllowRegisterRead() override { return true; }

    void ResetShift() {
        _shiftCount = 0;
        _writeBuffer = 0;
    }

    void ProcessRegisterWrite(uint16_t address, uint8_t value) {
        switch (address & 0xE000) {
            case 0x8000:
                _slotSelect = (value & 0x04) != 0;
                _prgMode = (value & 0x08) != 0;
                _chrMode = (value & 0x10) != 0;
                break;
            case 0xA000:
                _lastChrReg = 0xA000;
                _chrReg0 = value & 0x1F;
                break;
            case 0xC000:
                _lastChrReg = 0xC000;
                _chrReg1 = value & 0x1F;
                break;
            case 0xE000:
                _prgReg = value & 0x0F;
                _wramDisable = (value & 0x10) != 0;
                break;
        }
    }

    uint8_t ExtraRegister() const {
        return (_lastChrReg == 0xC000 && _chrMode) ? _chrReg1 : _chrReg0;
    }

    void UpdateState() {
        uint8_t extra = ExtraRegister();
        PrgMemoryType ramType = HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam;
        int8_t access = _wramDisable && !_forceWramOn ? NoAccess : ReadWrite;
        uint32_t totalRam = _saveRamSize + _workRamSize;
        if (totalRam > 0x4000) {
            SetCpuMemoryMapping(0x6000, 0x7FFF, (extra >> 2) & 3, ramType, access);
        } else if (totalRam > 0x2000) {
            if (_saveRamSize == 0x2000 && _workRamSize == 0x2000) {
                SetCpuMemoryMapping(0x6000, 0x7FFF, 0,
                    (extra & 8) ? PrgMemoryType::WorkRam : PrgMemoryType::SaveRam, access);
            } else {
                SetCpuMemoryMapping(0x6000, 0x7FFF, (extra >> 2) & 1, ramType, access);
            }
        } else if (totalRam) {
            SetCpuMemoryMapping(0x6000, 0x7FFF, 0, ramType, access);
        } else {
            RemoveCpuMemoryMapping(0x6000, 0x7FFF);
        }

        uint8_t outer = _prgSize == 0x80000 ? extra & 0x10 : 0;
        if (_romInfo.SubMapperID == 5) {
            SelectPrgPage2x(0, 0);
        } else if (_prgMode) {
            if (_slotSelect) {
                SelectPrgPage(0, _prgReg | outer);
                SelectPrgPage(1, 0x0F | outer);
            } else {
                SelectPrgPage(0, outer);
                SelectPrgPage(1, _prgReg | outer);
            }
        } else {
            SelectPrgPage2x(0, (_prgReg & 0x0E) | outer);
        }

        if (_chrMode) {
            SelectChrPage(0, _chrReg0);
            SelectChrPage(1, _chrReg1);
        } else {
            SelectChrPage(0, _chrReg0 & 0x1E);
            SelectChrPage(1, (_chrReg0 & 0x1E) + 1);
        }

        SetMirroringType(_mirroringSelect);
        SetPpuMemoryMapping(0x0000, 0x1FFF, ChrMemoryType::ChrRam,
                            _chrRamBank ? 0x2000 : 0, ReadWrite);
        if (_workRamEnable1 && _workRamEnable2)
            SetCpuMemoryMapping(0x6000, 0x7FFF, PrgMemoryType::WorkRam, 0, ReadWrite);
    }

    void InitMapper() override {
        ProcessRegisterWrite(0x8000, 0x0C);
        ProcessRegisterWrite(0xA000, 0);
        ProcessRegisterWrite(0xC000, 0);
        ProcessRegisterWrite(0xE000,
            _romInfo.DatabaseInfo.Board.find("MMC1B") != std::string::npos ? 0x10 : 0);
        _forceWramOn = _romInfo.DatabaseInfo.Board == "MMC1A";
        AddRegisterRange(0x40AD, 0x40C0, MemoryOperation::Any);
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Read);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        _kanjiRomData = FcnsKanjiFirmware();
        if (_kanjiRomData.empty()) _kanjiRomData.resize(0x40000);
        UpdateState();
    }

    uint8_t ReadRegister(uint16_t address) override {
        if (address < 0x5000) {
            if (address == 0x40B0) _kanjiRomPos = 0;
            if (address == 0x40C0) return 0x80;
            return GetOpenBus();
        }
        if (address < 0x6000) {
            uint32_t offset = (_kanjiRomBank ? 0x20000u : 0u)
                            | ((uint32_t)(address & 0x0FFFu) << 5) | _kanjiRomPos;
            uint8_t value = _kanjiRomData[offset];
            _kanjiRomPos = (_kanjiRomPos + 1) & 0x1F;
            return value;
        }
        return InternalRead(address);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x6000) {
            switch (address) {
                case 0x40AD:
                    _mirroringSelect = value & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical;
                    UpdateState();
                    break;
                case 0x40AE:
                    _workRamEnable1 = (value & 1) != 0;
                    UpdateState();
                    break;
                case 0x40B0:
                    _kanjiRomBank = value & 1;
                    break;
                case 0x40C0:
                    _workRamEnable2 = (value & 1) != 0;
                    _chrRamBank = (value >> 3) & 1;
                    UpdateState();
                    break;
                default:
                    break;
            }
            return;
        }

        uint64_t cycle = CpuClock();
        bool consecutive = _hasWriteCycle && cycle - _lastWriteCycle < 2;
        _lastWriteCycle = cycle;
        _hasWriteCycle = true;
        if (consecutive && !(value & 0x80)) return;
        if (value & 0x80) {
            ResetShift();
            _prgMode = true;
            _slotSelect = true;
            UpdateState();
            return;
        }
        _writeBuffer = static_cast<uint8_t>(((value & 1) << 4) | (_writeBuffer >> 1));
        if (++_shiftCount == 5) {
            ProcessRegisterWrite(address, _writeBuffer);
            ResetShift();
            UpdateState();
        }
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        uint8_t mirroring = static_cast<uint8_t>(_mirroringSelect);
        if (!Board::VisitState(state)
            || !state.Field("fcns_mmc1.write_buffer", _writeBuffer, 0x1F)
            || !state.Field("fcns_mmc1.shift_count", _shiftCount, 4)
            || !state.Field("fcns_mmc1.wram_disable", _wramDisable)
            || !state.Field("fcns_mmc1.chr_mode", _chrMode)
            || !state.Field("fcns_mmc1.prg_mode", _prgMode)
            || !state.Field("fcns_mmc1.slot_select", _slotSelect)
            || !state.InvariantBool("fcns_mmc1.force_wram_on", _forceWramOn)
            || !state.Field("fcns_mmc1.chr_reg0", _chrReg0, 31)
            || !state.Field("fcns_mmc1.chr_reg1", _chrReg1, 31)
            || !state.Field("fcns_mmc1.prg_reg", _prgReg, 15)
            || !state.Field("fcns_mmc1.last_chr_reg", _lastChrReg)
            || !state.Field("fcns_mmc1.last_write_cycle", _lastWriteCycle)
            || !state.Field("fcns_mmc1.has_write_cycle", _hasWriteCycle)
            || !state.InvariantBytes("fcns_mmc1.kanji_rom", _kanjiRomData.data(),
                                     _kanjiRomData.size())
            || !state.ValueU8("fcns_mmc1.mirroring_select", mirroring,
                              static_cast<uint8_t>(MirroringType::FourScreens))
            || !state.Field("fcns_mmc1.kanji_rom_pos", _kanjiRomPos, 31)
            || !state.Field("fcns_mmc1.kanji_rom_bank", _kanjiRomBank, 1)
            || !state.Field("fcns_mmc1.chr_ram_bank", _chrRamBank, 1)
            || !state.Field("fcns_mmc1.work_ram_enable1", _workRamEnable1)
            || !state.Field("fcns_mmc1.work_ram_enable2", _workRamEnable2))
            return false;
        if (state.GetMode() == BoardStateVisitor::Mode::Apply)
            _mirroringSelect = static_cast<MirroringType>(mirroring);
        return true;
    }
};

} // namespace cupid::boards
#endif
