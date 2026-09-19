/*
 * ffe.hpp - Front Fareast cartridge registers and IRQ counter
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
#ifndef CUPID_BOARDS_FFE_HPP
#define CUPID_BOARDS_FFE_HPP
#include "runtime.hpp"

namespace cupid::boards {

class FrontFareast final : public Board {
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;
    bool _alternateMode = true;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    uint32_t GetChrRamSize() override { return 0x8000; }
    uint16_t RegisterStartAddress() override { return 0x42FE; }
    uint16_t RegisterEndAddress() override { return 0x4517; }
    bool EnableCpuClockHook() override { return true; }

    void InitMapper() override {
        _irqCounter = 0;
        _irqEnabled = false;
        _alternateMode = true;
        switch (_romInfo.MapperID) {
            case 6:
                AddRegisterRange(0x8000, 0xFFFF, MemoryOperation::Write);
                SelectPrgPage2x(0, 0);
                SelectPrgPage2x(1, 14);
                break;
            case 8:
                AddRegisterRange(0x8000, 0xFFFF, MemoryOperation::Write);
                SelectPrgPage4x(0, 0);
                break;
            case 17:
                SelectPrgPage4x(0, static_cast<uint16_t>(-4));
                break;
        }
    }

    void ProcessCpuClock() override {
        if (_irqEnabled && ++_irqCounter == 0) {
            SetIrq(true);
            _irqEnabled = false;
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address) {
            case 0x42FE:
                _alternateMode = (value & 0x80) == 0;
                SetMirroringType(value & 0x10 ? MirroringType::ScreenBOnly : MirroringType::ScreenAOnly);
                break;
            case 0x42FF:
                SetMirroringType(value & 0x10 ? MirroringType::Horizontal : MirroringType::Vertical);
                break;
            case 0x4501:
                _irqEnabled = false;
                SetIrq(false);
                break;
            case 0x4502:
                _irqCounter = (_irqCounter & 0xFF00) | value;
                SetIrq(false);
                break;
            case 0x4503:
                _irqCounter = (_irqCounter & 0x00FF) | (value << 8);
                _irqEnabled = true;
                SetIrq(false);
                break;
            default:
                if (_romInfo.MapperID == 6 && address >= 0x8000) {
                    if (HasChrRam() || _alternateMode) {
                        SelectPrgPage2x(0, (value & 0xFC) >> 1);
                        value &= 3;
                    }
                    SelectChrPage8x(0, value << 3);
                } else if (_romInfo.MapperID == 8 && address >= 0x8000) {
                    SelectPrgPage2x(0, (value & 0xF8) >> 2);
                    SelectChrPage8x(0, (value & 7) << 3);
                } else if (_romInfo.MapperID == 17) {
                    if (address >= 0x4504 && address <= 0x4507)
                        SelectPrgPage(address - 0x4504, value);
                    else if (address >= 0x4510 && address <= 0x4517)
                        SelectChrPage(address - 0x4510, value);
                }
                break;
        }
    }
};

} // namespace cupid::boards
#endif
