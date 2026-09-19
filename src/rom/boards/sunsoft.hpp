/*
 * sunsoft.hpp - FME-7 cartridge banking, timer, and 5B audio
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
#ifndef CUPID_BOARDS_SUNSOFT_HPP
#define CUPID_BOARDS_SUNSOFT_HPP
#include "runtime.hpp"
extern "C" {
#include "../sunsoft5b.h"
}

namespace cupid::boards {

class SunsoftFme7 final : public Board {
    Sunsoft5B _audio{};
    uint8_t _command = 0;
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;
    bool _irqCounterEnabled = false;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    uint32_t GetWorkRamSize() override { return 0x8000; }
    uint32_t GetWorkRamPageSize() override { return 0x2000; }
    uint32_t GetSaveRamSize() override { return 0x8000; }
    uint32_t GetSaveRamPageSize() override { return 0x2000; }
    bool EnableCpuClockHook() override { return true; }

    void MapLowWindow(uint8_t value) {
        if (value & 0x40) {
            SetCpuMemoryMapping(0x6000, 0x7FFF, value & 0x3F,
                                HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam,
                                value & 0x80 ? ReadWrite : NoAccess);
        } else {
            SetCpuMemoryMapping(0x6000, 0x7FFF, value & 0x3F,
                                PrgMemoryType::PrgRom, Read);
        }
    }

    void InitMapper() override {
        sunsoft5b_reset(&_audio);
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        MapLowWindow(0);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xE000) {
            case 0x8000:
                _command = value & 0x0F;
                break;
            case 0xA000:
                if (_command < 8) {
                    SelectChrPage(_command, value);
                } else if (_command == 8) {
                    MapLowWindow(value);
                } else if (_command < 12) {
                    SelectPrgPage(_command - 9, value & 0x3F);
                } else if (_command == 12) {
                    static constexpr MirroringType modes[] = {
                        MirroringType::Vertical, MirroringType::Horizontal,
                        MirroringType::ScreenAOnly, MirroringType::ScreenBOnly
                    };
                    SetMirroringType(modes[value & 3]);
                } else if (_command == 13) {
                    _irqEnabled = (value & 1) != 0;
                    _irqCounterEnabled = (value & 0x80) != 0;
                    SetIrq(false);
                } else if (_command == 14) {
                    _irqCounter = (_irqCounter & 0xFF00) | value;
                } else {
                    _irqCounter = (_irqCounter & 0x00FF)
                                | (static_cast<uint16_t>(value) << 8);
                }
                break;
            case 0xC000:
            case 0xE000:
                sunsoft5b_write(&_audio, address, value);
                break;
        }
    }

public:
    void ProcessCpuClock() override {
        if (_irqCounterEnabled && --_irqCounter == 0xFFFF && _irqEnabled) SetIrq(true);
        sunsoft5b_clock(&_audio, 1);
    }

    float AudioOutput() const override { return sunsoft5b_output(&_audio); }
};

} // namespace cupid::boards
#endif
