/*
 * jy_small.hpp - JY Company small cartridge boards and interrupt counters
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
#ifndef CUPID_BOARDS_JY_SMALL_HPP
#define CUPID_BOARDS_JY_SMALL_HPP
#include "runtime.hpp"
#include "mmc3.hpp"

namespace cupid::boards {

class Jy35 final : public Board {
    uint8_t _counter = 0;
    bool _enabled = false;
    uint32_t _lastFrameCycle = 0, _cyclesDown = 0;

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    bool EnableVramAddressHook() override { return true; }
    void InitMapper() override {
        _counter = 0;
        _enabled = false;
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xF007) {
            case 0x8000: case 0x8001: case 0x8002: case 0x8003:
                SelectPrgPage(address & 3, value);
                break;
            case 0x9000: case 0x9001: case 0x9002: case 0x9003:
            case 0x9004: case 0x9005: case 0x9006: case 0x9007:
                SelectChrPage(address & 7, value);
                break;
            case 0xC002: _enabled = false; SetIrq(false); break;
            case 0xC003: _enabled = true; break;
            case 0xC005: _counter = value; break;
            case 0xD001:
                SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
                break;
        }
    }

public:
    void NotifyVramAddressChange(uint16_t address) override {
        uint32_t frameCycle = PpuFrameCycle();
        if (_cyclesDown) {
            _cyclesDown += _lastFrameCycle > frameCycle ? 89342u - _lastFrameCycle + frameCycle
                                                       : frameCycle - _lastFrameCycle;
        }
        bool rising = false;
        if (!(address & 0x1000)) {
            if (!_cyclesDown) _cyclesDown = 1;
        } else {
            rising = _cyclesDown > 10;
            _cyclesDown = 0;
        }
        _lastFrameCycle = frameCycle;
        if (rising && _enabled && --_counter == 0) {
            _enabled = false;
            SetIrq(true);
        }
    }
};

class Jy91 final : public Mmc3 {
    uint16_t RegisterStartAddress() override { return 0x6000; }
    uint16_t RegisterEndAddress() override { return 0x7FFF; }
    uint16_t GetChrPageSize() override { return 0x800; }
    void InitMapper() override {
        SelectPrgPage(2, static_cast<uint16_t>(-2));
        SelectPrgPage(3, static_cast<uint16_t>(-1));
    }
    void UpdateState() override {}
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0x7003) {
            case 0x6000: case 0x6001: case 0x6002: case 0x6003:
                SelectChrPage(address & 3, value);
                break;
            case 0x7000: case 0x7001:
                SelectPrgPage(address & 1, value & 0x0F);
                break;
            case 0x7002: Mmc3::WriteRegister(0xE000, value); break;
            case 0x7003:
                Mmc3::WriteRegister(0xC000, 7);
                Mmc3::WriteRegister(0xC001, value);
                Mmc3::WriteRegister(0xE001, value);
                break;
        }
    }
};

} // namespace cupid::boards
#endif
