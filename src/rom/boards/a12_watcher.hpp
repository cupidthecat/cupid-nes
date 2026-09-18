/*
 * a12_watcher.hpp - PPU-dot filtering for cartridge A12 transitions
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
#ifndef CUPID_BOARDS_A12_WATCHER_HPP
#define CUPID_BOARDS_A12_WATCHER_HPP
#include <cstdint>

namespace cupid::boards {

class A12Watcher {
    uint32_t _lastCycle = 0, _cyclesDown = 0;

public:
    bool Rising(uint16_t address, uint32_t frameCycle) {
        if (_cyclesDown) {
            _cyclesDown += _lastCycle > frameCycle
                ? 89342 - _lastCycle + frameCycle : frameCycle - _lastCycle;
        }
        bool rising = false;
        if (address & 0x1000) {
            rising = _cyclesDown > 10;
            _cyclesDown = 0;
        } else if (!_cyclesDown) _cyclesDown = 1;
        _lastCycle = frameCycle;
        return rising;
    }
};

} // namespace cupid::boards
#endif
