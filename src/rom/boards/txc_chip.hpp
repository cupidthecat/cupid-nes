/*
 * txc_chip.hpp - TXC and JV001 accumulator and protection logic
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
#ifndef CUPID_BOARDS_TXC_CHIP_HPP
#define CUPID_BOARDS_TXC_CHIP_HPP
#include <cstdint>

namespace cupid::boards {

class TxcChip {
    uint8_t _accumulator = 0, _inverter = 0, _staging = 0, _output = 0;
    bool _increase = false, _yFlag = false, _invert;
    uint8_t _mask;
    bool _isJv001;

public:
    explicit TxcChip(bool isJv001)
        : _invert(isJv001), _mask(isJv001 ? 0x0F : 0x07), _isJv001(isJv001) {}

    bool GetInvertFlag() const { return _invert; }
    bool GetY() const { return _yFlag; }
    uint8_t GetOutput() const { return _output; }

    uint8_t Read() {
        uint8_t value = (_accumulator & _mask)
                      | ((_inverter ^ (_invert ? 0xFF : 0)) & ~_mask);
        _yFlag = !_invert || (value & 0x10) != 0;
        return value;
    }

    void Write(uint16_t address, uint8_t value) {
        if (address < 0x8000) {
            switch (address & 0xE103) {
                case 0x4100:
                    if (_increase) ++_accumulator;
                    else _accumulator = ((_accumulator & ~_mask) | (_staging & _mask))
                                      ^ (_invert ? 0xFF : 0);
                    break;
                case 0x4101: _invert = (value & 1) != 0; break;
                case 0x4102:
                    _staging = value & _mask;
                    _inverter = value & ~_mask;
                    break;
                case 0x4103: _increase = (value & 1) != 0; break;
            }
        } else {
            _output = (_accumulator & 0x0F)
                    | (_isJv001 ? (_inverter & 0xF0) : ((_inverter & 8) << 1));
        }
        _yFlag = !_invert || (value & 0x10) != 0;
    }
};

} // namespace cupid::boards
#endif
