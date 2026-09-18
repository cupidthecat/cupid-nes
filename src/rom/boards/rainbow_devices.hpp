/*
 * rainbow_devices.hpp - Flash command decoder and cartridge audio generators
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
#ifndef CUPID_BOARDS_RAINBOW_DEVICES_HPP
#define CUPID_BOARDS_RAINBOW_DEVICES_HPP
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace cupid::boards {

class RainbowFlash {
    enum class Mode { Command, Program, Erase };
    uint8_t *_bytes = nullptr;
    uint32_t _size = 0;
    Mode _mode = Mode::Command;
    uint8_t _cycle = 0;
    bool _identify = false, _bypass = false;

    void ResetCommand() { _mode = Mode::Command; _cycle = 0; }

    void EraseSector(uint32_t address) {
        if (address >= _size) return;
        uint32_t first = address & ~0xFFFFu;
        uint32_t length = 0x10000;
        if (_size >= 0x10000 && first == (_size / 0x10000 - 1) * 0x10000) {
            // The boot sectors occupy the last 64 KiB of the chip.
            if (_size == 0x400000 || _size == 0x800000) {
                first = address & ~0x1FFFu;
                length = 0x2000;
            } else {
                static constexpr uint32_t lengths[] = {0x8000, 0x2000, 0x2000, 0x4000};
                for (uint32_t sector : lengths) {
                    length = sector;
                    if (address < first + length) break;
                    first += length;
                }
            }
        }
        if (first < _size) std::memset(_bytes + first, 0xFF, std::min(length, _size - first));
    }

public:
    void Initialize(uint8_t *bytes, uint32_t size) { _bytes = bytes; _size = size; }
    bool Identifying() const { return _identify; }

    uint8_t Read(uint32_t address) const {
        switch (address & 0x1FF) {
            case 0: return 0x01;
            case 2:
                if (_size == 0x200000) return 0x49;
                if (_size == 0x400000 || _size == 0x800000) return 0x7E;
                return 0x5B;
            case 0x1C:
                return _size == 0x400000 ? 0x0A : _size == 0x800000 ? 0x10 : 0xFF;
            case 0x1E: return _size == 0x400000 || _size == 0x800000 ? 0 : 0xFF;
            default: return 0xFF;
        }
    }

    void Write(uint32_t address, uint8_t value) {
        uint16_t command = address & 0xFFF;
        if (_mode == Mode::Program) {
            if (address < _size) _bytes[address] &= value;
            ResetCommand();
            return;
        }
        if (_mode == Mode::Erase) {
            if (_cycle == 3 && command == 0xAAA && value == 0xAA) ++_cycle;
            else if (_cycle == 4 && command == 0x555 && value == 0x55) ++_cycle;
            else if (_cycle == 5) {
                if (command == 0xAAA && value == 0x10) {
                    if (_size) std::memset(_bytes, 0xFF, _size);
                } else if (value == 0x30) EraseSector(address);
                ResetCommand();
            } else ResetCommand();
            return;
        }
        if (_bypass) {
            if (_cycle == 0) {
                if (value == 0xA0) _mode = Mode::Program;
                else if (value == 0x90) ++_cycle;
                else ResetCommand();
            } else {
                if (value == 0) _bypass = false;
                ResetCommand();
            }
            return;
        }
        if (_cycle == 0) {
            if (command == 0xAAA && value == 0xAA) ++_cycle;
            else if (value == 0xF0) { ResetCommand(); _identify = false; }
        } else if (_cycle == 1 && command == 0x555 && value == 0x55) ++_cycle;
        else if (_cycle == 2 && command == 0xAAA) {
            ++_cycle;
            switch (value) {
                case 0x20: ResetCommand(); _bypass = true; break;
                case 0x80: _mode = Mode::Erase; break;
                case 0x90: ResetCommand(); _identify = true; break;
                case 0xA0: _mode = Mode::Program; break;
                case 0xF0: ResetCommand(); _identify = false; break;
            }
        } else _cycle = 0;
    }
};

class RainbowPulse {
    uint8_t _volume = 0, _duty = 0, _step = 0;
    uint16_t _frequency = 1, _timer = 1;
    bool _enabled = false, _constant = false;
public:
    void Write(unsigned reg, uint8_t value) {
        if (reg == 0) {
            _volume = value & 15;
            _duty = (value >> 4) & 7;
            _constant = (value & 0x80) != 0;
        } else if (reg == 1) _frequency = (_frequency & 0xF00) | value;
        else {
            _frequency = (_frequency & 0xFF) | ((value & 15) << 8);
            _enabled = (value & 0x80) != 0;
            if (!_enabled) _step = 0;
        }
    }
    void Clock() {
        if (_enabled && --_timer == 0) {
            _step = (_step + 1) & 15;
            _timer = _frequency + 1;
        }
    }
    uint8_t Output() const { return _enabled && (_constant || _step <= _duty) ? _volume : 0; }
};

class RainbowSaw {
    uint8_t _rate = 0, _accumulator = 0, _step = 0;
    uint16_t _frequency = 1, _timer = 1;
    bool _enabled = false;
public:
    void Write(unsigned reg, uint8_t value) {
        if (reg == 0) _rate = value & 0x3F;
        else if (reg == 1) _frequency = (_frequency & 0xF00) | value;
        else {
            _frequency = (_frequency & 0xFF) | ((value & 15) << 8);
            _enabled = (value & 0x80) != 0;
            if (!_enabled) _step = _accumulator = 0;
        }
    }
    void Clock() {
        if (_enabled && --_timer == 0) {
            _step = (_step + 1) % 14;
            _timer = _frequency + 1;
            if (!_step) _accumulator = 0;
            else if (!(_step & 1)) _accumulator += _rate;
        }
    }
    uint8_t Output() const { return _enabled ? _accumulator >> 3 : 0; }
};

class RainbowAudio {
    std::array<RainbowPulse, 2> _pulse;
    RainbowSaw _saw;
    uint8_t _routing = 0, _volume = 0, _lastOutput = 0;
public:
    void Clock() {
        _pulse[0].Clock();
        _pulse[1].Clock();
        _saw.Clock();
        _lastOutput = _pulse[0].Output() + _pulse[1].Output() + _saw.Output();
    }
    void Write(uint16_t address, uint8_t value) {
        unsigned reg = address & 15;
        if (reg < 6) _pulse[reg / 3].Write(reg % 3, value);
        else if (reg < 9) _saw.Write(reg - 6, value);
        else if (reg == 9) _routing = value & 7;
        else if (reg == 10) _volume = value & 15;
    }
    uint8_t LastOutput() const { return _lastOutput; }
    float Output() const { return (_routing & 3) ? -_lastOutput * (_volume * 5.0f / 5000.0f) : 0.0f; }
};

} // namespace cupid::boards
#endif
