/*
 * drip_game.hpp - Drip Game FIFO audio, timer, and extended attributes
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
#ifndef CUPID_BOARDS_DRIP_GAME_HPP
#define CUPID_BOARDS_DRIP_GAME_HPP
#include "runtime.hpp"

namespace cupid::boards {

class DripFifoAudio {
    std::array<uint8_t, 256> _buffer{};
    uint8_t _read = 0, _write = 0, _volume = 0;
    uint16_t _period = 0, _timer = 0;
    int16_t _output = 0;
    bool _full = false, _empty = true;

    void UpdateOutput(uint8_t value) { _output = (static_cast<int>(value) - 0x80) * _volume; }

public:
    bool VisitState(BoardStateVisitor &state) {
        return state.Field("drip.buffer", _buffer)
            && state.Field("drip.read", _read)
            && state.Field("drip.write", _write)
            && state.Field("drip.volume", _volume, 15)
            && state.Field("drip.period", _period, 0x0FFF)
            && state.Field("drip.timer", _timer)
            && state.Field("drip.output", _output)
            && state.Field("drip.full", _full)
            && state.Field("drip.empty", _empty);
    }

    void Clock() {
        if (_empty) return;
        if (--_timer == 0) {
            _timer = _period;
            if (_read == _write) _full = false;
            ++_read;
            UpdateOutput(_buffer[_read]);
            if (_read == _write) _empty = true;
        }
    }

    uint8_t Status() const { return (_full ? 0x80 : 0) | (_empty ? 0x40 : 0); }
    int16_t Output() const { return _output; }

    void Write(uint16_t address, uint8_t value) {
        switch (address & 3) {
            case 0:
                _buffer.fill(0);
                _read = _write = 0;
                _full = false;
                _empty = true;
                _output = 0;
                _timer = _period;
                break;
            case 1:
                if (_read == _write) {
                    _empty = false;
                    UpdateOutput(value);
                    _timer = _period;
                }
                _buffer[_write++] = value;
                if (_read == _write) _full = true;
                break;
            case 2: _period = (_period & 0x0F00) | value; break;
            case 3:
                _period = (_period & 0xFF) | ((value & 0x0F) << 8);
                _volume = value >> 4;
                if (!_empty) UpdateOutput(_buffer[_read]);
                break;
        }
    }
};

class DripGame final : public Board {
    std::array<DripFifoAudio, 2> _audio;
    uint16_t _irqCounter = 0, _lastNametable = 0;
    uint8_t _irqLow = 0;
    bool _irqEnabled = false, _extendedAttributes = false, _workRamEnabled = false;

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x800; }
    uint32_t GetDipSwitchCount() override { return 1; }
    uint32_t GetMapperRamSize() override { return 0x800; }
    bool EnableCpuClockHook() override { return true; }
    bool EnableCustomVramRead() override { return true; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        _audio = {};
        _irqCounter = _lastNametable = 0;
        _irqLow = 0;
        _irqEnabled = _extendedAttributes = _workRamEnabled = false;
        AddRegisterRange(0x4800, 0x5FFF, MemoryOperation::Read);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        SelectPrgPage(1, static_cast<uint16_t>(-1));
    }

    uint8_t ReadRegister(uint16_t address) override {
        return PeekCpu(address, GetOpenBus());
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0xC000) {
            _mapperRam[address & 0x7FF] = value;
            return;
        }
        switch (address & 0x800F) {
            case 0x8000: case 0x8001: case 0x8002: case 0x8003:
                _audio[0].Write(address, value);
                break;
            case 0x8004: case 0x8005: case 0x8006: case 0x8007:
                _audio[1].Write(address, value);
                break;
            case 0x8008: _irqLow = value; break;
            case 0x8009:
                _irqCounter = ((value & 0x7F) << 8) | _irqLow;
                _irqEnabled = (value & 0x80) != 0;
                SetIrq(false);
                break;
            case 0x800A: {
                const MirroringType modes[] = {MirroringType::Vertical, MirroringType::Horizontal,
                                                MirroringType::ScreenAOnly, MirroringType::ScreenBOnly};
                SetMirroringType(modes[value & 3]);
                _extendedAttributes = (value & 4) != 0;
                _workRamEnabled = (value & 8) != 0;
                SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::WorkRam,
                                    _workRamEnabled ? ReadWrite : Read);
                break;
            }
            case 0x800B: SelectPrgPage(0, value & 0x0F); break;
            case 0x800C: case 0x800D: case 0x800E: case 0x800F:
                SelectChrPage(address & 3, value & 0x0F);
                break;
        }
    }

public:
    uint8_t PeekCpu(uint16_t address, uint8_t openBus) const override {
        if (address >= 0x4800 && address <= 0x5FFF) {
            switch (address & 0x5800) {
                case 0x4800: return (GetDipSwitches(1) ? 0x80 : 0) | 0x64;
                case 0x5000: return _audio[0].Status();
                case 0x5800: return _audio[1].Status();
            }
        }
        return Board::PeekCpu(address, openBus);
    }

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && _audio[0].VisitState(state)
            && _audio[1].VisitState(state)
            && state.Field("drip_game.irq_counter", _irqCounter)
            && state.Field("drip_game.last_nametable", _lastNametable, 0x03FF)
            && state.Field("drip_game.irq_low", _irqLow)
            && state.Field("drip_game.irq_enabled", _irqEnabled)
            && state.Field("drip_game.extended_attributes", _extendedAttributes)
            && state.Field("drip_game.work_ram_enabled", _workRamEnabled);
    }

    void ProcessCpuClock() override {
        if (_irqEnabled && _irqCounter && --_irqCounter == 0) {
            _irqEnabled = false;
            SetIrq(true);
        }
        _audio[0].Clock();
        _audio[1].Clock();
    }

    uint8_t MapperReadVram(uint16_t address, MemoryOperationType operation) override {
        if (_extendedAttributes && operation == MemoryOperationType::PpuRenderingRead && address >= 0x2000) {
            if ((address & 0x3FF) < 0x3C0) {
                _lastNametable = address & 0x3FF;
            } else {
                unsigned bank = 0;
                switch (GetMirroringType()) {
                    case MirroringType::Horizontal: bank = (address >> 11) & 1; break;
                    case MirroringType::Vertical: bank = (address >> 10) & 1; break;
                    case MirroringType::ScreenBOnly: bank = 1; break;
                    default: break;
                }
                return (_mapperRam[bank * 0x400 + _lastNametable] & 3) * 0x55;
            }
        }
        return InternalReadVram(address);
    }

    float AudioOutput() const override {
        return (_audio[0].Output() + _audio[1].Output()) * (3.0f / 5000.0f);
    }
};

} // namespace cupid::boards
#endif
