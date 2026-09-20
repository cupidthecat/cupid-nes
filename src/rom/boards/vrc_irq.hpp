/*
 * vrc_irq.hpp - VRC CPU and scanline interrupt counter
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
#ifndef CUPID_BOARDS_VRC_IRQ_HPP
#define CUPID_BOARDS_VRC_IRQ_HPP
#include "state_codec.hpp"
#include <cstdint>

namespace cupid::boards {

class VrcIrq {
    uint8_t _reload = 0, _counter = 0;
    uint16_t _prescaler = 0;
    bool _enabled = false, _enableAfterAck = false, _cycleMode = false;

public:
    bool VisitState(BoardStateVisitor &state) {
        return state.Field("vrc_irq.reload", _reload)
            && state.Field("vrc_irq.counter", _counter)
            && state.Field("vrc_irq.prescaler", _prescaler)
            && state.Field("vrc_irq.enabled", _enabled)
            && state.Field("vrc_irq.enable_after_ack", _enableAfterAck)
            && state.Field("vrc_irq.cycle_mode", _cycleMode);
    }

    bool Clock() {
        if (!_enabled) return false;
        _prescaler = static_cast<uint16_t>(_prescaler - 3);
        if (!_cycleMode && _prescaler && !(_prescaler & 0x8000)) return false;
        bool asserted = _counter == 0xFF;
        _counter = asserted ? _reload : static_cast<uint8_t>(_counter + 1);
        _prescaler = static_cast<uint16_t>(_prescaler + 341);
        return asserted;
    }

    void SetReloadNibble(uint8_t value, bool high) {
        _reload = static_cast<uint8_t>(high ? (_reload & 15) | ((value & 15) << 4)
                                           : (_reload & 0xF0) | (value & 15));
    }

    void SetControl(uint8_t value) {
        _enableAfterAck = (value & 1) != 0;
        _enabled = (value & 2) != 0;
        _cycleMode = (value & 4) != 0;
        if (_enabled) { _counter = _reload; _prescaler = 341; }
    }

    void Acknowledge() { _enabled = _enableAfterAck; }
};

} // namespace cupid::boards
#endif
