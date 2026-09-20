/*
 * bandai.hpp - Bandai 74161/7432 cartridge boards
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
#ifndef CUPID_BOARDS_BANDAI_HPP
#define CUPID_BOARDS_BANDAI_HPP
#include "runtime.hpp"

namespace cupid::boards {

class Bandai74161 final : public Board {
    bool _mirroringControl;

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        SelectPrgPage(1, static_cast<uint16_t>(-1));
        SelectChrPage(0, 0);
        // These boards boot vertically, including images with a horizontal header.
        SetMirroringType(MirroringType::Vertical);
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        bool screenB = (value & 0x80) != 0;
        // A set mirroring bit also identifies switchable boards carrying mapper 70.
        if (screenB) _mirroringControl = true;
        if (_mirroringControl)
            SetMirroringType(screenB ? MirroringType::ScreenBOnly : MirroringType::ScreenAOnly);
        SelectPrgPage(0, (value >> 4) & 7);
        SelectChrPage(0, value & 15);
    }

public:
    explicit Bandai74161(bool mirroringControl) : _mirroringControl(mirroringControl) {}

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bandai74161.mirroring_control", _mirroringControl);
    }
};

class BandaiKaraoke final : public Board {
    enum Input : unsigned { A, B, Microphone, InputCount };
    bool _inputs[InputCount]{};

    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }
    bool HasBusConflicts() override { return true; }

    void InitMapper() override {
        AddRegisterRange(0x6000, 0x7FFF, MemoryOperation::Read);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        SelectPrgPage(0, 0);
        SelectPrgPage(1, 0x07);
        SelectChrPage(0, 0);
    }

    uint8_t ReadRegister(uint16_t) override {
        return static_cast<uint8_t>((_inputs[A] ? 0 : 0x01)
            | (_inputs[B] ? 0 : 0x02)
            | (_inputs[Microphone] && (FrameCount() % 2) == 0 ? 0x04 : 0)
            | GetOpenBus(0xF8));
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        if (value & 0x10) {
            SelectPrgPage(0, value & 0x07);
        } else if (_prgSize >= 0x40000) {
            SelectPrgPage(0, (value & 0x07) | 0x08);
        } else {
            RemoveCpuMemoryMapping(0x8000, 0xBFFF);
        }
        SetMirroringType(value & 0x20 ? MirroringType::Horizontal : MirroringType::Vertical);
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("bandai_karaoke.inputs", _inputs);
    }

    bool SetMapperInput(unsigned input, bool pressed) override {
        if (input >= InputCount) return false;
        _inputs[input] = pressed;
        return true;
    }
};

} // namespace cupid::boards
#endif
