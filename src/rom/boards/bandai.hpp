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
};

} // namespace cupid::boards
#endif
