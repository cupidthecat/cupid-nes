/*
 * homebrew.hpp - Cartridge banking for homebrew boards
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
#ifndef CUPID_BOARDS_HOMEBREW_HPP
#define CUPID_BOARDS_HOMEBREW_HPP
#include "runtime.hpp"

namespace cupid::boards {

class SealieComputing final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetWorkRamSize() override { return 0x2000; }
    uint32_t GetChrRamSize() override { return 0x8000; }

    void InitMapper() override {
        SelectPrgPage(1, static_cast<uint16_t>(-1));
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::WorkRam, ReadWrite);
    }

    void WriteRegister(uint16_t, uint8_t value) override {
        SelectChrPage(0, value & 3);
        SelectPrgPage(0, (value >> 2) & 7);
    }
};

class NsfCartridge final : public Board {
    uint16_t RegisterStartAddress() override { return 0x5000; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    uint16_t GetPrgPageSize() override { return 0x1000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        WriteRegister(0x5FFF, 0xFF);
        SelectChrPage(0, 0);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        SelectPrgPage(address & 7, value);
    }
};

class MagicFloor final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }

    void InitMapper() override {
        SelectPrgPage(0, 0);
        if (GetMirroringType() == MirroringType::FourScreens)
            SetMirroringType((_romInfo.Header.Byte6 & 1)
                ? MirroringType::ScreenBOnly : MirroringType::ScreenAOnly);

        uint16_t mask = 0;
        switch (GetMirroringType()) {
            case MirroringType::Vertical: mask = 0x0400; break;
            case MirroringType::Horizontal: mask = 0x0800; break;
            case MirroringType::ScreenAOnly: mask = 0x1000; break;
            case MirroringType::ScreenBOnly: mask = 0x2000; break;
            case MirroringType::FourScreens: break;
        }
        for (uint16_t page = 0; page < 8; ++page) {
            uint16_t first = page * 0x400;
            SetPpuMemoryMapping(first, first + 0x3FF, (first & mask) ? 1 : 0,
                                ChrMemoryType::NametableRam);
        }
    }
};

} // namespace cupid::boards
#endif
