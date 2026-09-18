/*
 * unif.hpp - Named cartridge boards used by UNIF images
 *
 * Copyright (C) 2014-2026 Sour and contributors
 * Copyright (C) 2026 Francis Hagan
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_UNIF_BOARDS_HPP
#define CUPID_UNIF_BOARDS_HPP

#include "runtime.hpp"
#include "mmc3_96.hpp"

namespace cupid::boards {

class UnifMalee final : public Board {
protected:
    uint16_t GetPrgPageSize() override { return 0x0800; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        SelectPrgPage4x(0, 0); SelectPrgPage4x(1, 4);
        SelectPrgPage4x(2, 8); SelectPrgPage4x(3, 12);
        SelectChrPage(0, 0);
        SetCpuMemoryMapping(0x6000, 0x67FF, 16, PrgMemoryType::PrgRom);
    }
};

class UnifGs2013 final : public Board {
protected:
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectChrPage(0, 0); Reset(false); }
    void Reset(bool) override {
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0x1F, PrgMemoryType::PrgRom);
        SelectPrgPage4x(0, 0x0F << 2);
    }
    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage4x(0, static_cast<uint16_t>((value & 0x0F) << 2));
    }
};

class UnifGhostbusters63in1 final : public Board {
    uint8_t regs[2]{};
    void UpdateState() {
        uint8_t chip = static_cast<uint8_t>((regs[1] << 5 & 0x20) << (regs[0] >> 7));
        if (chip < (regs[0] >> 7)) {
            RemoveCpuMemoryMapping(0x8000, 0xFFFF);
        } else {
            SelectPrgPage(0, static_cast<uint16_t>(chip | (regs[0] & 0x1E) | (regs[0] >> 5 & regs[0])));
            SelectPrgPage(1, static_cast<uint16_t>(chip | (regs[0] & 0x1F) | (~regs[0] >> 5 & 0x01)));
        }
        SelectChrPage(0, 0);
        SetMirroringType(regs[0] & 0x40 ? MirroringType::Vertical : MirroringType::Horizontal);
    }
protected:
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { Reset(false); }
    void Reset(bool) override { regs[0] = regs[1] = 0; UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        regs[address & 1] = value; UpdateState();
    }
};

class UnifCc21 final : public Board {
protected:
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x1000; }
    void InitMapper() override {
        SelectPrgPage(0, 0); SelectChrPage(0, 0); SelectChrPage(1, 0);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        uint8_t latch = address == 0x8000 ? value : static_cast<uint8_t>(address);
        if (_chrRomSize == 0x2000) {
            SelectChrPage(0, latch & 1); SelectChrPage(1, latch & 1);
        } else {
            SelectChrPage2x(0, static_cast<uint16_t>((latch & 1) << 1));
        }
        SetMirroringType(latch & 1 ? MirroringType::ScreenBOnly : MirroringType::ScreenAOnly);
    }
};

class UnifAc08 final : public Board {
    uint8_t reg = 0;
    void UpdateState() { SetCpuMemoryMapping(0x6000, 0x7FFF, reg, PrgMemoryType::PrgRom); }
protected:
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        AddRegisterRange(0x4025, 0x4025, MemoryOperation::Write);
        reg = 0;
        SelectPrgPage4x(0, static_cast<uint16_t>(-4));
        SelectChrPage(0, 0);
        UpdateState();
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x4025) {
            SetMirroringType(value & 0x08 ? MirroringType::Horizontal : MirroringType::Vertical);
        } else {
            reg = address == 0x8001 ? static_cast<uint8_t>((value >> 1) & 0x0F)
                                    : static_cast<uint8_t>(value & 0x0F);
            UpdateState();
        }
    }
};

class UnifPuzzle final : public Board {
protected:
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4100; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override {
        SelectPrgPage(0, (value >> 3) & 1); SelectChrPage(0, value & 7);
    }
};

class Unif255in1 final : public Board {
protected:
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { WriteRegister(0x8000, 0); }
    void WriteRegister(uint16_t address, uint8_t) override {
        SelectChrPage(0, address & 7); SelectPrgPage(0, (address >> 2) & 3);
    }
};

class UnifFamicomBox final : public Board {
    uint8_t regs[8]{};
    std::array<uint8_t, 0x2000> cpuRam{};
protected:
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x5000; }
    uint16_t RegisterEndAddress() override { return 0x5FFF; }
    bool AllowRegisterRead() override { return true; }
    void InitMapper() override {
        regs[7] = 0xFF; SelectPrgPage(0, 0); SelectPrgPage(1, 1); SelectChrPage(0, 0);
    }
    void Reset(bool) override { for (unsigned i = 0; i < 6; ++i) regs[i] = 0; }
    uint8_t ReadRegister(uint16_t address) override {
        switch (address & 7) {
            case 0: regs[0] = 0xFF; return regs[0];
            case 2: return static_cast<uint8_t>(GetDipSwitches());
            case 3: case 4: case 5: case 6: return 0;
            case 7: return 0x22;
            default: return 0;
        }
    }
    void WriteRegister(uint16_t, uint8_t) override {}
public:
    uint8_t *CpuRam8K() override { return cpuRam.data(); }
};

class Unif8237A final : public Mmc3_215 {
    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        uint8_t sbank = 0, bank = 0, mask = 0;
        if (_exRegs[0] & 0x40) {
            mask = 0x0F; sbank = _exRegs[1] & 0x10;
            if (_exRegs[0] & 0x80)
                bank = static_cast<uint8_t>(((_exRegs[1] & 0x03) << 4) | ((_exRegs[1] & 0x08) << 3)
                    | (_exRegs[0] & 0x07) | (sbank >> 1));
        } else {
            mask = 0x1F;
            if (_exRegs[0] & 0x80)
                bank = static_cast<uint8_t>(((_exRegs[1] & 0x03) << 4) | ((_exRegs[1] & 0x08) << 3)
                    | (_exRegs[0] & 0x0F));
        }
        if (_exRegs[0] & 0x80) {
            bank <<= 1;
            if (_exRegs[0] & 0x20) {
                bank &= 0xFC;
                for (uint16_t index = 0; index < 4; ++index)
                    Board::SelectPrgPage(index, static_cast<uint16_t>(bank + index), type);
            } else {
                for (uint16_t index = 0; index < 4; ++index)
                    Board::SelectPrgPage(index, static_cast<uint16_t>(bank + (index & 1)), type);
            }
        } else {
            Board::SelectPrgPage(slot, static_cast<uint16_t>(((_exRegs[1] & 0x03) << 5)
                | ((_exRegs[1] & 0x08) << 4) | (page & mask) | sbank), type);
        }
    }
    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        uint16_t mapped = _exRegs[0] & 0x40
            ? static_cast<uint16_t>(((_exRegs[1] & 0x0E) << 7) | (page & 0x7F) | ((_exRegs[1] & 0x20) << 2))
            : static_cast<uint16_t>(((_exRegs[1] & 0x0E) << 7) | page);
        Board::SelectChrPage(slot, mapped, type);
    }
};

} // namespace cupid::boards

#endif
