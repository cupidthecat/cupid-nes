/*
 * taito.hpp - Taito X1-017 cartridge register and RAM wiring
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
#ifndef CUPID_BOARDS_TAITO_HPP
#define CUPID_BOARDS_TAITO_HPP
#include "runtime.hpp"

namespace cupid::boards {

class TaitoX1005 final : public Board {
    bool _alternateMirroring = false;
    uint8_t _ramPermission = 0;

    void UpdateRamAccess() {
        PrgMemoryType type = HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam;
        SetCpuMemoryMapping(0x7F00, 0x7FFF, 0, type,
                            _ramPermission == 0xA3 ? ReadWrite : NoAccess);
    }

    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    uint16_t RegisterStartAddress() override { return 0x7EF0; }
    uint16_t RegisterEndAddress() override { return 0x7EFF; }
    uint32_t GetWorkRamSize() override { return 0x100; }
    uint32_t GetWorkRamPageSize() override { return 0x100; }
    uint32_t GetSaveRamSize() override { return 0x100; }
    uint32_t GetSaveRamPageSize() override { return 0x100; }
    bool ForceSaveRamSize() override { return HasBattery(); }
    bool ForceWorkRamSize() override { return !HasBattery(); }

    void InitMapper() override {
        _ramPermission = 0;
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        UpdateRamAccess();
    }

    void WriteRam(uint16_t address, uint8_t value) override {
        if ((address & 0xFF00) == 0x7F00)
            Board::WriteRam(address ^ 0x80, value);
        Board::WriteRam(address, value);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address) {
            case 0x7EF0:
                SelectChrPage(0, value);
                SelectChrPage(1, value + 1);
                if (_alternateMirroring) {
                    SetNametable(0, value >> 7);
                    SetNametable(1, value >> 7);
                }
                break;
            case 0x7EF1:
                SelectChrPage(2, value);
                SelectChrPage(3, value + 1);
                if (_alternateMirroring) {
                    SetNametable(2, value >> 7);
                    SetNametable(3, value >> 7);
                }
                break;
            case 0x7EF2: SelectChrPage(4, value); break;
            case 0x7EF3: SelectChrPage(5, value); break;
            case 0x7EF4: SelectChrPage(6, value); break;
            case 0x7EF5: SelectChrPage(7, value); break;
            case 0x7EF6:
            case 0x7EF7:
                if (!_alternateMirroring)
                    SetMirroringType(value & 1 ? MirroringType::Vertical : MirroringType::Horizontal);
                break;
            case 0x7EF8:
            case 0x7EF9:
                _ramPermission = value;
                UpdateRamAccess();
                break;
            case 0x7EFA:
            case 0x7EFB: SelectPrgPage(0, value); break;
            case 0x7EFC:
            case 0x7EFD: SelectPrgPage(1, value); break;
            case 0x7EFE:
            case 0x7EFF: SelectPrgPage(2, value); break;
        }
    }

public:
    explicit TaitoX1005(bool alternateMirroring) : _alternateMirroring(alternateMirroring) {}

    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.InvariantBool("taito_x1005.alternate_mirroring", _alternateMirroring)
            && state.Field("taito_x1005.ram_permission", _ramPermission);
    }
};

class TaitoX1017 final : public Board {
    uint8_t _chrMode = 0;
    uint8_t _chrRegs[6]{}, _ramPermission[3]{};
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x400; }
    uint16_t RegisterStartAddress() override { return 0x7EF0; }
    uint16_t RegisterEndAddress() override { return 0x7EFF; }
    uint32_t GetSaveRamSize() override { return 0x1400; }
    uint32_t GetSaveRamPageSize() override { return 0x400; }

    void UpdateRamAccess() {
        int8_t first = _ramPermission[0] == 0xCA ? ReadWrite : NoAccess;
        int8_t second = _ramPermission[1] == 0x69 ? ReadWrite : NoAccess;
        int8_t third = _ramPermission[2] == 0x84 ? ReadWrite : NoAccess;
        SetCpuMemoryMapping(0x6000, 0x63FF, 0, PrgMemoryType::SaveRam, first);
        SetCpuMemoryMapping(0x6400, 0x67FF, 1, PrgMemoryType::SaveRam, first);
        SetCpuMemoryMapping(0x6800, 0x6BFF, 2, PrgMemoryType::SaveRam, second);
        SetCpuMemoryMapping(0x6C00, 0x6FFF, 3, PrgMemoryType::SaveRam, second);
        SetCpuMemoryMapping(0x7000, 0x73FF, 4, PrgMemoryType::SaveRam, third);
    }

    void UpdateChrBanking() {
        uint16_t paired = _chrMode ? 2 : 0;
        uint16_t single = _chrMode ? 0 : 4;
        SelectChrPage2x(paired, _chrRegs[0] & 0xFE);
        SelectChrPage2x(paired + 1, _chrRegs[1] & 0xFE);
        for (uint16_t reg = 0; reg < 4; ++reg)
            SelectChrPage(single + reg, _chrRegs[reg + 2]);
    }

    void InitMapper() override {
        SelectPrgPage(3, static_cast<uint16_t>(-1));
        UpdateRamAccess();
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0x7EF0 && address <= 0x7EF5) {
            _chrRegs[address & 15] = value;
            UpdateChrBanking();
        } else if (address == 0x7EF6) {
            SetMirroringType(value & 1 ? MirroringType::Vertical : MirroringType::Horizontal);
            _chrMode = (value >> 1) & 1;
            UpdateChrBanking();
        } else if (address >= 0x7EF7 && address <= 0x7EF9) {
            _ramPermission[address - 0x7EF7] = value;
            UpdateRamAccess();
        } else if (address >= 0x7EFA && address <= 0x7EFC) {
            uint8_t page = _romInfo.MapperID == 82 ? value >> 2
                : ((value & 0x20) >> 5) | ((value & 0x10) >> 3)
                | ((value & 8) >> 1) | ((value & 4) << 1)
                | ((value & 2) << 3) | ((value & 1) << 5);
            SelectPrgPage(address - 0x7EFA, page);
        }
    }

public:
    bool VisitState(BoardStateVisitor &state) override {
        return Board::VisitState(state)
            && state.Field("taito_x1017.chr_mode", _chrMode, 1)
            && state.Field("taito_x1017.chr_regs", _chrRegs)
            && state.Field("taito_x1017.ram_permission", _ramPermission);
    }
};

} // namespace cupid::boards
#endif
