/*
 * mmc3_97.hpp - Later MMC3-derived multicart and protection boards
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
#ifndef CUPID_BOARDS_MMC3_97_HPP
#define CUPID_BOARDS_MMC3_97_HPP

#include "mmc3.hpp"

namespace cupid::boards {

class Mmc3_217 final : public Mmc3 {
    uint8_t _exRegs[4]{};
    inline static constexpr uint8_t Lut[8] = {0, 6, 3, 7, 5, 2, 4, 1};

    void InitMapper() override {
        AddRegisterRange(0x5000, 0x5001, MemoryOperation::Write);
        AddRegisterRange(0x5007, 0x5007, MemoryOperation::Write);
        Mmc3::InitMapper();
    }

    void Reset(bool) override {
        _exRegs[0] = 0;
        _exRegs[1] = 0xFF;
        _exRegs[2] = 0x03;
        _exRegs[3] = 0;
        UpdateState();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (!(_exRegs[1] & 0x08))
            page = ((_exRegs[1] << 3) & 0x80) | (page & 0x7F);
        Mmc3::SelectChrPage(slot, ((_exRegs[1] << 8) & 0x0300) | page, type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_exRegs[1] & 0x08)
            page &= 0x1F;
        else
            page = (page & 0x0F) | (_exRegs[1] & 0x10);
        Mmc3::SelectPrgPage(slot, ((_exRegs[1] << 5) & 0x60) | page, type);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            switch (address) {
                case 0x5000:
                    _exRegs[0] = value;
                    if (value & 0x80) {
                        value = (value & 0x0F) | ((_exRegs[1] << 4) & 0x30);
                        value <<= 1;
                        SelectPrgPage(0, value);
                        SelectPrgPage(1, value + 1);
                        SelectPrgPage(2, value);
                        SelectPrgPage(3, value + 1);
                    } else {
                        UpdatePrgMapping();
                    }
                    break;
                case 0x5001:
                    if (_exRegs[1] != value) {
                        _exRegs[1] = value;
                        UpdatePrgMapping();
                    }
                    break;
                case 0x5007:
                    _exRegs[2] = value;
                    break;
            }
            return;
        }

        switch (address & 0xE001) {
            case 0x8000:
                Mmc3::WriteRegister(_exRegs[2] ? 0xC000 : 0x8000, value);
                break;
            case 0x8001:
                if (_exRegs[2]) {
                    value = (value & 0xC0) | Lut[value & 7];
                    _exRegs[3] = 1;
                    Mmc3::WriteRegister(0x8000, value);
                } else {
                    Mmc3::WriteRegister(0x8001, value);
                }
                break;
            case 0xA000:
                if (_exRegs[2]) {
                    if (_exRegs[3] && (!(_exRegs[0] & 0x80) || GetCurrentRegister() < 6)) {
                        _exRegs[3] = 0;
                        Mmc3::WriteRegister(0x8001, value);
                    }
                } else {
                    SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
                }
                break;
            case 0xA001:
                if (_exRegs[2])
                    SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
                else
                    Mmc3::WriteRegister(0xA001, value);
                break;
            default:
                Mmc3::WriteRegister(address, value);
                break;
        }
    }
};

class Mmc3_219 final : public Mmc3 {
    uint8_t _exRegs[3]{};

    void InitMapper() override {
        Mmc3::InitMapper();
        SelectPrgPage4x(0, static_cast<uint16_t>(-4));
        SelectChrPage8x(0, 0);
        std::memset(_exRegs, 0, sizeof(_exRegs));
    }

    void UpdatePrgMapping() override {}
    void UpdateChrMapping() override {}

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0xA000) {
            switch (address & 0xE003) {
                case 0x8000:
                    _exRegs[0] = 0;
                    _exRegs[1] = value;
                    break;
                case 0x8001:
                    if (_exRegs[0] >= 0x23 && _exRegs[0] <= 0x26) {
                        uint8_t bank = ((value & 0x20) >> 5) | ((value & 0x10) >> 3)
                                     | ((value & 0x08) >> 1) | ((value & 0x04) << 1);
                        SelectPrgPage(0x26 - _exRegs[0], bank);
                    }
                    switch (_exRegs[1]) {
                        case 0x08: case 0x0A: case 0x0E: case 0x12:
                        case 0x16: case 0x1A: case 0x1E:
                            _exRegs[2] = value << 4;
                            break;
                        case 0x09: SelectChrPage(0, _exRegs[2] | ((value >> 1) & 0x0E)); break;
                        case 0x0B: SelectChrPage(1, _exRegs[2] | ((value >> 1) | 0x01)); break;
                        case 0x0C: case 0x0D:
                            SelectChrPage(2, _exRegs[2] | ((value >> 1) & 0x0E)); break;
                        case 0x0F: SelectChrPage(3, _exRegs[2] | ((value >> 1) | 0x01)); break;
                        case 0x10: case 0x11:
                            SelectChrPage(4, _exRegs[2] | ((value >> 1) & 0x0F)); break;
                        case 0x14: case 0x15:
                            SelectChrPage(5, _exRegs[2] | ((value >> 1) & 0x0F)); break;
                        case 0x18: case 0x19:
                            SelectChrPage(6, _exRegs[2] | ((value >> 1) & 0x0F)); break;
                        case 0x1C: case 0x1D:
                            SelectChrPage(7, _exRegs[2] | ((value >> 1) & 0x0F)); break;
                    }
                    break;
                case 0x8002:
                    _exRegs[0] = value;
                    _exRegs[1] = 0;
                    break;
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

class Mmc3_224 final : public Mmc3 {
    uint8_t _outerBank = 0;

    void InitMapper() override {
        _outerBank = 0;
        AddRegisterRange(0x5000, 0x5003, MemoryOperation::Write);
        Mmc3::InitMapper();
    }

    void UpdatePrgMapping() override {
        uint16_t outer = static_cast<uint16_t>(_outerBank << 6);
        if (_prgMode == 0) {
            SelectPrgPage(0, (_registers[6] & 0x3F) | outer);
            SelectPrgPage(1, (_registers[7] & 0x3F) | outer);
            SelectPrgPage(2, 0x3E | outer);
            SelectPrgPage(3, 0x3F | outer);
        } else {
            SelectPrgPage(0, 0x3E | outer);
            SelectPrgPage(1, (_registers[6] & 0x3F) | outer);
            SelectPrgPage(2, (_registers[7] & 0x3F) | outer);
            SelectPrgPage(3, 0x3F | outer);
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (address == 0x5000) {
                _outerBank = (value >> 2) & 1;
                UpdatePrgMapping();
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

class Mmc3_238 final : public Mmc3 {
    inline static constexpr uint8_t SecurityLut[4] = {0x00, 0x02, 0x02, 0x03};
    uint8_t _exReg = 0;

    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        Mmc3::InitMapper();
        _exReg = 0;
        AddRegisterRange(0x4020, 0x7FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    uint8_t ReadRegister(uint16_t) override { return _exReg; }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000)
            _exReg = SecurityLut[value & 3];
        else
            Mmc3::WriteRegister(address, value);
    }
};

class Mmc3_245 final : public Mmc3 {
    void UpdateState() override {
        Mmc3::UpdateState();
        if (HasChrRam()) {
            if (_chrMode) {
                SelectChrPage4x(0, 4);
                SelectChrPage4x(1, 0);
            } else {
                SelectChrPage4x(0, 0);
                SelectChrPage4x(1, 4);
            }
        }
    }

    void UpdatePrgMapping() override {
        uint8_t outer = _registers[0] & 0x02 ? 0x40 : 0;
        _registers[6] = (_registers[6] & 0x3F) | outer;
        _registers[7] = (_registers[7] & 0x3F) | outer;
        uint16_t last = GetPrgPageCount() >= 0x40
                      ? static_cast<uint16_t>(0x3F | outer)
                      : static_cast<uint16_t>(-1);
        if (_prgMode == 0) {
            SelectPrgPage(0, _registers[6]);
            SelectPrgPage(1, _registers[7]);
            SelectPrgPage(2, last - 1);
            SelectPrgPage(3, last);
        } else {
            SelectPrgPage(0, last - 1);
            SelectPrgPage(1, _registers[7]);
            SelectPrgPage(2, _registers[6]);
            SelectPrgPage(3, last);
        }
    }
};

class Mmc3_249 final : public Mmc3 {
    uint8_t _exReg = 0;

    void InitMapper() override {
        Mmc3::InitMapper();
        AddRegisterRange(0x5000, 0x5000, MemoryOperation::Write);
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (_exReg & 0x02)
            page = (page & 0x03) | ((page >> 1) & 0x04) | ((page >> 4) & 0x08)
                 | ((page >> 2) & 0x10) | ((page << 3) & 0x20) | ((page << 2) & 0xC0);
        Board::SelectChrPage(slot, page, type);
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_exReg & 0x02) {
            if (page < 0x20)
                page = (page & 0x01) | ((page >> 3) & 0x02) | ((page >> 1) & 0x04)
                     | ((page << 2) & 0x18);
            else {
                page -= 0x20;
                page = (page & 0x03) | ((page >> 1) & 0x04) | ((page >> 4) & 0x08)
                     | ((page >> 2) & 0x10) | ((page << 3) & 0x20) | ((page << 2) & 0xC0);
            }
        }
        Board::SelectPrgPage(slot, page, type);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x5000) {
            _exReg = value;
            UpdatePrgMapping();
            UpdateChrMapping();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

class Mmc3_250 final : public Mmc3 {
    void WriteRegister(uint16_t address, uint8_t) override {
        Mmc3::WriteRegister((address & 0xE000) | ((address & 0x0400) >> 10),
                            static_cast<uint8_t>(address));
    }
};

class Mmc3_254 final : public Mmc3 {
    uint8_t _exRegs[2]{};

    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        Mmc3::InitMapper();
        AddRegisterRange(0x6000, 0x7FFF, MemoryOperation::Read);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    uint8_t ReadRegister(uint16_t address) override {
        uint8_t value = InternalReadRam(address);
        return _exRegs[0] ? value : static_cast<uint8_t>(value ^ _exRegs[1]);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x8000) _exRegs[0] = 0xFF;
        if (address == 0xA001) _exRegs[1] = value;
        Mmc3::WriteRegister(address, value);
    }
};

class Unl158B final : public Mmc3 {
    inline static constexpr uint8_t ProtectionLut[8] = {0, 0, 0, 1, 2, 4, 0x0F, 0};
    uint8_t _reg = 0;

    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        _reg = 0;
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        Mmc3::InitMapper();
    }

    void Reset(bool) override {
        _reg = 0;
        ResetMmc3();
        UpdateState();
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_reg & 0x80) {
            uint16_t bank = _reg & 7;
            if (_reg & 0x20)
                SelectPrgPage4x(0, (bank & 6) << 1);
            else {
                SelectPrgPage2x(0, bank << 1);
                SelectPrgPage2x(1, bank << 1);
            }
        } else {
            Mmc3::SelectPrgPage(slot, page & 0x0F, type);
        }
    }

    uint8_t ReadRegister(uint16_t address) override {
        return static_cast<uint8_t>(GetOpenBus() | ProtectionLut[address & 7]);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0x5FFF) {
            if ((address & 7) == 0) {
                _reg = value;
                UpdatePrgMapping();
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

class Mmc3_BmcF15 final : public Mmc3 {
    uint8_t _exReg = 0;

    void InitMapper() override {
        AddRegisterRange(0x6000, 0xFFFF, MemoryOperation::Write);
        _exReg = 0;
        Mmc3::InitMapper();
    }

    void UpdatePrgMapping() override {
        uint16_t bank = _exReg & 0x0F;
        uint16_t mode = (_exReg & 0x08) >> 3;
        uint16_t mask = static_cast<uint16_t>(~mode);
        SelectPrgPage2x(0, (bank & mask) << 1);
        SelectPrgPage2x(1, ((bank & mask) | mode) << 1);
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (GetState().regA001 & 0x80) {
                _exReg = value & 0x0F;
                UpdatePrgMapping();
            }
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

class BmcHpxx final : public Mmc3 {
    uint8_t _exRegs[5]{};
    bool _locked = false;

    uint32_t GetDipSwitchCount() override { return 4; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        std::memset(_exRegs, 0, sizeof(_exRegs));
        _locked = false;
        Mmc3::InitMapper();
        AddRegisterRange(0x5000, 0x5FFF, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    void Reset(bool) override {
        std::memset(_exRegs, 0, sizeof(_exRegs));
        _locked = false;
        ResetMmc3();
        UpdateState();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (_exRegs[0] & 0x04) {
            switch (_exRegs[0] & 3) {
                case 0: case 1: SelectChrPage8x(0, (_exRegs[2] & 0x3F) << 3); break;
                case 2: SelectChrPage8x(0, ((_exRegs[2] & 0x3E) | (_exRegs[4] & 1)) << 3); break;
                case 3: SelectChrPage8x(0, ((_exRegs[2] & 0x3C) | (_exRegs[4] & 3)) << 3); break;
            }
        } else {
            uint8_t base, mask;
            if (_exRegs[0] & 1) {
                base = _exRegs[2] & 0x30;
                mask = 0x7F;
            } else {
                base = _exRegs[2] & 0x20;
                mask = 0xFF;
            }
            Mmc3::SelectChrPage(slot, (page & mask) | (base << 3), type);
        }
    }

    void SelectPrgPage(uint16_t slot, uint16_t page,
                       PrgMemoryType type = PrgMemoryType::PrgRom) override {
        if (_exRegs[0] & 0x04) {
            if ((_exRegs[0] & 0x0F) == 0x04) {
                SelectPrgPage2x(0, (_exRegs[1] & 0x1F) << 1);
                SelectPrgPage2x(1, (_exRegs[1] & 0x1F) << 1);
            } else {
                SelectPrgPage4x(0, (_exRegs[1] & 0x1E) << 1);
            }
        } else {
            uint8_t base, mask;
            if (_exRegs[0] & 2) {
                base = _exRegs[1] & 0x18;
                mask = 0x0F;
            } else {
                base = _exRegs[1] & 0x10;
                mask = 0x1F;
            }
            Mmc3::SelectPrgPage(slot, (page & mask) | (base << 1), type);
        }
    }

    void UpdateMirroring() override {
        if (_exRegs[0] & 0x04)
            SetMirroringType(_exRegs[4] & 0x04 ? MirroringType::Vertical : MirroringType::Horizontal);
        else
            Mmc3::UpdateMirroring();
    }

    uint8_t ReadRegister(uint16_t) override { return static_cast<uint8_t>(GetDipSwitches()); }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) {
            if (!_locked) {
                _exRegs[address & 3] = value;
                _locked = (value & 0x80) != 0;
                UpdatePrgMapping();
                UpdateChrMapping();
            }
        } else if (_exRegs[0] & 0x04) {
            _exRegs[4] = value;
            UpdateChrMapping();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

class Mmc3_StreetHeroes final : public Mmc3 {
    uint8_t _exReg = 0;
    uint8_t _resetSwitch = 0;

    uint16_t GetChrRamPageSize() override { return 0x2000; }
    uint32_t GetChrRamSize() override { return 0x2000; }
    bool AllowRegisterRead() override { return true; }

    void InitMapper() override {
        _exReg = 0;
        _resetSwitch = 0;
        Mmc3::InitMapper();
        AddRegisterRange(0x4100, 0x4100, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
    }

    void Reset(bool softReset) override {
        if (softReset) _resetSwitch ^= 0xFF;
        UpdateState();
    }

    void SelectChrPage(uint16_t slot, uint16_t page,
                       ChrMemoryType type = ChrMemoryType::Default) override {
        if (_exReg & 0x40) {
            Mmc3::SelectChrPage(0, 0, ChrMemoryType::ChrRam);
            return;
        }
        switch (slot) {
            case 0: case 1: Mmc3::SelectChrPage(slot, page | ((_exReg & 0x08) << 5), type); break;
            case 2: case 3: Mmc3::SelectChrPage(slot, page | ((_exReg & 0x04) << 6), type); break;
            case 4: case 5: Mmc3::SelectChrPage(slot, page | ((_exReg & 0x01) << 8), type); break;
            default: Mmc3::SelectChrPage(slot, page | ((_exReg & 0x02) << 7), type); break;
        }
    }

    uint8_t ReadRegister(uint16_t) override { return _resetSwitch; }

    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x4100) {
            _exReg = value;
            UpdateState();
        } else {
            Mmc3::WriteRegister(address, value);
        }
    }
};

} // namespace cupid::boards
#endif
