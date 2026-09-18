/*
 * rainbow.hpp - Rainbow cartridge banking, interrupts, and extended video
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
#ifndef CUPID_BOARDS_RAINBOW_HPP
#define CUPID_BOARDS_RAINBOW_HPP
#include "runtime.hpp"
#include "rainbow_devices.hpp"

namespace cupid::boards {

class Rainbow final : public Board {
    struct NtControl {
        uint8_t packed = 0;
        bool Attribute() const { return (packed & 1) != 0; }
        bool Background() const { return (packed & 2) != 0; }
        unsigned ExtendedBank() const { return (packed >> 2) & 3; }
        bool Fill() const { return (packed & 0x20) != 0; }
        unsigned Source() const { return packed >> 6; }
    };

    RainbowFlash _prgFlash, _chrFlash;
    RainbowAudio _audio;
    std::array<uint16_t, 8> _highBanks{};
    std::array<uint16_t, 2> _lowBanks{};
    std::array<uint16_t, 16> _chrBanks{};
    uint8_t _highMode = 0, _lowMode = 0, _chrMode = 0, _chrSource = 0, _fpgaBank = 0;
    uint8_t _backgroundBank = 0, _fillTile = 0, _fillAttribute = 0;
    std::array<uint8_t, 4> _ntBanks{};
    std::array<NtControl, 4> _ntControl{};
    NtControl _windowControl;
    uint8_t _windowBank = 0, _windowX1 = 0, _windowX2 = 0, _windowY1 = 0, _windowY2 = 0;
    uint8_t _windowScrollX = 0, _windowScrollY = 0;
    bool _windowEnabled = false, _inWindow = false;
    bool _scanlineEnabled = false, _scanlinePending = false;
    uint8_t _scanlineTarget = 0, _scanlineOffset = 0;
    int16_t _scanline = 0;
    uint16_t _lastPpuAddress = 0;
    uint8_t _idleCounter = 0, _repeatReads = 0, _ppuReadCounter = 0, _ntFetchCounter = 0;
    bool _inFrame = false, _inHBlank = false;
    uint8_t _jitterCounter = 0;
    bool _parityCounter = false;
    uint16_t _cpuCounter = 0, _cpuReload = 0;
    bool _cpuEnabled = false, _cpuPending = false, _cpuEnableAfterAck = false, _cpuAckOn4011 = false;
    uint16_t _fpgaAddress = 0;
    uint8_t _fpgaIncrement = 0, _vectorControl = 0;
    uint16_t _nmiVector = 0, _irqVector = 0;
    bool _overrideTile = false;
    uint8_t _extendedData = 0;
    std::array<uint8_t, 64> _spriteData{}, _spriteY{};
    std::array<uint8_t, 8> _spriteMappings{};
    uint8_t _spriteBank = 0, _oamAddress = 0, _spriteLimit = 0, _oamSlowPage = 0, _oamExtPage = 0;
    std::array<uint8_t, 0x506> _oamCode{};
    bool _oamCodeLocked = false, _spriteExtended = false, _largeSprites = false;
    uint8_t _wifiControl = 0, _receivePage = 0, _sendPage = 0;

    uint16_t GetPrgPageSize() override { return 0x1000; }
    uint16_t GetChrPageSize() override { return 0x200; }
    uint32_t GetMapperRamSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x4100; }
    uint16_t RegisterEndAddress() override { return 0x4785; }
    bool AllowRegisterRead() override { return true; }
    bool EnableCpuClockHook() override { return true; }
    bool EnableCustomVramRead() override { return true; }

    void InitMapper() override {
        _prgFlash.Initialize(_prgRom, _prgSize);
        _chrFlash.Initialize(_chrRom, _chrRomSize);
        AddRegisterRange(0x6000, 0xFFFF);
        Reset(false);
    }
    void LoadBattery() override {
        Board::LoadBattery();
        ReadBattery(".flash.sav", _prgRom, _prgSize);
        ReadBattery(".chr.flash.sav", _chrRom, _chrRomSize);
    }
    void SaveBattery() override {
        Board::SaveBattery();
        WriteBattery(".flash.sav", _prgRom, _prgSize);
        WriteBattery(".chr.flash.sav", _chrRom, _chrRomSize);
    }
    PrgMemoryType RamType() const { return HasBattery() ? PrgMemoryType::SaveRam : PrgMemoryType::WorkRam; }
    void MapHigh(uint16_t start, uint16_t size, unsigned reg) {
        bool ram = (_highBanks[reg] & 0x8000) != 0;
        SetCpuMemoryMapping(start, start + size - 1, ram ? RamType() : PrgMemoryType::PrgRom,
                            (_highBanks[reg] & 0x7FFFu) * size, ram ? ReadWrite : Read);
    }
    void MapLow(uint16_t start, uint16_t size, unsigned reg) {
        unsigned source = _lowBanks[reg] >> 14;
        PrgMemoryType type = source < 2 ? PrgMemoryType::PrgRom
                            : source == 2 ? RamType() : PrgMemoryType::MapperRam;
        unsigned bank = _lowBanks[reg] & (source < 2 ? 0x7FFF : 0x3FFF);
        SetCpuMemoryMapping(start, start + size - 1, type, bank * size, source < 2 ? Read : ReadWrite);
    }
    void UpdateState() {
        switch (_highMode) {
            case 0: MapHigh(0x8000, 0x8000, 0); break;
            case 1: MapHigh(0x8000, 0x4000, 0); MapHigh(0xC000, 0x4000, 4); break;
            case 2:
                MapHigh(0x8000, 0x4000, 0);
                MapHigh(0xC000, 0x2000, 4);
                MapHigh(0xE000, 0x2000, 6);
                break;
            case 3:
                for (unsigned reg = 0; reg < 8; reg += 2) MapHigh(0x8000 + reg * 0x1000, 0x2000, reg);
                break;
            default:
                for (unsigned reg = 0; reg < 8; ++reg) MapHigh(0x8000 + reg * 0x1000, 0x1000, reg);
                break;
        }
        if (_lowMode) { MapLow(0x6000, 0x1000, 0); MapLow(0x7000, 0x1000, 1); }
        else MapLow(0x6000, 0x2000, 0);
        SetCpuMemoryMapping(0x5000, 0x5FFF, PrgMemoryType::MapperRam, _fpgaBank * 0x1000, ReadWrite);
        SetCpuMemoryMapping(0x4800, 0x4FFF, PrgMemoryType::MapperRam, 0x1800, ReadWrite);

        if (_chrSource == 3) {
            for (unsigned start = 0; start < 0x2000; start += 0x800)
                SetPpuMemoryMapping(start, start + 0x7FF, ChrMemoryType::NametableRam, 0, ReadWrite);
        } else if (_chrSource == 2) {
            SetPpuMemoryMapping(0, 0x0FFF, ChrMemoryType::MapperRam, 0, ReadWrite);
            SetPpuMemoryMapping(0x1000, 0x1FFF, ChrMemoryType::MapperRam, 0, ReadWrite);
        } else {
            // The three 1xx encodings all select the sixteen 512-byte banks.
            unsigned mode = std::min<unsigned>(_chrMode, 4);
            unsigned count = 1u << mode, size = 0x2000u >> mode;
            for (unsigned reg = 0; reg < count; ++reg)
                SetPpuMemoryMapping(reg * size, (reg + 1) * size - 1,
                                    _chrSource ? ChrMemoryType::ChrRam : ChrMemoryType::ChrRom,
                                    _chrBanks[reg] * size, _chrSource ? ReadWrite : Read);
        }
        static constexpr ChrMemoryType sources[] = {ChrMemoryType::NametableRam, ChrMemoryType::ChrRam,
                                                    ChrMemoryType::MapperRam, ChrMemoryType::ChrRom};
        for (unsigned quadrant = 0; quadrant < 4; ++quadrant) {
            unsigned source = _ntControl[quadrant].Source();
            unsigned bank = source == 2 ? _ntBanks[quadrant] & 3 : _ntBanks[quadrant];
            for (unsigned mirror = 0x2000; mirror <= 0x3000; mirror += 0x1000) {
                uint16_t start = mirror + quadrant * 0x400;
                SetPpuMemoryMapping(start, start + 0x3FF, sources[source], bank * 0x400,
                                    source == 3 ? Read : ReadWrite);
            }
        }
    }
    void UpdateIrq() {
        bool active = (_cpuEnabled && _cpuPending) || (_scanlineEnabled && _scanlinePending);
        if (active && !IrqPending()) _jitterCounter = 0;
        SetIrq(active);
    }
    void AckCpuIrq() {
        _cpuEnabled = _cpuEnableAfterAck;
        _cpuPending = false;
        UpdateIrq();
    }
    void GenerateOam(bool extended) {
        if (_oamCodeLocked) return;
        _oamCodeLocked = true;
        unsigned position = extended ? 2 : 0;
        unsigned last = extended ? _spriteLimit : (_spriteLimit << 2) | 3;
        unsigned page = extended ? _oamExtPage : _oamSlowPage;
        for (unsigned index = 0; index <= last; ++index) {
            _oamCode[position++] = 0xA9;
            _oamCode[position++] = _mapperRam[0x1800 + page * 0x100 + (extended ? index * 4 : index)];
            _oamCode[position++] = 0x8D;
            _oamCode[position++] = extended ? index : 4;
            _oamCode[position++] = extended ? 0x42 : 0x20;
        }
        _oamCode[position] = 0x60;
    }
    void EvaluateSprites() {
        _spriteMappings.fill(0);
        unsigned count = 0;
        for (unsigned sprite = 0; sprite < 64; ++sprite) {
            if (_scanline >= _spriteY[sprite] && _scanline < _spriteY[sprite] + (_largeSprites ? 16 : 8)) {
                _spriteMappings[count++] = sprite;
                if (count == 8) break;
            }
        }
    }
    void DetectScanline(uint16_t address) {
        if (address >= 0x2000 && address <= 0x2FFF && address == _lastPpuAddress) {
            if (++_repeatReads >= 2) {
                if (_inFrame) ++_scanline;
                else { _inFrame = true; _scanline = 0; }
                EvaluateSprites();
                _ntFetchCounter = _ppuReadCounter = _repeatReads = 0;
                _inHBlank = false;
            }
        } else _repeatReads = 0;
    }
    uint8_t WindowScanline() const {
        uint8_t scanline = static_cast<uint8_t>(_scanline + (_ntFetchCounter >= 49 ? 1 : 0));
        return (scanline + _windowScrollY) % 240;
    }
    unsigned WindowColumn() const { return (_ntFetchCounter + 1u) % 50; }
    void UpdateWindow() {
        uint8_t scanline = static_cast<uint8_t>(_scanline + (_ntFetchCounter >= 49 ? 1 : 0));
        unsigned column = WindowColumn();
        bool y = _windowY1 >= _windowY2 ? scanline <= _windowY2 || scanline > _windowY1
                                      : scanline >= _windowY1 && scanline <= _windowY2;
        bool x = _windowX1 >= _windowX2 ? column <= _windowX2 || column > _windowX1
                                      : column >= _windowX1 && column <= _windowX2;
        _inWindow = x && y;
    }
    uint8_t ReadChr(uint32_t offset, uint16_t ppuAddress) {
        switch (_chrSource) {
            case 0: return _chrRomSize ? _chrRom[offset & (_chrRomSize - 1)] : 0;
            case 1: return _chrRamSize ? _chrRam[offset & (_chrRamSize - 1)] : 0;
            case 2: return _mapperRam[offset & 0x1FFF];
            default: return GetNametable((ppuAddress & 0x400) >> 10)[ppuAddress & 0x3FF];
        }
    }
    uint8_t ReadRegister(uint16_t address) override;
    void WriteRegister(uint16_t address, uint8_t value) override;

public:
    void Reset(bool) override {
        static constexpr uint16_t registers[] = {
            0x4100, 0x4108, 0x4118, 0x4120, 0x4130, 0x4140, 0x4126, 0x4127, 0x4128, 0x4129,
            0x412E, 0x412A, 0x412B, 0x412C, 0x412D, 0x412F, 0x4241, 0x4242, 0x4152, 0x4153,
            0x415A, 0x416B, 0x4190, 0x41A9, 0x41AA
        };
        static constexpr uint8_t values[] = {
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0x80, 7, 6, 0, 0x87, 0, 0, 0, 0, 15
        };
        for (unsigned reg = 0; reg < sizeof(values); ++reg) WriteRegister(registers[reg], values[reg]);
    }
    void ProcessCpuClock() override {
        ++_jitterCounter;
        _parityCounter = !_parityCounter;
        _audio.Clock();
        if (_cpuEnabled && _cpuCounter && --_cpuCounter == 0) {
            _cpuCounter = _cpuReload;
            _cpuPending = true;
            UpdateIrq();
        }
        if (_idleCounter && --_idleCounter == 0) {
            _inFrame = _inHBlank = false;
            _scanline = -1;
            _ntFetchCounter = _repeatReads = _oamAddress = 0;
        }
    }
    bool ReadCpuRegister(uint16_t address, uint8_t &value) override {
        if (address != 0x4011) return false;
        if (_cpuAckOn4011) AckCpuIrq();
        value = _audio.LastOutput() << 1;
        return true;
    }
    void ObserveCpuWrite(uint16_t address, uint8_t value) override {
        if (address == 0x2000) _largeSprites = (value & 0x20) != 0;
        else if (address == 0x2003) _oamAddress = value;
        else if (address == 0x2004) {
            if (!(_oamAddress & 3)) _spriteY[_oamAddress >> 2] = value;
            ++_oamAddress;
        }
    }
    uint8_t MapperReadVram(uint16_t address, MemoryOperationType) override;
    void MapperWriteVram(uint16_t address, uint8_t value) override {
        int64_t offset = address < 0x2000 ? ChrRomOffset(address) : -1;
        if (offset >= 0) _chrFlash.Write(static_cast<uint32_t>(offset), value);
        else InternalWriteVram(address, value);
    }
    float AudioOutput() const override { return _audio.Output(); }
};

inline uint8_t Rainbow::ReadRegister(uint16_t address) {
    switch (address) {
        case 0x4100: return _highMode | (_lowMode << 7);
        case 0x4120: return _chrMode | (_windowEnabled ? 0x10 : 0) | (_spriteExtended ? 0x20 : 0) | (_chrSource << 6);
        case 0x412A: case 0x412B: case 0x412C: case 0x412D: return _ntControl[address - 0x412A].packed;
        case 0x412F: return _windowControl.packed;
        case 0x4150: return static_cast<uint8_t>(_scanline);
        case 0x4151:
            _scanlinePending = false;
            UpdateIrq();
            return (_inFrame ? 0x40 : 0) | (_inHBlank ? 0x80 : 0);
        case 0x4154: return _jitterCounter;
        case 0x4157: return _parityCounter ? 0x80 : 0;
        case 0x415F: {
            uint8_t value = _mapperRam[_fpgaAddress];
            _fpgaAddress = (_fpgaAddress + _fpgaIncrement) & 0x1FFF;
            return value;
        }
        case 0x4160: return 0x21;
        case 0x4161: return (_cpuPending ? 0x40 : 0) | (_scanlinePending ? 0x80 : 0);
        case 0x4280: GenerateOam(false); break;
        case 0x4282: GenerateOam(true); break;
        case 0x4190: return _wifiControl;
        case 0x4191: case 0x4192: return 0;
        case 0xFFFA: case 0xFFFB:
            _inFrame = false;
            _lastPpuAddress = 0;
            _scanline = 0;
            _scanlinePending = false;
            UpdateIrq();
            return (_vectorControl & 1) ? static_cast<uint8_t>(_nmiVector >> ((address & 1) * 8))
                                        : InternalReadRam(address);
        case 0xFFFE: case 0xFFFF:
            return (_vectorControl & 2) ? static_cast<uint8_t>(_irqVector >> ((address & 1) * 8))
                                        : InternalReadRam(address);
    }
    if (address >= 0x4280 && address < 0x4800) {
        if (address >= 0x4282) _oamCodeLocked = false;
        if (address - 0x4280 < static_cast<int>(_oamCode.size())) return _oamCode[address - 0x4280];
    }
    if (address >= 0x6000) {
        int64_t offset = _prgFlash.Identifying() ? PrgRomOffset(address) : -1;
        return offset >= 0 ? _prgFlash.Read(static_cast<uint32_t>(offset)) : InternalReadRam(address);
    }
    return GetOpenBus();
}

inline void Rainbow::WriteRegister(uint16_t address, uint8_t value) {
    switch (address) {
        case 0x4100: _highMode = value & 7; _lowMode = value >> 7; UpdateState(); break;
        case 0x4115: _fpgaBank = value & 1; UpdateState(); break;
        case 0x4120:
            _chrMode = value & 7;
            _windowEnabled = (value & 0x10) != 0;
            _spriteExtended = (value & 0x20) != 0;
            _chrSource = value >> 6;
            UpdateState();
            break;
        case 0x4121: _backgroundBank = value & 0x1F; break;
        case 0x4124: _fillTile = value; break;
        case 0x4125: _fillAttribute = value & 3; break;
        case 0x412E: _windowBank = value; break;
        case 0x412F:
            _windowControl.packed = (value & 0xEC) | ((value & 1) << 1) | ((value & 2) >> 1);
            break;
        case 0x4150: _scanlineTarget = value; break;
        case 0x4151: _scanlineEnabled = true; break;
        case 0x4152: _scanlineEnabled = _scanlinePending = false; UpdateIrq(); break;
        case 0x4153: _scanlineOffset = std::clamp<uint8_t>(value, 1, 170); break;
        case 0x4157: _parityCounter = true; break;
        case 0x4158: _cpuReload = (_cpuReload & 0xFF) | (value << 8); break;
        case 0x4159: _cpuReload = (_cpuReload & 0xFF00) | value; break;
        case 0x415A:
            _cpuEnabled = (value & 1) != 0;
            _cpuEnableAfterAck = (value & 2) != 0;
            _cpuAckOn4011 = (value & 4) != 0;
            _cpuPending = false;
            if (_cpuEnabled) _cpuCounter = _cpuReload;
            UpdateIrq();
            break;
        case 0x415B: AckCpuIrq(); break;
        case 0x415C: _fpgaAddress = (_fpgaAddress & 0xFF) | ((value & 0x1F) << 8); break;
        case 0x415D: _fpgaAddress = (_fpgaAddress & 0x1F00) | value; break;
        case 0x415E: _fpgaIncrement = value; break;
        case 0x415F:
            _mapperRam[_fpgaAddress] = value;
            _fpgaAddress = (_fpgaAddress + _fpgaIncrement) & 0x1FFF;
            break;
        case 0x416B: _vectorControl = value & 3; break;
        case 0x416C: _nmiVector = (_nmiVector & 0xFF) | (value << 8); break;
        case 0x416D: _nmiVector = (_nmiVector & 0xFF00) | value; break;
        case 0x416E: _irqVector = (_irqVector & 0xFF) | (value << 8); break;
        case 0x416F: _irqVector = (_irqVector & 0xFF00) | value; break;
        case 0x4170: _windowX1 = value & 0x1F; break;
        case 0x4171: _windowX2 = value & 0x1F; break;
        case 0x4172: _windowY1 = value; break;
        case 0x4173: _windowY2 = value; break;
        case 0x4174: _windowScrollX = value & 0x1F; break;
        case 0x4175: _windowScrollY = value; break;
        case 0x4190: _wifiControl = value & 3; break;
        case 0x4193: _receivePage = value & 7; break;
        case 0x4194: _sendPage = value & 7; break;
        case 0x4240: _spriteBank = value & 7; break;
        case 0x4241: _oamSlowPage = value & 7; break;
        case 0x4242: _oamExtPage = value & 7; break;
        case 0x4243: _spriteLimit = value & 0x3F; break;
    }
    if (address >= 0x4106 && address <= 0x4107) {
        uint16_t &bank = _lowBanks[address - 0x4106];
        bank = (bank & 0xFF) | (value << 8);
        UpdateState();
    } else if (address >= 0x4116 && address <= 0x4117) {
        uint16_t &bank = _lowBanks[address - 0x4116];
        bank = (bank & 0xFF00) | value;
        UpdateState();
    } else if (address >= 0x4108 && address <= 0x410F) {
        uint16_t &bank = _highBanks[address - 0x4108];
        bank = (bank & 0xFF) | (value << 8);
        UpdateState();
    } else if (address >= 0x4118 && address <= 0x411F) {
        uint16_t &bank = _highBanks[address - 0x4118];
        bank = (bank & 0xFF00) | value;
        UpdateState();
    } else if (address >= 0x4130 && address <= 0x413F) {
        uint16_t &bank = _chrBanks[address - 0x4130];
        bank = (bank & 0xFF) | (value << 8);
        UpdateState();
    } else if (address >= 0x4140 && address <= 0x414F) {
        uint16_t &bank = _chrBanks[address - 0x4140];
        bank = (bank & 0xFF00) | value;
        UpdateState();
    } else if (address >= 0x4126 && address <= 0x4129) {
        _ntBanks[address - 0x4126] = value;
        UpdateState();
    } else if (address >= 0x412A && address <= 0x412D) {
        _ntControl[address - 0x412A].packed = value & 0xEF;
        UpdateState();
    } else if (address >= 0x41A0 && address <= 0x41AA) _audio.Write(address, value);
    else if (address >= 0x4200 && address <= 0x423F) _spriteData[address - 0x4200] = value;

    if (address >= 0x6000) {
        int64_t offset = PrgRomOffset(address);
        if (offset >= 0) _prgFlash.Write(static_cast<uint32_t>(offset), value);
        else WritePrgRam(address, value);
    }
}

inline uint8_t Rainbow::MapperReadVram(uint16_t address, MemoryOperationType) {
    int64_t offset = _chrFlash.Identifying() ? ChrRomOffset(address) : -1;
    if (offset >= 0) return _chrFlash.Read(static_cast<uint32_t>(offset));
    ++_ppuReadCounter;
    DetectScanline(address);
    if (_scanlineTarget == _scanline && _scanlineOffset == _ppuReadCounter) {
        _scanlinePending = true;
        UpdateIrq();
    }
    _idleCounter = 3;
    _lastPpuAddress = address;
    if (!_inFrame) return InternalReadVram(address);

    if (address >= 0x2000 && address <= 0x2FFF) {
        bool attribute = (address & 0x3FF) >= 0x3C0;
        if (!attribute) {
            ++_ntFetchCounter;
            if (_ntFetchCounter == 33) _inHBlank = true;
            _inWindow = false;
            if (_windowEnabled) UpdateWindow();
        }
        const NtControl &control = _inWindow ? _windowControl : _ntControl[(address >> 10) & 3];
        unsigned shift = 0;
        if (_inWindow) {
            unsigned line = WindowScanline(), column = (WindowColumn() + _windowScrollX) & 31;
            uint16_t ntAddress = (line / 8) * 32 + column;
            if (!attribute) address = ntAddress;
            else {
                address = 0x3C0 + (((line >> 2) & 0xF8) | (column >> 2));
                shift = ((ntAddress >> 4) & 4) | (ntAddress & 2);
            }
        }
        if (!attribute) {
            _overrideTile = control.Background();
            if (control.Attribute() || control.Background())
                _extendedData = _mapperRam[control.ExtendedBank() * 0x400 + (address & 0x3FF)];
            if (control.Fill()) return _fillTile;
        } else {
            if (control.Fill()) return _fillAttribute * 0x55;
            if (control.Attribute()) return (_extendedData >> 6) * 0x55;
        }
        if (_inWindow) {
            uint8_t value = _mapperRam[(_windowBank * 0x400 + address) & 0x1FFF];
            return attribute ? ((value >> shift) & 3) * 0x55 : value;
        }
    } else {
        bool background = _ntFetchCounter < 33 || _ntFetchCounter >= 49;
        if (_inWindow && background) {
            unsigned fineY = WindowScanline() & 7;
            if (_overrideTile)
                return ReadChr((address & 0xFF8) | fineY | ((_extendedData & 0x3F) << 12)
                               | (_backgroundBank << 18), address);
            return InternalReadVram((address & 0x1FF8) | fineY);
        }
        if (_overrideTile && background)
            return ReadChr((address & 0xFFF) | ((_extendedData & 0x3F) << 12)
                           | (_backgroundBank << 18), address);
        if (_spriteExtended && !background) {
            uint8_t sprite = _spriteMappings[(_ntFetchCounter - 33) >> 1];
            uint32_t absolute = _largeSprites
                ? (_spriteBank << 21) | (_spriteData[sprite] << 13) | (address & 0x1FFF)
                : (_spriteBank << 20) | (_spriteData[sprite] << 12) | (address & 0xFFF);
            return ReadChr(absolute, address);
        }
    }
    return InternalReadVram(address);
}

} // namespace cupid::boards
#endif
