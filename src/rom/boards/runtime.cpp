/*
 * runtime.cpp - Cartridge page mapping and bus integration
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
#include "runtime.hpp"
#include <cstdio>
#include <limits>
#include <stdexcept>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif
extern "C" {
#include "../mapper.h"
#include "../../cpu/cpu.h"
#include "../../system/hardware.h"
}

namespace cupid::boards {

bool Board::AlignedRange(uint16_t first, uint16_t last) {
    return !(first & 0xFF) && (last & 0xFF) == 0xFF && last > first;
}

void Board::MapCpuBytes(uint16_t first, uint16_t last, uint8_t *bytes,
                        uint32_t offset, uint32_t size, int8_t access) {
    if (!AlignedRange(first, last)) return;
    for (unsigned slot = first >> 8; slot <= (last >> 8); ++slot) {
        bool valid = access != 0 && bytes && offset <= size && size - offset >= 0x100;
        _cpuPages[slot] = valid ? Page{bytes + offset, static_cast<uint8_t>(access < 0 ? static_cast<int>(ReadWrite) : access)}
                                : Page{};
        if (offset <= UINT32_MAX - 0x100) offset += 0x100;
        else bytes = nullptr;
    }
}

void Board::MapPpuBytes(uint16_t first, uint16_t last, uint8_t *bytes,
                        uint32_t offset, uint32_t size, int8_t access) {
    if (!AlignedRange(first, last) || last > 0x3FFF) return;
    for (unsigned slot = first >> 8; slot <= (last >> 8); ++slot) {
        bool valid = access != 0 && bytes && offset <= size && size - offset >= 0x100;
        _ppuPages[slot] = valid ? Page{bytes + offset, static_cast<uint8_t>(access < 0 ? static_cast<int>(ReadWrite) : access)}
                                : Page{};
        if (offset <= UINT32_MAX - 0x100) offset += 0x100;
        else bytes = nullptr;
    }
}

void Board::SetCpuMemoryMapping(uint16_t first, uint16_t last, uint8_t *bytes,
                                uint32_t offset, uint32_t size, int8_t access) {
    MapCpuBytes(first, last, bytes, offset, size, access);
}

void Board::SetCpuMemoryMapping(uint16_t first, uint16_t last, PrgMemoryType type,
                                uint32_t offset, int8_t access) {
    uint8_t *bytes = nullptr;
    uint32_t size = 0;
    switch (type) {
        case PrgMemoryType::PrgRom: bytes = _prgRom; size = _prgSize; break;
        case PrgMemoryType::SaveRam: bytes = _saveRam; size = _saveRamSize; break;
        case PrgMemoryType::WorkRam: bytes = _workRam; size = _workRamSize; break;
        case PrgMemoryType::MapperRam: bytes = _mapperRam; size = _mapperRamSize; break;
    }
    if (size) offset %= size;
    MapCpuBytes(first, last, bytes, offset, size, access);
}

void Board::SetCpuMemoryMapping(uint16_t first, uint16_t last, int16_t page,
                                PrgMemoryType type, int8_t access) {
    if (!AlignedRange(first, last)) return;
    uint32_t pageSize = 0, size = 0;
    switch (type) {
        case PrgMemoryType::PrgRom: pageSize = _prgPageSize; size = _prgSize; break;
        case PrgMemoryType::SaveRam: pageSize = _savePageSize; size = _saveRamSize; break;
        case PrgMemoryType::WorkRam: pageSize = _workPageSize; size = _workRamSize; break;
        case PrgMemoryType::MapperRam: return;
    }
    if (!pageSize || size < pageSize) return;
    int32_t pageCount = static_cast<int32_t>(size / pageSize);
    int32_t selected = page < 0 ? pageCount + page : page % pageCount;
    if (access < 0) access = type == PrgMemoryType::PrgRom ? Read : ReadWrite;
    uint32_t length = static_cast<uint32_t>(last) - first + 1;
    uint32_t chunk = std::min(length, pageSize);
    for (uint32_t addr = first; addr + chunk - 1 <= last; addr += chunk) {
        uint32_t offset = static_cast<uint32_t>(selected) * pageSize;
        SetCpuMemoryMapping(static_cast<uint16_t>(addr), static_cast<uint16_t>(addr + chunk - 1),
                            type, offset, access);
        selected = (selected + 1) % pageCount;
    }
}

void Board::SetPpuMemoryMapping(uint16_t first, uint16_t last, uint8_t *bytes,
                                uint32_t offset, uint32_t size, int8_t access) {
    MapPpuBytes(first, last, bytes, offset, size, access);
}

void Board::SetPpuMemoryMapping(uint16_t first, uint16_t last, ChrMemoryType type,
                                uint32_t offset, int8_t access) {
    if (type == ChrMemoryType::Default)
        type = _chrRomSize ? ChrMemoryType::ChrRom : ChrMemoryType::ChrRam;
    uint8_t *bytes = nullptr;
    uint32_t size = 0;
    switch (type) {
        case ChrMemoryType::ChrRom: bytes = _chrRom; size = _chrRomSize; break;
        case ChrMemoryType::ChrRam: bytes = _chrRam; size = _chrRamSize; break;
        case ChrMemoryType::NametableRam:
            bytes = _nametableStorage.data(); size = static_cast<uint32_t>(_nametableStorage.size()); break;
        case ChrMemoryType::MapperRam: bytes = _mapperRam; size = _mapperRamSize; break;
        case ChrMemoryType::Default: break;
    }
    if (size) offset %= size;
    MapPpuBytes(first, last, bytes, offset, size, access);
}

void Board::SetPpuMemoryMapping(uint16_t first, uint16_t last, uint16_t page,
                                ChrMemoryType type, int8_t access) {
    if (!AlignedRange(first, last) || last > 0x3FFF) return;
    if (type == ChrMemoryType::Default)
        type = _chrRomSize ? ChrMemoryType::ChrRom : ChrMemoryType::ChrRam;
    uint32_t pageSize = 0, size = 0;
    switch (type) {
        case ChrMemoryType::ChrRom: pageSize = _chrRomPageSize; size = _chrRomSize; break;
        case ChrMemoryType::ChrRam: pageSize = _chrRamPageSize; size = _chrRamSize; break;
        case ChrMemoryType::NametableRam: pageSize = 0x400; size = _nametableCount * 0x400; break;
        case ChrMemoryType::MapperRam: case ChrMemoryType::Default: return;
    }
    if (!pageSize || size < pageSize) return;
    uint32_t pageCount = size / pageSize;
    uint32_t selected = page % pageCount;
    if (access < 0) access = type == ChrMemoryType::ChrRom ? Read : ReadWrite;
    uint32_t length = static_cast<uint32_t>(last) - first + 1;
    uint32_t chunk = std::min(length, pageSize);
    for (uint32_t addr = first; addr + chunk - 1 <= last; addr += chunk) {
        SetPpuMemoryMapping(static_cast<uint16_t>(addr), static_cast<uint16_t>(addr + chunk - 1),
                            type, selected * pageSize, access);
        selected = (selected + 1) % pageCount;
    }
}

void Board::RemoveCpuMemoryMapping(uint16_t first, uint16_t last) {
    MapCpuBytes(first, last, nullptr, 0, 0, NoAccess);
}

void Board::RemovePpuMemoryMapping(uint16_t first, uint16_t last) {
    MapPpuBytes(first, last, nullptr, 0, 0, NoAccess);
}

void Board::SelectPrgPage(uint16_t slot, uint16_t page, PrgMemoryType type) {
    if (!_prgSize || !_prgPageSize) return;
    if (_prgSize < 0x8000 && GetPrgPageSize() > _prgSize) {
        for (uint32_t start = 0x8000; start + _prgSize <= 0x10000; start += _prgSize)
            SetCpuMemoryMapping(static_cast<uint16_t>(start),
                                static_cast<uint16_t>(start + _prgSize - 1), 0, type);
    } else {
        uint32_t start = 0x8000 + static_cast<uint32_t>(slot) * _prgPageSize;
        if (start + _prgPageSize <= 0x10000)
            SetCpuMemoryMapping(static_cast<uint16_t>(start),
                                static_cast<uint16_t>(start + _prgPageSize - 1),
                                static_cast<int16_t>(page), type);
    }
}

void Board::SelectPrgPage2x(uint16_t slot, uint16_t page, PrgMemoryType type) {
    Board::SelectPrgPage(slot * 2, page, type);
    Board::SelectPrgPage(slot * 2 + 1, page + 1, type);
}

void Board::SelectPrgPage4x(uint16_t slot, uint16_t page, PrgMemoryType type) {
    Board::SelectPrgPage2x(slot * 2, page, type);
    Board::SelectPrgPage2x(slot * 2 + 1, page + 2, type);
}

void Board::SelectChrPage(uint16_t slot, uint16_t page, ChrMemoryType type) {
    if (type == ChrMemoryType::Default)
        type = _chrRomSize ? ChrMemoryType::ChrRom : ChrMemoryType::ChrRam;
    uint32_t pageSize = type == ChrMemoryType::NametableRam ? 0x400
                      : type == ChrMemoryType::ChrRam ? _chrRamPageSize : _chrRomPageSize;
    uint32_t start = static_cast<uint32_t>(slot) * pageSize;
    if (pageSize && start + pageSize <= 0x4000)
        SetPpuMemoryMapping(static_cast<uint16_t>(start),
                            static_cast<uint16_t>(start + pageSize - 1), page, type);
}

void Board::SelectChrPage2x(uint16_t slot, uint16_t page, ChrMemoryType type) {
    Board::SelectChrPage(slot * 2, page, type);
    Board::SelectChrPage(slot * 2 + 1, page + 1, type);
}

void Board::SelectChrPage4x(uint16_t slot, uint16_t page, ChrMemoryType type) {
    Board::SelectChrPage2x(slot * 2, page, type);
    Board::SelectChrPage2x(slot * 2 + 1, page + 2, type);
}

void Board::SelectChrPage8x(uint16_t slot, uint16_t page, ChrMemoryType type) {
    Board::SelectChrPage4x(slot, page, type);
    Board::SelectChrPage4x(slot * 2 + 1, page + 4, type);
}

uint32_t Board::GetPrgPageCount() const { return _prgPageSize ? _prgSize / _prgPageSize : 0; }
uint32_t Board::GetChrRomPageCount() const { return _chrRomPageSize ? _chrRomSize / _chrRomPageSize : 0; }
uint64_t Board::CpuClock() const { return cpu_get_bus_cycle(); }

uint32_t Board::GetDipSwitches() {
    unsigned count = GetDipSwitchCount();
    uint32_t mask = count >= 32 ? UINT32_MAX : (uint32_t{1} << count) - 1;
    return cart_dip_switches() & mask;
}

void Board::AddRegisterRange(uint16_t first, uint16_t last, MemoryOperation operation) {
    for (uint32_t addr = first; addr <= last; ++addr)
        _registerAccess[addr] |= static_cast<uint8_t>(operation);
}

void Board::RemoveRegisterRange(uint16_t first, uint16_t last, MemoryOperation operation) {
    for (uint32_t addr = first; addr <= last; ++addr)
        _registerAccess[addr] &= ~static_cast<uint8_t>(operation);
}

uint8_t Board::InternalReadRam(uint16_t addr) const {
    const Page &page = _cpuPages[addr >> 8];
    return page.data ? page.data[addr & 0xFF] : 0;
}

uint8_t Board::InternalRead(uint16_t addr) {
    if (_registerReads && (_registerAccess[addr] & Read)) return ReadRegister(addr);
    const Page &page = _cpuPages[addr >> 8];
    return page.access & Read ? page.data[addr & 0xFF] : _openBus;
}

void Board::WritePrgRam(uint16_t addr, uint8_t value) {
    Page &page = _cpuPages[addr >> 8];
    if (page.access & Write) page.data[addr & 0xFF] = value;
}

void Board::WriteRam(uint16_t addr, uint8_t value) {
    if (_registerAccess[addr] & Write) {
        if (_busConflicts) {
            const Page &page = _cpuPages[addr >> 8];
            if (page.access & Read) value &= page.data[addr & 0xFF];
        }
        WriteRegister(addr, value);
    } else {
        WritePrgRam(addr, value);
    }
}

uint8_t Board::InternalReadVram(uint16_t addr) const {
    addr &= 0x3FFF;
    const Page &page = _ppuPages[addr >> 8];
    return page.access & Read ? page.data[addr & 0xFF] : static_cast<uint8_t>(addr);
}

void Board::InternalWriteVram(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF;
    Page &page = _ppuPages[addr >> 8];
    if (page.access & Write) page.data[addr & 0xFF] = value;
}

uint8_t Board::ReadCpu(uint16_t addr, uint8_t openBus) {
    _openBus = openBus;
    return _customCpuRead || addr < 0x6000 ? ReadRam(addr) : InternalRead(addr);
}

void Board::WriteCpu(uint16_t addr, uint8_t value) {
    _openBus = value;
    WriteRam(addr, value);
}

uint8_t Board::ReadPpu(uint16_t addr, unsigned fetchSource) {
    MemoryOperationType type = fetchSource ? MemoryOperationType::PpuRenderingRead : MemoryOperationType::Read;
    return _customRead ? MapperReadVram(addr & 0x3FFF, type) : InternalReadVram(addr);
}

void Board::ClockCpu(bool writeCycle) {
    _writeCycle = writeCycle;
    if (_clockHook) ProcessCpuClock();
}

void Board::NotifyPpu(uint16_t addr, uint64_t cycle) {
    _ppuClock = cycle;
    if (_addressHook) NotifyVramAddressChange(addr & 0x3FFF);
}

uint8_t *Board::GetNametable(uint8_t index) {
    if (index >= _nametableCount) index = 0;
    return _nametableStorage.data() + static_cast<uint32_t>(index) * 0x400;
}

void Board::SetNametable(uint8_t index, uint8_t page) {
    if (index >= 4 || page >= _nametableCount) return;
    SetPpuMemoryMapping(0x2000 + index * 0x400, 0x23FF + index * 0x400, page, ChrMemoryType::NametableRam);
    SetPpuMemoryMapping(0x3000 + index * 0x400, 0x33FF + index * 0x400, page, ChrMemoryType::NametableRam);
}

void Board::SetNametables(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    SetNametable(0, a);
    SetNametable(1, b);
    SetNametable(2, c);
    SetNametable(3, d);
}

void Board::SetMirroringType(MirroringType type) {
    _mirroring = type;
    switch (type) {
        case MirroringType::Horizontal: SetNametables(0, 0, 1, 1); break;
        case MirroringType::Vertical: SetNametables(0, 1, 0, 1); break;
        case MirroringType::ScreenAOnly: SetNametables(0, 0, 0, 0); break;
        case MirroringType::ScreenBOnly: SetNametables(1, 1, 1, 1); break;
        case MirroringType::FourScreens: SetNametables(0, 1, 2, 3); break;
    }
}

void Board::SetupDefaultWorkRam() {
    if (HasBattery() && _saveRamSize)
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::SaveRam);
    else if (_workRamSize)
        SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::WorkRam);
}

void Board::UpdatePageSizes() {
    _prgPageSize = std::min<uint32_t>(GetPrgPageSize(), _prgSize);
    _chrRomPageSize = std::min<uint32_t>(GetChrPageSize(), _chrRomSize);
    _chrRamPageSize = std::min<uint32_t>(GetChrRamPageSize(), _chrRamSize);
    _workPageSize = std::min(GetWorkRamPageSize(), _workRamSize);
    _savePageSize = std::min(GetSaveRamPageSize(), _saveRamSize);
}

void Board::Initialize(const iNESHeader &header, uint8_t *prg, size_t prgBytes,
                        uint8_t *chr, size_t chrBytes) {
    if (!prg || !prgBytes || prgBytes > UINT32_MAX || chrBytes > UINT32_MAX)
        throw std::invalid_argument("unsupported cartridge buffer size");
    _romInfo.MapperID = static_cast<unsigned>(rom_mapper_number(&header));
    _romInfo.IsNes20Header = (header.flags7 & 0x0C) == 8;
    _romInfo.SubMapperID = IsNes20() ? header.prg_ram_size >> 4 : 0;
    _romInfo.HasBattery = (header.flags6 & 2) != 0;
    _romInfo.Header.Byte6 = header.flags6;
    _prgRom = prg;
    _prgSize = static_cast<uint32_t>(prgBytes);
    bool chrRom = header.chr_rom_chunks || (IsNes20() && (header.flags9 & 0xF0));
    if (chrRom && (!chr || !chrBytes)) throw std::invalid_argument("missing CHR ROM");
    _chrRom = chrRom ? chr : nullptr;
    _chrRomSize = chrRom ? static_cast<uint32_t>(chrBytes) : 0;
    RomRamSizes ram{};
    rom_ram_sizes(&header, &ram);
    _saveRamSize = !IsNes20() || ForceSaveRamSize()
                 ? (HasBattery() || ForceSaveRamSize() ? GetSaveRamSize() : 0)
                 : static_cast<uint32_t>(ram.prg_nvram);
    _workRamSize = !IsNes20() || ForceWorkRamSize()
                 ? (!HasBattery() || ForceWorkRamSize() ? GetWorkRamSize() : 0)
                 : static_cast<uint32_t>(ram.prg_ram);
    _saveChrRamSize = IsNes20() ? static_cast<uint32_t>(ram.chr_nvram) : 0;
    _chrRamSize = IsNes20() ? static_cast<uint32_t>(ram.chr_ram + ram.chr_nvram)
                : GetChrRamSize() ? GetChrRamSize() : !chrRom ? 0x2000 : 0;
    if (!_saveChrRamSize && ForceChrBattery()) _saveChrRamSize = _chrRamSize;
    _mapperRamSize = GetMapperRamSize();
    _nametableCount = GetNametableCount();
    if (!_nametableCount) _nametableCount = header.flags6 & 8 ? 4 : 2;
    if (_nametableCount > 256) throw std::invalid_argument("unsupported nametable count");
    _workStorage.resize(_workRamSize);
    _saveStorage.resize(_saveRamSize);
    _chrStorage.resize(_chrRamSize);
    _mapperStorage.resize(_mapperRamSize);
    _nametableStorage.resize(_nametableCount * 0x400);
    _workRam = _workStorage.data();
    _saveRam = _saveStorage.data();
    _chrRam = _chrStorage.data();
    _mapperRam = _mapperStorage.data();
    nes_initialize_power_on_ram(_saveRam, _saveRamSize, 0);
    nes_initialize_power_on_ram(_workRam, _workRamSize, 0);
    nes_initialize_power_on_ram(_mapperRam, _mapperRamSize, 0);
    nes_initialize_power_on_ram(_nametableStorage.data(), _nametableStorage.size(), 0);
    nes_initialize_power_on_ram(_chrRam, _chrRamSize, 0);
    _romInfo.HasChrRam = HasChrRam();
    _registerReads = AllowRegisterRead();
    _busConflicts = HasBusConflicts();
    _clockHook = EnableCpuClockHook();
    _addressHook = EnableVramAddressHook();
    _customRead = EnableCustomVramRead();
    _customCpuRead = EnableCustomRamRead();
    AddRegisterRange(RegisterStartAddress(), RegisterEndAddress());
    UpdatePageSizes();
    if (!chrRom && _chrRamSize) SetPpuMemoryMapping(0, 0x1FFF, 0, ChrMemoryType::ChrRam);
    SetupDefaultWorkRam();
    SetMirroringType(header.flags6 & 8 ? MirroringType::FourScreens
                     : header.flags6 & 1 ? MirroringType::Vertical : MirroringType::Horizontal);
    InitMapper();
}

void Board::ApplyTrainer(const uint8_t trainer[512]) {
    if (!trainer) return;
    uint8_t *bytes = _workRamSize >= 0x2000 ? _workRam : _saveRamSize >= 0x2000 ? _saveRam : nullptr;
    if (bytes) std::memcpy(bytes + 0x1000, trainer, 512);
}

void Board::ReadBattery(const char *suffix, uint8_t *bytes, uint32_t size) {
    if (_saveStem.empty() || !bytes || !size) return;
    std::string path = _saveStem + suffix;
    if (FILE *file = std::fopen(path.c_str(), "rb")) {
        size_t count = std::fread(bytes, 1, size, file);
        if (count < size && std::ferror(file))
            std::fprintf(stderr, "Failed to read cartridge save '%s'\n", path.c_str());
        std::fclose(file);
    }
    _savedBytes[suffix].assign(bytes, bytes + size);
}

bool Board::WriteBattery(const char *suffix, const uint8_t *bytes, uint32_t size) {
    if (_saveStem.empty() || !bytes || !size) return true;
    auto previous = _savedBytes.find(suffix);
    if (previous != _savedBytes.end() && previous->second.size() == size
        && !std::memcmp(previous->second.data(), bytes, size)) return true;
    std::string path = _saveStem + suffix;
    std::string temporary = path + ".tmp";
    FILE *file = std::fopen(temporary.c_str(), "wb");
    if (!file) {
        std::fprintf(stderr, "Cannot write cartridge save '%s'\n", path.c_str());
        return false;
    }
    bool complete = std::fwrite(bytes, 1, size, file) == size;
    if (std::fclose(file) != 0) complete = false;
#ifdef _WIN32
    if (complete) complete = MoveFileExA(temporary.c_str(), path.c_str(),
                                       MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
    if (complete) complete = std::rename(temporary.c_str(), path.c_str()) == 0;
#endif
    if (!complete) {
        std::fprintf(stderr, "Cannot replace cartridge save '%s'; changes remain unsaved\n", path.c_str());
        std::remove(temporary.c_str());
        return false;
    }
    _savedBytes[suffix].assign(bytes, bytes + size);
    return true;
}

void Board::LoadBattery() {
    if (HasBattery()) ReadBattery(".sav", _saveRam, _saveRamSize);
    if (_saveChrRamSize)
        ReadBattery(".chr.sav", _chrRam + (_chrRamSize - _saveChrRamSize), _saveChrRamSize);
}

void Board::SaveBattery() {
    if (HasBattery()) WriteBattery(".sav", _saveRam, _saveRamSize);
    if (_saveChrRamSize)
        WriteBattery(".chr.sav", _chrRam + (_chrRamSize - _saveChrRamSize), _saveChrRamSize);
}

void Board::ConfigureBattery(const char *romPath) {
    FlushBattery();
    _saveStem.clear();
    _savedBytes.clear();
    if (!romPath) return;
    _saveStem = romPath;
    size_t slash = _saveStem.find_last_of("/\\");
    size_t dot = _saveStem.find_last_of('.');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) _saveStem.resize(dot);
    LoadBattery();
}

void Board::FlushBattery() {
    if (!_saveStem.empty()) SaveBattery();
}

} // namespace cupid::boards

struct CartridgeBoard { std::unique_ptr<cupid::boards::Board> instance; };

CartridgeBoard *board_create(const iNESHeader *header, uint8_t *prg, size_t prgBytes,
                             uint8_t *chr, size_t chrBytes) {
    if (!header) return nullptr;
    try {
        auto board = std::make_unique<CartridgeBoard>();
        board->instance = cupid::boards::CreateBoard(static_cast<unsigned>(rom_mapper_number(header)));
        if (!board->instance) return nullptr;
        board->instance->Initialize(*header, prg, prgBytes, chr, chrBytes);
        return board.release();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "Cartridge initialization failed: %s\n", error.what());
        return nullptr;
    }
}

void board_destroy(CartridgeBoard *board) { delete board; }
uint8_t board_cpu_read(CartridgeBoard *board, uint16_t address, uint8_t openBus) {
    return board ? board->instance->ReadCpu(address, openBus) : openBus;
}
void board_cpu_write(CartridgeBoard *board, uint16_t address, uint8_t value) {
    if (board) board->instance->WriteCpu(address, value);
}
uint8_t board_ppu_read(CartridgeBoard *board, uint16_t address, unsigned source) {
    return board ? board->instance->ReadPpu(address, source) : static_cast<uint8_t>(address);
}
void board_ppu_write(CartridgeBoard *board, uint16_t address, uint8_t value) {
    if (board) board->instance->MapperWriteVram(address & 0x3FFF, value);
}
void board_clock_cpu(CartridgeBoard *board, bool writeCycle) { if (board) board->instance->ClockCpu(writeCycle); }
void board_notify_ppu_address(CartridgeBoard *board, uint16_t address, uint64_t cycle) {
    if (board) board->instance->NotifyPpu(address, cycle);
}
void board_reset(CartridgeBoard *board, bool softReset) { if (board) board->instance->Reset(softReset); }
void board_after_reset(CartridgeBoard *board) { if (board) board->instance->OnAfterResetPowerOn(); }
bool board_irq_pending(const CartridgeBoard *board) { return board && board->instance->PendingIrq(); }
void board_irq_ack(CartridgeBoard *board) { if (board) board->instance->AcknowledgeIrq(); }
float board_audio(const CartridgeBoard *board) { return board ? board->instance->AudioOutput() : 0.0f; }
Mirroring board_mirroring(const CartridgeBoard *board) {
    return board ? board->instance->MirroringMode() : MIRROR_HORIZONTAL;
}
void board_set_mirroring(CartridgeBoard *board, Mirroring mirroring) {
    if (board) board->instance->SetMirroring(mirroring);
}
void board_apply_trainer(CartridgeBoard *board, const uint8_t trainer[512]) {
    if (board) board->instance->ApplyTrainer(trainer);
}
void board_battery_configure(CartridgeBoard *board, const char *romPath) {
    if (!board) return;
    try { board->instance->ConfigureBattery(romPath); }
    catch (const std::exception &error) {
        std::fprintf(stderr, "Cartridge save setup failed: %s\n", error.what());
    }
}
void board_battery_flush(CartridgeBoard *board) {
    if (!board) return;
    try { board->instance->FlushBattery(); }
    catch (const std::exception &error) {
        std::fprintf(stderr, "Cartridge save failed: %s\n", error.what());
    }
}
