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
#include "board_internal.hpp"
#include "studybox.hpp"
#include "../../util/file_io.h"
#include "../../system/execution_policy.h"
#include "../../video/video_trace.h"
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
#include "../../ppu/ppu.h"
#include "../../system/hardware.h"
#include "../../system/timing.h"
}

namespace cupid::boards {

static std::vector<uint8_t> fcnsKanjiFirmware;

const std::vector<uint8_t> &FcnsKanjiFirmware() { return fcnsKanjiFirmware; }

bool SetFcnsKanjiFirmware(const uint8_t *data, size_t size) {
    if (!data && !size) {
        fcnsKanjiFirmware.clear();
        return true;
    }
    if (!data || size != 0x40000) return false;
    fcnsKanjiFirmware.assign(data, data + size);
    return true;
}

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
uint64_t Board::FrameCount() const { return ppu.frame_count; }
void Board::WriteCpuBus(uint16_t address, uint8_t value) { write_mem(address, value); }
uint32_t Board::PpuFrameCycle() const {
    int line = ppu.scanline == (int)nes_timing()->scanlines - 1 ? -1 : ppu.scanline;
    return static_cast<uint32_t>((line + 1) * 341 + ppu.dot);
}

bool Board::VisitState(BoardStateVisitor &state) {
    enum class Backing : uint8_t {
        None,
        PrgRom,
        ChrRom,
        SaveRam,
        WorkRam,
        ChrRam,
        MapperRam,
        NametableRam
    };

    auto visitPage = [&](const char *prefix, unsigned index, Page &page, bool cpu) {
        uint8_t kind = static_cast<uint8_t>(Backing::None);
        uint32_t offset = 0;
        auto identify = [&](Backing candidate, uint8_t *base, uint32_t size) {
            if (!page.data || !base || size < 0x100) return false;
            uintptr_t mapped = reinterpret_cast<uintptr_t>(page.data);
            uintptr_t first = reinterpret_cast<uintptr_t>(base);
            if (mapped < first || mapped - first > size - 0x100) return false;
            kind = static_cast<uint8_t>(candidate);
            offset = static_cast<uint32_t>(mapped - first);
            return true;
        };
        if (state.GetMode() == BoardStateVisitor::Mode::Capture && page.data) {
            bool found = cpu
                ? identify(Backing::PrgRom, _prgRom, _prgSize)
                    || identify(Backing::SaveRam, _saveRam, _saveRamSize)
                    || identify(Backing::WorkRam, _workRam, _workRamSize)
                    || identify(Backing::MapperRam, _mapperRam, _mapperRamSize)
                : identify(Backing::ChrRom, _chrRom, _chrRomSize)
                    || identify(Backing::ChrRam, _chrRam, _chrRamSize)
                    || identify(Backing::MapperRam, _mapperRam, _mapperRamSize)
                    || identify(Backing::NametableRam, _nametableStorage.data(),
                                static_cast<uint32_t>(_nametableStorage.size()));
            if (!found) return false;
        }

        char name[48];
        std::snprintf(name, sizeof(name), "%s.kind.%u", prefix, index);
        if (!state.ValueU8(name, kind, static_cast<uint8_t>(Backing::NametableRam))) return false;

        uint8_t *base = nullptr;
        uint32_t size = 0;
        switch (static_cast<Backing>(kind)) {
            case Backing::None: break;
            case Backing::PrgRom: base = _prgRom; size = _prgSize; break;
            case Backing::ChrRom: base = _chrRom; size = _chrRomSize; break;
            case Backing::SaveRam: base = _saveRam; size = _saveRamSize; break;
            case Backing::WorkRam: base = _workRam; size = _workRamSize; break;
            case Backing::ChrRam: base = _chrRam; size = _chrRamSize; break;
            case Backing::MapperRam: base = _mapperRam; size = _mapperRamSize; break;
            case Backing::NametableRam:
                base = _nametableStorage.data();
                size = static_cast<uint32_t>(_nametableStorage.size());
                break;
        }
        bool validKind = kind == static_cast<uint8_t>(Backing::None)
                      || (cpu && (kind == static_cast<uint8_t>(Backing::PrgRom)
                               || kind == static_cast<uint8_t>(Backing::SaveRam)
                               || kind == static_cast<uint8_t>(Backing::WorkRam)
                               || kind == static_cast<uint8_t>(Backing::MapperRam)))
                      || (!cpu && (kind == static_cast<uint8_t>(Backing::ChrRom)
                                || kind == static_cast<uint8_t>(Backing::ChrRam)
                                || kind == static_cast<uint8_t>(Backing::MapperRam)
                                || kind == static_cast<uint8_t>(Backing::NametableRam)));
        if (!validKind || (kind != static_cast<uint8_t>(Backing::None) && (!base || size < 0x100)))
            return false;

        uint32_t maximum = size >= 0x100 ? size - 0x100 : 0;
        std::snprintf(name, sizeof(name), "%s.offset.%u", prefix, index);
        if (!state.ValueU32(name, offset, maximum)) return false;
        std::snprintf(name, sizeof(name), "%s.access.%u", prefix, index);
        uint8_t access = page.access;
        if (!state.ValueU8(name, access, static_cast<uint8_t>(ReadWrite))) return false;
        if (kind == static_cast<uint8_t>(Backing::None) && (offset != 0 || access != 0)) return false;
        if (state.GetMode() == BoardStateVisitor::Mode::Apply)
            page = kind == static_cast<uint8_t>(Backing::None) ? Page{} : Page{base + offset, access};
        return true;
    };

    if (!state.InvariantU32("board.mapper", _romInfo.MapperID)
        || !state.InvariantU8("board.submapper", _romInfo.SubMapperID)
        || !state.InvariantBool("board.nes20", _romInfo.IsNes20Header)
        || !state.InvariantBool("board.battery", _romInfo.HasBattery)
        || !state.InvariantU8("board.header6", _romInfo.Header.Byte6)
        || !state.InvariantString("board.db.board", _romInfo.DatabaseInfo.Board)
        || !state.InvariantString("board.db.chip", _romInfo.DatabaseInfo.Chip)
        || !state.InvariantU32("board.prg.size", _prgSize)
        || !state.InvariantU32("board.chrrom.size", _chrRomSize)
        || !state.InvariantU32("board.chrram.size", _chrRamSize)
        || !state.InvariantU32("board.saveram.size", _saveRamSize)
        || !state.InvariantU32("board.workram.size", _workRamSize)
        || !state.InvariantU32("board.mapperram.size", _mapperRamSize)
        || !state.InvariantU32("board.nametable.count", _nametableCount))
        return false;

    if (_prgSize && !(StatePrgRomContentsMutable()
        ? state.Bytes("board.prg.mutable", _prgRom, _prgSize)
        : state.InvariantBytes("board.prg", _prgRom, _prgSize)))
        return false;
    if (_chrRomSize && !(StateChrRomContentsMutable()
        ? state.Bytes("board.chrrom.mutable", _chrRom, _chrRomSize)
        : state.InvariantBytes("board.chrrom", _chrRom, _chrRomSize)))
        return false;

    if ((_workRamSize && !state.Bytes("board.workram", _workRam, _workRamSize))
        || (_saveRamSize && !state.Bytes("board.saveram", _saveRam, _saveRamSize))
        || (_chrRamSize && !state.Bytes("board.chrram", _chrRam, _chrRamSize))
        || (_mapperRamSize && !state.Bytes("board.mapperram", _mapperRam, _mapperRamSize))
        || (!_nametableStorage.empty()
            && !state.Bytes("board.nametables", _nametableStorage.data(), _nametableStorage.size()))
        || !state.Bytes("board.register_access", _registerAccess.data(), _registerAccess.size()))
        return false;

    uint8_t mirroring = static_cast<uint8_t>(_mirroring);
    if (!state.ValueU8("board.mirroring", mirroring,
                       static_cast<uint8_t>(MirroringType::FourScreens))
        || !state.Field("board.open_bus", _openBus)
        || !state.Field("board.irq", _irq)
        || !state.Field("board.ppu_clock", _ppuClock)
        || !state.Field("board.write_cycle", _writeCycle))
        return false;
    if (state.GetMode() == BoardStateVisitor::Mode::Apply)
        _mirroring = static_cast<MirroringType>(mirroring);

    for (unsigned i = 0; i < _cpuPages.size(); ++i)
        if (!visitPage("board.cpu", i, _cpuPages[i], true)) return false;
    for (unsigned i = 0; i < _ppuPages.size(); ++i)
        if (!visitPage("board.ppu", i, _ppuPages[i], false)) return false;
    return true;
}

uint32_t Board::GetDipSwitches() {
    return GetDipSwitches(GetDipSwitchCount());
}

uint32_t Board::GetDipSwitches(unsigned count) const {
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

int64_t Board::PrgRomOffset(uint16_t addr) const {
    const Page &page = _cpuPages[addr >> 8];
    uintptr_t mapped = reinterpret_cast<uintptr_t>(page.data);
    uintptr_t base = reinterpret_cast<uintptr_t>(_prgRom);
    if (!(page.access & Read) || mapped < base || mapped - base >= _prgSize) return -1;
    return static_cast<int64_t>(mapped - base + (addr & 0xFF));
}

int64_t Board::ChrRomOffset(uint16_t addr) const {
    const Page &page = _ppuPages[(addr & 0x3FFF) >> 8];
    uintptr_t mapped = reinterpret_cast<uintptr_t>(page.data);
    uintptr_t base = reinterpret_cast<uintptr_t>(_chrRom);
    if (!(page.access & Read) || mapped < base || mapped - base >= _chrRomSize) return -1;
    return static_cast<int64_t>(mapped - base + (addr & 0xFF));
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
    if (nes_video_trace_active && (page.access & Read) && page.data) {
        uintptr_t pointer = reinterpret_cast<uintptr_t>(page.data) + (addr & 0xFF);
        auto record = [&](const uint8_t *base, size_t size, bool ram) {
            uintptr_t start = reinterpret_cast<uintptr_t>(base);
            if (!base || pointer < start || pointer - start >= size) return false;
            nes_video_trace_chr_read(base, size, pointer - start, ram);
            return true;
        };
        if (!record(_chrRom, _chrRomSize, false) && !record(_chrRam, _chrRamSize, true)) {
            (void)record(_mapperRam, _mapperRamSize, true);
        }
    }
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

uint8_t Board::PeekCpu(uint16_t addr, uint8_t openBus) const {
    const Page &page = _cpuPages[addr >> 8];
    return page.data && (page.access & Read) ? page.data[addr & 0xFF] : openBus;
}

void Board::WriteCpu(uint16_t addr, uint8_t value) {
    _openBus = value;
    WriteRam(addr, value);
}

uint8_t Board::ReadPpu(uint16_t addr, unsigned fetchSource) {
    MemoryOperationType type = fetchSource ? MemoryOperationType::PpuRenderingRead : MemoryOperationType::Read;
    return _customRead ? MapperReadVram(addr & 0x3FFF, type) : InternalReadVram(addr);
}

uint8_t Board::PeekPpu(uint16_t addr) const {
    addr &= 0x3FFF;
    const Page &page = _ppuPages[addr >> 8];
    return page.data && (page.access & Read) ? page.data[addr & 0xFF]
                                              : static_cast<uint8_t>(addr);
}

bool Board::DebugWritePpu(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF;
    Page &page = _ppuPages[addr >> 8];
    if (!page.data || !(page.access & Write)) return false;
    page.data[addr & 0xFF] = value;
    return true;
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
                        uint8_t *chr, size_t chrBytes,
                        const RomDatabaseInfo *database) {
    if (!prg || !prgBytes || prgBytes > UINT32_MAX || chrBytes > UINT32_MAX)
        throw std::invalid_argument("unsupported cartridge buffer size");
    _romInfo.MapperID = database && database->present
                      ? database->mapper : static_cast<unsigned>(rom_mapper_number(&header));
    /* Database-corrected legacy and headerless images keep legacy mapper
       semantics even though their synthesized transport header carries the
       NES 2.0 marker for extended metadata. */
    _romInfo.IsNes20Header = !(database && database->present)
                           && (header.flags7 & 0x0C) == 8;
    _romInfo.SubMapperID = database && database->present && database->submapper_present
                         ? database->submapper : IsNes20() ? header.prg_ram_size >> 4 : 0;
    _romInfo.HasBattery = (header.flags6 & 2) != 0;
    _romInfo.Header.Byte6 = header.flags6;
    if (database && database->present) {
        _romInfo.DatabaseInfo.Board = database->board;
        _romInfo.DatabaseInfo.Chip = database->chip;
        _romInfo.DatabaseInfo.BusConflicts = database->bus_conflicts;
    }
    _prgRom = prg;
    _prgSize = static_cast<uint32_t>(prgBytes);
    bool chrRom = database && database->present
                ? database->chr_rom_size != 0
                : header.chr_rom_chunks || (IsNes20() && (header.flags9 & 0xF0));
    if (chrRom && (!chr || !chrBytes)) throw std::invalid_argument("missing CHR ROM");
    _chrRom = chrRom ? chr : nullptr;
    _chrRomSize = chrRom ? static_cast<uint32_t>(chrBytes) : 0;
    RomRamSizes ram{};
    rom_ram_sizes_with_metadata(&header, database, &ram);
    if (database && database->present) {
        _saveRamSize = !database->save_ram_override ? (HasBattery() ? GetSaveRamSize() : 0)
                     : ForceSaveRamSize() ? GetSaveRamSize() : static_cast<uint32_t>(ram.prg_nvram);
        _workRamSize = !database->work_ram_override ? (HasBattery() ? 0 : GetWorkRamSize())
                     : ForceWorkRamSize() ? GetWorkRamSize() : static_cast<uint32_t>(ram.prg_ram);
        _saveChrRamSize = static_cast<uint32_t>(ram.chr_nvram);
        _chrRamSize = database->chr_ram_override ? static_cast<uint32_t>(ram.chr_ram + ram.chr_nvram)
                    : GetChrRamSize() ? GetChrRamSize() : chrRom ? 0 : 0x2000;
    } else {
        _saveRamSize = !IsNes20() || ForceSaveRamSize()
                     ? (HasBattery() || ForceSaveRamSize() ? GetSaveRamSize() : 0)
                     : static_cast<uint32_t>(ram.prg_nvram);
        _workRamSize = !IsNes20() || ForceWorkRamSize()
                     ? (!HasBattery() || ForceWorkRamSize() ? GetWorkRamSize() : 0)
                     : static_cast<uint32_t>(ram.prg_ram);
        _saveChrRamSize = IsNes20() ? static_cast<uint32_t>(ram.chr_nvram) : 0;
        _chrRamSize = IsNes20() ? static_cast<uint32_t>(ram.chr_ram + ram.chr_nvram)
                    : GetChrRamSize() ? GetChrRamSize() : !chrRom ? 0x2000 : 0;
    }
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
    _busConflicts = database && database->present && database->bus_conflicts >= 0
                  ? database->bus_conflicts != 0 : HasBusConflicts();
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
    if (database && database->present && database->mirroring_override)
        SetMirroringType(static_cast<MirroringType>(database->mirroring));
    InitMapper();
}

void Board::InitializeReplayMemory(CartReplayMemoryInitializer initialize, void *context) {
    if (!initialize) {
        return;
    }

    initialize(_workStorage.data(), _workStorage.size(), true, context);
    initialize(_saveStorage.data(), _saveStorage.size(), true, context);
    initialize(_chrStorage.data(), _chrStorage.size(), false, context);
    initialize(_mapperStorage.data(), _mapperStorage.size(), true, context);
    initialize(_nametableStorage.data(), _nametableStorage.size(), true, context);
}

size_t Board::ReplaySaveRamSize() const {
    return _saveStorage.empty() ? _workStorage.size() : _saveStorage.size();
}

bool Board::SetReplaySaveRam(const uint8_t *bytes, size_t size) {
    auto &storage = _saveStorage.empty() ? _workStorage : _saveStorage;
    if ((!bytes && size) || size != storage.size()) {
        return false;
    }

    if (size) {
        std::memcpy(storage.data(), bytes, size);
    }

    return true;
}

bool Board::DebugLocation(bool cpu, uint16_t addr, NesMemoryLocation &out) const {
    if (!cpu && addr >= 0x4000) return false;
    /* Custom reads need mapper-specific observational resolution. A page
     * pointer alone does not identify the storage supplying such a read. */
    if (cpu ? (_customCpuRead || (_registerReads && (_registerAccess[addr] & Read))) : _customRead)
        return false;
    const Page &page = cpu ? _cpuPages[addr >> 8] : _ppuPages[addr >> 8];
    if (!page.data || !(page.access & Read)) return false;
    uintptr_t pointer = reinterpret_cast<uintptr_t>(page.data) + (addr & 0xFF);
    auto identify = [&](uint8_t *base, size_t size, const char *name, bool ram) {
        uintptr_t start = reinterpret_cast<uintptr_t>(base);
        if (!base || pointer < start || pointer - start >= size) return false;
        bool writable = ram && (page.access & Write);
        if (cpu && (_registerAccess[addr] & Write)) writable = false;
        return nes_memory_location(&out, base, size, pointer - start, name, writable, nullptr);
    };
    if (cpu) {
        return identify(_prgRom, _prgSize, "PRG ROM", false)
            || identify(_saveRam, _saveRamSize, "PRG NVRAM", true)
            || identify(_workRam, _workRamSize, "PRG RAM", true)
            || identify(_mapperRam, _mapperRamSize, "Mapper RAM", true);
    }
    if (identify(_chrRom, _chrRomSize, "CHR ROM", false)) return true;
    if (identify(_chrRam, _chrRamSize, "CHR RAM", true)) {
        if (_saveChrRamSize && out.offset >= _chrRamSize - _saveChrRamSize) {
            out.backing = "CHR NVRAM";
            out.offset -= _chrRamSize - _saveChrRamSize;
        }
        return true;
    }
    return identify(_mapperRam, _mapperRamSize, "Mapper RAM", true)
        || identify(const_cast<uint8_t *>(_nametableStorage.data()), _nametableStorage.size(), "Nametable RAM", true);
}

void Board::ApplyTrainer(const uint8_t trainer[512]) {
    if (!trainer) return;
    uint8_t *bytes = _workRamSize >= 0x2000 ? _workRam : _saveRamSize >= 0x2000 ? _saveRam : nullptr;
    if (bytes) std::memcpy(bytes + 0x1000, trainer, 512);
}

void Board::ReadBattery(const char *suffix, uint8_t *bytes, uint32_t size) {
    if (_saveStem.empty() || !bytes || !size) return;
    std::string path = _saveStem + suffix;
    if (FILE *file = nes_file_open(path.c_str(), "rb")) {
        size_t count = std::fread(bytes, 1, size, file);
        if (count < size && std::ferror(file))
            std::fprintf(stderr, "Failed to read cartridge save '%s'\n", path.c_str());
        std::fclose(file);
    }
    _savedBytes[suffix].assign(bytes, bytes + size);
}

bool Board::WriteBattery(const char *suffix, const uint8_t *bytes, uint32_t size) {
    if (!nes_execution_allows_persistence()) return true;
    if (_saveStem.empty() || !bytes || !size) return true;
    auto previous = _savedBytes.find(suffix);
    if (previous != _savedBytes.end() && previous->second.size() == size
        && !std::memcmp(previous->second.data(), bytes, size)) return true;
    std::string path = _saveStem + suffix;
    if (nes_file_write_atomic(path.c_str(), bytes, size) != NES_FILE_OK) {
        std::fprintf(stderr, "Cannot replace cartridge save '%s'; changes remain unsaved\n", path.c_str());
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

bool Board::SaveBattery() {
    bool saved = !HasBattery() || WriteBattery(".sav", _saveRam, _saveRamSize);
    if (_saveChrRamSize) {
        saved = WriteBattery(".chr.sav", _chrRam + (_chrRamSize - _saveChrRamSize), _saveChrRamSize) && saved;
    }

    return saved;
}

void Board::ConfigureBattery(const char *romPath) {
    if (!FlushBattery()) {
        throw std::runtime_error("Current cartridge save could not be written");
    }
    _saveStem.clear();
    _savedBytes.clear();
    if (!romPath) return;
    _saveStem = romPath;
    size_t slash = _saveStem.find_last_of("/\\");
    size_t dot = _saveStem.find_last_of('.');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) _saveStem.resize(dot);
    LoadBattery();
}

bool Board::FlushBattery() {
    return _saveStem.empty() || SaveBattery();
}

} // namespace cupid::boards

CartridgeBoard *board_create(const iNESHeader *header, uint8_t *prg, size_t prgBytes,
                             uint8_t *chr, size_t chrBytes) {
    return board_create_with_metadata(header, prg, prgBytes, chr, chrBytes, nullptr);
}

CartridgeBoard *board_create_with_metadata(const iNESHeader *header,
                                           uint8_t *prg, size_t prgBytes,
                                           uint8_t *chr, size_t chrBytes,
                                           const RomDatabaseInfo *database) {
    if (!header) return nullptr;
    if (!prg || !prgBytes || prgBytes > UINT32_MAX || chrBytes > UINT32_MAX) {
        std::fprintf(stderr, "Unsupported cartridge buffer size\n");
        return nullptr;
    }
    bool nes20 = (header->flags7 & 0x0C) == 8;
    bool hasChrRom = header->chr_rom_chunks || (nes20 && (header->flags9 & 0xF0));
    if (hasChrRom && (!chr || !chrBytes)) {
        std::fprintf(stderr, "Missing cartridge CHR ROM\n");
        return nullptr;
    }
    const bool corrected = database && database->present;
    const unsigned mapper = corrected ? database->mapper
        : static_cast<unsigned>(rom_mapper_number(header));
    const unsigned submapper = corrected && database->submapper_present ? database->submapper
        : !corrected && nes20 ? header->prg_ram_size >> 4 : 0;
    if (!board_is_fcns_header(header) && (mapper == 34 || mapper == 69)) {
        RomRamSizes ram{};
        rom_ram_sizes_with_metadata(header, database, &ram);
        if ((ram.prg_nvram || ram.chr_nvram) && !(header->flags6 & 2)) {
            std::fprintf(stderr, "Nonvolatile RAM declared without the battery flag\n");
            return nullptr;
        }
        if (mapper == 34 && submapper > 2) {
            std::fprintf(stderr, "Unsupported mapper/submapper: 34/%u\n", submapper);
            return nullptr;
        }
    }
    try {
        auto board = std::make_unique<CartridgeBoard>();
        board->instance = board_is_fcns_header(header)
            ? cupid::boards::CreateFcnsBoard()
            : cupid::boards::CreateBoard(database && database->present
                ? database->mapper : static_cast<unsigned>(rom_mapper_number(header)));
        if (!board->instance) return nullptr;
        board->instance->Initialize(*header, prg, prgBytes, chr, chrBytes, database);
        return board.release();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "Cartridge initialization failed: %s\n", error.what());
        return nullptr;
    }
}

void board_destroy(CartridgeBoard *board) { delete board; }
uint8_t *board_cpu_ram_8k(CartridgeBoard *board) {
    return board ? board->instance->CpuRam8K() : nullptr;
}
uint8_t board_cpu_read(CartridgeBoard *board, uint16_t address, uint8_t openBus) {
    return board ? board->instance->ReadCpu(address, openBus) : openBus;
}
uint8_t board_cpu_peek(const CartridgeBoard *board, uint16_t address, uint8_t openBus) {
    return board ? board->instance->PeekCpu(address, openBus) : openBus;
}
void board_cpu_write(CartridgeBoard *board, uint16_t address, uint8_t value) {
    if (board) board->instance->WriteCpu(address, value);
}
bool board_read_cpu_register(CartridgeBoard *board, uint16_t address, uint8_t *value) {
    return board && value && board->instance->ReadCpuRegister(address, *value);
}
void board_observe_cpu_write(CartridgeBoard *board, uint16_t address, uint8_t value) {
    if (board) board->instance->ObserveCpuWrite(address, value);
}
uint8_t board_ppu_read(CartridgeBoard *board, uint16_t address, unsigned source) {
    return board ? board->instance->ReadPpu(address, source) : static_cast<uint8_t>(address);
}
uint8_t board_ppu_peek(const CartridgeBoard *board, uint16_t address) {
    return board ? board->instance->PeekPpu(address) : static_cast<uint8_t>(address);
}
bool board_debug_write_ppu(CartridgeBoard *board, uint16_t address, uint8_t value) {
    return board && board->instance->DebugWritePpu(address, value);
}
bool board_debug_location(CartridgeBoard *board, bool cpu, uint16_t address, NesMemoryLocation *out) {
    return board && out && board->instance->DebugLocation(cpu, address, *out);
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
unsigned board_audio_mix_channel(const CartridgeBoard *board) {
    return board ? board->instance->AudioMixChannel() : static_cast<unsigned>(NES_AUDIO_CARTRIDGE_PCM);
}
bool board_set_mapper_input(CartridgeBoard *board, unsigned input, bool pressed) {
    return board && board->instance->SetMapperInput(input, pressed);
}

CartridgeBoard *board_create_studybox(const uint8_t *bios, size_t biosSize,
                                      const uint8_t *media, size_t mediaSize) {
    if (!bios || biosSize != 0x40000 || !media) {
        std::fprintf(stderr, "StudyBox BIOS must be exactly 256 KiB\n");
        return nullptr;
    }
    try {
        cupid::boards::StudyBoxTape tape;
        std::string error;
        if (!cupid::boards::ParseStudyBoxTape(media, mediaSize, tape, error)) {
            std::fprintf(stderr, "Invalid StudyBox media: %s\n", error.c_str());
            return nullptr;
        }

        auto board = std::make_unique<CartridgeBoard>();
        board->ownedPrg.assign(bios, bios + biosSize);
        board->instance = std::make_unique<cupid::boards::StudyBox>(std::move(tape));
        iNESHeader header{};
        std::memcpy(header.signature, "NES\x1A", 4);
        header.prg_rom_chunks = 16;
        board->instance->Initialize(header, board->ownedPrg.data(), board->ownedPrg.size(), nullptr, 0);
        return board.release();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "StudyBox initialization failed: %s\n", error.what());
        return nullptr;
    }
}
bool board_set_fcns_kanji_firmware(const uint8_t *data, size_t size) {
    return cupid::boards::SetFcnsKanjiFirmware(data, size);
}
Mirroring board_mirroring(const CartridgeBoard *board) {
    return board ? board->instance->MirroringMode() : MIRROR_HORIZONTAL;
}
void board_set_mirroring(CartridgeBoard *board, Mirroring mirroring) {
    if (board) board->instance->SetMirroring(mirroring);
}
void board_apply_trainer(CartridgeBoard *board, const uint8_t trainer[512]) {
    if (board) board->instance->ApplyTrainer(trainer);
}

void board_replay_initialize_memory(CartridgeBoard *board, CartReplayMemoryInitializer initialize,
                                     void *context) {
    if (board) {
        board->instance->InitializeReplayMemory(initialize, context);
    }
}

size_t board_replay_save_ram_size(const CartridgeBoard *board) {
    return board ? board->instance->ReplaySaveRamSize() : 0;
}

bool board_replay_set_save_ram(CartridgeBoard *board, const uint8_t *bytes, size_t size) {
    return board && board->instance->SetReplaySaveRam(bytes, size);
}
void board_battery_configure(CartridgeBoard *board, const char *romPath) {
    if (!board) return;
    try { board->instance->ConfigureBattery(romPath); }
    catch (const std::exception &error) {
        std::fprintf(stderr, "Cartridge save setup failed: %s\n", error.what());
    }
}
bool board_battery_flush(CartridgeBoard *board) {
    if (!board) return true;
    try { return board->instance->FlushBattery(); }
    catch (const std::exception &error) {
        std::fprintf(stderr, "Cartridge save failed: %s\n", error.what());
        return false;
    }
}
