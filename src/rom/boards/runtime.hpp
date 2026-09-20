/*
 * runtime.hpp - Cartridge page mapping and board hooks
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
#ifndef CUPID_BOARD_RUNTIME_HPP
#define CUPID_BOARD_RUNTIME_HPP

#include "../board.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cupid::boards {

enum class PrgMemoryType { PrgRom, SaveRam, WorkRam, MapperRam };
enum class ChrMemoryType { Default, ChrRom, ChrRam, NametableRam, MapperRam };
enum class MirroringType { Horizontal, Vertical, ScreenAOnly, ScreenBOnly, FourScreens };
enum class MemoryOperation { Read = 1, Write = 2, Any = 3 };
enum class MemoryOperationType { Read, PpuRenderingRead, PpuRenderingSpriteRead };
enum MemoryAccessType : int8_t { Unspecified = -1, NoAccess = 0, Read = 1, Write = 2, ReadWrite = 3 };

struct BoardInfo {
    unsigned MapperID = 0;
    uint8_t SubMapperID = 0;
    bool IsNes20Header = false;
    bool HasBattery = false;
    bool HasChrRam = false;
    struct { uint8_t Byte6 = 0; } Header;
    struct { std::string Chip, Board; int8_t BusConflicts = -1; } DatabaseInfo;
};

class Board {
    struct Page {
        uint8_t *data = nullptr;
        uint8_t access = 0;
    };

    std::array<Page, 256> _cpuPages{};
    std::array<Page, 64> _ppuPages{};
    std::array<uint8_t, 65536> _registerAccess{};
    std::vector<uint8_t> _workStorage, _saveStorage, _chrStorage;
    std::vector<uint8_t> _mapperStorage, _nametableStorage;
    std::string _saveStem;
    std::map<std::string, std::vector<uint8_t>> _savedBytes;
    MirroringType _mirroring = MirroringType::Horizontal;
    uint32_t _prgPageSize = 0, _chrRomPageSize = 0, _chrRamPageSize = 0;
    uint32_t _workPageSize = 0, _savePageSize = 0;
    uint8_t _openBus = 0xFF;
    bool _registerReads = false, _busConflicts = false;
    bool _clockHook = false, _addressHook = false, _customRead = false, _customCpuRead = false;
    bool _irq = false;
    uint64_t _ppuClock = 0;
    bool _writeCycle = false;

    static bool AlignedRange(uint16_t first, uint16_t last);
    void MapCpuBytes(uint16_t first, uint16_t last, uint8_t *bytes,
                     uint32_t offset, uint32_t size, int8_t access);
    void MapPpuBytes(uint16_t first, uint16_t last, uint8_t *bytes,
                     uint32_t offset, uint32_t size, int8_t access);

protected:
    BoardInfo _romInfo;
    uint8_t *_prgRom = nullptr, *_chrRom = nullptr, *_chrRam = nullptr;
    uint8_t *_saveRam = nullptr, *_workRam = nullptr, *_mapperRam = nullptr;
    uint32_t _prgSize = 0, _chrRomSize = 0, _chrRamSize = 0, _saveChrRamSize = 0;
    uint32_t _saveRamSize = 0, _workRamSize = 0, _mapperRamSize = 0;
    uint32_t _nametableCount = 0;

    virtual void InitMapper() = 0;
    virtual uint16_t GetPrgPageSize() = 0;
    virtual uint16_t GetChrPageSize() = 0;
    virtual uint16_t GetChrRamPageSize() { return GetChrPageSize(); }
    virtual uint32_t GetSaveRamSize() { return 0x2000; }
    virtual uint32_t GetSaveRamPageSize() { return 0x2000; }
    virtual uint32_t GetWorkRamSize() { return 0x2000; }
    virtual uint32_t GetWorkRamPageSize() { return 0x2000; }
    virtual uint32_t GetChrRamSize() { return 0; }
    virtual uint32_t GetMapperRamSize() { return 0; }
    virtual uint32_t GetNametableCount() { return 0; }
    virtual uint32_t GetDipSwitchCount() { return 0; }
    virtual bool ForceSaveRamSize() { return false; }
    virtual bool ForceWorkRamSize() { return false; }
    virtual bool ForceChrBattery() { return false; }
    virtual bool HasBusConflicts() { return false; }
    virtual bool AllowRegisterRead() { return false; }
    virtual bool EnableCpuClockHook() { return false; }
    virtual bool EnableVramAddressHook() { return false; }
    virtual bool EnableCustomVramRead() { return false; }
    virtual bool EnableCustomRamRead() { return false; }
    virtual uint16_t RegisterStartAddress() { return 0x8000; }
    virtual uint16_t RegisterEndAddress() { return 0xFFFF; }
    virtual uint8_t ReadRegister(uint16_t) { return 0; }
    virtual void WriteRegister(uint16_t, uint8_t) {}

    bool IsNes20() const { return _romInfo.IsNes20Header; }
    bool HasBattery() const { return _romInfo.HasBattery; }
    bool HasChrRam() const { return _chrRamSize != 0; }
    bool HasChrRom() const { return _chrRomSize != 0; }
    uint32_t GetPrgPageCount() const;
    uint32_t GetChrRomPageCount() const;
    uint32_t GetDipSwitches();
    uint8_t GetPowerOnByte(uint8_t defaultValue = 0) const { return defaultValue; }
    uint8_t GetOpenBus(uint8_t mask = 0xFF) const { return _openBus & mask; }
    uint64_t CpuClock() const;
    uint64_t FrameCount() const;
    void WriteCpuBus(uint16_t address, uint8_t value);
    uint32_t PpuFrameCycle() const;
    uint64_t PpuClock() const { return _ppuClock; }
    bool CpuWriteCycle() const { return _writeCycle; }
    void SetIrq(bool asserted) { _irq = asserted; }
    bool IrqPending() const { return _irq; }

    uint8_t InternalReadRam(uint16_t addr) const;
    int64_t PrgRomOffset(uint16_t addr) const;
    int64_t ChrRomOffset(uint16_t addr) const;
    uint8_t InternalRead(uint16_t addr);
    uint8_t InternalReadVram(uint16_t addr) const;
    void InternalWriteVram(uint16_t addr, uint8_t value);
    void WritePrgRam(uint16_t addr, uint8_t value);

    virtual void SelectPrgPage(uint16_t slot, uint16_t page,
                               PrgMemoryType type = PrgMemoryType::PrgRom);
    void SelectPrgPage2x(uint16_t slot, uint16_t page,
                         PrgMemoryType type = PrgMemoryType::PrgRom);
    void SelectPrgPage4x(uint16_t slot, uint16_t page,
                         PrgMemoryType type = PrgMemoryType::PrgRom);
    virtual void SelectChrPage(uint16_t slot, uint16_t page,
                               ChrMemoryType type = ChrMemoryType::Default);
    void SelectChrPage2x(uint16_t slot, uint16_t page,
                         ChrMemoryType type = ChrMemoryType::Default);
    void SelectChrPage4x(uint16_t slot, uint16_t page,
                         ChrMemoryType type = ChrMemoryType::Default);
    void SelectChrPage8x(uint16_t slot, uint16_t page,
                         ChrMemoryType type = ChrMemoryType::Default);
    void SetCpuMemoryMapping(uint16_t first, uint16_t last, int16_t page,
                              PrgMemoryType type, int8_t access = -1);
    void SetCpuMemoryMapping(uint16_t first, uint16_t last, PrgMemoryType type,
                              uint32_t offset, int8_t access);
    void SetCpuMemoryMapping(uint16_t first, uint16_t last, uint8_t *bytes,
                              uint32_t offset, uint32_t size, int8_t access = -1);
    void SetPpuMemoryMapping(uint16_t first, uint16_t last, uint16_t page,
                              ChrMemoryType type = ChrMemoryType::Default, int8_t access = -1);
    void SetPpuMemoryMapping(uint16_t first, uint16_t last, ChrMemoryType type,
                              uint32_t offset, int8_t access);
    void SetPpuMemoryMapping(uint16_t first, uint16_t last, uint8_t *bytes,
                              uint32_t offset, uint32_t size, int8_t access = -1);
    void RemoveCpuMemoryMapping(uint16_t first, uint16_t last);
    void RemovePpuMemoryMapping(uint16_t first, uint16_t last);
    void AddRegisterRange(uint16_t first, uint16_t last,
                          MemoryOperation operation = MemoryOperation::Any);
    void RemoveRegisterRange(uint16_t first, uint16_t last,
                             MemoryOperation operation = MemoryOperation::Any);
    uint8_t *GetNametable(uint8_t index);
    void SetNametable(uint8_t index, uint8_t page);
    void SetNametables(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
    void SetMirroringType(MirroringType type);
    MirroringType GetMirroringType() const { return _mirroring; }
    void SetupDefaultWorkRam();
    void UpdatePageSizes();
    void ReadBattery(const char *suffix, uint8_t *bytes, uint32_t size);
    bool WriteBattery(const char *suffix, const uint8_t *bytes, uint32_t size);
    virtual void LoadBattery();
    virtual bool SaveBattery();

public:
    virtual ~Board() = default;
    void Initialize(const iNESHeader &header, uint8_t *prg, size_t prgBytes,
                     uint8_t *chr, size_t chrBytes,
                     const RomDatabaseInfo *database = nullptr);
    virtual void Reset(bool) {}
    virtual void OnAfterResetPowerOn() {}
    virtual void ProcessCpuClock() {}
    virtual void NotifyVramAddressChange(uint16_t) {}
    virtual uint8_t ReadRam(uint16_t addr) { return InternalRead(addr); }
    virtual void WriteRam(uint16_t addr, uint8_t value);
    virtual uint8_t MapperReadVram(uint16_t addr, MemoryOperationType) {
        return InternalReadVram(addr);
    }
    virtual void MapperWriteVram(uint16_t addr, uint8_t value) {
        InternalWriteVram(addr, value);
    }
    virtual float AudioOutput() const { return 0.0f; }
    virtual bool SetMapperInput(unsigned, bool) { return false; }
    virtual uint8_t *CpuRam8K() { return nullptr; }
    virtual bool ReadCpuRegister(uint16_t, uint8_t &) { return false; }
    virtual void ObserveCpuWrite(uint16_t, uint8_t) {}
    uint8_t ReadCpu(uint16_t addr, uint8_t openBus);
    void WriteCpu(uint16_t addr, uint8_t value);
    uint8_t ReadPpu(uint16_t addr, unsigned fetchSource);
    void ClockCpu(bool writeCycle);
    void NotifyPpu(uint16_t addr, uint64_t cycle);
    bool PendingIrq() const { return _irq; }
    void AcknowledgeIrq() { _irq = false; }
    Mirroring MirroringMode() const { return static_cast<Mirroring>(_mirroring); }
    void SetMirroring(Mirroring type) { SetMirroringType(static_cast<MirroringType>(type)); }
    void ApplyTrainer(const uint8_t trainer[512]);
    void ConfigureBattery(const char *romPath);
    bool FlushBattery();
};

std::unique_ptr<Board> CreateBoard(unsigned mapper);
std::unique_ptr<Board> CreateFcnsBoard();
const std::vector<uint8_t> &FcnsKanjiFirmware();
bool SetFcnsKanjiFirmware(const uint8_t *data, size_t size);

} // namespace cupid::boards
#endif
