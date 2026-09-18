/*
 * unlicensed_110.hpp - Unlicensed cartridge boards for issue 110
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
#ifndef CUPID_BOARDS_UNLICENSED_110_HPP
#define CUPID_BOARDS_UNLICENSED_110_HPP

#include "runtime.hpp"
#include "a12_watcher.hpp"
#include <cstring>
extern "C" {
#include "../../ppu/ppu.h"
}

namespace cupid::boards {

class Unl60 final : public Board {
    uint8_t _resetCounter = 0;
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override {
        _resetCounter = 0;
        SelectPrgPage(0, 0); SelectPrgPage(1, 0); SelectChrPage(0, 0);
    }
    void Reset(bool softReset) override {
        if (!softReset) return;
        _resetCounter = static_cast<uint8_t>((_resetCounter + 1) & 3);
        SelectPrgPage(0, _resetCounter); SelectPrgPage(1, _resetCounter);
        SelectChrPage(0, _resetCounter);
    }
};

class Unl62 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { Reset(true); }
    void Reset(bool softReset) override {
        if (!softReset) return;
        SelectPrgPage(0, 0); SelectPrgPage(1, 1); SelectChrPage(0, 0);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        uint8_t prg = static_cast<uint8_t>(((address & 0x3F00) >> 8) | (address & 0x40));
        uint8_t chr = static_cast<uint8_t>(((address & 0x1F) << 2) | (value & 3));
        if (address & 0x20) {
            SelectPrgPage(0, prg); SelectPrgPage(1, prg);
        } else {
            SelectPrgPage(0, prg & 0xFE); SelectPrgPage(1, static_cast<uint8_t>((prg & 0xFE) + 1));
        }
        SelectChrPage(0, chr);
        SetMirroringType(address & 0x80 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Unl83 final : public Board {
    uint8_t _regs[11]{};
    uint8_t _exRegs[4]{};
    bool _is2kBank = false, _isNot2kBank = false;
    uint8_t _mode = 0, _bank = 0;
    uint16_t _irqCounter = 0;
    bool _irqEnabled = false;

    uint32_t GetDipSwitchCount() override { return 2; }
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    bool AllowRegisterRead() override { return true; }
    bool EnableCpuClockHook() override { return true; }

    void UpdateState() {
        switch (_mode & 3) {
            case 0: SetMirroringType(MirroringType::Vertical); break;
            case 1: SetMirroringType(MirroringType::Horizontal); break;
            case 2: SetMirroringType(MirroringType::ScreenAOnly); break;
            default: SetMirroringType(MirroringType::ScreenBOnly); break;
        }
        if (_is2kBank && !_isNot2kBank) {
            SelectChrPage2x(0, static_cast<uint16_t>(_regs[0] << 1));
            SelectChrPage2x(1, static_cast<uint16_t>(_regs[1] << 1));
            SelectChrPage2x(2, static_cast<uint16_t>(_regs[6] << 1));
            SelectChrPage2x(3, static_cast<uint16_t>(_regs[7] << 1));
        } else {
            for (unsigned i = 0; i < 8; ++i)
                SelectChrPage(i, static_cast<uint16_t>(_regs[i] | ((_bank & 0x30) << 4)));
        }
        if (_mode & 0x40) {
            SelectPrgPage2x(0, static_cast<uint16_t>((_bank & 0x3F) << 1));
            SelectPrgPage2x(1, static_cast<uint16_t>(((_bank & 0x30) | 0x0F) << 1));
        } else {
            SelectPrgPage(0, _regs[8]); SelectPrgPage(1, _regs[9]);
            SelectPrgPage(2, _regs[10]); SelectPrgPage(3, static_cast<uint16_t>(-1));
        }
    }

    void InitMapper() override {
        std::memset(_regs, 0, sizeof(_regs)); std::memset(_exRegs, 0, sizeof(_exRegs));
        _is2kBank = _isNot2kBank = false; _mode = _bank = 0; _irqCounter = 0; _irqEnabled = false;
        AddRegisterRange(0x5000, 0x5000, MemoryOperation::Read);
        AddRegisterRange(0x5100, 0x5103, MemoryOperation::Any);
        RemoveRegisterRange(0x8000, 0xFFFF, MemoryOperation::Read);
        UpdateState();
    }
    void ProcessCpuClock() override {
        if (_irqEnabled && --_irqCounter == 0) {
            _irqEnabled = false; _irqCounter = 0xFFFF; SetIrq(true);
        }
    }
    uint8_t ReadRegister(uint16_t address) override {
        return address == 0x5000 ? static_cast<uint8_t>((GetOpenBus() & 0xFC) | GetDipSwitches())
                                 : _exRegs[address & 3];
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address < 0x8000) { _exRegs[address & 3] = value; return; }
        if (address >= 0x8300 && address <= 0x8302) {
            _mode &= 0xBF; _regs[address - 0x8300 + 8] = value; UpdateState(); return;
        }
        if (address >= 0x8310 && address <= 0x8317) {
            _regs[address - 0x8310] = value;
            if (address >= 0x8312 && address <= 0x8315) _isNot2kBank = true;
            UpdateState(); return;
        }
        switch (address) {
            case 0x8000: _is2kBank = true; _bank = value; _mode |= 0x40; UpdateState(); break;
            case 0xB000: case 0xB0FF: case 0xB1FF: _bank = value; _mode |= 0x40; UpdateState(); break;
            case 0x8100: _mode = static_cast<uint8_t>(value | (_mode & 0x40)); UpdateState(); break;
            case 0x8200: _irqCounter = static_cast<uint16_t>((_irqCounter & 0xFF00) | value); SetIrq(false); break;
            case 0x8201: _irqEnabled = (_mode & 0x80) != 0; _irqCounter = static_cast<uint16_t>((_irqCounter & 0xFF) | (value << 8)); break;
        }
    }
};

class Unl103 final : public Board {
    bool _prgRamDisabled = false;
    uint8_t _prgReg = 0;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetWorkRamSize() override { return 0x4000; }
    uint32_t GetWorkRamPageSize() override { return 0x2000; }
    uint16_t RegisterStartAddress() override { return 0x6000; }
    void UpdateState() {
        SelectPrgPage4x(0, static_cast<uint16_t>(-4));
        if (_prgRamDisabled) SetCpuMemoryMapping(0x6000, 0x7FFF, _prgReg, PrgMemoryType::PrgRom);
        else {
            SetCpuMemoryMapping(0x6000, 0x7FFF, 0, PrgMemoryType::WorkRam);
            SetCpuMemoryMapping(0xB800, 0xD7FF, 1, PrgMemoryType::WorkRam);
        }
    }
    void InitMapper() override { _prgRamDisabled = false; _prgReg = 0; SelectChrPage(0, 0); UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0xF000) {
            case 0x6000: case 0x7000:
                if (static_cast<uint32_t>(address - 0x6000) < _workRamSize)
                    _workRam[address - 0x6000] = value;
                break;
            case 0x8000: _prgReg = value & 0x0F; UpdateState(); break;
            case 0xB000: case 0xC000: case 0xD000:
                if (address >= 0xB800 && address < 0xD800
                    && 0x2000u + address - 0xB800u < _workRamSize)
                    _workRam[0x2000 + address - 0xB800] = value;
                break;
            case 0xE000: SetMirroringType(value & 8 ? MirroringType::Horizontal : MirroringType::Vertical); break;
            case 0xF000: _prgRamDisabled = (value & 0x10) != 0; UpdateState(); break;
        }
    }
};

class Unl106 final : public Board {
    uint16_t _irqCounter = 0; bool _irqEnabled = false;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    bool EnableCpuClockHook() override { return true; }
    void InitMapper() override {
        _irqCounter = 0; _irqEnabled = false;
        for (unsigned i = 0; i < 4; ++i) SelectPrgPage(i, static_cast<uint16_t>(-1));
    }
    void ProcessCpuClock() override {
        if (_irqEnabled && ++_irqCounter == 0) { SetIrq(true); _irqEnabled = false; }
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address & 0x0F) {
            case 0: case 2: SelectChrPage(address & 0x0F, value & 0xFE); break;
            case 1: case 3: SelectChrPage(address & 0x0F, value | 1); break;
            case 4: case 5: case 6: case 7: SelectChrPage(address & 0x0F, value); break;
            case 8: case 0x0B: SelectPrgPage((address & 0x0F) - 8, (value & 0x0F) | 0x10); break;
            case 9: case 0x0A: SelectPrgPage((address & 0x0F) - 8, value & 0x1F); break;
            case 0x0D: _irqEnabled = false; _irqCounter = 0; SetIrq(false); break;
            case 0x0E: _irqCounter = static_cast<uint16_t>((_irqCounter & 0xFF00) | value); break;
            case 0x0F: _irqCounter = static_cast<uint16_t>((_irqCounter & 0xFF) | (value << 8)); _irqEnabled = true; break;
        }
    }
};

class Unl107 final : public Board {
    uint16_t GetPrgPageSize() override { return 0x8000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void InitMapper() override { SelectPrgPage(0, 0); SelectChrPage(0, 0); }
    void WriteRegister(uint16_t, uint8_t value) override { SelectPrgPage(0, value >> 1); SelectChrPage(0, value); }
};

class Unl108 final : public Board {
    uint8_t _prgReg = 0xFF, _chrReg = 0;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    void UpdateState() { SetCpuMemoryMapping(0x6000, 0x7FFF, _prgReg, PrgMemoryType::PrgRom); SelectChrPage(0, _chrReg); }
    void InitMapper() override { _prgReg = 0xFF; _chrReg = 0; SelectPrgPage4x(0, static_cast<uint16_t>(-4)); UpdateState(); }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if ((address & 0x9000) == 0x8000 || address >= 0xF000) _prgReg = _chrReg = value;
        else _chrReg = value & 1;
        UpdateState();
    }
};

class Unl116 final : public Board {
    uint8_t _mode = 0, _vrc2Chr[8]{}, _vrc2Prg[2]{}, _vrc2Mirroring = 0;
    uint8_t _mmc3Regs[10]{}, _mmc3Ctrl = 0, _mmc3Mirroring = 0;
    uint8_t _mmc1Regs[4]{}, _mmc1Buffer = 0, _mmc1Shift = 0;
    uint8_t _irqCounter = 0, _irqReloadValue = 0;
    bool _irqReload = false, _irqEnabled = false;
    A12Watcher _a12;

    uint16_t RegisterStartAddress() override { return 0x4100; }
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    bool EnableVramAddressHook() override { return true; }
    void UpdatePrg() {
        switch (_mode & 3) {
            case 0: SelectPrgPage(0,_vrc2Prg[0]); SelectPrgPage(1,_vrc2Prg[1]); SelectPrgPage(2,static_cast<uint16_t>(-2)); SelectPrgPage(3,static_cast<uint16_t>(-1)); break;
            case 1: { unsigned m = (_mmc3Ctrl >> 5) & 2; SelectPrgPage(0,_mmc3Regs[6+m]); SelectPrgPage(1,_mmc3Regs[7]); SelectPrgPage(2,_mmc3Regs[6+(m^2)]); SelectPrgPage(3,_mmc3Regs[9]); break; }
            default: { uint8_t bank = _mmc1Regs[3] & 0x0F; if (_mmc1Regs[0] & 8) { if (_mmc1Regs[0] & 4) { SelectPrgPage2x(0, bank << 1); SelectPrgPage2x(1, 0x1E); } else { SelectPrgPage2x(0,0); SelectPrgPage2x(1,bank << 1); } } else SelectPrgPage4x(0, (bank & 0x0E) << 1); break; }
        }
    }
    void UpdateChr() {
        uint16_t outer = static_cast<uint16_t>((_mode & 4) << 6);
        switch (_mode & 3) {
            case 0: for(unsigned i=0;i<8;++i) SelectChrPage(i, outer | _vrc2Chr[i]); break;
            case 1: { unsigned swap = (_mmc3Ctrl & 0x80) ? 4 : 0; SelectChrPage(0^swap,outer|(_mmc3Regs[0]&0xFE)); SelectChrPage(1^swap,outer|(_mmc3Regs[0]|1)); SelectChrPage(2^swap,outer|(_mmc3Regs[1]&0xFE)); SelectChrPage(3^swap,outer|(_mmc3Regs[1]|1)); for(unsigned i=2;i<6;++i) SelectChrPage((i+2)^swap,outer|_mmc3Regs[i]); break; }
            default: if (_mmc1Regs[0] & 0x10) { SelectChrPage4x(0,_mmc1Regs[1]<<2); SelectChrPage4x(1,_mmc1Regs[2]<<2); } else SelectChrPage8x(0,(_mmc1Regs[1]&0xFE)<<2); break;
        }
    }
    void UpdateMirroring() {
        if ((_mode & 3) == 0) SetMirroringType(_vrc2Mirroring & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
        else if ((_mode & 3) == 1) SetMirroringType(_mmc3Mirroring & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
        else switch(_mmc1Regs[0]&3){case 0:SetMirroringType(MirroringType::ScreenAOnly);break;case 1:SetMirroringType(MirroringType::ScreenBOnly);break;case 2:SetMirroringType(MirroringType::Vertical);break;default:SetMirroringType(MirroringType::Horizontal);break;}
    }
    void UpdateState(){UpdatePrg();UpdateChr();UpdateMirroring();}
    void InitMapper() override {
        _mode=0; uint8_t c[8]={0xFF,0xFF,0xFF,0xFF,4,5,6,7}; std::memcpy(_vrc2Chr,c,8); _vrc2Prg[0]=0;_vrc2Prg[1]=1;_vrc2Mirroring=0;
        uint8_t m[10]={0,2,4,5,6,7,0xFC,0xFD,0xFE,0xFF};std::memcpy(_mmc3Regs,m,10);_mmc3Ctrl=_mmc3Mirroring=0;_irqCounter=_irqReloadValue=0;_irqEnabled=_irqReload=false;
        _mmc1Regs[0]=0x0C;_mmc1Regs[1]=_mmc1Regs[2]=_mmc1Regs[3]=_mmc1Buffer=_mmc1Shift=0;_a12={};UpdateState();
    }
    void NotifyVramAddressChange(uint16_t address) override {
        if ((_mode & 3) != 1 || !_a12.Rising(address, PpuFrameCycle())) return;
        if (_irqCounter == 0 || _irqReload) _irqCounter = _irqReloadValue; else --_irqCounter;
        if (_irqCounter == 0 && _irqEnabled) SetIrq(true); _irqReload=false;
    }
    void WriteVrc2(uint16_t a,uint8_t v){if(a>=0xB000&&a<=0xE003){int i=((((a&2)|(a>>10))>>1)+2)&7;int s=(a&1)<<2;_vrc2Chr[i]=static_cast<uint8_t>((_vrc2Chr[i]&(0xF0>>s))|((v&0x0F)<<s));UpdateChr();}else switch(a&0xF000){case 0x8000:_vrc2Prg[0]=v;UpdatePrg();break;case 0xA000:_vrc2Prg[1]=v;UpdatePrg();break;case 0x9000:_vrc2Mirroring=v;UpdateMirroring();break;}}
    void WriteMmc3(uint16_t a,uint8_t v){switch(a&0xE001){case 0x8000:_mmc3Ctrl=v;UpdateState();break;case 0x8001:_mmc3Regs[_mmc3Ctrl&7]=v;UpdateState();break;case 0xA000:_mmc3Mirroring=v;UpdateState();break;case 0xC000:_irqReloadValue=v;break;case 0xC001:_irqReload=true;break;case 0xE000:SetIrq(false);_irqEnabled=false;break;case 0xE001:_irqEnabled=true;break;}}
    void WriteMmc1(uint16_t a,uint8_t v){if(v&0x80){_mmc1Regs[0]|=0x0C;_mmc1Buffer=_mmc1Shift=0;UpdateState();return;}uint8_t i=static_cast<uint8_t>((a>>13)-4);_mmc1Buffer|=static_cast<uint8_t>((v&1)<<_mmc1Shift++);if(_mmc1Shift==5){_mmc1Regs[i]=_mmc1Buffer;_mmc1Buffer=_mmc1Shift=0;UpdateState();}}
    void WriteRegister(uint16_t a,uint8_t v) override {if(a<0x8000){if((a&0x4100)==0x4100){_mode=v;if(a&1){_mmc1Regs[0]=0x0C;_mmc1Regs[3]=0;_mmc1Buffer=_mmc1Shift=0;}UpdateState();}}else switch(_mode&3){case 0:WriteVrc2(a,v);break;case 1:WriteMmc3(a,v);break;default:WriteMmc1(a,v);break;}}
};

class Unl117 final : public Board {
    uint8_t _irqCounter = 0, _irqReloadValue = 0;
    bool _irqEnabled = false, _irqEnabledAlt = false;
    A12Watcher _a12;
    uint16_t GetPrgPageSize() override { return 0x2000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    bool EnableVramAddressHook() override { return true; }
    void InitMapper() override { SelectPrgPage4x(0, static_cast<uint16_t>(-4)); }
    void NotifyVramAddressChange(uint16_t address) override {
        if (!_a12.Rising(address, PpuFrameCycle()) || !_irqEnabled || !_irqEnabledAlt || !_irqCounter) return;
        if (--_irqCounter == 0) { SetIrq(true); _irqEnabledAlt = false; }
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address >= 0x8000 && address <= 0x8003) SelectPrgPage(address & 3, value);
        else if (address >= 0xA000 && address <= 0xA007) SelectChrPage(address & 7, value);
        else switch (address) {
            case 0xC001: _irqReloadValue = value; break;
            case 0xC002: SetIrq(false); break;
            case 0xC003: _irqCounter = _irqReloadValue; _irqEnabledAlt = true; break;
            case 0xD000: SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical); break;
            case 0xE000: _irqEnabled = (value & 1) != 0; SetIrq(false); break;
        }
    }
};

class Unl120 final : public Board {
    uint8_t _prgReg=0;uint16_t RegisterStartAddress() override{return 0x41FF;}uint16_t RegisterEndAddress() override{return 0x41FF;}uint16_t GetPrgPageSize() override{return 0x2000;}uint16_t GetChrPageSize() override{return 0x2000;}
    void UpdatePrg(){SetCpuMemoryMapping(0x6000,0x7FFF,_prgReg,PrgMemoryType::PrgRom);}void InitMapper() override{_prgReg=0;UpdatePrg();SelectPrgPage4x(0,8);SelectChrPage(0,0);}void WriteRegister(uint16_t,uint8_t v) override{_prgReg=v;UpdatePrg();}
};

class Unl156 final : public Board {
    std::array<uint8_t, 8> _chrLow{}, _chrHigh{};
    uint16_t RegisterStartAddress() override { return 0xC000; }
    uint16_t RegisterEndAddress() override { return 0xC014; }
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x0400; }
    void InitMapper() override {
        SelectPrgPage(1, static_cast<uint16_t>(-1));
        SetMirroringType(MirroringType::ScreenAOnly);
    }
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address <= 0xC00F) {
            unsigned bank = (address & 3) + (address >= 0xC008 ? 4 : 0);
            (address & 4 ? _chrHigh : _chrLow)[bank] = value;
            for (unsigned slot = 0; slot < 8; ++slot)
                SelectChrPage(slot, (_chrHigh[slot] << 8) | _chrLow[slot]);
        } else if (address == 0xC010) SelectPrgPage(0, value);
        else if (address == 0xC014)
            SetMirroringType(value & 1 ? MirroringType::Horizontal : MirroringType::Vertical);
    }
};

class Unl163 final : public Board {
    uint8_t _regs[5]{};bool _toggle=true,_autoSwitch=false;uint16_t GetPrgPageSize() override{return 0x8000;}uint16_t GetChrPageSize() override{return 0x1000;}bool AllowRegisterRead() override{return true;}bool EnableVramAddressHook() override{return true;}uint16_t RegisterStartAddress() override{return 0x5000;}uint16_t RegisterEndAddress() override{return 0x5FFF;}
    void UpdateState(){uint8_t p=static_cast<uint8_t>((_regs[0]&0x0F)|((_regs[2]&0x0F)<<4));_autoSwitch=(_regs[0]&0x80)!=0;SelectPrgPage(0,p);}void InitMapper() override{std::memset(_regs,0,5);_toggle=true;_autoSwitch=false;SelectPrgPage(0,0);SelectChrPage(0,0);SelectChrPage(1,0);}
    void WriteRegister(uint16_t address, uint8_t value) override {
        if (address == 0x5101) {
            if (_regs[4] && !value) _toggle = !_toggle;
            _regs[4] = value;
            return;
        }
        if (address == 0x5100 && value == 6) { SelectPrgPage(0, 3); return; }
        switch (address & 0x7300) {
            case 0x5000: {
                _regs[0] = value;
                int scanline = static_cast<int>(PpuFrameCycle() / 341) - 1;
                if (!(_regs[0] & 0x80) && scanline < 128) { SelectChrPage(0, 0); SelectChrPage(1, 1); }
                UpdateState();
                break;
            }
            case 0x5100: _regs[1] = value; if (value == 6) SelectPrgPage(0, 3); break;
            case 0x5200: _regs[2] = value; UpdateState(); break;
            case 0x5300: _regs[3] = value; break;
        }
    }
    uint8_t ReadRegister(uint16_t a) override{switch(a&0x7700){case 0x5100:return static_cast<uint8_t>(_regs[3]|_regs[1]|_regs[0]|(_regs[2]^0xFF));case 0x5500:return _toggle?static_cast<uint8_t>(_regs[3]|_regs[0]):0;default:return 4;}}
    void NotifyVramAddressChange(uint16_t) override{if(!_autoSwitch||ppu.dot<=256)return;if(ppu.scanline==239){SelectChrPage(0,0);SelectChrPage(1,0);}else if(ppu.scanline==127){SelectChrPage(0,1);SelectChrPage(1,1);}}
};

} // namespace cupid::boards
#endif
