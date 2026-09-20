/*
 * studybox.hpp - StudyBox tape cartridge hardware and media parser
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
#ifndef CUPID_BOARDS_STUDYBOX_HPP
#define CUPID_BOARDS_STUDYBOX_HPP

#include "runtime.hpp"
#include <string>
#include <utility>

extern "C" {
#include "../../system/timing.h"
}

namespace cupid::boards {

struct StudyBoxPage {
    uint32_t LeadInOffset = 0;
    uint32_t AudioOffset = 0;
    std::vector<uint8_t> Data;
};

struct StudyBoxTape {
    std::vector<uint8_t> AudioFile;
    std::vector<StudyBoxPage> Pages;
};

inline uint32_t StudyBoxRead32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0])
         | (static_cast<uint32_t>(data[1]) << 8)
         | (static_cast<uint32_t>(data[2]) << 16)
         | (static_cast<uint32_t>(data[3]) << 24);
}

inline bool ParseStudyBoxTape(const uint8_t *data, size_t size, StudyBoxTape &out,
                              std::string &error) {
    out = StudyBoxTape{};
    if (!data || size < 16) {
        error = "file is too small";
        return false;
    }
    if (std::memcmp(data, "STBX", 4) != 0) {
        error = "invalid STBX signature";
        return false;
    }
    if (StudyBoxRead32(data + 4) != 4) {
        error = "invalid STBX header length";
        return false;
    }
    if (StudyBoxRead32(data + 8) != 0x100) {
        error = "unsupported STBX version";
        return false;
    }

    size_t offset = 12;
    uint32_t previousAudio = 0;
    uint32_t previousLeadIn = 0;
    while (offset + 4 <= size) {
        const uint8_t *tag = data + offset;
        offset += 4;
        if (std::memcmp(tag, "PAGE", 4) == 0) {
            if (offset + 12 > size) {
                error = "truncated PAGE chunk";
                return false;
            }
            uint32_t pageSize = StudyBoxRead32(data + offset);
            uint32_t leadIn = StudyBoxRead32(data + offset + 4);
            uint32_t audio = StudyBoxRead32(data + offset + 8);
            offset += 12;
            if (pageSize < 14) {
                error = "PAGE chunk is too small";
                return false;
            }
            size_t payload = static_cast<size_t>(pageSize - 8);
            if (payload > size - offset) {
                error = "PAGE chunk exceeds file size";
                return false;
            }
            if (audio < leadIn) {
                error = "PAGE audio offset precedes lead-in";
                return false;
            }
            if (audio < previousAudio || leadIn < previousLeadIn) {
                error = "PAGE chunks are out of tape order";
                return false;
            }
            previousAudio = audio;
            previousLeadIn = leadIn;
            StudyBoxPage page;
            page.LeadInOffset = leadIn;
            page.AudioOffset = audio;
            page.Data.assign(data + offset, data + offset + payload);
            out.Pages.push_back(std::move(page));
            offset += payload;
        } else if (std::memcmp(tag, "AUDI", 4) == 0) {
            if (offset + 8 > size) {
                error = "truncated AUDI chunk";
                return false;
            }
            uint32_t audioSize = StudyBoxRead32(data + offset);
            uint32_t fileType = StudyBoxRead32(data + offset + 4);
            offset += 8;
            if (audioSize < 4) {
                error = "AUDI chunk is too small";
                return false;
            }
            size_t payload = static_cast<size_t>(audioSize - 4);
            if (payload > size - offset) {
                error = "AUDI chunk exceeds file size";
                return false;
            }
            if (fileType != 0) {
                error = "unsupported STBX audio type";
                return false;
            }
            out.AudioFile.assign(data + offset, data + offset + payload);
            offset += payload;
            if (offset != size) {
                error = "AUDI chunk must be last";
                return false;
            }
            break;
        } else {
            error = "unsupported STBX chunk";
            return false;
        }
    }
    if (out.Pages.empty()) {
        error = "STBX image contains no PAGE chunks";
        return false;
    }
    return true;
}

class StudyBox final : public Board {
    StudyBoxTape _tapeData;

    bool _wavValid = false;
    uint32_t _audioSampleRate = 44100;
    uint32_t _audioDataOffset = 0;
    uint32_t _audioSampleCount = 0;
    uint32_t _audioPosition = 0;
    uint64_t _audioPhase = 0;
    bool _audioPlaying = false;

    bool _readyForBit = false;
    uint16_t _processBitDelay = 0;
    uint8_t _reg4202 = 0;
    uint8_t _commandCounter = 0;
    uint8_t _command = 0;
    uint8_t _currentPage = 0;
    int16_t _seekPage = 0;
    uint32_t _seekPageDelay = 0;
    bool _enableDecoder = false;
    bool _audioEnabled = false;
    bool _motorDisabled = true;
    uint16_t _byteReadDelay = 0;
    bool _irqEnabled = false;
    bool _pageFound = false;
    int32_t _pageIndex = 0;
    int32_t _pagePosition = -1;
    uint32_t _inDataDelay = 0;
    bool _inDataRegion = false;

    static uint16_t Read16(const uint8_t *data) {
        return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
    }

    void DecodeWav() {
        _wavValid = false;
        if (_tapeData.AudioFile.size() < 100) return;
        const uint8_t *wav = _tapeData.AudioFile.data();
        size_t length = _tapeData.AudioFile.size();
        if (std::memcmp(wav, "RIFF", 4) || std::memcmp(wav + 8, "WAVE", 4)
            || std::memcmp(wav + 12, "fmt ", 4)) return;
        if (StudyBoxRead32(wav + 4) + 8u != length) return;
        uint32_t fmtSize = StudyBoxRead32(wav + 16);
        if (fmtSize > 50 || 20u + fmtSize + 8u > length) return;
        if (std::memcmp(wav + 20 + fmtSize, "data", 4)) return;
        uint32_t dataSize = StudyBoxRead32(wav + 24 + fmtSize);
        uint32_t headerSize = 28 + fmtSize;
        if (headerSize > length || dataSize > length - headerSize) return;
        uint16_t channels = Read16(wav + 22);
        uint16_t bits = Read16(wav + 34);
        uint32_t sampleRate = StudyBoxRead32(wav + 24);
        if (channels != 1 || bits != 16 || !sampleRate || (dataSize & 1u)) return;
        _audioSampleRate = sampleRate;
        _audioDataOffset = headerSize;
        _audioSampleCount = dataSize / 2;
        _wavValid = true;
    }

    int16_t CurrentAudioSample() const {
        if (!_wavValid || !_audioPlaying || _audioPosition >= _audioSampleCount) return 0;
        size_t offset = static_cast<size_t>(_audioDataOffset) + static_cast<size_t>(_audioPosition) * 2;
        if (offset + 1 >= _tapeData.AudioFile.size()) return 0;
        return static_cast<int16_t>(Read16(_tapeData.AudioFile.data() + offset));
    }

    void PlayAudio(uint32_t startSample) {
        _audioPosition = startSample;
        _audioPhase = 0;
        _audioPlaying = _wavValid && startSample < _audioSampleCount;
    }

    void AdvanceAudio() {
        if (_motorDisabled || !_audioPlaying || !_wavValid) return;
        uint32_t cpuRate = static_cast<uint32_t>(nes_timing()->cpu_hz + 0.5);
        _audioPhase += _audioSampleRate;
        while (_audioPhase >= cpuRate && _audioPlaying) {
            _audioPhase -= cpuRate;
            if (++_audioPosition >= _audioSampleCount) _audioPlaying = false;
        }
    }

    bool PageIndexValid() const {
        return _pageIndex >= 0 && static_cast<size_t>(_pageIndex) < _tapeData.Pages.size();
    }

    void ReadLeadInTrack() {
        if (!PageIndexValid()) {
            _motorDisabled = true;
            _pageFound = false;
            return;
        }
        const StudyBoxPage &page = _tapeData.Pages[static_cast<size_t>(_pageIndex)];
        uint64_t delay = static_cast<uint64_t>(page.AudioOffset - page.LeadInOffset)
                       * static_cast<uint64_t>(nes_timing()->cpu_hz + 0.5)
                       / _audioSampleRate;
        _inDataDelay = delay > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(delay);
        _pagePosition = -1;
        _byteReadDelay = 0;
        _motorDisabled = false;
        _pageFound = true;
    }

    uint16_t RegisterStartAddress() override { return 0x4200; }
    uint16_t RegisterEndAddress() override { return 0x4203; }
    bool AllowRegisterRead() override { return true; }
    bool EnableCpuClockHook() override { return true; }
    uint16_t GetPrgPageSize() override { return 0x4000; }
    uint16_t GetChrPageSize() override { return 0x2000; }
    uint32_t GetWorkRamSize() override { return 0x10000; }
    uint32_t GetWorkRamPageSize() override { return 0x1000; }
    uint32_t GetNametableCount() override { return 4; }

    void InitMapper() override {
        DecodeWav();
        SelectPrgPage(1, 0);
        SelectChrPage(0, 0);
        SetCpuMemoryMapping(0x4000, 0x4FFF, 8, PrgMemoryType::WorkRam);
        RemoveCpuMemoryMapping(0x4000, 0x43FF);
        SetMirroringType(MirroringType::FourScreens);
    }

    void Reset(bool softReset) override {
        if (!softReset) return;
        _readyForBit = false;
        _processBitDelay = 0;
        _reg4202 = 0;
        _commandCounter = 0;
        _command = 0;
        _currentPage = 0;
        _seekPage = 0;
        _seekPageDelay = 0;
        _enableDecoder = false;
        _audioEnabled = false;
        _motorDisabled = true;
        _byteReadDelay = 0;
        _irqEnabled = false;
        _pageFound = false;
        _pageIndex = 0;
        _pagePosition = -1;
        _inDataDelay = 0;
        _inDataRegion = false;
        _audioPosition = 0;
        _audioPhase = 0;
        _audioPlaying = false;
        SetIrq(false);
    }

    void ProcessCpuClock() override {
        AdvanceAudio();
        if (_processBitDelay) {
            if (--_processBitDelay == 0) _readyForBit = true;
        }
        if (_motorDisabled) return;

        if (_seekPage != _currentPage) {
            if (_seekPageDelay && --_seekPageDelay == 0) {
                _seekPageDelay = 3000000;
                if (_seekPage > _currentPage) _currentPage++;
                else _currentPage--;

                _pageIndex = 0;
                for (size_t i = 0; i < _tapeData.Pages.size(); ++i) {
                    if (_tapeData.Pages[i].Data[5] == _currentPage - 1) {
                        _pageIndex = static_cast<int32_t>(i);
                        break;
                    }
                }
                ReadLeadInTrack();
            }
        } else if (_inDataDelay) {
            _inDataRegion = true;
            if (--_inDataDelay == 0) {
                _byteReadDelay = 7820;
                if (PageIndexValid())
                    PlayAudio(_tapeData.Pages[static_cast<size_t>(_pageIndex)].AudioOffset);
                SetIrq(true);
            }
        } else if (_byteReadDelay) {
            if (--_byteReadDelay == 0) {
                _byteReadDelay = 3355;
                _pagePosition++;
                if (!PageIndexValid()
                    || _pagePosition >= static_cast<int32_t>(_tapeData.Pages[static_cast<size_t>(_pageIndex)].Data.size())) {
                    _pageFound = false;
                    _inDataRegion = false;
                    _motorDisabled = true;
                }
                if (_irqEnabled) SetIrq(true);
            }
        }
    }

    uint8_t ReadRegister(uint16_t address) override {
        switch (address) {
            case 0x4200:
                SetIrq(false);
                if (PageIndexValid() && _pagePosition >= 0
                    && _pagePosition < static_cast<int32_t>(_tapeData.Pages[static_cast<size_t>(_pageIndex)].Data.size()))
                    return _tapeData.Pages[static_cast<size_t>(_pageIndex)].Data[static_cast<size_t>(_pagePosition)];
                return 0xAA;
            case 0x4201: {
                uint8_t value = static_cast<uint8_t>((_inDataRegion ? 0x20 : 0)
                    | (_pageFound ? 0x40 : 0) | (_enableDecoder ? 0x80 : 0));
                _pageFound = false;
                return value;
            }
            case 0x4202:
                return _readyForBit ? 0x40 : 0;
            case 0x4203:
                return 0;
            default:
                return 0;
        }
    }

    void WriteRegister(uint16_t address, uint8_t value) override {
        switch (address) {
            case 0x4200:
                SetCpuMemoryMapping(0x6000, 0x6FFF, (value & 0xC0) >> 5, PrgMemoryType::WorkRam);
                SetCpuMemoryMapping(0x7000, 0x7FFF, ((value & 0xC0) >> 5) + 1, PrgMemoryType::WorkRam);
                SetCpuMemoryMapping(0x5000, 0x5FFF, (value & 0x07) + 8, PrgMemoryType::WorkRam);
                break;
            case 0x4201:
                SelectPrgPage(0, value);
                break;
            case 0x4202:
                if ((_reg4202 & 0x10) && !(value & 0x10)) {
                    _command = static_cast<uint8_t>((_command << 1) | ((value & 0x80) >> 7));
                    if (++_commandCounter == 8) {
                        _commandCounter = 0;
                        if (_command >= 1 && _command < 0x40) {
                            _seekPage = static_cast<int16_t>(_currentPage + _command);
                            _seekPageDelay = 3000000;
                            _motorDisabled = false;
                        } else if (_command > 0x40 && _command < 0x80) {
                            _seekPage = static_cast<int16_t>(_currentPage - (_command - 0x40));
                            _seekPageDelay = 3000000;
                            _motorDisabled = false;
                        } else if (_command == 0) {
                            _seekPage = _currentPage;
                            _currentPage--;
                            _seekPageDelay = 3000000;
                            _motorDisabled = false;
                        } else if (_command == 0x86) {
                            if (_pageIndex < static_cast<int32_t>(_tapeData.Pages.size()) - 1) _pageIndex++;
                            else _pageIndex = 0;
                            ReadLeadInTrack();
                        }
                    }
                }
                if (value & 0x10) {
                    _readyForBit = false;
                    _processBitDelay = 100;
                }
                if ((_reg4202 & 0x20) && !(value & 0x20)) {
                    _command = 0;
                    _commandCounter = 0;
                    _readyForBit = true;
                }
                if ((value & 0x04) != (_reg4202 & 0x04))
                    _audioEnabled = (value & 0x04) == 0;
                _reg4202 = value;
                _enableDecoder = (value & 0x01) != 0;
                _irqEnabled = (value & 0x02) != 0;
                SetIrq(false);
                break;
            case 0x4203:
                break;
        }
    }

public:
    explicit StudyBox(StudyBoxTape tape) : _tapeData(std::move(tape)) {}

    bool VisitState(BoardStateVisitor &state) override {
        if (!Board::VisitState(state)
            || !state.InvariantU32("studybox.page_count",
                                   static_cast<uint32_t>(_tapeData.Pages.size()))
            || !state.InvariantBytes("studybox.audio_file", _tapeData.AudioFile.data(),
                                     _tapeData.AudioFile.size()))
            return false;
        for (size_t i = 0; i < _tapeData.Pages.size(); ++i) {
            char name[64];
            std::snprintf(name, sizeof(name), "studybox.page.%zu.lead_in", i);
            if (!state.InvariantU32(name, _tapeData.Pages[i].LeadInOffset)) return false;
            std::snprintf(name, sizeof(name), "studybox.page.%zu.audio_offset", i);
            if (!state.InvariantU32(name, _tapeData.Pages[i].AudioOffset)) return false;
            std::snprintf(name, sizeof(name), "studybox.page.%zu.data", i);
            if (!state.InvariantBytes(name, _tapeData.Pages[i].Data.data(),
                                      _tapeData.Pages[i].Data.size()))
                return false;
        }
        return state.Field("studybox.wav_valid", _wavValid)
            && state.Field("studybox.audio_sample_rate", _audioSampleRate)
            && state.Field("studybox.audio_data_offset", _audioDataOffset)
            && state.Field("studybox.audio_sample_count", _audioSampleCount)
            && state.Field("studybox.audio_position", _audioPosition)
            && state.Field("studybox.audio_phase", _audioPhase)
            && state.Field("studybox.audio_playing", _audioPlaying)
            && state.Field("studybox.ready_for_bit", _readyForBit)
            && state.Field("studybox.process_bit_delay", _processBitDelay)
            && state.Field("studybox.reg4202", _reg4202)
            && state.Field("studybox.command_counter", _commandCounter, 7)
            && state.Field("studybox.command", _command)
            && state.Field("studybox.current_page", _currentPage)
            && state.Field("studybox.seek_page", _seekPage)
            && state.Field("studybox.seek_page_delay", _seekPageDelay)
            && state.Field("studybox.enable_decoder", _enableDecoder)
            && state.Field("studybox.audio_enabled", _audioEnabled)
            && state.Field("studybox.motor_disabled", _motorDisabled)
            && state.Field("studybox.byte_read_delay", _byteReadDelay)
            && state.Field("studybox.irq_enabled", _irqEnabled)
            && state.Field("studybox.page_found", _pageFound)
            && state.Field("studybox.page_index", _pageIndex)
            && state.Field("studybox.page_position", _pagePosition)
            && state.Field("studybox.in_data_delay", _inDataDelay)
            && state.Field("studybox.in_data_region", _inDataRegion);
    }

    float AudioOutput() const override {
        (void)_audioEnabled;
        if (_motorDisabled || !_audioPlaying) return 0.0f;
        return static_cast<float>(CurrentAudioSample()) / 32768.0f;
    }
};

} // namespace cupid::boards
#endif
