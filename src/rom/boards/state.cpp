/*
 * state.cpp - Transactional cartridge board save-state codec
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "board_state.h"

#include "board_internal.hpp"
#include "state_codec.hpp"

#include <cassert>
#include <cstring>
#include <new>
#include <vector>

namespace cupid::boards {
namespace {

enum class WireType : uint8_t {
    U8 = 1,
    U16,
    U32,
    U64,
    I16,
    I32,
    Bool,
    Bytes,
    String,
    Hash64
};

uint64_t HashBytes(const uint8_t *bytes, size_t size) {
    uint64_t hash = 1469598103934665603ull;
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

bool WriteHeader(NesStateWriter *writer, const char *name, WireType type, size_t count) {
    if (!writer || !name) return false;
    size_t length = std::strlen(name);
    if (length > UINT16_MAX) return false;
    return nes_state_write_u16(writer, static_cast<uint16_t>(length))
        && nes_state_write_bytes(writer, name, length)
        && nes_state_write_u8(writer, static_cast<uint8_t>(type))
        && nes_state_write_u64(writer, static_cast<uint64_t>(count));
}

bool ReadHeader(NesStateReader *reader, const char *name, WireType type, size_t count) {
    uint16_t length = 0;
    uint8_t savedType = 0;
    uint64_t savedCount = 0;
    size_t expectedLength = name ? std::strlen(name) : 0;
    if (!reader || !name || expectedLength > UINT16_MAX
        || !nes_state_read_u16(reader, &length)
        || length != expectedLength
        || length > nes_state_reader_remaining(reader)
        || std::memcmp(reader->data + reader->offset, name, length) != 0) {
        if (reader) reader->failed = true;
        return false;
    }
    reader->offset += length;
    if (!nes_state_read_u8(reader, &savedType)
        || !nes_state_read_u64(reader, &savedCount)
        || savedType != static_cast<uint8_t>(type)
        || savedCount != count) {
        reader->failed = true;
        return false;
    }
    return true;
}

class CaptureVisitor final : public BoardStateVisitor {
    NesStateWriter *_writer;
    NesStateResult _result = NES_STATE_OK;

    bool Fail(NesStateResult result) {
        if (_result == NES_STATE_OK) _result = result;
        return false;
    }

public:
    explicit CaptureVisitor(NesStateWriter *writer) : _writer(writer) {}
    Mode GetMode() const override { return Mode::Capture; }
    NesStateResult Result() const { return _result; }

    bool U8(const char *name, uint8_t &value, uint8_t maximum) override {
        if (value > maximum) return Fail(NES_STATE_ERROR_CORRUPT);
        return (WriteHeader(_writer, name, WireType::U8, 1)
            && nes_state_write_u8(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool U16(const char *name, uint16_t &value, uint16_t maximum) override {
        if (value > maximum) return Fail(NES_STATE_ERROR_CORRUPT);
        return (WriteHeader(_writer, name, WireType::U16, 1)
            && nes_state_write_u16(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool U32(const char *name, uint32_t &value, uint32_t maximum) override {
        if (value > maximum) return Fail(NES_STATE_ERROR_CORRUPT);
        return (WriteHeader(_writer, name, WireType::U32, 1)
            && nes_state_write_u32(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool U64(const char *name, uint64_t &value) override {
        return (WriteHeader(_writer, name, WireType::U64, 1)
            && nes_state_write_u64(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool I16(const char *name, int16_t &value, int16_t minimum, int16_t maximum) override {
        if (value < minimum || value > maximum) return Fail(NES_STATE_ERROR_CORRUPT);
        return (WriteHeader(_writer, name, WireType::I16, 1)
            && nes_state_write_u16(_writer, static_cast<uint16_t>(value)))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool I32(const char *name, int32_t &value, int32_t minimum, int32_t maximum) override {
        if (value < minimum || value > maximum) return Fail(NES_STATE_ERROR_CORRUPT);
        return (WriteHeader(_writer, name, WireType::I32, 1)
            && nes_state_write_u32(_writer, static_cast<uint32_t>(value)))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool Bool(const char *name, bool &value) override {
        return (WriteHeader(_writer, name, WireType::Bool, 1)
            && nes_state_write_u8(_writer, value ? 1 : 0))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool ValueU8(const char *name, uint8_t &value, uint8_t maximum) override {
        return U8(name, value, maximum);
    }
    bool ValueU32(const char *name, uint32_t &value, uint32_t maximum) override {
        return U32(name, value, maximum);
    }

    bool U8Array(const char *name, uint8_t *values, size_t count, uint8_t maximum) override {
        if ((!values && count) || !WriteHeader(_writer, name, WireType::U8, count))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        for (size_t i = 0; i < count; ++i) {
            if (values[i] > maximum) return Fail(NES_STATE_ERROR_CORRUPT);
        }
        return nes_state_write_bytes(_writer, values, count)
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool U16Array(const char *name, uint16_t *values, size_t count, uint16_t maximum) override {
        if ((!values && count) || !WriteHeader(_writer, name, WireType::U16, count))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        for (size_t i = 0; i < count; ++i) {
            if (values[i] > maximum || !nes_state_write_u16(_writer, values[i]))
                return Fail(values[i] > maximum ? NES_STATE_ERROR_CORRUPT
                                                : NES_STATE_ERROR_OUT_OF_MEMORY);
        }
        return true;
    }
    bool U32Array(const char *name, uint32_t *values, size_t count, uint32_t maximum) override {
        if ((!values && count) || !WriteHeader(_writer, name, WireType::U32, count))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        for (size_t i = 0; i < count; ++i) {
            if (values[i] > maximum || !nes_state_write_u32(_writer, values[i]))
                return Fail(values[i] > maximum ? NES_STATE_ERROR_CORRUPT
                                                : NES_STATE_ERROR_OUT_OF_MEMORY);
        }
        return true;
    }
    bool BoolArray(const char *name, bool *values, size_t count) override {
        if ((!values && count) || !WriteHeader(_writer, name, WireType::Bool, count))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        for (size_t i = 0; i < count; ++i) {
            if (!nes_state_write_u8(_writer, values[i] ? 1 : 0))
                return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        }
        return true;
    }
    bool Bytes(const char *name, uint8_t *values, size_t count) override {
        if ((!values && count) || !WriteHeader(_writer, name, WireType::Bytes, count)
            || !nes_state_write_bytes(_writer, values, count))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        return true;
    }

    bool InvariantU8(const char *name, uint8_t value) override {
        return (WriteHeader(_writer, name, WireType::U8, 1)
            && nes_state_write_u8(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool InvariantU16(const char *name, uint16_t value) override {
        return (WriteHeader(_writer, name, WireType::U16, 1)
            && nes_state_write_u16(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool InvariantU32(const char *name, uint32_t value) override {
        return (WriteHeader(_writer, name, WireType::U32, 1)
            && nes_state_write_u32(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool InvariantU64(const char *name, uint64_t value) override {
        return (WriteHeader(_writer, name, WireType::U64, 1)
            && nes_state_write_u64(_writer, value))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool InvariantBool(const char *name, bool value) override {
        return (WriteHeader(_writer, name, WireType::Bool, 1)
            && nes_state_write_u8(_writer, value ? 1 : 0))
            || Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
    }
    bool InvariantString(const char *name, const std::string &value) override {
        if (!WriteHeader(_writer, name, WireType::String, value.size())
            || !nes_state_write_bytes(_writer, value.data(), value.size()))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        return true;
    }
    bool InvariantBytes(const char *name, const uint8_t *values, size_t count) override {
        if ((!values && count) || !WriteHeader(_writer, name, WireType::Hash64, count)
            || !nes_state_write_u64(_writer, HashBytes(values, count)))
            return Fail(NES_STATE_ERROR_OUT_OF_MEMORY);
        return true;
    }
};

class ValidateVisitor final : public BoardStateVisitor {
    NesStateReader *_reader;
    NesStateResult _result = NES_STATE_OK;

    bool Fail(NesStateResult result) {
        if (_result == NES_STATE_OK) _result = result;
        return false;
    }

    template <typename T>
    bool Range(T value, T minimum, T maximum) {
        return value >= minimum && value <= maximum;
    }

public:
    explicit ValidateVisitor(NesStateReader *reader) : _reader(reader) {}
    Mode GetMode() const override { return Mode::Validate; }
    NesStateResult Result() const { return _result; }

    bool U8(const char *name, uint8_t &, uint8_t maximum) override {
        uint8_t value = 0;
        if (!ReadHeader(_reader, name, WireType::U8, 1) || !nes_state_read_u8(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return value <= maximum || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool U16(const char *name, uint16_t &, uint16_t maximum) override {
        uint16_t value = 0;
        if (!ReadHeader(_reader, name, WireType::U16, 1) || !nes_state_read_u16(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return value <= maximum || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool U32(const char *name, uint32_t &, uint32_t maximum) override {
        uint32_t value = 0;
        if (!ReadHeader(_reader, name, WireType::U32, 1) || !nes_state_read_u32(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return value <= maximum || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool U64(const char *name, uint64_t &) override {
        uint64_t value = 0;
        if (!ReadHeader(_reader, name, WireType::U64, 1) || !nes_state_read_u64(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return true;
    }
    bool I16(const char *name, int16_t &, int16_t minimum, int16_t maximum) override {
        uint16_t raw = 0;
        if (!ReadHeader(_reader, name, WireType::I16, 1) || !nes_state_read_u16(_reader, &raw))
            return Fail(NES_STATE_ERROR_CORRUPT);
        int16_t value = static_cast<int16_t>(raw);
        return Range(value, minimum, maximum) || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool I32(const char *name, int32_t &, int32_t minimum, int32_t maximum) override {
        uint32_t raw = 0;
        if (!ReadHeader(_reader, name, WireType::I32, 1) || !nes_state_read_u32(_reader, &raw))
            return Fail(NES_STATE_ERROR_CORRUPT);
        int32_t value = static_cast<int32_t>(raw);
        return Range(value, minimum, maximum) || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool Bool(const char *name, bool &) override {
        uint8_t value = 0;
        if (!ReadHeader(_reader, name, WireType::Bool, 1) || !nes_state_read_u8(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return value <= 1 || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool ValueU8(const char *name, uint8_t &value, uint8_t maximum) override {
        if (!ReadHeader(_reader, name, WireType::U8, 1) || !nes_state_read_u8(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return value <= maximum || Fail(NES_STATE_ERROR_CORRUPT);
    }
    bool ValueU32(const char *name, uint32_t &value, uint32_t maximum) override {
        if (!ReadHeader(_reader, name, WireType::U32, 1) || !nes_state_read_u32(_reader, &value))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return value <= maximum || Fail(NES_STATE_ERROR_CORRUPT);
    }

    bool U8Array(const char *name, uint8_t *, size_t count, uint8_t maximum) override {
        if (!ReadHeader(_reader, name, WireType::U8, count)) return Fail(NES_STATE_ERROR_CORRUPT);
        for (size_t i = 0; i < count; ++i) {
            uint8_t value = 0;
            if (!nes_state_read_u8(_reader, &value) || value > maximum)
                return Fail(NES_STATE_ERROR_CORRUPT);
        }
        return true;
    }
    bool U16Array(const char *name, uint16_t *, size_t count, uint16_t maximum) override {
        if (!ReadHeader(_reader, name, WireType::U16, count)) return Fail(NES_STATE_ERROR_CORRUPT);
        for (size_t i = 0; i < count; ++i) {
            uint16_t value = 0;
            if (!nes_state_read_u16(_reader, &value) || value > maximum)
                return Fail(NES_STATE_ERROR_CORRUPT);
        }
        return true;
    }
    bool U32Array(const char *name, uint32_t *, size_t count, uint32_t maximum) override {
        if (!ReadHeader(_reader, name, WireType::U32, count)) return Fail(NES_STATE_ERROR_CORRUPT);
        for (size_t i = 0; i < count; ++i) {
            uint32_t value = 0;
            if (!nes_state_read_u32(_reader, &value) || value > maximum)
                return Fail(NES_STATE_ERROR_CORRUPT);
        }
        return true;
    }
    bool BoolArray(const char *name, bool *, size_t count) override {
        if (!ReadHeader(_reader, name, WireType::Bool, count)) return Fail(NES_STATE_ERROR_CORRUPT);
        for (size_t i = 0; i < count; ++i) {
            uint8_t value = 0;
            if (!nes_state_read_u8(_reader, &value) || value > 1)
                return Fail(NES_STATE_ERROR_CORRUPT);
        }
        return true;
    }
    bool Bytes(const char *name, uint8_t *, size_t count) override {
        if (!ReadHeader(_reader, name, WireType::Bytes, count)
            || count > nes_state_reader_remaining(_reader))
            return Fail(NES_STATE_ERROR_CORRUPT);
        _reader->offset += count;
        return true;
    }

    bool InvariantU8(const char *name, uint8_t live) override {
        uint8_t saved = 0;
        if (!ReadHeader(_reader, name, WireType::U8, 1) || !nes_state_read_u8(_reader, &saved))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return saved == live || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
    bool InvariantU16(const char *name, uint16_t live) override {
        uint16_t saved = 0;
        if (!ReadHeader(_reader, name, WireType::U16, 1) || !nes_state_read_u16(_reader, &saved))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return saved == live || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
    bool InvariantU32(const char *name, uint32_t live) override {
        uint32_t saved = 0;
        if (!ReadHeader(_reader, name, WireType::U32, 1) || !nes_state_read_u32(_reader, &saved))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return saved == live || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
    bool InvariantU64(const char *name, uint64_t live) override {
        uint64_t saved = 0;
        if (!ReadHeader(_reader, name, WireType::U64, 1) || !nes_state_read_u64(_reader, &saved))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return saved == live || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
    bool InvariantBool(const char *name, bool live) override {
        uint8_t saved = 0;
        if (!ReadHeader(_reader, name, WireType::Bool, 1) || !nes_state_read_u8(_reader, &saved)
            || saved > 1)
            return Fail(NES_STATE_ERROR_CORRUPT);
        return (saved != 0) == live || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
    bool InvariantString(const char *name, const std::string &live) override {
        if (!ReadHeader(_reader, name, WireType::String, live.size())
            || live.size() > nes_state_reader_remaining(_reader))
            return Fail(NES_STATE_ERROR_CORRUPT);
        bool equal = std::memcmp(_reader->data + _reader->offset, live.data(), live.size()) == 0;
        _reader->offset += live.size();
        return equal || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
    bool InvariantBytes(const char *name, const uint8_t *live, size_t count) override {
        uint64_t saved = 0;
        if ((!live && count) || !ReadHeader(_reader, name, WireType::Hash64, count)
            || !nes_state_read_u64(_reader, &saved))
            return Fail(NES_STATE_ERROR_CORRUPT);
        return saved == HashBytes(live, count) || Fail(NES_STATE_ERROR_INCOMPATIBLE);
    }
};

class ApplyVisitor final : public BoardStateVisitor {
    NesStateReader *_reader;
    bool _ok = true;

    bool Header(const char *name, WireType type, size_t count) {
        _ok = _ok && ReadHeader(_reader, name, type, count);
        return _ok;
    }

public:
    explicit ApplyVisitor(NesStateReader *reader) : _reader(reader) {}
    Mode GetMode() const override { return Mode::Apply; }
    bool Ok() const { return _ok && !_reader->failed; }

    bool U8(const char *name, uint8_t &value, uint8_t) override {
        return Header(name, WireType::U8, 1) && (_ok = nes_state_read_u8(_reader, &value));
    }
    bool U16(const char *name, uint16_t &value, uint16_t) override {
        return Header(name, WireType::U16, 1) && (_ok = nes_state_read_u16(_reader, &value));
    }
    bool U32(const char *name, uint32_t &value, uint32_t) override {
        return Header(name, WireType::U32, 1) && (_ok = nes_state_read_u32(_reader, &value));
    }
    bool U64(const char *name, uint64_t &value) override {
        return Header(name, WireType::U64, 1) && (_ok = nes_state_read_u64(_reader, &value));
    }
    bool I16(const char *name, int16_t &value, int16_t, int16_t) override {
        uint16_t raw = 0;
        if (!Header(name, WireType::I16, 1) || !nes_state_read_u16(_reader, &raw)) return _ok = false;
        value = static_cast<int16_t>(raw);
        return true;
    }
    bool I32(const char *name, int32_t &value, int32_t, int32_t) override {
        uint32_t raw = 0;
        if (!Header(name, WireType::I32, 1) || !nes_state_read_u32(_reader, &raw)) return _ok = false;
        value = static_cast<int32_t>(raw);
        return true;
    }
    bool Bool(const char *name, bool &value) override {
        uint8_t raw = 0;
        if (!Header(name, WireType::Bool, 1) || !nes_state_read_u8(_reader, &raw)) return _ok = false;
        value = raw != 0;
        return true;
    }
    bool ValueU8(const char *name, uint8_t &value, uint8_t maximum) override {
        return U8(name, value, maximum);
    }
    bool ValueU32(const char *name, uint32_t &value, uint32_t maximum) override {
        return U32(name, value, maximum);
    }

    bool U8Array(const char *name, uint8_t *values, size_t count, uint8_t) override {
        return Header(name, WireType::U8, count)
            && (_ok = nes_state_read_bytes(_reader, values, count));
    }
    bool U16Array(const char *name, uint16_t *values, size_t count, uint16_t) override {
        if (!Header(name, WireType::U16, count)) return false;
        for (size_t i = 0; i < count; ++i) {
            if (!nes_state_read_u16(_reader, &values[i])) return _ok = false;
        }
        return true;
    }
    bool U32Array(const char *name, uint32_t *values, size_t count, uint32_t) override {
        if (!Header(name, WireType::U32, count)) return false;
        for (size_t i = 0; i < count; ++i) {
            if (!nes_state_read_u32(_reader, &values[i])) return _ok = false;
        }
        return true;
    }
    bool BoolArray(const char *name, bool *values, size_t count) override {
        if (!Header(name, WireType::Bool, count)) return false;
        for (size_t i = 0; i < count; ++i) {
            uint8_t raw = 0;
            if (!nes_state_read_u8(_reader, &raw)) return _ok = false;
            values[i] = raw != 0;
        }
        return true;
    }
    bool Bytes(const char *name, uint8_t *values, size_t count) override {
        return Header(name, WireType::Bytes, count)
            && (_ok = nes_state_read_bytes(_reader, values, count));
    }

    bool InvariantU8(const char *name, uint8_t) override {
        uint8_t value = 0;
        return Header(name, WireType::U8, 1) && (_ok = nes_state_read_u8(_reader, &value));
    }
    bool InvariantU16(const char *name, uint16_t) override {
        uint16_t value = 0;
        return Header(name, WireType::U16, 1) && (_ok = nes_state_read_u16(_reader, &value));
    }
    bool InvariantU32(const char *name, uint32_t) override {
        uint32_t value = 0;
        return Header(name, WireType::U32, 1) && (_ok = nes_state_read_u32(_reader, &value));
    }
    bool InvariantU64(const char *name, uint64_t) override {
        uint64_t value = 0;
        return Header(name, WireType::U64, 1) && (_ok = nes_state_read_u64(_reader, &value));
    }
    bool InvariantBool(const char *name, bool) override {
        uint8_t value = 0;
        return Header(name, WireType::Bool, 1) && (_ok = nes_state_read_u8(_reader, &value));
    }
    bool InvariantString(const char *name, const std::string &live) override {
        if (!Header(name, WireType::String, live.size())
            || live.size() > nes_state_reader_remaining(_reader))
            return _ok = false;
        _reader->offset += live.size();
        return true;
    }
    bool InvariantBytes(const char *name, const uint8_t *, size_t count) override {
        uint64_t hash = 0;
        return Header(name, WireType::Hash64, count)
            && (_ok = nes_state_read_u64(_reader, &hash));
    }
};

} // namespace
} // namespace cupid::boards

struct BoardStateRestore {
    std::vector<uint8_t> payload;
    const CartridgeBoard *validatedBoard = nullptr;
};

NesStateResult board_state_capture(const CartridgeBoard *board, NesStateWriter *writer) {
    if (!board || !board->instance || !writer) return NES_STATE_ERROR_ARGUMENT;
    cupid::boards::CaptureVisitor visitor(writer);
    if (!const_cast<cupid::boards::Board *>(board->instance.get())->VisitState(visitor))
        return visitor.Result() == NES_STATE_OK ? NES_STATE_ERROR_CORRUPT : visitor.Result();
    return visitor.Result();
}

NesStateResult board_state_validate(const CartridgeBoard *board, NesStateReader *reader,
                                    BoardStateRestore **outRestore) {
    if (!board || !board->instance || !reader || !outRestore) return NES_STATE_ERROR_ARGUMENT;
    *outRestore = nullptr;

    NesStateReader probe = *reader;
    size_t start = probe.offset;
    cupid::boards::ValidateVisitor visitor(&probe);
    if (!const_cast<cupid::boards::Board *>(board->instance.get())->VisitState(visitor)) {
        return visitor.Result() == NES_STATE_OK ? NES_STATE_ERROR_CORRUPT : visitor.Result();
    }
    if (visitor.Result() != NES_STATE_OK) return visitor.Result();
    if (probe.failed || nes_state_reader_remaining(&probe) != 0) return NES_STATE_ERROR_CORRUPT;

    BoardStateRestore *restore = new (std::nothrow) BoardStateRestore();
    if (!restore) return NES_STATE_ERROR_OUT_OF_MEMORY;
    try {
        restore->payload.assign(reader->data + start, reader->data + probe.offset);
    } catch (const std::bad_alloc &) {
        delete restore;
        return NES_STATE_ERROR_OUT_OF_MEMORY;
    }
    restore->validatedBoard = board;
    *reader = probe;
    *outRestore = restore;
    return NES_STATE_OK;
}

void board_state_apply(CartridgeBoard *board, const BoardStateRestore *restore) {
    if (!board || !board->instance || !restore || restore->validatedBoard != board) return;
    NesStateReader reader;
    nes_state_reader_init(&reader, restore->payload.data(), restore->payload.size());
    cupid::boards::ApplyVisitor visitor(&reader);
    bool complete = board->instance->VisitState(visitor);
    assert(complete && visitor.Ok() && nes_state_reader_remaining(&reader) == 0);
    (void)complete;
}

void board_state_restore_free(BoardStateRestore *restore) {
    delete restore;
}
