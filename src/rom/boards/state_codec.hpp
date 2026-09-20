/*
 * state_codec.hpp - Cartridge board save-state field visitor
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
#ifndef CUPID_BOARD_STATE_CODEC_HPP
#define CUPID_BOARD_STATE_CODEC_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace cupid::boards {

class BoardStateVisitor {
public:
    enum class Mode { Capture, Validate, Apply };

    virtual ~BoardStateVisitor() = default;
    virtual Mode GetMode() const = 0;

    virtual bool U8(const char *name, uint8_t &value, uint8_t maximum = UINT8_MAX) = 0;
    virtual bool U16(const char *name, uint16_t &value, uint16_t maximum = UINT16_MAX) = 0;
    virtual bool U32(const char *name, uint32_t &value, uint32_t maximum = UINT32_MAX) = 0;
    virtual bool U64(const char *name, uint64_t &value) = 0;
    virtual bool I16(const char *name, int16_t &value, int16_t minimum = INT16_MIN,
                     int16_t maximum = INT16_MAX) = 0;
    virtual bool I32(const char *name, int32_t &value, int32_t minimum = INT32_MIN,
                     int32_t maximum = INT32_MAX) = 0;
    virtual bool Bool(const char *name, bool &value) = 0;
    virtual bool ValueU8(const char *name, uint8_t &value,
                         uint8_t maximum = UINT8_MAX) = 0;
    virtual bool ValueU32(const char *name, uint32_t &value,
                          uint32_t maximum = UINT32_MAX) = 0;

    virtual bool U8Array(const char *name, uint8_t *values, size_t count,
                         uint8_t maximum = UINT8_MAX) = 0;
    virtual bool U16Array(const char *name, uint16_t *values, size_t count,
                          uint16_t maximum = UINT16_MAX) = 0;
    virtual bool U32Array(const char *name, uint32_t *values, size_t count,
                          uint32_t maximum = UINT32_MAX) = 0;
    virtual bool BoolArray(const char *name, bool *values, size_t count) = 0;
    virtual bool Bytes(const char *name, uint8_t *values, size_t count) = 0;

    virtual bool InvariantU8(const char *name, uint8_t value) = 0;
    virtual bool InvariantU16(const char *name, uint16_t value) = 0;
    virtual bool InvariantU32(const char *name, uint32_t value) = 0;
    virtual bool InvariantU64(const char *name, uint64_t value) = 0;
    virtual bool InvariantBool(const char *name, bool value) = 0;
    virtual bool InvariantString(const char *name, const std::string &value) = 0;
    virtual bool InvariantBytes(const char *name, const uint8_t *values, size_t count) = 0;

    bool Field(const char *name, uint8_t &value, uint8_t maximum = UINT8_MAX) {
        return U8(name, value, maximum);
    }
    bool Field(const char *name, uint16_t &value, uint16_t maximum = UINT16_MAX) {
        return U16(name, value, maximum);
    }
    bool Field(const char *name, uint32_t &value, uint32_t maximum = UINT32_MAX) {
        return U32(name, value, maximum);
    }
    bool Field(const char *name, uint64_t &value) { return U64(name, value); }
    bool Field(const char *name, int16_t &value, int16_t minimum = INT16_MIN,
               int16_t maximum = INT16_MAX) {
        return I16(name, value, minimum, maximum);
    }
    bool Field(const char *name, int32_t &value, int32_t minimum = INT32_MIN,
               int32_t maximum = INT32_MAX) {
        return I32(name, value, minimum, maximum);
    }
    bool Field(const char *name, bool &value) { return Bool(name, value); }

    template <size_t N>
    bool U8Array(const char *name, std::array<uint8_t, N> &values,
                 uint8_t maximum = UINT8_MAX) {
        return U8Array(name, values.data(), values.size(), maximum);
    }

    template <size_t N>
    bool U16Array(const char *name, std::array<uint16_t, N> &values,
                  uint16_t maximum = UINT16_MAX) {
        return U16Array(name, values.data(), values.size(), maximum);
    }

    template <size_t N>
    bool U32Array(const char *name, std::array<uint32_t, N> &values,
                  uint32_t maximum = UINT32_MAX) {
        return U32Array(name, values.data(), values.size(), maximum);
    }

    template <size_t N>
    bool U8Array(const char *name, uint8_t (&values)[N], uint8_t maximum = UINT8_MAX) {
        return U8Array(name, values, N, maximum);
    }

    template <size_t N>
    bool U16Array(const char *name, uint16_t (&values)[N], uint16_t maximum = UINT16_MAX) {
        return U16Array(name, values, N, maximum);
    }

    template <size_t N>
    bool U32Array(const char *name, uint32_t (&values)[N], uint32_t maximum = UINT32_MAX) {
        return U32Array(name, values, N, maximum);
    }

    template <size_t N>
    bool BoolArray(const char *name, bool (&values)[N]) {
        return BoolArray(name, values, N);
    }

    template <size_t N>
    bool Field(const char *name, std::array<uint8_t, N> &values,
               uint8_t maximum = UINT8_MAX) {
        return U8Array(name, values, maximum);
    }

    template <size_t N>
    bool Field(const char *name, std::array<uint16_t, N> &values,
               uint16_t maximum = UINT16_MAX) {
        return U16Array(name, values, maximum);
    }

    template <size_t N>
    bool Field(const char *name, std::array<uint32_t, N> &values,
               uint32_t maximum = UINT32_MAX) {
        return U32Array(name, values, maximum);
    }

    template <size_t N>
    bool Field(const char *name, uint8_t (&values)[N], uint8_t maximum = UINT8_MAX) {
        return U8Array(name, values, maximum);
    }

    template <size_t N>
    bool Field(const char *name, uint16_t (&values)[N], uint16_t maximum = UINT16_MAX) {
        return U16Array(name, values, maximum);
    }

    template <size_t N>
    bool Field(const char *name, uint32_t (&values)[N], uint32_t maximum = UINT32_MAX) {
        return U32Array(name, values, maximum);
    }

    template <size_t N>
    bool Field(const char *name, bool (&values)[N]) {
        return BoolArray(name, values);
    }
};

} // namespace cupid::boards

#endif
