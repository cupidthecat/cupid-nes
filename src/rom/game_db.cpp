/*
 * game_db.cpp - Optional NES cartridge metadata database
 *
 * Author: @frankischilling
 *
 * The accepted CSV layout is the common 18-field NES database format:
 * CRC, system, board, PCB, chip, mapper, PRG ROM, CHR ROM, CHR RAM,
 * work RAM, save RAM, battery, mirroring, input, bus conflicts,
 * submapper, VS hardware type, and PPU model.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "game_db.h"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

std::unordered_map<uint32_t, GameDbEntry> entries;

bool parse_unsigned(const std::string &text, unsigned long long limit,
                    int base, unsigned long long &value) {
    if (text.empty()) {
        value = 0;
        return true;
    }
    char *end = nullptr;
    errno = 0;
    unsigned long long parsed = std::strtoull(text.c_str(), &end, base);
    if (errno || end == text.c_str() || *end || parsed > limit) return false;
    value = parsed;
    return true;
}

bool parse_size(const std::string &text, uint32_t &value) {
    if (text.empty()) {
        value = 0;
        return true;
    }
    bool bytes = text[0] == 'b' || text[0] == 'B';
    std::string number = bytes ? text.substr(1) : text;
    unsigned long long parsed = 0;
    if (!parse_unsigned(number, UINT32_MAX, 10, parsed)) return false;
    if (!bytes) {
        if (parsed > UINT32_MAX / 1024u) return false;
        parsed *= 1024u;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
}

bool copy_field(char *destination, size_t capacity, const std::string &value) {
    if (!destination || !capacity || value.size() >= capacity) return false;
    std::memcpy(destination, value.c_str(), value.size() + 1);
    return true;
}

std::vector<std::string> split_csv(const std::string &line) {
    std::vector<std::string> fields;
    size_t start = 0;
    for (;;) {
        size_t comma = line.find(',', start);
        fields.push_back(line.substr(start, comma == std::string::npos ? comma : comma - start));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return fields;
}

bool parse_entry(const std::string &line, GameDbEntry &entry) {
    std::vector<std::string> fields = split_csv(line);
    if (fields.size() < 18) return false;
    entry = GameDbEntry{};
    entry.bus_conflicts = -1;

    unsigned long long number = 0;
    if (!parse_unsigned(fields[0], UINT32_MAX, 16, number)) return false;
    entry.crc32 = static_cast<uint32_t>(number);
    if (!copy_field(entry.system, sizeof(entry.system), fields[1])
        || !copy_field(entry.board, sizeof(entry.board), fields[2])
        || !copy_field(entry.pcb, sizeof(entry.pcb), fields[3])
        || !copy_field(entry.chip, sizeof(entry.chip), fields[4])) return false;

    if (!parse_unsigned(fields[5], UINT16_MAX, 10, number)) return false;
    entry.mapper = static_cast<uint16_t>(number);
    if (!parse_size(fields[6], entry.prg_rom_size)
        || !parse_size(fields[7], entry.chr_rom_size)
        || !parse_size(fields[8], entry.chr_ram_size)
        || !parse_size(fields[9], entry.work_ram_size)
        || !parse_size(fields[10], entry.save_ram_size)) return false;

    if (!parse_unsigned(fields[11], 1, 10, number)) return false;
    entry.battery = number != 0;
    entry.mirroring = fields[12].empty() ? '\0' : fields[12][0];
    if (fields[12].size() > 1 || (entry.mirroring && entry.mirroring != 'h'
        && entry.mirroring != 'v' && entry.mirroring != '4'
        && entry.mirroring != '0' && entry.mirroring != '1')) return false;

    if (!parse_unsigned(fields[13], UINT8_MAX, 10, number)) return false;
    entry.input_type = static_cast<uint8_t>(number);
    if (fields[14] == "Y") entry.bus_conflicts = 1;
    else if (fields[14] == "N") entry.bus_conflicts = 0;
    else if (!fields[14].empty()) return false;

    if (!fields[15].empty()) {
        if (!parse_unsigned(fields[15], 15, 10, number)) return false;
        entry.submapper_present = true;
        entry.submapper = static_cast<uint8_t>(number);
    }
    if (!parse_unsigned(fields[16], UINT8_MAX, 10, number)) return false;
    entry.vs_type = static_cast<uint8_t>(number);
    if (!parse_unsigned(fields[17], UINT8_MAX, 10, number)) return false;
    entry.ppu_model = static_cast<uint8_t>(number);
    return true;
}

bool parse_database(const std::string &text,
                    std::unordered_map<uint32_t, GameDbEntry> &parsed) {
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        GameDbEntry entry{};
        if (!parse_entry(line, entry)) return false;
        parsed[entry.crc32] = entry;
    }
    return true;
}

} // namespace

extern "C" {

bool game_db_load_file(const char *path) {
    if (!path) return false;
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (file.bad()) return false;
    std::unordered_map<uint32_t, GameDbEntry> parsed;
    if (!parse_database(buffer.str(), parsed)) return false;
    entries.swap(parsed);
    return true;
}

bool game_db_load_memory(const char *text, size_t size) {
    if (!text && size) return false;
    std::string input(text ? text : "", size);
    std::unordered_map<uint32_t, GameDbEntry> parsed;
    if (!parse_database(input, parsed)) return false;
    entries.swap(parsed);
    return true;
}

void game_db_clear(void) { entries.clear(); }

bool game_db_lookup(uint32_t crc32, GameDbEntry *entry) {
    auto found = entries.find(crc32);
    if (found == entries.end()) return false;
    if (entry) *entry = found->second;
    return true;
}

size_t game_db_entry_count(void) { return entries.size(); }

uint32_t game_db_crc32(const uint8_t *data, size_t size) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data ? data[i] : 0;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc ^ 0xFFFFFFFFu;
}

} // extern "C"
