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
#include "unif.h"

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

const std::unordered_map<std::string, int32_t> unif_boards = {
    {"11160", 299}, {"12-IN-1", 331}, {"13in1JY110", UNIF_BOARD_UNKNOWN},
    {"190in1", 300}, {"22211", 132}, {"255in1", UNIF_BOARD_255IN1},
    {"3D-BLOCK", UNIF_BOARD_UNKNOWN}, {"411120-C", 287}, {"42in1ResetSwitch", 226},
    {"43272", 227}, {"603-5052", 238}, {"64in1NoRepeat", 314},
    {"70in1", 236}, {"70in1B", 236}, {"810544-C-A1", 261},
    {"830425C-4391T", 320}, {"8157", 301}, {"8237", 215},
    {"8237A", UNIF_BOARD_8237A}, {"830118C", 348}, {"A65AS", 285},
    {"AC08", UNIF_BOARD_AC08}, {"ANROM", 7}, {"AX5705", 530},
    {"BB", 108}, {"BS-5", 286}, {"CC-21", UNIF_BOARD_CC21},
    {"CITYFIGHT", 266}, {"COOLBOY", 268}, {"10-24-C-A1", UNIF_BOARD_UNKNOWN},
    {"CNROM", 3}, {"CPROM", 13}, {"D1038", 59}, {"DANCE", UNIF_BOARD_UNKNOWN},
    {"DANCE2000", 518}, {"DREAMTECH01", 521}, {"EDU2000", 329},
    {"EKROM", 5}, {"ELROM", 5}, {"ETROM", 5}, {"EWROM", 5},
    {"FARID_SLROM_8-IN-1", 323}, {"FARID_UNROM_8-IN-1", 324},
    {"FK23C", 176}, {"FK23CA", 176}, {"FS304", 162}, {"G-146", 349},
    {"GK-192", 58}, {"GS-2004", 283}, {"GS-2013", UNIF_BOARD_GS2013},
    {"Ghostbusters63in1", UNIF_BOARD_GHOSTBUSTERS_63IN1}, {"H2288", 123},
    {"HKROM", 4}, {"KOF97", 263}, {"KONAMI-QTAI", 190}, {"K-3046", 336},
    {"KS7010", UNIF_BOARD_UNKNOWN}, {"KS7012", 346}, {"KS7013B", 312},
    {"KS7016", 306}, {"KS7017", 303}, {"KS7030", UNIF_BOARD_UNKNOWN},
    {"KS7031", 305}, {"KS7032", 142}, {"KS7037", 307}, {"KS7057", 302},
    {"LE05", UNIF_BOARD_UNKNOWN}, {"LH10", 522}, {"LH32", 125}, {"LH51", 309},
    {"LH53", UNIF_BOARD_UNKNOWN}, {"MALISB", 325}, {"MARIO1-MALEE2", UNIF_BOARD_MALEE},
    {"MHROM", 66}, {"N625092", 221}, {"NROM", 0}, {"NROM-128", 0},
    {"NROM-256", 0}, {"NTBROM", 68}, {"NTD-03", 290}, {"NovelDiamond9999999in1", 201},
    {"OneBus", UNIF_BOARD_UNKNOWN}, {"PEC-586", UNIF_BOARD_UNKNOWN},
    {"PUZZLE", UNIF_BOARD_PUZZLE}, {"RESET-TXROM", 313}, {"RET-CUFROM", 29},
    {"RROM", 0}, {"RROM-128", 0}, {"SA-002", 136}, {"SA-0036", 149},
    {"SA-0037", 148}, {"SA-009", 160}, {"SA-016-1M", 146},
    {"SA-72007", 145}, {"SA-72008", 133}, {"SA-9602B", 513}, {"SA-NROM", 143},
    {"SAROM", 1}, {"SBROM", 1}, {"SC-127", 35}, {"SCROM", 1}, {"SEROM", 1},
    {"SGROM", 1}, {"SHERO", 262}, {"SKROM", 1}, {"SL12", 116}, {"SL1632", 14},
    {"SL1ROM", 1}, {"SLROM", 1}, {"SMB2J", 304}, {"SNROM", 1}, {"SOROM", 1},
    {"SSS-NROM-256", UNIF_BOARD_SSS_NROM_256}, {"SUNSOFT_UNROM", 93},
    {"Sachen-74LS374N", 150}, {"Sachen-74LS374NA", 243}, {"Sachen-8259A", 141},
    {"Sachen-8259B", 138}, {"Sachen-8259C", 139}, {"Sachen-8259D", 137},
    {"Super24in1SC03", 176}, {"SuperHIK8in1", 45}, {"Supervision16in1", 53},
    {"T-227-1", UNIF_BOARD_UNKNOWN}, {"T-230", 529}, {"T-262", 265},
    {"TBROM", 4}, {"TC-U01-1.5M", 147}, {"TEK90", 90}, {"TEROM", 4},
    {"TF1201", 298}, {"TFROM", 4}, {"TGROM", 4}, {"TKROM", 4},
    {"TKSROM", 4}, {"TLROM", 4}, {"TLSROM", 4}, {"TQROM", 4},
    {"TR1ROM", 4}, {"TSROM", 4}, {"TVROM", 4}, {"Transformer", UNIF_BOARD_UNKNOWN},
    {"UNROM", 2}, {"UNROM-512-8", 30}, {"UNROM-512-16", 30},
    {"UNROM-512-32", 30}, {"UOROM", 2}, {"VRC7", 85}, {"YOKO", 264},
    {"SB-2000", UNIF_BOARD_UNKNOWN}, {"158B", 258}, {"DRAGONFIGHTER", 292},
    {"EH8813A", 519}, {"HP898F", 319}, {"F-15", 259}, {"RT-01", 328},
    {"81-01-31-C", UNIF_BOARD_UNKNOWN}, {"8-IN-1", 333}, {"WS", 332},
    {"80013-B", 274}, {"WAIXING-FW01", 227}, {"WAIXING-FS005", UNIF_BOARD_UNKNOWN},
    {"HPxx", 260}, {"HP2018A", 260}, {"DRIPGAME", 284}, {"60311C", 289},
    {"CHINA_ER_SAN2", 19}
};

int32_t resolve_unif_board(std::string name) {
    if (name.rfind("NES-", 0) == 0 || name.rfind("UNL-", 0) == 0
        || name.rfind("HVC-", 0) == 0 || name.rfind("BTL-", 0) == 0
        || name.rfind("BMC-", 0) == 0)
        name.erase(0, 4);
    auto found = unif_boards.find(name);
    return found == unif_boards.end() ? UNIF_BOARD_UNKNOWN : found->second;
}

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
    if (entry.mapper == 65000)
        entry.mapper = static_cast<uint16_t>(resolve_unif_board(entry.board));
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

int32_t unif_board_mapper_id(const char *board_name) {
    return board_name ? resolve_unif_board(board_name) : UNIF_BOARD_UNKNOWN;
}

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
