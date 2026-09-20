/*
 * hd_pack_loader.cpp - bounded HD pack definition and archive loading
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "hd_pack_loader.hpp"

#include "../third_party/miniz/miniz.h"
#include "../util/file_io.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cupid::hd {
namespace {

constexpr const char *kDefinitionName = "hires.txt";
constexpr std::uint32_t kNesWidth = 256;
constexpr std::uint32_t kNesHeight = 240;

bool valid_utf8(const std::string &text) {
    const auto *bytes = reinterpret_cast<const unsigned char *>(text.data());
    std::size_t i = 0;
    while (i < text.size()) {
        std::uint32_t value = bytes[i++];
        if (value < 0x80u) continue;
        unsigned continuation = 0;
        std::uint32_t minimum = 0;
        if (value >= 0xc2u && value <= 0xdfu) {
            continuation = 1; minimum = 0x80u; value &= 0x1fu;
        } else if (value >= 0xe0u && value <= 0xefu) {
            continuation = 2; minimum = 0x800u; value &= 0x0fu;
        } else if (value >= 0xf0u && value <= 0xf4u) {
            continuation = 3; minimum = 0x10000u; value &= 0x07u;
        } else {
            return false;
        }
        if (continuation > text.size() - i) return false;
        for (unsigned j = 0; j < continuation; ++j) {
            const std::uint32_t next = bytes[i++];
            if ((next & 0xc0u) != 0x80u) return false;
            value = (value << 6) | (next & 0x3fu);
        }
        if (value < minimum || value > 0x10ffffu || (value >= 0xd800u && value <= 0xdfffu))
            return false;
    }
    return true;
}

std::string ascii_alias(const std::string &path) {
    std::string result = path;
    for (char &c : result) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return result;
}

bool normalize_member_path(const std::string &input, std::string &output, std::string &error) {
    output.clear();
    if (input.empty() || input.size() >= NES_FILE_PATH_LIMIT || !valid_utf8(input)) {
        error = "asset path is empty, too long, or is not valid UTF-8";
        return false;
    }
    if (input.front() == '/' || input.front() == '\\' || input.find('\\') != std::string::npos) {
        error = "asset path must be a relative path using '/' separators";
        return false;
    }
    std::size_t start = 0;
    while (start < input.size()) {
        const std::size_t slash = input.find('/', start);
        const std::size_t end = slash == std::string::npos ? input.size() : slash;
        const std::string segment = input.substr(start, end - start);
        if (segment.empty() || segment == "." || segment == "..") {
            error = "asset path contains an empty or traversal segment";
            return false;
        }
        if (segment.back() == '.' || segment.back() == ' ') {
            error = "asset path contains a segment with an unsafe trailing character";
            return false;
        }
        for (unsigned char c : segment) {
            if (c < 0x20u || c == 0x7fu || c == ':' || c == '*' || c == '?' || c == '"'
                || c == '<' || c == '>' || c == '|') {
                error = "asset path contains an unsafe character";
                return false;
            }
        }
        if (!output.empty()) output.push_back('/');
        output += segment;
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    return !output.empty();
}

std::string join_path(const std::string &root, const std::string &relative) {
    if (root.empty()) return relative;
    const char last = root.back();
    if (last == '/' || last == '\\') return root + relative;
#ifdef _WIN32
    return root + "\\" + relative;
#else
    return root + "/" + relative;
#endif
}

enum class ReadStatus { Ok, Missing, Error };

class AssetSource {
public:
    virtual ~AssetSource() = default;
    virtual ReadStatus read(const std::string &path, std::size_t limit,
                            std::vector<std::uint8_t> &data, std::string &error) = 0;
};

class DirectorySource final : public AssetSource {
public:
    explicit DirectorySource(std::string root) : root_(std::move(root)) {}

    ReadStatus read(const std::string &path, std::size_t limit,
                    std::vector<std::uint8_t> &data, std::string &error) override {
        data.clear();
        const std::string full = join_path(root_, path);
        std::uint8_t *raw = nullptr;
        std::size_t size = 0;
        const NesFileResult result = nes_file_read_all(full.c_str(), limit, &raw, &size);
        if (result == NES_FILE_NOT_FOUND) return ReadStatus::Missing;
        if (result != NES_FILE_OK) {
            error = "could not read asset '" + path + "': " + nes_file_result_message(result);
            return ReadStatus::Error;
        }
        data.assign(raw, raw + size);
        std::free(raw);
        return ReadStatus::Ok;
    }

private:
    std::string root_;
};

struct ZipAllocationHeader { std::size_t size; };
struct ZipBudget { std::size_t used = 0; std::size_t limit = 0; bool exhausted = false; };

void *zip_alloc(void *opaque, size_t count, size_t size) {
    auto *budget = static_cast<ZipBudget *>(opaque);
    if (size && count > std::numeric_limits<std::size_t>::max() / size) {
        budget->exhausted = true;
        return nullptr;
    }
    const std::size_t bytes = count * size;
    if (bytes > budget->limit - std::min(budget->used, budget->limit)
        || bytes > std::numeric_limits<std::size_t>::max() - sizeof(ZipAllocationHeader)) {
        budget->exhausted = true;
        return nullptr;
    }
    auto *header = static_cast<ZipAllocationHeader *>(std::malloc(sizeof(ZipAllocationHeader) + bytes));
    if (!header) return nullptr;
    header->size = bytes;
    budget->used += bytes;
    return header + 1;
}

void zip_free(void *opaque, void *address) {
    if (!address) return;
    auto *budget = static_cast<ZipBudget *>(opaque);
    auto *header = static_cast<ZipAllocationHeader *>(address) - 1;
    budget->used = header->size <= budget->used ? budget->used - header->size : 0;
    std::free(header);
}

void *zip_realloc(void *opaque, void *address, size_t count, size_t size) {
    if (!address) return zip_alloc(opaque, count, size);
    auto *budget = static_cast<ZipBudget *>(opaque);
    if (size && count > std::numeric_limits<std::size_t>::max() / size) {
        budget->exhausted = true;
        return nullptr;
    }
    const std::size_t bytes = count * size;
    auto *old_header = static_cast<ZipAllocationHeader *>(address) - 1;
    const std::size_t old_size = old_header->size;
    const std::size_t base = old_size <= budget->used ? budget->used - old_size : 0;
    if (bytes > budget->limit - std::min(base, budget->limit)
        || bytes > std::numeric_limits<std::size_t>::max() - sizeof(ZipAllocationHeader)) {
        budget->exhausted = true;
        return nullptr;
    }
    auto *header = static_cast<ZipAllocationHeader *>(
        std::realloc(old_header, sizeof(ZipAllocationHeader) + bytes));
    if (!header) return nullptr;
    header->size = bytes;
    budget->used = base + bytes;
    return header + 1;
}

class ZipSource final : public AssetSource {
public:
    explicit ZipSource(std::vector<std::uint8_t> bytes) : bytes_(std::move(bytes)) {
        mz_zip_zero_struct(&zip_);
    }
    ~ZipSource() override {
        if (initialized_) mz_zip_reader_end(&zip_);
    }

    bool initialize(const LoadLimits &limits, std::string &error) {
        budget_.limit = std::min<std::size_t>(limits.max_archive_bytes, 64u * 1024u * 1024u);
        zip_.m_pAlloc = zip_alloc;
        zip_.m_pFree = zip_free;
        zip_.m_pRealloc = zip_realloc;
        zip_.m_pAlloc_opaque = &budget_;
        if (!mz_zip_reader_init_mem(&zip_, bytes_.data(), bytes_.size(), 0)) {
            error = budget_.exhausted ? "ZIP metadata exceeds the memory limit"
                                      : "pack ZIP has an invalid central directory";
            return false;
        }
        initialized_ = true;
        const mz_uint count = mz_zip_reader_get_num_files(&zip_);
        if (count > limits.max_entries) {
            error = "pack ZIP contains too many entries";
            return false;
        }
        std::size_t total_uncompressed = 0;
        std::unordered_set<std::string> aliases;
        for (mz_uint i = 0; i < count; ++i) {
            mz_zip_archive_file_stat stat{};
            if (!mz_zip_reader_file_stat(&zip_, i, &stat)) {
                error = "pack ZIP contains unreadable entry metadata";
                return false;
            }
            const mz_uint name_size = mz_zip_reader_get_filename(&zip_, i, nullptr, 0);
            if (name_size < 2 || name_size > NES_FILE_PATH_LIMIT) {
                error = "pack ZIP contains an invalid entry name";
                return false;
            }
            std::vector<char> name_buffer(name_size);
            if (mz_zip_reader_get_filename(&zip_, i, name_buffer.data(), name_size) != name_size
                || name_buffer.back() != '\0') {
                error = "pack ZIP contains an invalid entry name";
                return false;
            }
            std::string raw(name_buffer.data());
            if (!(stat.m_bit_flag & 0x800u)) {
                for (unsigned char c : raw) {
                    if (c >= 0x80u) {
                        error = "pack ZIP contains a non-UTF-8 entry name";
                        return false;
                    }
                }
            }
            const bool directory = stat.m_is_directory != 0;
            if (directory && !raw.empty() && raw.back() == '/') raw.pop_back();
            std::string normalized;
            std::string path_error;
            if (!normalize_member_path(raw, normalized, path_error)) {
                error = "unsafe ZIP entry '" + raw + "': " + path_error;
                return false;
            }
            const std::string alias = ascii_alias(normalized);
            if (!aliases.insert(alias).second) {
                error = "pack ZIP contains duplicate or case-aliased entry '" + normalized + "'";
                return false;
            }
            const bool symlink = ((stat.m_external_attr >> 16) & 0170000u) == 0120000u;
            if (symlink) {
                error = "pack ZIP contains symbolic link entry '" + normalized + "'";
                return false;
            }
            if (directory) continue;
            if (!stat.m_is_supported || mz_zip_reader_is_file_encrypted(&zip_, i)) {
                error = "pack ZIP contains unsupported or encrypted entry '" + normalized + "'";
                return false;
            }
            if (stat.m_uncomp_size > limits.max_member_bytes) {
                error = "pack ZIP entry '" + normalized + "' exceeds the member size limit";
                return false;
            }
            if (stat.m_uncomp_size > limits.max_total_asset_bytes - std::min(total_uncompressed, limits.max_total_asset_bytes)) {
                error = "pack ZIP exceeds the total uncompressed asset limit";
                return false;
            }
            total_uncompressed += static_cast<std::size_t>(stat.m_uncomp_size);
            entries_.emplace(normalized, Entry{static_cast<unsigned>(i), static_cast<std::size_t>(stat.m_uncomp_size)});
        }
        if (!mz_zip_validate_archive(&zip_, 0)) {
            error = "pack ZIP failed CRC or compressed-stream validation";
            return false;
        }
        return true;
    }

    ReadStatus read(const std::string &path, std::size_t limit,
                    std::vector<std::uint8_t> &data, std::string &error) override {
        data.clear();
        const auto it = entries_.find(path);
        if (it == entries_.end()) return ReadStatus::Missing;
        if (it->second.size > limit) {
            error = "asset '" + path + "' exceeds the requested size limit";
            return ReadStatus::Error;
        }
        data.resize(it->second.size);
        if (!mz_zip_reader_extract_to_mem(&zip_, it->second.index,
                                          data.data(), data.size(), 0)) {
            error = budget_.exhausted ? "ZIP extraction exceeded its memory limit for '" + path + "'"
                                      : "ZIP extraction failed CRC validation for '" + path + "'";
            data.clear();
            return ReadStatus::Error;
        }
        return ReadStatus::Ok;
    }

private:
    struct Entry { unsigned index; std::size_t size; };
    std::vector<std::uint8_t> bytes_;
    mz_zip_archive zip_{};
    ZipBudget budget_{};
    bool initialized_ = false;
    std::unordered_map<std::string, Entry> entries_;
};

std::vector<std::string> split(const std::string &text, char delimiter) {
    std::vector<std::string> result;
    std::size_t start = 0;
    while (true) {
        const std::size_t position = text.find(delimiter, start);
        result.push_back(text.substr(start, position == std::string::npos ? position : position - start));
        if (position == std::string::npos) break;
        start = position + 1;
    }
    return result;
}

std::string trim(const std::string &text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

bool parse_u32_dec(const std::string &text, std::uint32_t &value) {
    if (text.empty()) return false;
    const char *begin = text.data();
    const char *end = begin + text.size();
    auto result = std::from_chars(begin, end, value, 10);
    return result.ec == std::errc{} && result.ptr == end;
}

bool parse_i32_dec(const std::string &text, std::int32_t &value) {
    if (text.empty()) return false;
    const char *begin = text.data();
    const char *end = begin + text.size();
    auto result = std::from_chars(begin, end, value, 10);
    return result.ec == std::errc{} && result.ptr == end;
}

bool parse_hex(const std::string &text, std::uint32_t &value) {
    if (text.empty() || text.size() > 8) return false;
    std::string_view view(text);
    if (view.size() > 2 && view[0] == '0' && (view[1] == 'x' || view[1] == 'X')) view.remove_prefix(2);
    if (view.empty() || view.size() > 8) return false;
    value = 0;
    const char *begin = view.data();
    const char *end = begin + view.size();
    auto result = std::from_chars(begin, end, value, 16);
    return result.ec == std::errc{} && result.ptr == end;
}

bool parse_float(const std::string &text, float &value) {
    if (text.empty()) return false;
    char *end = nullptr;
    errno = 0;
    value = std::strtof(text.c_str(), &end);
    return errno != ERANGE && end == text.c_str() + text.size() && std::isfinite(value);
}

bool parse_bool(const std::string &text, bool &value) {
    if (text == "Y") { value = true; return true; }
    if (text == "N") { value = false; return true; }
    return false;
}

bool parse_compare(const std::string &text, CompareOperator &op) {
    if (text == "==") op = CompareOperator::Equal;
    else if (text == "!=") op = CompareOperator::NotEqual;
    else if (text == ">") op = CompareOperator::GreaterThan;
    else if (text == "<") op = CompareOperator::LessThan;
    else if (text == "<=") op = CompareOperator::LessThanOrEqual;
    else if (text == ">=") op = CompareOperator::GreaterThanOrEqual;
    else return false;
    return true;
}

bool parse_brightness(const std::string &text, std::int32_t &brightness) {
    float factor = 0.0f;
    if (!parse_float(text, factor) || factor < 0.0f) return false;
    const double scaled = static_cast<double>(factor) * 255.0;
    if (scaled > static_cast<double>(std::numeric_limits<std::int32_t>::max())) return false;
    brightness = static_cast<std::int32_t>(scaled);
    return true;
}

bool parse_sha1(const std::string &text) {
    if (text.size() != 40) return false;
    for (char c : text) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
}

class Parser {
public:
    Parser(AssetSource &source, const LoadLimits &limits, LoadResult &result)
        : source_(source), limits_(limits), result_(result), pack_(std::make_unique<Pack>()) {}

    bool run() {
        std::vector<std::uint8_t> definition;
        if (!read_asset(kDefinitionName, AssetKind::Definition, true,
                        limits_.max_definition_bytes, definition)) return false;
        if (std::find(definition.begin(), definition.end(), 0) != definition.end()
            || !valid_utf8(std::string(reinterpret_cast<const char *>(definition.data()), definition.size()))) {
            return fail("hires.txt is not valid UTF-8 text");
        }
        initialize_builtin_conditions();
        const std::string text(reinterpret_cast<const char *>(definition.data()), definition.size());
        std::size_t start = 0;
        while (start <= text.size()) {
            if (++line_number_ > limits_.max_lines) return fail("hires.txt contains too many lines");
            const std::size_t newline = text.find('\n', start);
            std::string line = text.substr(start, newline == std::string::npos ? newline : newline - start);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!parse_line(line)) return false;
            if (newline == std::string::npos) break;
            start = newline + 1;
        }
        if (!version_seen_) return fail("hires.txt must declare <ver>109");
        if (!load_palette()) return false;
        result_.candidate = std::move(pack_);
        return true;
    }

private:
    bool fail(const std::string &message) {
        if (result_.error.empty()) {
            if (line_number_) result_.error = "hires.txt line " + std::to_string(line_number_) + ": " + message;
            else result_.error = message;
        }
        return false;
    }

    bool ensure_version() {
        return version_seen_ || fail("<ver>109 must appear before pack data");
    }

    bool count_ok(std::size_t count, std::size_t limit, const char *kind) {
        if (count < limit) return true;
        return fail(std::string("pack contains too many ") + kind);
    }

    bool read_asset(const std::string &input, AssetKind kind, bool required,
                    std::size_t limit, std::vector<std::uint8_t> &data) {
        std::string path;
        std::string path_error;
        if (!normalize_member_path(input, path, path_error)) {
            return fail("invalid asset path '" + input + "': " + path_error);
        }
        const auto cached = asset_index_.find(path);
        if (cached != asset_index_.end()) {
            data = pack_->assets[cached->second].bytes;
            return true;
        }
        std::string read_error;
        const ReadStatus status = source_.read(path, limit, data, read_error);
        if (status == ReadStatus::Missing) {
            if (required) return fail("required asset '" + path + "' was not found");
            return false;
        }
        if (status == ReadStatus::Error) return fail(read_error);
        if (data.size() > limits_.max_total_asset_bytes
            || total_asset_bytes_ > limits_.max_total_asset_bytes - data.size()) {
            return fail("pack exceeds the total encoded asset memory limit");
        }
        total_asset_bytes_ += data.size();
        asset_index_.emplace(path, pack_->assets.size());
        pack_->assets.push_back(PackAsset{path, kind, data});
        return true;
    }

    bool add_decoded(std::size_t bytes) {
        if (bytes > limits_.max_decoded_bytes
            || decoded_bytes_ > limits_.max_decoded_bytes - bytes) {
            return fail("pack exceeds the total decoded asset memory limit");
        }
        decoded_bytes_ += bytes;
        return true;
    }

    void add_builtin(std::string name, ConditionType type, std::uint8_t palette = 0) {
        Condition normal;
        normal.name = name;
        normal.type = type;
        normal.sprite_palette = palette;
        condition_index_[normal.name] = pack_->conditions.size();
        pack_->conditions.push_back(normal);
        Condition inverted = normal;
        inverted.name = "!" + name;
        inverted.inverted = true;
        condition_index_[inverted.name] = pack_->conditions.size();
        pack_->conditions.push_back(std::move(inverted));
    }

    void initialize_builtin_conditions() {
        add_builtin("hmirror", ConditionType::HorizontalMirror);
        add_builtin("vmirror", ConditionType::VerticalMirror);
        add_builtin("bgpriority", ConditionType::BackgroundPriority);
        for (std::uint8_t i = 0; i < 4; ++i)
            add_builtin("sppalette" + std::to_string(i), ConditionType::SpritePalette, i);
    }

    bool parse_condition_refs(const std::string &expression, std::vector<ConditionRef> &refs) {
        if (expression.empty()) return fail("condition list may not be empty");
        for (std::string name : split(expression, '&')) {
            name.erase(std::remove_if(name.begin(), name.end(), [](unsigned char c) {
                return c == ' ' || c == '\t' || c == '\r' || c == '\n';
            }), name.end());
            if (name.empty()) return fail("condition list contains an empty condition name");
            const auto it = condition_index_.find(name);
            if (it == condition_index_.end()) return fail("condition '" + name + "' is not defined");
            refs.push_back(it->second);
        }
        return true;
    }

    bool parse_line(std::string line) {
        if (line.empty()) return true;
        const std::string leading_trimmed = trim(line);
        if (leading_trimmed.empty() || leading_trimmed.front() == '#') return true;
        line = leading_trimmed;

        std::vector<ConditionRef> condition_refs;
        if (!line.empty() && line.front() == '[') {
            const std::size_t close = line.find(']');
            if (close == std::string::npos) return fail("condition prefix is missing ']'");
            if (!parse_condition_refs(line.substr(1, close - 1), condition_refs)) return false;
            line = line.substr(close + 1);
        }

        if (line.rfind("<ver>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<ver> may not have conditions");
            if (version_seen_) return fail("duplicate <ver> tag");
            std::uint32_t version = 0;
            if (!parse_u32_dec(trim(line.substr(5)), version) || version != kSupportedPackVersion)
                return fail("only HD pack format version 109 is supported");
            version_seen_ = true;
            pack_->version = version;
            return true;
        }
        if (!ensure_version()) return false;

        if (line.rfind("<scale>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<scale> may not have conditions");
            std::uint32_t scale = 0;
            if (!parse_u32_dec(trim(line.substr(7)), scale) || scale < 1 || scale > 10)
                return fail("scale must be between 1 and 10");
            pack_->scale = scale;
            return true;
        }
        if (line.rfind("<supportedRom>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<supportedRom> may not have conditions");
            std::string sha1 = trim(line.substr(14));
            if (!parse_sha1(sha1)) return fail("<supportedRom> requires a 40-digit SHA-1 hash");
            std::transform(sha1.begin(), sha1.end(), sha1.begin(), [](unsigned char c) {
                return static_cast<char>(std::toupper(c));
            });
            pack_->supported_rom_sha1.push_back(std::move(sha1));
            return true;
        }
        if (line.rfind("<img>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<img> may not have conditions");
            return parse_image(trim(line.substr(5)));
        }
        if (line.rfind("<tile>", 0) == 0)
            return parse_tile(split_tokens(line.substr(6)), std::move(condition_refs));
        if (line.rfind("<condition>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<condition> may not have a condition prefix");
            return parse_condition(split_tokens(line.substr(11)));
        }
        if (line.rfind("<background>", 0) == 0)
            return parse_background(split_tokens(line.substr(12)), std::move(condition_refs));
        if (line.rfind("<addition>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<addition> may not have conditions");
            return parse_addition(split_tokens(line.substr(10)));
        }
        if (line.rfind("<fallback>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<fallback> may not have conditions");
            return parse_fallback(split_tokens(line.substr(10)));
        }
        if (line.rfind("<bgm>", 0) == 0 || line.rfind("<sfx>", 0) == 0) {
            if (!condition_refs.empty()) return fail("audio tags may not have conditions");
            const bool bgm = line.rfind("<bgm>", 0) == 0;
            return parse_audio(split_tokens(line.substr(5)), bgm);
        }
        if (line.rfind("<overscan>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<overscan> may not have conditions");
            return parse_overscan(split_tokens(line.substr(10)));
        }
        if (line.rfind("<options>", 0) == 0) {
            if (!condition_refs.empty()) return fail("<options> may not have conditions");
            return parse_options(split_tokens(line.substr(9)));
        }
        if (line.rfind("<patch>", 0) == 0)
            return fail("<patch> is unsupported because ROM patching changes emulated hardware");
        if (!condition_refs.empty()) return fail("condition prefix is attached to an unsupported tag");
        return fail("unsupported HD pack tag '" + line.substr(0, std::min<std::size_t>(line.size(), 64)) + "'");
    }

    std::vector<std::string> split_tokens(const std::string &text) {
        auto tokens = split(text, ',');
        for (std::string &token : tokens) token = trim(token);
        return tokens;
    }

    bool parse_tile_key(const std::string &tile_text, const std::string &palette_text, TileKey &key) {
        std::uint32_t palette = 0;
        if (!parse_hex(palette_text, palette)) return fail("invalid tile palette '" + palette_text + "'");
        key.palette = palette;
        if (tile_text.size() == 32) {
            key.source = TileSource::ChrRam;
            for (std::size_t i = 0; i < key.chr_ram.size(); ++i) {
                std::uint32_t byte = 0;
                if (!parse_hex(tile_text.substr(i * 2, 2), byte) || byte > 0xff)
                    return fail("invalid 16-byte CHR-RAM tile key '" + tile_text + "'");
                key.chr_ram[i] = static_cast<std::uint8_t>(byte);
            }
        } else {
            std::uint32_t index = 0;
            if (!parse_hex(tile_text, index)) return fail("invalid physical CHR-ROM tile index '" + tile_text + "'");
            key.source = TileSource::ChrRom;
            key.chr_rom_index = index;
        }
        return true;
    }

    bool parse_image(const std::string &input_path) {
        if (!count_ok(pack_->tile_images.size(), limits_.max_images, "tile images")) return false;
        std::vector<std::uint8_t> encoded;
        if (!read_asset(input_path, AssetKind::TileImage, true, limits_.max_member_bytes, encoded)) return false;
        std::string path;
        std::string path_error;
        if (!normalize_member_path(input_path, path, path_error)) return fail("invalid image path: " + path_error);
        Image image;
        std::string error;
        if (!decode_png(encoded, path, limits_, image, error)) return fail(error);
        if (!add_decoded(image.argb32.size() * sizeof(std::uint32_t))) return false;
        pack_->tile_images.push_back(std::move(image));
        return true;
    }

    bool parse_tile(const std::vector<std::string> &tokens, std::vector<ConditionRef> conditions) {
        if (!count_ok(pack_->tiles.size(), limits_.max_tiles, "tile rules")) return false;
        if (tokens.size() < 7 || tokens.size() > 9) return fail("<tile> requires 7 to 9 parameters");
        std::uint32_t bitmap = 0;
        if (!parse_u32_dec(tokens[0], bitmap) || bitmap >= pack_->tile_images.size())
            return fail("<tile> references an invalid bitmap index");
        TileRule rule;
        rule.bitmap_index = bitmap;
        if (!parse_tile_key(tokens[1], tokens[2], rule.key)) return false;
        std::int32_t x = 0, y = 0;
        if (!parse_i32_dec(tokens[3], x) || !parse_i32_dec(tokens[4], y) || x < 0 || y < 0)
            return fail("<tile> x/y coordinates must be non-negative integers");
        rule.x = static_cast<std::uint32_t>(x);
        rule.y = static_cast<std::uint32_t>(y);
        if (!parse_brightness(tokens[5], rule.brightness))
            return fail("<tile> brightness must be a finite non-negative number");
        if (!parse_bool(tokens[6], rule.default_tile)) return fail("<tile> default flag must be Y or N");
        rule.conditions = std::move(conditions);
        if (rule.key.source == TileSource::ChrRam) {
            if (tokens.size() >= 8 && !parse_u32_dec(tokens[7], rule.chr_bank_id))
                return fail("<tile> CHR-RAM bank id must be a non-negative integer");
            if (tokens.size() >= 9 && !parse_i32_dec(tokens[8], rule.runtime_tile_index))
                return fail("<tile> CHR-RAM runtime tile index is invalid");
        } else if (tokens.size() != 7) {
            return fail("CHR-ROM <tile> may not contain CHR-RAM mapping parameters");
        }
        const Image &image = pack_->tile_images[rule.bitmap_index];
        const std::uint32_t dimension = pack_->scale * 8u;
        if (rule.x > image.width || rule.y > image.height
            || dimension > image.width - rule.x || dimension > image.height - rule.y)
            return fail("<tile> source rectangle lies outside its bitmap");
        pack_->tiles.push_back(std::move(rule));
        return true;
    }

    bool parse_condition(const std::vector<std::string> &tokens) {
        if (!count_ok(pack_->conditions.size(), limits_.max_conditions, "conditions")) return false;
        if (tokens.size() < 4 || tokens[0].empty() || tokens[0].find('!') != std::string::npos)
            return fail("<condition> has an invalid name or parameter count");
        if (condition_index_.count(tokens[0]) || condition_index_.count("!" + tokens[0]))
            return fail("condition '" + tokens[0] + "' is defined more than once");
        Condition base;
        base.name = tokens[0];
        const std::string &type = tokens[1];
        if (type == "tileAtPosition" || type == "tileNearby" || type == "spriteAtPosition" || type == "spriteNearby") {
            if (tokens.size() != 6 && tokens.size() != 7) return fail("tile/sprite conditions require 6 or 7 parameters");
            if (!parse_i32_dec(tokens[2], base.x) || !parse_i32_dec(tokens[3], base.y)) return fail("condition coordinates are invalid");
            if (!parse_tile_key(tokens[4], tokens[5], base.tile)) return false;
            if (tokens.size() == 7 && !parse_bool(tokens[6], base.ignore_palette)) return fail("condition ignore-palette flag must be Y or N");
            if ((type == "tileAtPosition" || type == "spriteAtPosition")
                && (base.x < 0 || base.y < 0 || base.x >= static_cast<std::int32_t>(kNesWidth)
                    || base.y >= static_cast<std::int32_t>(kNesHeight)))
                return fail("absolute tile/sprite condition position is outside the NES frame");
            base.type = type == "tileAtPosition" ? ConditionType::TileAtPosition
                      : type == "tileNearby" ? ConditionType::TileNearby
                      : type == "spriteAtPosition" ? ConditionType::SpriteAtPosition
                      : ConditionType::SpriteNearby;
        } else if (type == "memoryCheck" || type == "ppuMemoryCheck"
                   || type == "memoryCheckConstant" || type == "ppuMemoryCheckConstant") {
            if (tokens.size() != 5 && tokens.size() != 6) return fail("memory conditions require 5 or 6 parameters");
            const bool ppu = type.rfind("ppu", 0) == 0;
            const bool constant = type.find("Constant") != std::string::npos;
            base.type = constant ? ConditionType::MemoryCheckConstant : ConditionType::MemoryCheck;
            base.memory_space = ppu ? MemorySpace::Ppu : MemorySpace::Cpu;
            if (!parse_hex(tokens[2], base.operand_a) || base.operand_a > (ppu ? 0x3fffu : 0xffffu)) return fail("memory condition operand A is out of range");
            if (!parse_compare(tokens[3], base.comparison)) return fail("memory condition has an invalid comparison operator");
            if (!parse_hex(tokens[4], base.operand_b)) return fail("memory condition operand B is invalid");
            if ((constant && base.operand_b > 0xffu) || (!constant && base.operand_b > (ppu ? 0x3fffu : 0xffffu)))
                return fail("memory condition operand B is out of range");
            if (tokens.size() == 6) {
                std::uint32_t mask = 0;
                if (!parse_hex(tokens[5], mask) || mask > 0xffu) return fail("memory condition mask is out of range");
                base.mask = static_cast<std::uint8_t>(mask);
            }
        } else if (type == "frameRange") {
            if (tokens.size() != 4 || !parse_u32_dec(tokens[2], base.operand_a) || !parse_u32_dec(tokens[3], base.operand_b)
                || base.operand_a == 0 || base.operand_a > 0xffffu || base.operand_b > 0xffffu)
                return fail("frameRange requires period 1..65535 and offset 0..65535");
            base.type = ConditionType::FrameRange;
        } else if (type == "positionCheckX" || type == "positionCheckY"
                   || type == "originPositionCheckX" || type == "originPositionCheckY") {
            if (tokens.size() != 4 || !parse_compare(tokens[2], base.comparison)
                || !parse_u32_dec(tokens[3], base.operand_b) || base.operand_b > 0xffffu)
                return fail("position condition has invalid parameters");
            base.type = type == "positionCheckX" ? ConditionType::PositionCheckX
                      : type == "positionCheckY" ? ConditionType::PositionCheckY
                      : type == "originPositionCheckX" ? ConditionType::OriginPositionCheckX
                      : ConditionType::OriginPositionCheckY;
        } else {
            return fail("unsupported condition type '" + type + "'");
        }
        condition_index_[base.name] = pack_->conditions.size();
        pack_->conditions.push_back(base);
        Condition inverted = base;
        inverted.name = "!" + base.name;
        inverted.inverted = true;
        condition_index_[inverted.name] = pack_->conditions.size();
        pack_->conditions.push_back(std::move(inverted));
        return true;
    }

    bool parse_background(const std::vector<std::string> &tokens, std::vector<ConditionRef> refs) {
        if (!count_ok(pack_->backgrounds.size(), limits_.max_backgrounds, "backgrounds")) return false;
        if (tokens.size() < 2 || tokens.size() > 8) return fail("<background> requires 2 to 8 parameters");
        for (ConditionRef ref : refs) {
            const auto type = pack_->conditions[ref].type;
            if (type != ConditionType::TileAtPosition && type != ConditionType::SpriteAtPosition
                && type != ConditionType::MemoryCheck && type != ConditionType::MemoryCheckConstant
                && type != ConditionType::FrameRange)
                return fail("background uses a condition type unsupported by format 109");
        }
        std::string path, path_error;
        if (!normalize_member_path(tokens[0], path, path_error)) return fail("invalid background path: " + path_error);
        std::size_t image_index;
        auto existing = background_index_.find(path);
        if (existing == background_index_.end()) {
            std::vector<std::uint8_t> encoded;
            if (!read_asset(path, AssetKind::BackgroundImage, true, limits_.max_member_bytes, encoded)) return false;
            Image image;
            std::string error;
            if (!decode_png(encoded, path, limits_, image, error)) return fail(error);
            if (!add_decoded(image.argb32.size() * sizeof(std::uint32_t))) return false;
            image_index = pack_->background_images.size();
            background_index_[path] = image_index;
            pack_->background_images.push_back(std::move(image));
        } else image_index = existing->second;
        Background bg;
        bg.image_index = image_index;
        bg.conditions = std::move(refs);
        if (!parse_brightness(tokens[1], bg.brightness)) return fail("background brightness must be finite and non-negative");
        if (tokens.size() > 2 && !parse_float(tokens[2], bg.horizontal_scroll_ratio)) return fail("background horizontal scroll ratio is invalid");
        if (tokens.size() > 3 && !parse_float(tokens[3], bg.vertical_scroll_ratio)) return fail("background vertical scroll ratio is invalid");
        if (tokens.size() > 4) {
            std::uint32_t priority = 0;
            if (!parse_u32_dec(tokens[4], priority) || priority >= kBackgroundPriorityCount) return fail("background priority must be 0..39");
            bg.priority = static_cast<std::uint8_t>(priority);
        }
        if (tokens.size() > 6) {
            std::int32_t left = 0, top = 0;
            if (!parse_i32_dec(tokens[5], left) || !parse_i32_dec(tokens[6], top)) return fail("background offsets are invalid");
            bg.left = static_cast<std::uint32_t>(std::max(0, left));
            bg.top = static_cast<std::uint32_t>(std::max(0, top));
        } else if (tokens.size() == 6) return fail("background left/top offsets must be specified together");
        if (tokens.size() > 7) {
            if (tokens[7] == "Alpha") bg.blend_mode = BlendMode::Alpha;
            else if (tokens[7] == "Add") bg.blend_mode = BlendMode::Add;
            else if (tokens[7] == "Subtract") bg.blend_mode = BlendMode::Subtract;
            else return fail("background blend mode must be Alpha, Add, or Subtract");
        }
        pack_->backgrounds.push_back(std::move(bg));
        return true;
    }

    bool parse_addition(const std::vector<std::string> &tokens) {
        if (!count_ok(pack_->additions.size(), limits_.max_additions, "additions") || (tokens.size() != 6 && tokens.size() != 7))
            return tokens.size() == 6 || tokens.size() == 7 ? false : fail("<addition> requires 6 or 7 parameters");
        Addition item;
        if (!parse_tile_key(tokens[0], tokens[1], item.original) || !parse_i32_dec(tokens[2], item.offset_x)
            || !parse_i32_dec(tokens[3], item.offset_y) || !parse_tile_key(tokens[4], tokens[5], item.additional))
            return false;
        if (tokens.size() == 7 && !parse_bool(tokens[6], item.ignore_palette)) return fail("addition ignore-palette flag must be Y or N");
        pack_->additions.push_back(std::move(item));
        return true;
    }

    bool parse_fallback(const std::vector<std::string> &tokens) {
        if (!count_ok(pack_->fallbacks.size(), limits_.max_fallbacks, "fallbacks")) return false;
        if (tokens.size() != 2) return fail("<fallback> requires exactly 2 parameters");
        Fallback item;
        if (!parse_hex(tokens[0], item.tile_index) || !parse_hex(tokens[1], item.fallback_tile_index)) return fail("fallback tile indices must be hexadecimal");
        pack_->fallbacks.push_back(item);
        return true;
    }

    bool parse_audio(const std::vector<std::string> &tokens, bool bgm) {
        if (!count_ok(pack_->audio_tracks.size(), limits_.max_audio_tracks, "audio tracks")) return false;
        if ((bgm && tokens.size() != 3 && tokens.size() != 4) || (!bgm && tokens.size() != 3)) return fail("audio tag has an invalid parameter count");
        std::uint32_t album = 0, track = 0;
        if (!parse_u32_dec(tokens[0], album) || !parse_u32_dec(tokens[1], track) || album > 255 || track > 255)
            return fail("audio album and track must be 0..255");
        std::vector<std::uint8_t> encoded;
        if (!read_asset(tokens[2], AssetKind::Audio, true, limits_.max_member_bytes, encoded)) return false;
        AudioTrack decoded;
        decoded.role = bgm ? AudioRole::BackgroundMusic : AudioRole::SoundEffect;
        decoded.album = static_cast<std::uint8_t>(album);
        decoded.track = static_cast<std::uint8_t>(track);
        std::string path, path_error;
        if (!normalize_member_path(tokens[2], path, path_error)) return fail("invalid audio path: " + path_error);
        decoded.path = path;
        std::string error;
        if (!decode_audio(encoded, path, limits_, decoded, error)) return fail(error);
        if (bgm && tokens.size() == 4 && !parse_u32_dec(tokens[3], decoded.loop_position)) return fail("BGM loop position is invalid");
        const std::size_t frames = decoded.channels ? decoded.pcm.size() / decoded.channels : 0;
        if (decoded.loop_position > frames) return fail("BGM loop position lies past end of decoded audio");
        if (!add_decoded(decoded.pcm.size() * sizeof(std::int16_t))) return false;
        for (auto &entry : pack_->audio_tracks) {
            if (entry.role == decoded.role && entry.album == decoded.album && entry.track == decoded.track) { entry = std::move(decoded); return true; }
        }
        pack_->audio_tracks.push_back(std::move(decoded));
        return true;
    }

    bool parse_overscan(const std::vector<std::string> &tokens) {
        if (tokens.size() != 4) return fail("<overscan> requires top,right,bottom,left");
        Overscan o;
        if (!parse_u32_dec(tokens[0], o.top) || !parse_u32_dec(tokens[1], o.right)
            || !parse_u32_dec(tokens[2], o.bottom) || !parse_u32_dec(tokens[3], o.left)
            || o.top > kNesHeight || o.bottom > kNesHeight || o.left > kNesWidth || o.right > kNesWidth)
            return fail("overscan values exceed NES frame dimensions");
        pack_->overscan = o;
        return true;
    }

    bool parse_options(const std::vector<std::string> &tokens) {
        for (const auto &token : tokens) {
            if (token.empty()) continue;
            if (token == "disableSpriteLimit") return fail("option disableSpriteLimit is unsupported because it changes emulated hardware");
            if (token == "alternateRegisterRange") pack_->hardware_options |= HardwareOption::AlternateRegisterRange;
            else if (token == "disableCache") pack_->render_options |= RenderOption::DisableCache;
            else if (token == "disableOriginalTiles") pack_->render_options |= RenderOption::DisableOriginalTiles;
            else if (token == "automaticFallbackTiles") pack_->render_options |= RenderOption::AutomaticFallbackTiles;
            else if (token == "disableContours") result_.warnings.push_back("hires.txt line " + std::to_string(line_number_) + ": obsolete option disableContours is ignored");
            else return fail("unsupported HD pack option '" + token + "'");
        }
        return true;
    }

    bool load_palette() {
        std::vector<std::uint8_t> bytes;
        const std::string path = "palette.dat";
        std::string read_error;
        const ReadStatus status = source_.read(path, 192, bytes, read_error);
        if (status == ReadStatus::Missing) return true;
        if (status == ReadStatus::Error) return fail(read_error);
        if (bytes.size() != 192) return fail("palette.dat must be exactly 192 bytes");
        if (total_asset_bytes_ > limits_.max_total_asset_bytes - bytes.size()) return fail("pack exceeds total encoded asset memory limit");
        total_asset_bytes_ += bytes.size();
        asset_index_[path] = pack_->assets.size();
        pack_->assets.push_back(PackAsset{path, AssetKind::Palette, bytes});
        std::array<std::uint32_t, 64> palette{};
        for (std::size_t i = 0; i < palette.size(); ++i)
            palette[i] = 0xff000000u | (static_cast<std::uint32_t>(bytes[i * 3]) << 16)
                       | (static_cast<std::uint32_t>(bytes[i * 3 + 1]) << 8) | bytes[i * 3 + 2];
        pack_->palette = palette;
        return true;
    }

    AssetSource &source_;
    const LoadLimits &limits_;
    LoadResult &result_;
    std::unique_ptr<Pack> pack_;
    std::size_t line_number_ = 0, total_asset_bytes_ = 0, decoded_bytes_ = 0;
    bool version_seen_ = false;
    std::unordered_map<std::string, std::size_t> asset_index_;
    std::unordered_map<std::string, ConditionRef> condition_index_;
    std::unordered_map<std::string, std::size_t> background_index_;
};

LoadResult parse_source(AssetSource &source, const LoadLimits &limits) {
    LoadResult result;
    Parser parser(source, limits, result);
    parser.run();
    return result;
}

}  // namespace

LoadResult load_pack_directory(const std::string &directory, const LoadLimits &limits) {
    DirectorySource source(directory);
    return parse_source(source, limits);
}

LoadResult load_pack_zip_bytes(const std::uint8_t *data, std::size_t size, const LoadLimits &limits) {
    LoadResult result;
    if ((!data && size) || size == 0 || size > limits.max_archive_bytes) {
        result.error = "pack ZIP is empty or exceeds the archive size limit";
        return result;
    }
    std::vector<std::uint8_t> bytes(data, data + size);
    ZipSource source(std::move(bytes));
    if (!source.initialize(limits, result.error)) return result;
    return parse_source(source, limits);
}

LoadResult load_pack_zip(const std::string &zip_path, const LoadLimits &limits) {
    LoadResult result;
    std::uint8_t *raw = nullptr;
    std::size_t size = 0;
    const NesFileResult file = nes_file_read_all(zip_path.c_str(), limits.max_archive_bytes, &raw, &size);
    if (file != NES_FILE_OK) {
        result.error = std::string("could not read pack ZIP: ") + nes_file_result_message(file);
        return result;
    }
    result = load_pack_zip_bytes(raw, size, limits);
    std::free(raw);
    return result;
}

bool create_archive(const Pack &pack, std::vector<std::uint8_t> &zip_bytes, std::string &error) {
    zip_bytes.clear(); error.clear();
    if (pack.assets.empty()) { error = "validated pack has no source assets"; return false; }
    std::unordered_set<std::string> aliases;
    bool definition = false;
    mz_zip_archive zip{};
    if (!mz_zip_writer_init_heap(&zip, 0, 64 * 1024)) { error = "could not initialize ZIP writer"; return false; }
    bool ok = true;
    for (const auto &asset : pack.assets) {
        std::string path, path_error;
        if (!normalize_member_path(asset.path, path, path_error) || path != asset.path) { error = "validated asset has unsafe path '" + asset.path + "'"; ok = false; break; }
        if (!aliases.insert(ascii_alias(path)).second) { error = "validated assets contain duplicate path alias '" + path + "'"; ok = false; break; }
        if (path == kDefinitionName) definition = true;
        if (!mz_zip_writer_add_mem(&zip, path.c_str(), asset.bytes.data(), asset.bytes.size(), MZ_DEFAULT_COMPRESSION)) {
            error = "could not add asset '" + path + "' to ZIP"; ok = false; break;
        }
    }
    if (ok && !definition) { error = "validated pack assets do not contain hires.txt"; ok = false; }
    void *heap = nullptr; size_t size = 0;
    if (ok && !mz_zip_writer_finalize_heap_archive(&zip, &heap, &size)) { error = "could not finalize pack ZIP"; ok = false; }
    if (ok) zip_bytes.assign(static_cast<std::uint8_t *>(heap), static_cast<std::uint8_t *>(heap) + size);
    if (heap) mz_free(heap);
    mz_zip_writer_end(&zip);
    return ok;
}

}  // namespace cupid::hd
