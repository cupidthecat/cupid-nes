/*
 * update_metadata.c - Release metadata and semantic version validation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "update_checker.h"
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t number[3];
    const char *pre;
    size_t pre_size;
} Version;

static bool identifier(const char *s, size_t size, bool pre) {
    if (!size) {
        return false;
    }
    size_t first = 0;
    for (size_t i = 0; i <= size; ++i) {
        if (i == size || s[i] == '.') {
            if (i == first) {
                return false;
            }
            bool numeric = true;
            for (size_t k = first; k < i; ++k) {
                if (!isdigit((unsigned char)s[k])) {
                    numeric = false;
                }
            }
            if (pre && numeric && i - first > 1 && s[first] == '0') {
                return false;
            }
            first = i + 1;
        } else if (!isalnum((unsigned char)s[i]) && s[i] != '-') {
            return false;
        }
    }
    return true;
}

static bool parse_version(const char *s, Version *v) {
    if (!s || strlen(s) > 95) {
        return false;
    }
    memset(v, 0, sizeof(*v));
    if (*s == 'v') {
        ++s;
    }
    for (unsigned i = 0; i < 3; ++i) {
        const char *first = s;
        if (!isdigit((unsigned char)*s)) {
            return false;
        }
        while (isdigit((unsigned char)*s)) {
            unsigned digit = (unsigned)(*s++ - '0');
            if (v->number[i] > (UINT32_MAX - digit) / 10) {
                return false;
            }
            v->number[i] = v->number[i] * 10 + digit;
        }
        if (s - first > 1 && *first == '0') {
            return false;
        }
        if (i < 2 && *s++ != '.') {
            return false;
        }
    }
    if (*s == '-') {
        v->pre = ++s;
        while (*s && *s != '+') {
            ++s;
        }
        v->pre_size = (size_t)(s - v->pre);
        if (!identifier(v->pre, v->pre_size, true)) {
            return false;
        }
    }
    if (*s == '+') {
        ++s;
        return identifier(s, strlen(s), false);
    }
    return !*s;
}

bool update_version_compare(const char *a, const char *b, int *order) {
    Version x, y;
    if (!order || !parse_version(a, &x) || !parse_version(b, &y)) {
        return false;
    }
    *order = 0;
    for (unsigned i = 0; i < 3; ++i) {
        if (x.number[i] != y.number[i]) {
            *order = x.number[i] > y.number[i] ? 1 : -1;
            return true;
        }
    }
    if (!x.pre_size || !y.pre_size) {
        *order = (x.pre_size == 0) - (y.pre_size == 0);
        return true;
    }
    size_t xi = 0, yi = 0;
    while (xi < x.pre_size && yi < y.pre_size) {
        size_t xe = xi, ye = yi;
        bool xn = true, yn = true;
        while (xe < x.pre_size && x.pre[xe] != '.') {
            if (!isdigit((unsigned char)x.pre[xe])) {
                xn = false;
            }
            ++xe;
        }
        while (ye < y.pre_size && y.pre[ye] != '.') {
            if (!isdigit((unsigned char)y.pre[ye])) {
                yn = false;
            }
            ++ye;
        }
        size_t xl = xe - xi, yl = ye - yi;
        int c = 0;
        if (xn != yn) {
            c = xn ? -1 : 1;
        } else if (xn && xl != yl) {
            c = xl > yl ? 1 : -1;
        } else {
            c = memcmp(x.pre + xi, y.pre + yi, xl < yl ? xl : yl);
            if (!c && xl != yl) {
                c = xl > yl ? 1 : -1;
            }
        }
        if (c) {
            *order = c > 0 ? 1 : -1;
            return true;
        }
        xi = xe == x.pre_size ? xe : xe + 1;
        yi = ye == y.pre_size ? ye : ye + 1;
    }
    *order = (xi < x.pre_size) - (yi < y.pre_size);
    return true;
}

typedef struct {
    const char *p, *end;
    unsigned depth;
} Json;

static void ws(Json *j) {
    while (j->p < j->end && strchr(" \t\r\n", *j->p)) {
        ++j->p;
    }
}

static bool string(Json *j, char *out, size_t capacity) {
    if (j->p == j->end || *j->p++ != '"') {
        return false;
    }
    size_t used = 0;
    while (j->p < j->end && *j->p != '"') {
        unsigned char c = (unsigned char)*j->p++;
        if (c < 32) {
            return false;
        }
        if (c == '\\') {
            if (j->p == j->end) {
                return false;
            }
            c = (unsigned char)*j->p++;
            if (c == 'u') {
                unsigned code = 0;
                for (unsigned i = 0; i < 4; ++i) {
                    if (j->p == j->end || !isxdigit((unsigned char)*j->p)) {
                        return false;
                    }
                    unsigned char d = (unsigned char)*j->p++;
                    code = code * 16 + (isdigit(d) ? (unsigned)(d - '0') : (unsigned)(tolower(d) - 'a' + 10));
                }
                if (out && (code < 32 || code > 126)) {
                    return false;
                }
                c = (unsigned char)code;
            } else if (!strchr("\"\\/bfnrt", c)) {
                return false;
            } else if (out && strchr("bfnrt", c)) {
                return false;
            }
        }
        if (out) {
            if (used + 1 >= capacity) {
                return false;
            }
            out[used++] = (char)c;
        }
    }
    if (j->p == j->end) {
        return false;
    }
    ++j->p;
    if (out) {
        out[used] = 0;
    }
    return true;
}

static bool value(Json *j);

static bool container(Json *j, char close) {
    if (++j->depth > 32) {
        return false;
    }
    ++j->p;
    ws(j);
    if (j->p < j->end && *j->p == close) {
        ++j->p;
        --j->depth;
        return true;
    }
    for (;;) {
        if (close == '}') {
            if (!string(j, NULL, 0)) {
                return false;
            }
            ws(j);
            if (j->p == j->end || *j->p++ != ':') {
                return false;
            }
        }
        if (!value(j)) {
            return false;
        }
        ws(j);
        if (j->p == j->end) {
            return false;
        }
        char c = *j->p++;
        if (c == close) {
            --j->depth;
            return true;
        }
        if (c != ',') {
            return false;
        }
        ws(j);
    }
}

static bool value(Json *j) {
    ws(j);
    if (j->p == j->end) {
        return false;
    }
    if (*j->p == '"') {
        return string(j, NULL, 0);
    }
    if (*j->p == '{') {
        return container(j, '}');
    }
    if (*j->p == '[') {
        return container(j, ']');
    }
    const char *words[] = {"true", "false", "null"};
    for (unsigned i = 0; i < 3; ++i) {
        size_t n = strlen(words[i]);
        if ((size_t)(j->end - j->p) >= n && !memcmp(j->p, words[i], n)) {
            j->p += n;
            return true;
        }
    }
    if (*j->p == '-') {
        ++j->p;
    }
    if (j->p == j->end || !isdigit((unsigned char)*j->p)) {
        return false;
    }
    if (*j->p == '0') {
        ++j->p;
    } else {
        while (j->p < j->end && isdigit((unsigned char)*j->p)) {
            ++j->p;
        }
    }
    if (j->p < j->end && *j->p == '.') {
        ++j->p;
        if (j->p == j->end || !isdigit((unsigned char)*j->p)) {
            return false;
        }
        while (j->p < j->end && isdigit((unsigned char)*j->p)) {
            ++j->p;
        }
    }
    if (j->p < j->end && (*j->p == 'e' || *j->p == 'E')) {
        ++j->p;
        if (j->p < j->end && (*j->p == '+' || *j->p == '-')) {
            ++j->p;
        }
        if (j->p == j->end || !isdigit((unsigned char)*j->p)) {
            return false;
        }
        while (j->p < j->end && isdigit((unsigned char)*j->p)) {
            ++j->p;
        }
    }
    return true;
}


UpdateChannel update_version_channel(const char *version) {
    Version parsed;
    return parse_version(version, &parsed) && parsed.pre_size ? UPDATE_CHANNEL_PREVIEW : UPDATE_CHANNEL_STABLE;
}

bool update_release_url_valid(const char *url, const char *version, bool package) {
    int ignored;
    char prefix[640];
    if (!url || !update_version_compare(version, version, &ignored)) return false;
    snprintf(prefix, sizeof(prefix), "https://github.com/cupidthecat/cupid-nes/releases/%s/%s%s",
             package ? "download" : "tag", version, package ? "/" : "");
    if (!package) return !strcmp(url, prefix);
    size_t n = strlen(prefix);
    if (strncmp(url, prefix, n) || !url[n]) return false;
    const char *name = url + n;
    /* A single literal filename: no escaping, traversal, query or fragment. */
    for (const unsigned char *p = (const unsigned char *)name; *p; ++p)
        if (!((*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z') ||
              (*p >= '0' && *p <= '9') || *p == '-' || *p == '_' || *p == '.')) return false;
    return !strstr(name, "..") && name[0] != '.';
}

static bool boolean(Json *j, bool *out) {
    if (j->end - j->p >= 4 && !memcmp(j->p, "true", 4)) {
        j->p += 4; *out = true; return true;
    }
    if (j->end - j->p >= 5 && !memcmp(j->p, "false", 5)) {
        j->p += 5; *out = false; return true;
    }
    return false;
}

static bool assets(Json *j, char *package, size_t capacity) {
    if (j->p == j->end || *j->p++ != '[') return false;
    ws(j);
    if (j->p < j->end && *j->p == ']') { ++j->p; return true; }
    for (;;) {
        char name[256] = {0}, url[768] = {0};
        bool seen_name = false, seen_url = false;
        if (j->p == j->end || *j->p++ != '{') return false;
        ws(j);
        if (j->p < j->end && *j->p == '}') ++j->p;
        else for (;;) {
            char key[128];
            if (!string(j, key, sizeof(key))) return false;
            ws(j);
            if (j->p == j->end || *j->p++ != ':') return false;
            ws(j);
            if (!strcmp(key, "name")) {
                if (seen_name || !string(j, name, sizeof(name))) return false;
                seen_name = true;
            } else if (!strcmp(key, "browser_download_url")) {
                if (seen_url || !string(j, url, sizeof(url))) return false;
                seen_url = true;
            } else if (!value(j)) return false;
            ws(j);
            if (j->p == j->end) return false;
            char delimiter = *j->p++;
            if (delimiter == '}') break;
            if (delimiter != ',') return false;
            ws(j);
        }
        /* Packaging contract: exactly one architecture-specific portable ZIP. */
#if defined(_M_ARM64) || defined(__aarch64__)
        const char *wanted = "cupid-windows-arm64.zip";
#elif defined(_WIN64)
        const char *wanted = "cupid-windows-x64.zip";
#elif defined(_WIN32)
        const char *wanted = "cupid-windows-x86.zip";
#else
        const char *wanted = "";
#endif
        if (*wanted && !strcmp(name, wanted)) {
            if (*package || !*url) return false;
            const char *filename = strrchr(url, '/');
            if (!filename || strcmp(filename + 1, name)) return false;
            snprintf(package, capacity, "%s", url);
        }
        ws(j);
        if (j->p == j->end) return false;
        char delimiter = *j->p++;
        if (delimiter == ']') return true;
        if (delimiter != ',') return false;
        ws(j);
    }
}

static bool release_object(Json *j, UpdateChannel channel, UpdateRelease *parsed, bool *eligible) {
    memset(parsed, 0, sizeof(*parsed));
    unsigned seen = 0;
    bool draft = false, prerelease = false;
    ws(j);
    if (j->p == j->end || *j->p++ != '{') return false;
    for (;;) {
        ws(j);
        char key[128];
        if (!string(j, key, sizeof(key))) return false;
        ws(j);
        if (j->p == j->end || *j->p++ != ':') return false;
        ws(j);
        unsigned bit = !strcmp(key, "tag_name") ? 1 : !strcmp(key, "html_url") ? 2 :
                       !strcmp(key, "draft") ? 4 : !strcmp(key, "prerelease") ? 8 :
                       !strcmp(key, "assets") ? 16 : 0;
        if (bit && (seen & bit)) return false;
        seen |= bit;
        if (bit == 1) { if (!string(j, parsed->version, sizeof(parsed->version))) return false; }
        else if (bit == 2) { if (!string(j, parsed->url, sizeof(parsed->url))) return false; }
        else if (bit == 4) { if (!boolean(j, &draft)) return false; }
        else if (bit == 8) { if (!boolean(j, &prerelease)) return false; }
        else if (bit == 16) { if (!assets(j, parsed->package_url, sizeof(parsed->package_url))) return false; }
        else if (!value(j)) return false;
        ws(j);
        if (j->p == j->end) return false;
        char delimiter = *j->p++;
        if (delimiter == '}') break;
        if (delimiter != ',') return false;
    }
    *eligible = (seen & 15) == 15 && !draft &&
        (channel == UPDATE_CHANNEL_PREVIEW || (!prerelease && update_version_channel(parsed->version) == UPDATE_CHANNEL_STABLE)) &&
        update_release_url_valid(parsed->url, parsed->version, false);
    if (*parsed->package_url && !update_release_url_valid(parsed->package_url, parsed->version, true)) return false;
    return true;
}

bool update_metadata_select(const char *text, size_t size, UpdateChannel channel, UpdateRelease *release) {
    if (!text || !release || size > UPDATE_METADATA_LIMIT || memchr(text, 0, size)) return false;
    Json j = {text, text + size, 0};
    UpdateRelease best = {0};
    ws(&j);
    bool list = j.p < j.end && *j.p == '[';
    if (list) { ++j.p; ws(&j); }
    if (list && j.p < j.end && *j.p == ']') { ++j.p; }
    else for (;;) {
        UpdateRelease candidate;
        bool eligible;
        if (!release_object(&j, channel, &candidate, &eligible)) return false;
        int order = 0;
        if (eligible && (!best.version[0] ||
            (update_version_compare(candidate.version, best.version, &order) && order > 0))) best = candidate;
        ws(&j);
        if (!list) break;
        if (j.p == j.end) return false;
        char delimiter = *j.p++;
        if (delimiter == ']') break;
        if (delimiter != ',') return false;
    }
    ws(&j);
    if (j.p != j.end) return false;
    *release = best;
    return true;
}

bool update_metadata_parse(const char *text, size_t size, UpdateRelease *release) {
    UpdateRelease parsed;
    if (!update_metadata_select(text, size, UPDATE_CHANNEL_STABLE, &parsed) || !parsed.version[0]) return false;
    *release = parsed;
    return true;
}
