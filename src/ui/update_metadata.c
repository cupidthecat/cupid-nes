/*
 * update_metadata.c - Release metadata and semantic version validation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "update_checker.h"
#include <ctype.h>
#include <stdint.h>
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

bool update_metadata_parse(const char *text, size_t size, UpdateRelease *release) {
    if (!text || !release || size > UPDATE_METADATA_LIMIT || memchr(text, 0, size)) {
        return false;
    }
    Json j = {text, text + size, 0};
    UpdateRelease parsed = {{0}, {0}};
    bool tag = false, url = false;
    ws(&j);
    if (j.p == j.end || *j.p++ != '{') {
        return false;
    }
    for (;;) {
        ws(&j);
        char key[128];
        if (!string(&j, key, sizeof(key))) {
            return false;
        }
        ws(&j);
        if (j.p == j.end || *j.p++ != ':') {
            return false;
        }
        ws(&j);
        if (!strcmp(key, "tag_name")) {
            if (tag || !string(&j, parsed.version, sizeof(parsed.version))) {
                return false;
            }
            tag = true;
        } else if (!strcmp(key, "html_url")) {
            if (url || !string(&j, parsed.url, sizeof(parsed.url))) {
                return false;
            }
            url = true;
        } else if (!strcmp(key, "draft") || !strcmp(key, "prerelease")) {
            if (j.end - j.p < 5 || memcmp(j.p, "false", 5)) {
                return false;
            }
            j.p += 5;
        } else if (!value(&j)) {
            return false;
        }
        ws(&j);
        if (j.p == j.end) {
            return false;
        }
        char c = *j.p++;
        if (c == '}') {
            break;
        }
        if (c != ',') {
            return false;
        }
    }
    ws(&j);
    int ignored;
    const char prefix[] = "https://github.com/cupidthecat/cupid-nes/releases/tag/";
    if (j.p != j.end || !tag || !url || !update_version_compare(parsed.version, parsed.version, &ignored) ||
        strncmp(parsed.url, prefix, sizeof(prefix) - 1) || !parsed.url[sizeof(prefix) - 1]) {
        return false;
    }
    for (const unsigned char *p = (const unsigned char *)parsed.url; *p; ++p) {
        if (*p <= 32 || *p >= 127 || *p == '\\') {
            return false;
        }
    }
    *release = parsed;
    return true;
}
