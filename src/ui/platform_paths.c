/*
 * platform_paths.c - Resolve folders and open them with the desktop file manager
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef _WIN32
#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 700
#endif
#endif
#include "platform_frontend.h"
#include "../util/file_io.h"
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/stat.h>
#endif

static void report(char *error, size_t capacity, const char *message) {
    if (error && capacity) snprintf(error, capacity, "%s", message);
}

static char *absolute_directory(const char *path) {
#ifdef _WIN32
    int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1, NULL, 0);
    if (count <= 0 || count > NES_FILE_PATH_LIMIT) return NULL;
    wchar_t *input = malloc((size_t)count * sizeof(*input));
    if (!input) return NULL;
    if (!MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1, input, count)) {
        free(input);
        return NULL;
    }
    DWORD length = GetFullPathNameW(input, 0, NULL, NULL);
    wchar_t *absolute = length && length <= NES_FILE_PATH_LIMIT
        ? malloc((size_t)length * sizeof(*absolute)) : NULL;
    DWORD written = absolute ? GetFullPathNameW(input, length, absolute, NULL) : 0;
    free(input);
    if (!written || written >= length) { free(absolute); return NULL; }
    DWORD attributes = GetFileAttributesW(absolute);
    if (attributes == INVALID_FILE_ATTRIBUTES || !(attributes & FILE_ATTRIBUTE_DIRECTORY)) {
        free(absolute);
        return NULL;
    }
    count = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, absolute, -1, NULL, 0, NULL, NULL);
    char *utf8 = count > 0 ? malloc((size_t)count) : NULL;
    if (utf8 && !WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, absolute, -1, utf8, count, NULL, NULL)) {
        free(utf8);
        utf8 = NULL;
    }
    free(absolute);
    if (utf8) for (char *p = utf8; *p; ++p) if (*p == '\\') *p = '/';
    return utf8;
#else
    char *absolute = realpath(path, NULL);
    struct stat metadata;
    if (absolute && (stat(absolute, &metadata) || !S_ISDIR(metadata.st_mode))) {
        free(absolute);
        absolute = NULL;
    }
    return absolute;
#endif
}

static bool uri_plain(unsigned char value) {
    return (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z')
        || (value >= '0' && value <= '9') || strchr("/-._~:", value) != NULL;
}

bool frontend_folder_uri(const char *path, char *uri, size_t uri_size,
                          char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!path || !*path || !uri || !uri_size || strlen(path) >= NES_FILE_PATH_LIMIT) {
        report(error, error_size, "Choose a valid folder first");
        return false;
    }
    for (const unsigned char *p = (const unsigned char *)path; *p; ++p) {
        if (*p < 0x20 || *p == 0x7F) {
            report(error, error_size, "The folder path contains a control character");
            return false;
        }
    }
    char *absolute = absolute_directory(path);
    if (!absolute) {
        report(error, error_size, "The folder does not exist or cannot be accessed");
        return false;
    }
    const char *value = absolute;
    const char *prefix = "file://";
#ifdef _WIN32
    if (!strncmp(value, "//?/UNC/", 8)) value += 8;
    else {
        if (!strncmp(value, "//?/", 4)) value += 4;
        if (value[0] == '/' && value[1] == '/') value += 2;
        else prefix = "file:///";
    }
#endif
    size_t length = strlen(prefix);
    for (const unsigned char *p = (const unsigned char *)value; *p; ++p)
        length += uri_plain(*p) ? 1u : 3u;
    if (length >= uri_size) {
        free(absolute);
        report(error, error_size, "The folder URI is too long");
        return false;
    }
    char *cursor = uri;
    size_t prefix_size = strlen(prefix);
    memcpy(cursor, prefix, prefix_size);
    cursor += prefix_size;
    static const char digits[] = "0123456789ABCDEF";
    for (const unsigned char *p = (const unsigned char *)value; *p; ++p) {
        if (uri_plain(*p)) *cursor++ = (char)*p;
        else { *cursor++ = '%'; *cursor++ = digits[*p >> 4]; *cursor++ = digits[*p & 15]; }
    }
    *cursor = '\0';
    free(absolute);
    return true;
}

bool frontend_open_folder(const char *path, char *error, size_t error_size) {
    size_t capacity = (size_t)NES_FILE_PATH_LIMIT * 3u + 16u;
    char *uri = malloc(capacity);
    if (!uri) {
        report(error, error_size, "Not enough memory to open the folder");
        return false;
    }
    bool opened = frontend_folder_uri(path, uri, capacity, error, error_size);
    if (opened && SDL_OpenURL(uri)) {
        report(error, error_size, SDL_GetError());
        opened = false;
    }
    free(uri);
    return opened;
}
