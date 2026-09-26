/*
 * update_transport.c - Bounded HTTPS release metadata transport
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "update_checker.h"
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winhttp.h>

bool update_fetch_native(void *context, char *body, size_t capacity, unsigned *status, char *error, size_t error_size) {
    (void)context;
    HINTERNET session = NULL, connection = NULL, request = NULL;
    bool ok = false;
    size_t used = 0;
    if (!body || capacity < 2 || !status) {
        return false;
    }
    *status = 0;
    session = WinHttpOpen(L"Cupid release checker", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME,
                          WINHTTP_NO_PROXY_BYPASS, 0);
    if (!session) {
        goto done;
    }
    WinHttpSetTimeouts(session, 5000, 5000, 5000, 5000);
    connection = WinHttpConnect(session, L"api.github.com", INTERNET_DEFAULT_HTTPS_PORT, 0);
    if (!connection) {
        goto done;
    }
    request = WinHttpOpenRequest(connection, L"GET", L"/repos/cupidthecat/cupid-nes/releases/latest", NULL,
                                 WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
    if (!request) {
        goto done;
    }
    DWORD redirect = WINHTTP_OPTION_REDIRECT_POLICY_NEVER;
    WinHttpSetOption(request, WINHTTP_OPTION_REDIRECT_POLICY, &redirect, sizeof(redirect));
    if (!WinHttpSendRequest(request, L"Accept: application/vnd.github+json\r\n", (DWORD)-1L, WINHTTP_NO_REQUEST_DATA, 0,
                            0, 0) ||
        !WinHttpReceiveResponse(request, NULL)) {
        goto done;
    }
    DWORD code = 0, code_size = sizeof(code);
    if (!WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                             WINHTTP_HEADER_NAME_BY_INDEX, &code, &code_size, WINHTTP_NO_HEADER_INDEX)) {
        goto done;
    }
    *status = (unsigned)code;
    if (code != 200) {
        body[0] = 0;
        ok = true;
        goto done;
    }
    Uint32 started = SDL_GetTicks();
    for (;;) {
        DWORD read = 0;
        if (used + 1 >= capacity || SDL_GetTicks() - started > 15000) {
            goto done;
        }
        DWORD available = (DWORD)(capacity - used - 1);
        if (available > 8192) {
            available = 8192;
        }
        if (!WinHttpReadData(request, body + used, available, &read)) {
            goto done;
        }
        if (!read) {
            break;
        }
        if (memchr(body + used, 0, read)) {
            goto done;
        }
        used += read;
    }
    body[used] = 0;
    ok = true;
done:
    if (!ok && error && error_size) {
        snprintf(error, error_size, "Update server unavailable or response exceeds limits (network error %lu)",
                 (unsigned long)GetLastError());
    }
    if (request) {
        WinHttpCloseHandle(request);
    }
    if (connection) {
        WinHttpCloseHandle(connection);
    }
    if (session) {
        WinHttpCloseHandle(session);
    }
    return ok;
}
#else
#include <curl/curl.h>

typedef struct {
    char *body;
    size_t used, capacity;
} Download;

static size_t receive(void *data, size_t size, size_t count, void *context) {
    Download *d = context;
    if (count && size > SIZE_MAX / count) {
        return 0;
    }
    size_t bytes = size * count;
    if (bytes >= d->capacity - d->used || memchr(data, 0, bytes)) {
        return 0;
    }
    memcpy(d->body + d->used, data, bytes);
    d->used += bytes;
    d->body[d->used] = 0;
    return bytes;
}

bool update_fetch_native(void *context, char *body, size_t capacity, unsigned *status, char *error, size_t error_size) {
    (void)context;
    if (!body || capacity < 2 || !status) {
        return false;
    }
    *status = 0;
    body[0] = 0;
    CURL *curl = curl_easy_init();
    if (!curl) {
        return false;
    }
    Download download = {body, 0, capacity};
    curl_easy_setopt(curl, CURLOPT_URL, "https://api.github.com/repos/cupidthecat/cupid-nes/releases/latest");
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "Cupid release checker");
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 15000L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, receive);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &download);
    CURLcode result = curl_easy_perform(curl);
    long code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    *status = (unsigned)code;
    if (result != CURLE_OK && error && error_size) {
        snprintf(error, error_size, "Update check failed: %s", curl_easy_strerror(result));
    }
    curl_easy_cleanup(curl);
    return result == CURLE_OK;
}
#endif
