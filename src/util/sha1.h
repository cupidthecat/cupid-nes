/* SHA-1 image identity. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_SHA1_H
#define CUPID_SHA1_H
#include <stddef.h>
void nes_sha1(const void *data, size_t size, char hex[41]);
#endif
