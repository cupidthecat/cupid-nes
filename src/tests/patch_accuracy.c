/*
 * patch_accuracy.c - Image patch formats and malformed input
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "board_tests.h"
#include "../media/patch.h"
#include "../rom/game_db.h"

/* These fixed examples exercise every BPS command and both directions of a
 * growing UPS patch. Their CRC fields were computed independently. */
static const uint8_t bps_commands[] = {
    0x42,0x50,0x53,0x31,0x86,0x91,0x80,0x88,0x85,0x58,0x59,0x8a,0x86,0x93,0x86,0x82,
    0x89,0x8b,0x91,0xef,0x39,0x8e,0x4b,0xca,0xa4,0xca,0xcd,0x33,0x06,0x22,0x63
};
static const uint8_t bps_overlap[] = {
    0x42,0x50,0x53,0x31,0x81,0x86,0x80,0x81,0x61,0x93,0x80,0x83,0x16,0xdc,0x8c,0xf8,
    0x19,0xe4,0x5a,0xcc,0x0d,0xe8,0x12
};
static const uint8_t ups_growing[] = {
    0x55,0x50,0x53,0x31,0x86,0x89,0x82,0x32,0x00,0x82,0x58,0x59,0x5a,0x00,0xef,0x39,
    0x8e,0x4b,0xdd,0xf8,0xef,0xe4,0xbe,0x3e,0x80,0x49
};

static void refresh_patch_crc(uint8_t *patch, size_t size) {
    uint32_t crc = game_db_crc32(patch, size - 4);
    for (unsigned i = 0; i < 4; i++) {
        patch[size - 4 + i] = (uint8_t)(crc >> (i * 8));
    }
}

static int expect_patch(const uint8_t *source, size_t source_size,
                         const uint8_t *patch, size_t patch_size,
                         const uint8_t *expected, size_t expected_size) {
    uint8_t *output = NULL;
    size_t size = 0;
    uint32_t original_crc = game_db_crc32(source, source_size);
    BOARD_CHECK(nes_patch_apply(source, source_size, patch, patch_size,
                                16 * 1024 * 1024, &output, &size) == NES_PATCH_OK);
    BOARD_CHECK(size == expected_size && (!size || !memcmp(output, expected, size)));
    BOARD_CHECK(game_db_crc32(source, source_size) == original_crc);
    free(output);
    return 0;
}

static int test_ips_records(void) {
    const uint8_t source[] = {'a','b','c','d'};
    const uint8_t patch[] = {
        'P','A','T','C','H',
        0,0,6, 0,3, 'X','Y','Z',
        0,0,1, 0,0, 0,4, 0x7f,
        'E','O','F'
    };
    const uint8_t expected[] = {'a',0x7f,0x7f,0x7f,0x7f,0,'X','Y','Z'};
    BOARD_CHECK(expect_patch(source, sizeof(source), patch, sizeof(patch), expected, sizeof(expected)) == 0);
    uint8_t truncated[sizeof(patch) + 3];
    memcpy(truncated, patch, sizeof(patch));
    truncated[sizeof(patch)] = 0;
    truncated[sizeof(patch) + 1] = 0;
    truncated[sizeof(patch) + 2] = 7;
    BOARD_CHECK(expect_patch(source, sizeof(source), truncated, sizeof(truncated), expected, 7) == 0);
    truncated[sizeof(patch) + 2] = 20;
    BOARD_CHECK(expect_patch(source, sizeof(source), truncated, sizeof(truncated), expected, sizeof(expected)) == 0);

    uint8_t *output = NULL;
    size_t size = 0;
    BOARD_CHECK(nes_patch_apply(source, sizeof(source), patch, sizeof(patch), 8, &output, &size) == NES_PATCH_TOO_LARGE);
    BOARD_CHECK(!output && !size);
    for (size_t prefix = 5; prefix < sizeof(patch); prefix++) {
        BOARD_CHECK(nes_patch_apply(source, sizeof(source), patch, prefix, 32, &output, &size) == NES_PATCH_INVALID);
        BOARD_CHECK(!output && !size);
    }

    BOARD_CHECK(nes_patch_apply(source, sizeof(source), truncated, sizeof(truncated) - 1,
                                32, &output, &size) == NES_PATCH_INVALID);
    const uint8_t bad_rle[] = {'P','A','T','C','H',0,0,0,0,0,0,0,0,'E','O','F'};
    BOARD_CHECK(nes_patch_apply(source, sizeof(source), bad_rle, sizeof(bad_rle),
                                32, &output, &size) == NES_PATCH_INVALID);
    return 0;
}

static int test_bps_commands(void) {
    const uint8_t source[] = "abcdef";
    const uint8_t expected[] = "abcXYdefXYdefcabc";
    BOARD_CHECK(expect_patch(source, 6, bps_commands, sizeof(bps_commands), expected, 17) == 0);
    BOARD_CHECK(expect_patch((const uint8_t *)"x", 1, bps_overlap, sizeof(bps_overlap),
                             (const uint8_t *)"aaaaaa", 6) == 0);
    uint8_t broken[sizeof(bps_commands)];
    uint8_t *output = NULL;
    size_t size = 0;
    memcpy(broken, bps_commands, sizeof(broken));
    broken[12] = 0x83; /* Source-copy cursor would move before byte zero. */
    refresh_patch_crc(broken, sizeof(broken));
    BOARD_CHECK(nes_patch_apply(source, 6, broken, sizeof(broken), 64, &output, &size) == NES_PATCH_INVALID);
    BOARD_CHECK(!output && !size);
    memcpy(broken, bps_commands, sizeof(broken));
    broken[7] = 0x98; /* SourceRead requests seven bytes from a six-byte image. */
    refresh_patch_crc(broken, sizeof(broken));
    BOARD_CHECK(nes_patch_apply(source, 6, broken, sizeof(broken), 64, &output, &size) == NES_PATCH_INVALID);
    memcpy(broken, bps_commands, sizeof(broken));
    broken[6] = 0xff; /* Metadata extends into or beyond the checksum footer. */
    refresh_patch_crc(broken, sizeof(broken));
    BOARD_CHECK(nes_patch_apply(source, 6, broken, sizeof(broken), 64, &output, &size) == NES_PATCH_INVALID);

    uint8_t forward[sizeof(bps_overlap)];
    memcpy(forward, bps_overlap, sizeof(forward));
    forward[10] = 0x82; /* A target-copy cannot begin at an unwritten byte. */
    refresh_patch_crc(forward, sizeof(forward));
    BOARD_CHECK(nes_patch_apply((const uint8_t *)"x", 1, forward, sizeof(forward),
                                64, &output, &size) == NES_PATCH_INVALID);
    forward[10] = 0x83;
    refresh_patch_crc(forward, sizeof(forward));
    BOARD_CHECK(nes_patch_apply((const uint8_t *)"x", 1, forward, sizeof(forward),
                                64, &output, &size) == NES_PATCH_INVALID);

    uint8_t extra[sizeof(bps_overlap) + 1];
    size_t instructions = sizeof(bps_overlap) - 12;
    memcpy(extra, bps_overlap, instructions);
    extra[instructions] = 0x80;
    memcpy(extra + instructions + 1, bps_overlap + instructions, 12);
    refresh_patch_crc(extra, sizeof(extra));
    BOARD_CHECK(nes_patch_apply((const uint8_t *)"x", 1, extra, sizeof(extra),
                                64, &output, &size) == NES_PATCH_INVALID);
    return 0;
}

static int test_ups_directions(void) {
    const uint8_t source[] = "abcdef";
    const uint8_t expected[] = "abQdefXYZ";
    BOARD_CHECK(expect_patch(source, 6, ups_growing, sizeof(ups_growing), expected, 9) == 0);
    BOARD_CHECK(expect_patch(expected, 9, ups_growing, sizeof(ups_growing), source, 6) == 0);
    uint8_t broken[sizeof(ups_growing)];
    memcpy(broken, ups_growing, sizeof(broken));
    broken[6] = 0xff; /* Relative offset outside either image. */
    refresh_patch_crc(broken, sizeof(broken));
    uint8_t *output = NULL;
    size_t size = 0;
    BOARD_CHECK(nes_patch_apply(source, 6, broken, sizeof(broken), 256, &output, &size) == NES_PATCH_INVALID);
    memcpy(broken, ups_growing, sizeof(broken));
    broken[13] = 1; /* Nonzero XOR data outside the declared span. */
    refresh_patch_crc(broken, sizeof(broken));
    BOARD_CHECK(nes_patch_apply(source, 6, broken, sizeof(broken), 256, &output, &size) == NES_PATCH_INVALID);
    BOARD_CHECK(!output && !size);
    return 0;
}

static int test_patch_checksums(void) {
    const uint8_t *patches[] = {bps_commands, ups_growing};
    const size_t sizes[] = {sizeof(bps_commands), sizeof(ups_growing)};
    for (size_t p = 0; p < 2; p++) {
        uint8_t *output = NULL;
        size_t size = 0;
        BOARD_CHECK(nes_patch_apply((const uint8_t *)"abcdeg", 6, patches[p], sizes[p],
                                    64, &output, &size) == NES_PATCH_CHECKSUM);
        BOARD_CHECK(!output && !size);
        uint8_t broken[64];
        memcpy(broken, patches[p], sizes[p]);
        broken[sizes[p] - 1] ^= 1;
        BOARD_CHECK(nes_patch_apply((const uint8_t *)"abcdef", 6, broken, sizes[p],
                                    64, &output, &size) == NES_PATCH_CHECKSUM);
        memcpy(broken, patches[p], sizes[p]);
        broken[sizes[p] - 8] ^= 1;
        refresh_patch_crc(broken, sizes[p]);
        BOARD_CHECK(nes_patch_apply((const uint8_t *)"abcdef", 6, broken, sizes[p],
                                    64, &output, &size) == NES_PATCH_CHECKSUM);
        BOARD_CHECK(!output && !size);
    }

    return 0;
}

static int test_patch_limits(void) {
    uint8_t *output = NULL;
    size_t size = 0;
    BOARD_CHECK(nes_patch_apply((const uint8_t *)"abcdef", 6, bps_commands, sizeof(bps_commands),
                                16, &output, &size) == NES_PATCH_TOO_LARGE);
    BOARD_CHECK(nes_patch_apply((const uint8_t *)"abcdef", 6, ups_growing, sizeof(ups_growing),
                                8, &output, &size) == NES_PATCH_TOO_LARGE);
    uint8_t overflow[48] = {'B','P','S','1'};
    memset(overflow + 4, 0x7f, sizeof(overflow) - 16);
    refresh_patch_crc(overflow, sizeof(overflow));
    BOARD_CHECK(nes_patch_apply((const uint8_t *)"abcdef", 6, overflow, sizeof(overflow),
                                64, &output, &size) == NES_PATCH_INVALID);
    BOARD_CHECK(nes_patch_apply(NULL, 1, bps_commands, sizeof(bps_commands),
                                64, &output, &size) == NES_PATCH_INVALID);
    BOARD_CHECK(nes_patch_apply(NULL, 0, (const uint8_t *)"bad!", 4,
                                64, &output, &size) == NES_PATCH_UNSUPPORTED);
    BOARD_CHECK(!output && !size);
    return 0;
}

static int test_ips_creation(void) {
    const size_t reserved = 0x454f46u;
    size_t source_size = reserved + 10;
    uint8_t *source = (uint8_t *)calloc(source_size, 1);
    uint8_t *target = (uint8_t *)calloc(source_size + 64, 1);
    BOARD_CHECK(source && target);
    target[reserved] = 0x99;
    memset(target + 16, 0x5d, 70000); /* More than one maximal RLE record. */
    target[source_size + 63] = 0x7a;
    uint8_t *patch = NULL;
    size_t patch_size = 0;
    BOARD_CHECK(nes_patch_create_ips(source, source_size, target, source_size + 64,
                                     source_size * 2, &patch, &patch_size) == NES_PATCH_OK);
    BOARD_CHECK(expect_patch(source, source_size, patch, patch_size, target, source_size + 64) == 0);
    free(patch);
    BOARD_CHECK(nes_patch_create_ips(target, source_size + 64, source, source_size,
                                     source_size * 2, &patch, &patch_size) == NES_PATCH_OK);
    BOARD_CHECK(expect_patch(target, source_size + 64, patch, patch_size, source, source_size) == 0);
    free(patch);
    BOARD_CHECK(nes_patch_create_ips(source, source_size, NULL, 0,
                                     64, &patch, &patch_size) == NES_PATCH_OK);
    BOARD_CHECK(expect_patch(source, source_size, patch, patch_size, NULL, 0) == 0);
    free(patch);
    BOARD_CHECK(nes_patch_create_ips(source, source_size, target, source_size + 64,
                                     8, &patch, &patch_size) == NES_PATCH_TOO_LARGE);
    BOARD_CHECK(!patch && !patch_size);
    free(source);
    free(target);
    return 0;
}

int test_patch_accuracy(void) {
    int failures = test_ips_records() + test_bps_commands() + test_ups_directions()
                 + test_patch_checksums() + test_patch_limits() + test_ips_creation();
    printf("Image patches: 6 groups, %d failures\n", failures);
    return failures;
}
