/*
 * fm2_accuracy.c - FM2/FM3 container codec regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../replay/fm2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition);                                            \
            ok = false;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static char *dup_text(const char *text) {
    size_t size = strlen(text) + 1;
    char *copy = malloc(size);

    if (copy) {
        memcpy(copy, text, size);
    }
    return copy;
}

static uint32_t read_u32le(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static bool equal_string(const char *left, const char *right) {
    const char *a = left ? left : "";
    const char *b = right ? right : "";

    return strcmp(a, b) == 0;
}

static bool equal_blob(const NesFm2Blob *left, const NesFm2Blob *right) {
    return left->size == right->size && (left->size == 0 || memcmp(left->data, right->data, left->size) == 0);
}

static bool equal_strings(const NesFm2StringList *left, const NesFm2StringList *right) {
    if (left->count != right->count) {
        return false;
    }
    for (size_t i = 0; i < left->count; ++i) {
        if (!equal_string(left->items[i], right->items[i])) {
            return false;
        }
    }
    return true;
}

static bool equal_frame(const NesFm2Frame *left, const NesFm2Frame *right) {
    if (left->commands != right->commands || memcmp(left->pads, right->pads, 4) != 0) {
        return false;
    }
    for (size_t i = 0; i < 2; ++i) {
        if (left->zappers[i].x != right->zappers[i].x || left->zappers[i].y != right->zappers[i].y ||
            left->zappers[i].button != right->zappers[i].button || left->zappers[i].bogo != right->zappers[i].bogo ||
            left->zappers[i].zaphit != right->zappers[i].zaphit) {
            return false;
        }
    }
    return true;
}

static bool equal_movie(const NesFm2Movie *left, const NesFm2Movie *right) {
    if (left->version != right->version || left->emu_version != right->emu_version ||
        left->rerecord_count != right->rerecord_count || left->pal != right->pal ||
        !equal_string(left->rom_filename, right->rom_filename) ||
        memcmp(left->rom_md5, right->rom_md5, NES_FM2_MD5_SIZE) != 0 ||
        memcmp(left->guid, right->guid, NES_FM2_GUID_SIZE) != 0 || left->fourscore != right->fourscore ||
        left->microphone != right->microphone || memcmp(left->ports, right->ports, sizeof(left->ports)) != 0 ||
        left->fds != right->fds || left->new_ppu != right->new_ppu || left->ram_init_option != right->ram_init_option ||
        left->ram_init_seed != right->ram_init_seed || !equal_strings(&left->comments, &right->comments) ||
        !equal_strings(&left->subtitles, &right->subtitles) || !equal_blob(&left->savestate, &right->savestate) ||
        !equal_blob(&left->saveram, &right->saveram) || left->extension_count != right->extension_count ||
        left->frame_count != right->frame_count || left->project_present != right->project_present) {
        return false;
    }
    for (size_t i = 0; i < left->extension_count; ++i) {
        if (!equal_string(left->extensions[i].key, right->extensions[i].key) ||
            !equal_string(left->extensions[i].value, right->extensions[i].value)) {
            return false;
        }
    }
    for (size_t i = 0; i < left->frame_count; ++i) {
        if (!equal_frame(&left->frames[i], &right->frames[i])) {
            return false;
        }
    }
    if (!left->project_present) {
        return true;
    }
    if (left->project_version != right->project_version ||
        left->project_saved_modules != right->project_saved_modules) {
        return false;
    }
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        if (left->project_modules[i].size != right->project_modules[i].size ||
            (left->project_modules[i].size != 0 && memcmp(left->project_modules[i].data, right->project_modules[i].data,
                                                          left->project_modules[i].size) != 0)) {
            return false;
        }
    }
    return true;
}

static bool export_movie(const NesFm2Movie *movie, NesFm2Container container, uint8_t **data, size_t *size) {
    NesFm2Diagnostic diagnostic;
    size_t measured = 0;
    size_t written = 0;
    uint8_t *output;

    *data = NULL;
    *size = 0;
    if (nes_fm2_measure(movie, container, &measured, &diagnostic) != NES_FM2_OK) {
        return false;
    }
    output = malloc(measured ? measured : 1);
    if (!output) {
        return false;
    }
    if (nes_fm2_export(movie, container, output, measured, &written, &diagnostic) != NES_FM2_OK ||
        written != measured) {
        free(output);
        return false;
    }
    *data = output;
    *size = written;
    return true;
}

static bool contains_bytes(const uint8_t *data, size_t size, const char *needle) {
    size_t length = strlen(needle);

    if (length == 0 || length > size) {
        return false;
    }
    for (size_t i = 0; i <= size - length; ++i) {
        if (memcmp(data + i, needle, length) == 0) {
            return true;
        }
    }
    return false;
}

static bool test_text_round_trip(void) {
    static const char text[] = "version 3\n"
                               "emuVersion 20606\n"
                               "rerecordCount 7\n"
                               "palFlag 1\n"
                               "romFilename fixture.nes\n"
                               "romChecksum base64:AAECAwQFBgcICQoLDA0ODw==\n"
                               "guid 00112233-4455-6677-8899-AABBCCDDEEFF\n"
                               "fourscore 0\n"
                               "microphone 1\n"
                               "port0 1\n"
                               "port1 2\n"
                               "port2 0\n"
                               "FDS 1\n"
                               "NewPPU 1\n"
                               "RAMInitOption 2\n"
                               "RAMInitSeed -1\n"
                               "comment author test\n"
                               "subtitle 12 caption\n"
                               "x-extra retained\n"
                               "savestate base64:AQIDBA==\n"
                               "saveram base64:qrvM\n"
                               "|65|R......A|010 020 1 0 12345||\n"
                               "|0|.L....B.|255 000 0 1 18446744073709551615||\n";
    NesFm2Movie movie;
    NesFm2Movie reparsed;
    NesFm2Diagnostic diagnostic;
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    bool ok = true;

    nes_fm2_movie_init(&movie);
    nes_fm2_movie_init(&reparsed);
    CHECK(nes_fm2_parse((const uint8_t *)text, sizeof(text) - 1, NULL, &movie, &diagnostic) == NES_FM2_OK);
    CHECK(movie.original_container == NES_FM2_TEXT);
    CHECK(movie.frame_count == 2 && movie.frames[0].commands == 65);
    CHECK(movie.frames[0].pads[0] == 0x81 && movie.frames[1].pads[0] == 0x42);
    CHECK(movie.frames[0].zappers[1].x == 10 && movie.frames[0].zappers[1].y == 20);
    CHECK(movie.frames[0].zappers[1].button == 1 && movie.frames[0].zappers[1].zaphit == 12345);
    CHECK(movie.frames[1].zappers[1].zaphit == UINT64_MAX);
    CHECK(movie.ram_init_seed == UINT32_MAX);
    CHECK(movie.comments.count == 1 && strcmp(movie.comments.items[0], "author test") == 0);
    CHECK(movie.subtitles.count == 1 && strcmp(movie.subtitles.items[0], "12 caption") == 0);
    CHECK(movie.extension_count == 1 && strcmp(movie.extensions[0].key, "x-extra") == 0 &&
          strcmp(movie.extensions[0].value, "retained") == 0);
    CHECK(movie.savestate.size == 4 && movie.savestate.data[3] == 4);
    CHECK(movie.saveram.size == 3 && movie.saveram.data[0] == 0xaa && movie.saveram.data[2] == 0xcc);
    for (size_t i = 0; i < NES_FM2_MD5_SIZE; ++i) {
        CHECK(movie.rom_md5[i] == (uint8_t)i);
    }
    CHECK(export_movie(&movie, NES_FM2_TEXT, &encoded, &encoded_size));
    CHECK(encoded_size == sizeof(text) - 1 && memcmp(encoded, text, encoded_size) == 0);
    CHECK(nes_fm2_parse(encoded, encoded_size, NULL, &reparsed, &diagnostic) == NES_FM2_OK);
    CHECK(equal_movie(&movie, &reparsed));

cleanup:
    free(encoded);
    nes_fm2_movie_free(&reparsed);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool test_binary_round_trip(void) {
    NesFm2Movie movie;
    NesFm2Movie reparsed;
    NesFm2Diagnostic diagnostic;
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    bool ok = true;

    nes_fm2_movie_init(&movie);
    nes_fm2_movie_init(&reparsed);
    movie.emu_version = 20700;
    movie.ports[0] = NES_FM2_PORT_ZAPPER;
    movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.rom_filename = dup_text("binary.nes");
    movie.frames = calloc(2, sizeof(*movie.frames));
    CHECK(movie.rom_filename && movie.frames);
    movie.frame_count = 2;
    movie.frames[0].commands = NES_FM2_COMMAND_RESET | NES_FM2_COMMAND_VS_COIN_1;
    movie.frames[0].pads[1] = 0xa5;
    movie.frames[0].zappers[0].x = 17;
    movie.frames[0].zappers[0].y = 250;
    movie.frames[0].zappers[0].button = 1;
    movie.frames[0].zappers[0].bogo = 2;
    movie.frames[0].zappers[0].zaphit = UINT64_C(0x8877665544332211);
    movie.frames[1].commands = NES_FM2_COMMAND_POWER;
    movie.frames[1].pads[1] = 0x5a;
    movie.frames[1].zappers[0].zaphit = UINT64_MAX;

    CHECK(export_movie(&movie, NES_FM2_BINARY, &encoded, &encoded_size));
    CHECK(contains_bytes(encoded, encoded_size, "binary 1\n"));
    CHECK(!contains_bytes(encoded, encoded_size, "length "));
    CHECK(nes_fm2_parse(encoded, encoded_size, NULL, &reparsed, &diagnostic) == NES_FM2_OK);
    CHECK(reparsed.original_container == NES_FM2_BINARY);
    CHECK(equal_movie(&movie, &reparsed));

cleanup:
    free(encoded);
    nes_fm2_movie_free(&reparsed);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool test_fourscore_round_trip(void) {
    NesFm2Movie movie;
    NesFm2Movie reparsed;
    NesFm2Diagnostic diagnostic;
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    bool ok = true;

    nes_fm2_movie_init(&movie);
    nes_fm2_movie_init(&reparsed);
    movie.fourscore = true;
    movie.frames = calloc(1, sizeof(*movie.frames));
    CHECK(movie.frames);
    movie.frame_count = 1;
    movie.frames[0].commands = NES_FM2_COMMAND_POWER;
    movie.frames[0].pads[0] = 0x81;
    movie.frames[0].pads[1] = 0x42;
    movie.frames[0].pads[2] = 0x24;
    movie.frames[0].pads[3] = 0x18;

    CHECK(export_movie(&movie, NES_FM2_BINARY, &encoded, &encoded_size));
    CHECK(nes_fm2_parse(encoded, encoded_size, NULL, &reparsed, &diagnostic) == NES_FM2_OK);
    CHECK(reparsed.fourscore && reparsed.frame_count == 1);
    CHECK(equal_movie(&movie, &reparsed));

cleanup:
    free(encoded);
    nes_fm2_movie_free(&reparsed);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool install_module(NesFm2Movie *movie, size_t index, size_t size, uint8_t seed) {
    uint8_t *data = size ? malloc(size) : NULL;

    if (size && !data) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        data[i] = (uint8_t)(seed + i * 17u);
    }
    movie->project_modules[index].data = data;
    movie->project_modules[index].size = size;
    return true;
}

static bool test_fm3_round_trip_and_offsets(void) {
    static const size_t module_sizes[NES_FM3_PROJECT_MODULE_COUNT] = {3, 0, 5, 2, 7, 1};
    NesFm2Movie movie;
    NesFm2Movie reparsed;
    NesFm2Diagnostic diagnostic;
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    size_t modules_size = 0;
    size_t project_start;
    size_t expected_offset;
    size_t measured = 0;
    bool ok = true;

    nes_fm2_movie_init(&movie);
    nes_fm2_movie_init(&reparsed);
    movie.ports[0] = NES_FM2_PORT_GAMEPAD;
    movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.rom_filename = dup_text("project.nes");
    movie.frames = calloc(3, sizeof(*movie.frames));
    CHECK(movie.rom_filename && movie.frames);
    movie.frame_count = 3;
    movie.frames[0].pads[0] = 0x81;
    movie.frames[1].pads[1] = 0x42;
    movie.frames[2].commands = NES_FM2_COMMAND_RESET;
    movie.project_present = true;
    movie.project_timeline_valid = true;
    movie.project_version = 3;
    movie.project_saved_modules = 0x3f;
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        CHECK(install_module(&movie, i, module_sizes[i], (uint8_t)(0x20 + i * 13u)));
        modules_size += module_sizes[i];
    }

    CHECK(export_movie(&movie, NES_FM3_PROJECT, &encoded, &encoded_size));
    CHECK(contains_bytes(encoded, encoded_size, "binary 1\n"));
    CHECK(contains_bytes(encoded, encoded_size, "length 3\n"));
    CHECK(encoded_size >= 36 + modules_size);
    project_start = encoded_size - 36 - modules_size;
    CHECK(read_u32le(encoded + project_start) == 3);
    CHECK(read_u32le(encoded + project_start + 4) == 0x3f);
    CHECK(read_u32le(encoded + project_start + 8) == NES_FM3_PROJECT_MODULE_COUNT);
    expected_offset = project_start + 36;
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        CHECK(read_u32le(encoded + project_start + 12 + i * 4) == expected_offset);
        expected_offset += module_sizes[i];
    }
    CHECK(expected_offset == encoded_size);
    CHECK(nes_fm2_parse(encoded, encoded_size, NULL, &reparsed, &diagnostic) == NES_FM2_OK);
    CHECK(reparsed.original_container == NES_FM3_PROJECT);
    CHECK(equal_movie(&movie, &reparsed));

    nes_fm2_invalidate_project_timeline(&movie);
    CHECK(nes_fm2_measure(&movie, NES_FM3_PROJECT, &measured, &diagnostic) == NES_FM2_UNSUPPORTED);
    CHECK(nes_fm2_measure(&movie, NES_FM2_BINARY, &measured, &diagnostic) == NES_FM2_OK);

cleanup:
    free(encoded);
    nes_fm2_movie_free(&reparsed);
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool test_transactional_failures(void) {
    static const char truncated[] = "version 3\nport0 1\nport1 1\nport2 0\n|0|........|";
    NesFm2Movie destination;
    NesFm2Movie invalid_source;
    NesFm2Diagnostic diagnostic;
    char *sentinel;
    bool ok = true;

    nes_fm2_movie_init(&destination);
    nes_fm2_movie_init(&invalid_source);
    destination.emu_version = UINT32_C(0x12345678);
    destination.rom_filename = dup_text("sentinel");
    CHECK(destination.rom_filename);
    sentinel = destination.rom_filename;

    CHECK(nes_fm2_parse((const uint8_t *)truncated, sizeof(truncated) - 1, NULL, &destination, &diagnostic) ==
          NES_FM2_TRUNCATED);
    CHECK(destination.emu_version == UINT32_C(0x12345678));
    CHECK(destination.rom_filename == sentinel && strcmp(sentinel, "sentinel") == 0);

    invalid_source.frame_count = 1;
    CHECK(nes_fm2_movie_clone(&invalid_source, &destination, &diagnostic) == NES_FM2_INVALID_ARGUMENT);
    CHECK(destination.emu_version == UINT32_C(0x12345678));
    CHECK(destination.rom_filename == sentinel && strcmp(sentinel, "sentinel") == 0);

cleanup:
    nes_fm2_movie_free(&invalid_source);
    nes_fm2_movie_free(&destination);
    return ok;
}

static bool expect_parse_result(const uint8_t *data, size_t size, const NesFm2Limits *limits, NesFm2Result expected) {
    NesFm2Movie movie;
    NesFm2Diagnostic diagnostic;
    NesFm2Result result;

    nes_fm2_movie_init(&movie);
    result = nes_fm2_parse(data, size, limits, &movie, &diagnostic);
    nes_fm2_movie_free(&movie);
    return result == expected;
}

static bool test_malformed_and_limits(void) {
    static const char two_frames[] = "version 3\nport0 1\nport1 1\nport2 0\n"
                                     "|0|........|........||\n"
                                     "|0|........|........||\n";
    static const char large_blob[] = "version 3\nport0 1\nport1 1\nport2 0\n"
                                     "savestate base64:AQIDBA==\n"
                                     "|0|........|........||\n";
    static const char unsupported_port[] = "version 3\nport0 1\nport1 1\nport2 1\n|0|........|........||\n";
    static const char bad_command[] = "version 3\nport0 1\nport1 1\nport2 0\n|256|........|........||\n";
    static const uint8_t truncated_binary[] = "version 3\nport0 1\nport1 1\nport2 0\nbinary 1\n|\x01\x02";
    NesFm2Limits limits;
    bool ok = true;

    limits = nes_fm2_default_limits();
    limits.max_frames = 1;
    CHECK(expect_parse_result((const uint8_t *)two_frames, sizeof(two_frames) - 1, &limits, NES_FM2_LIMIT));

    limits = nes_fm2_default_limits();
    limits.max_blob_bytes = 3;
    CHECK(expect_parse_result((const uint8_t *)large_blob, sizeof(large_blob) - 1, &limits, NES_FM2_LIMIT));

    CHECK(expect_parse_result((const uint8_t *)unsupported_port, sizeof(unsupported_port) - 1, NULL,
                              NES_FM2_UNSUPPORTED));
    CHECK(expect_parse_result((const uint8_t *)bad_command, sizeof(bad_command) - 1, NULL, NES_FM2_CORRUPT));
    CHECK(expect_parse_result(truncated_binary, sizeof(truncated_binary) - 1, NULL, NES_FM2_TRUNCATED));

cleanup:
    return ok;
}

static bool make_small_fm3(uint8_t **data, size_t *size, size_t *project_start) {
    static const size_t module_sizes[NES_FM3_PROJECT_MODULE_COUNT] = {1, 2, 3, 4, 5, 6};
    NesFm2Movie movie;
    bool ok = false;
    size_t module_total = 0;

    nes_fm2_movie_init(&movie);
    movie.ports[0] = NES_FM2_PORT_GAMEPAD;
    movie.ports[1] = NES_FM2_PORT_GAMEPAD;
    movie.frames = calloc(2, sizeof(*movie.frames));
    if (!movie.frames) {
        goto cleanup;
    }
    movie.frame_count = 2;
    movie.project_present = true;
    movie.project_timeline_valid = true;
    movie.project_version = 3;
    movie.project_saved_modules = 0x3f;
    for (size_t i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        if (!install_module(&movie, i, module_sizes[i], (uint8_t)(0x40 + i))) {
            goto cleanup;
        }
        module_total += module_sizes[i];
    }
    if (!export_movie(&movie, NES_FM3_PROJECT, data, size) || *size < 36 + module_total) {
        goto cleanup;
    }
    *project_start = *size - 36 - module_total;
    ok = true;

cleanup:
    nes_fm2_movie_free(&movie);
    return ok;
}

static bool test_fm3_malformed_and_limits(void) {
    NesFm2Limits limits;
    uint8_t *valid = NULL;
    uint8_t *mutated = NULL;
    size_t size = 0;
    size_t project_start = 0;
    size_t project_size;
    bool ok = true;

    CHECK(make_small_fm3(&valid, &size, &project_start));
    CHECK(project_start + 36 <= size);
    project_size = size - project_start;

    limits = nes_fm2_default_limits();
    limits.max_project_bytes = project_size - 1;
    CHECK(expect_parse_result(valid, size, &limits, NES_FM2_LIMIT));
    CHECK(expect_parse_result(valid, project_start + 35, NULL, NES_FM2_TRUNCATED));

    mutated = malloc(size);
    CHECK(mutated);
    memcpy(mutated, valid, size);
    mutated[project_start + 8] = 5;
    mutated[project_start + 9] = 0;
    mutated[project_start + 10] = 0;
    mutated[project_start + 11] = 0;
    CHECK(expect_parse_result(mutated, size, NULL, NES_FM2_UNSUPPORTED));

    memcpy(mutated, valid, size);
    {
        uint32_t first = read_u32le(mutated + project_start + 12) + 1;

        mutated[project_start + 12] = (uint8_t)first;
        mutated[project_start + 13] = (uint8_t)(first >> 8);
        mutated[project_start + 14] = (uint8_t)(first >> 16);
        mutated[project_start + 15] = (uint8_t)(first >> 24);
    }
    CHECK(expect_parse_result(mutated, size, NULL, NES_FM2_CORRUPT));

cleanup:
    free(mutated);
    free(valid);
    return ok;
}

static bool test_invalid_export(void) {
    NesFm2Movie movie;
    NesFm2Diagnostic diagnostic;
    size_t measured = 0;
    bool ok = true;

    nes_fm2_movie_init(&movie);
    movie.ports[2] = NES_FM2_PORT_GAMEPAD;
    CHECK(nes_fm2_measure(&movie, NES_FM2_TEXT, &measured, &diagnostic) == NES_FM2_UNSUPPORTED);
    movie.ports[2] = NES_FM2_PORT_NONE;
    movie.project_present = true;
    movie.project_timeline_valid = false;
    movie.project_version = 3;
    CHECK(nes_fm2_measure(&movie, NES_FM3_PROJECT, &measured, &diagnostic) == NES_FM2_UNSUPPORTED);

cleanup:
    nes_fm2_movie_free(&movie);
    return ok;
}

int test_fm2_accuracy(void) {
    int failures = 0;

    failures += !test_text_round_trip();
    failures += !test_binary_round_trip();
    failures += !test_fourscore_round_trip();
    failures += !test_fm3_round_trip_and_offsets();
    failures += !test_transactional_failures();
    failures += !test_malformed_and_limits();
    failures += !test_fm3_malformed_and_limits();
    failures += !test_invalid_export();
    if (failures == 0) {
        printf("FM2/FM3 codec accuracy: PASS\n");
    }
    return failures;
}

#ifdef CUPID_FM2_ACCURACY_STANDALONE
int main(void) {
    return test_fm2_accuracy();
}
#endif
