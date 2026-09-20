/*
 * board_state_accuracy.c - Cartridge board save-state regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "board_tests.h"
#include "../rom/board.h"
#include "../rom/boards/board_state.h"
#include "../rom/unif.h"
#include "../state/state_io.h"

#include <math.h>

enum {
    STATE_TEST_PRG_SIZE = 0x200000,
    STATE_TEST_CHR_SIZE = 0x80000
};

static uint8_t state_prg[STATE_TEST_PRG_SIZE];
static uint8_t state_chr[STATE_TEST_CHR_SIZE];

static void state_fill_media(uint8_t salt) {
    for (size_t i = 0; i < sizeof(state_prg); ++i)
        state_prg[i] = (uint8_t)((i >> 12) + salt);
    for (size_t i = 0; i < sizeof(state_chr); ++i)
        state_chr[i] = (uint8_t)((i >> 10) + salt);
}

static iNESHeader state_header(unsigned mapper) {
    iNESHeader header = {0};
    memcpy(header.signature, "NES\x1A", 4);
    header.prg_rom_chunks = STATE_TEST_PRG_SIZE / 0x4000;
    header.chr_rom_chunks = STATE_TEST_CHR_SIZE / 0x2000;
    header.flags6 = (uint8_t)((mapper & 0x0F) << 4);
    header.flags7 = (uint8_t)((mapper & 0xF0) | 0x08);
    header.prg_ram_size = (uint8_t)((mapper >> 8) & 0x0F);
    return header;
}

static int state_capture(CartridgeBoard *board, uint8_t **data, size_t *size) {
    NesStateWriter writer;
    nes_state_writer_init(&writer, NES_STATE_MAX_SIZE);
    NesStateResult result = board_state_capture(board, &writer);
    if (result != NES_STATE_OK) {
        fprintf(stderr, "board state capture failed: %d\n", (int)result);
        nes_state_writer_destroy(&writer);
        return 1;
    }
    *data = nes_state_writer_release(&writer, size);
    nes_state_writer_destroy(&writer);
    if (!*data || !*size) {
        fprintf(stderr, "board state capture produced no data\n");
        return 1;
    }
    return 0;
}

static int state_apply(CartridgeBoard *board, const uint8_t *data, size_t size) {
    NesStateReader reader;
    nes_state_reader_init(&reader, data, size);
    BoardStateRestore *restore = NULL;
    NesStateResult result = board_state_validate(board, &reader, &restore);
    if (result != NES_STATE_OK || !restore || nes_state_reader_remaining(&reader) != 0) {
        fprintf(stderr, "board state validation failed: %d\n", (int)result);
        board_state_restore_free(restore);
        return 1;
    }
    board_state_apply(board, restore);
    board_state_restore_free(restore);
    return 0;
}

static void state_seed(CartridgeBoard *board) {
    static const uint16_t addresses[] = {
        0x4020, 0x4100, 0x4101, 0x4102, 0x4103, 0x4200, 0x4201, 0x4202,
        0x4800, 0x5000, 0x5001, 0x5100, 0x5800, 0x6000, 0x7EF0, 0x7EF1,
        0x8000, 0x8001, 0x9000, 0xA000, 0xA001, 0xB000, 0xC000, 0xC001,
        0xD000, 0xE000, 0xE001, 0xF000, 0xFFFF
    };
    board_reset(board, true);
    board_after_reset(board);
    board_set_mapper_input(board, 0, true);
    board_set_mapper_input(board, 1, false);
    board_set_mapper_input(board, 2, true);
    for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
        cpu_total_cycles += 3;
        board_cpu_write(board, addresses[i], (uint8_t)(0x31u + i * 17u));
        board_clock_cpu(board, (i & 1) != 0);
    }
    for (unsigned i = 0; i < 12; ++i) {
        ppu.scanline = (int)(i * 7);
        ppu.dot = (int)(i * 19);
        board_notify_ppu_address(board, 0x0000, i * 32);
        board_notify_ppu_address(board, 0x1000, i * 32 + 16);
        board_ppu_write(board, (uint16_t)(i * 0x101), (uint8_t)(0xA0 + i));
        cpu_total_cycles += 3;
        board_clock_cpu(board, false);
    }
    (void)board_cpu_read(board, 0x5000, 0xA5);
    (void)board_cpu_read(board, 0x6000, 0x5A);
    (void)board_ppu_read(board, 0x0000, 1);
    (void)board_ppu_read(board, 0x2000, 1);
}

static uint64_t state_hash_byte(uint64_t hash, uint8_t value) {
    return (hash ^ value) * 1099511628211ull;
}

static uint64_t state_future_trace(CartridgeBoard *board, uint64_t start_cycle) {
    static const uint16_t write_addresses[] = {
        0x4100, 0x4202, 0x5000, 0x6000, 0x7EF0, 0x8000, 0x8001,
        0xA000, 0xC000, 0xC001, 0xE000, 0xE001
    };
    static const uint16_t read_addresses[] = {0x5000, 0x6000, 0x8000, 0xC000, 0xFFFF};
    uint64_t hash = 1469598103934665603ull;
    cpu_total_cycles = start_cycle;
    for (unsigned step = 0; step < 24; ++step) {
        cpu_total_cycles += 3;
        uint16_t write_address = write_addresses[step % (sizeof(write_addresses) / sizeof(write_addresses[0]))];
        board_cpu_write(board, write_address, (uint8_t)(0x5D + step * 23));
        board_clock_cpu(board, (step & 1) != 0);
        ppu.scanline = (int)((step * 11) % 240);
        ppu.dot = (int)((step * 29) % 341);
        board_notify_ppu_address(board, step & 1 ? 0x1000 : 0x0000, step * 64);
        for (size_t i = 0; i < sizeof(read_addresses) / sizeof(read_addresses[0]); ++i)
            hash = state_hash_byte(hash, board_cpu_read(board, read_addresses[i], (uint8_t)(step + i)));
        hash = state_hash_byte(hash, board_ppu_read(board, (uint16_t)((step * 0x111) & 0x3FFF), 1));
        hash = state_hash_byte(hash, board_irq_pending(board) ? 1 : 0);
        hash = state_hash_byte(hash, (uint8_t)board_mirroring(board));
        float audio = board_audio(board);
        uint32_t audio_bits = 0;
        memcpy(&audio_bits, &audio, sizeof(audio_bits));
        for (unsigned byte = 0; byte < 4; ++byte)
            hash = state_hash_byte(hash, (uint8_t)(audio_bits >> (byte * 8)));
    }
    return hash;
}

static int state_round_trip_board(CartridgeBoard *board, unsigned mapper) {
    state_seed(board);
    uint64_t saved_cycle = cpu_total_cycles;
    uint8_t *saved = NULL;
    size_t saved_size = 0;
    if (state_capture(board, &saved, &saved_size) != 0) {
        fprintf(stderr, "mapper %u failed seeded board-state capture\n", mapper);
        board_destroy(board);
        return 1;
    }

    NesStateReader reader;
    nes_state_reader_init(&reader, saved, saved_size);
    BoardStateRestore *restore = NULL;
    BOARD_CHECK(board_state_validate(board, &reader, &restore) == NES_STATE_OK);
    BOARD_CHECK(restore != NULL && nes_state_reader_remaining(&reader) == 0);
    board_state_apply(board, restore);
    board_state_restore_free(restore);

    uint8_t *round_trip = NULL;
    size_t round_trip_size = 0;
    BOARD_CHECK(state_capture(board, &round_trip, &round_trip_size) == 0);
    if (saved_size != round_trip_size || memcmp(saved, round_trip, saved_size) != 0) {
        fprintf(stderr, "mapper %u changed during immediate board-state round trip\n", mapper);
        free(round_trip);
        free(saved);
        return 1;
    }
    free(round_trip);

    uint64_t expected = state_future_trace(board, saved_cycle);
    board_destroy(board);

    state_fill_media(0);
    iNESHeader header = state_header(mapper);
    board = board_create(&header, state_prg, sizeof(state_prg), state_chr, sizeof(state_chr));
    if (!board) {
        fprintf(stderr, "mapper %u could not be recreated for board-state test\n", mapper);
        free(saved);
        return 1;
    }
    if (state_apply(board, saved, saved_size) != 0) {
        fprintf(stderr,
                "mapper %u recreation restore failed (flags6=%02X flags7=%02X mapperByte=%02X)\n",
                mapper, header.flags6, header.flags7, header.prg_ram_size);
        board_destroy(board);
        free(saved);
        return 1;
    }
    uint64_t observed = state_future_trace(board, saved_cycle);
    if (expected != observed) {
        fprintf(stderr, "mapper %u diverged after board-state restore (%llx != %llx)\n",
                mapper, (unsigned long long)expected, (unsigned long long)observed);
        board_destroy(board);
        free(saved);
        return 1;
    }
    board_destroy(board);
    free(saved);
    return 0;
}

static int test_all_mapper_board_states(void) {
    cart_set_dip_switches(0xA5);
    for (unsigned mapper = 0; mapper <= 0xFFF; ++mapper) {
        if (!board_handles_mapper(mapper)) continue;
        state_fill_media(0);
        iNESHeader header = state_header(mapper);
        CartridgeBoard *board = board_create(&header, state_prg, sizeof(state_prg),
                                             state_chr, sizeof(state_chr));
        if (!board) {
            fprintf(stderr, "mapper %u failed board-state construction\n", mapper);
            return 1;
        }
        if (state_round_trip_board(board, mapper)) return 1;
    }
    return 0;
}

static int test_unif_board_states(void) {
    static const uint16_t ids[] = {
        UNIF_BOARD_MALEE, UNIF_BOARD_GS2013, UNIF_BOARD_GHOSTBUSTERS_63IN1,
        UNIF_BOARD_CC21, UNIF_BOARD_AC08, UNIF_BOARD_PUZZLE, UNIF_BOARD_255IN1,
        UNIF_BOARD_8237A, UNIF_BOARD_SSS_NROM_256
    };
    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        state_fill_media(0);
        iNESHeader header = state_header(0);
        RomDatabaseInfo database = {0};
        database.present = true;
        database.mapper = ids[i];
        database.prg_rom_size = sizeof(state_prg);
        database.chr_rom_size = sizeof(state_chr);
        database.bus_conflicts = -1;
        CartridgeBoard *board = board_create_with_metadata(
            &header, state_prg, sizeof(state_prg), state_chr, sizeof(state_chr), &database);
        BOARD_CHECK(board != NULL);
        state_seed(board);
        uint8_t *saved = NULL;
        size_t saved_size = 0;
        BOARD_CHECK(state_capture(board, &saved, &saved_size) == 0);
        BOARD_CHECK(state_apply(board, saved, saved_size) == 0);
        free(saved);
        board_destroy(board);
    }
    return 0;
}

static int test_fcns_board_state(void) {
    state_fill_media(0);
    iNESHeader header = state_header(0);
    header.flags7 = (uint8_t)((header.flags7 & 0xF0) | 0x0B);
    header.zero[2] = 0x0C;
    CartridgeBoard *board = board_create(&header, state_prg, sizeof(state_prg),
                                         state_chr, sizeof(state_chr));
    BOARD_CHECK(board != NULL);
    state_seed(board);
    uint8_t *saved = NULL;
    size_t saved_size = 0;
    BOARD_CHECK(state_capture(board, &saved, &saved_size) == 0);
    BOARD_CHECK(state_apply(board, saved, saved_size) == 0);
    free(saved);
    board_destroy(board);
    return 0;
}

static int test_fme7_restoration(void) {
    state_fill_media(0);
    iNESHeader header = state_header(69);
    header.flags10 = 9; /* 32 KiB work RAM across four 8 KiB banks. */
    CartridgeBoard *board = board_create(&header, state_prg, sizeof(state_prg),
                                         state_chr, sizeof(state_chr));
    BOARD_CHECK(board != NULL);

    board_cpu_write(board, 0x8000, 8);
    board_cpu_write(board, 0xA000, 0xC0);
    board_cpu_write(board, 0x6123, 0x5A);
    board_cpu_write(board, 0x8000, 9);
    board_cpu_write(board, 0xA000, 3);
    board_cpu_write(board, 0x8000, 0);
    board_cpu_write(board, 0xA000, 5);
    board_cpu_write(board, 0xC000, 0);
    board_cpu_write(board, 0xE000, 1);
    board_cpu_write(board, 0xC000, 1);
    board_cpu_write(board, 0xE000, 0);
    board_cpu_write(board, 0xC000, 7);
    board_cpu_write(board, 0xE000, 0x3E);
    board_cpu_write(board, 0xC000, 8);
    board_cpu_write(board, 0xE000, 0x0F);
    board_clock_cpu(board, false);
    board_clock_cpu(board, false);

    board_cpu_write(board, 0x8000, 14);
    board_cpu_write(board, 0xA000, 2);
    board_cpu_write(board, 0x8000, 15);
    board_cpu_write(board, 0xA000, 0);
    board_cpu_write(board, 0x8000, 13);
    board_cpu_write(board, 0xA000, 0x81);

    uint8_t expected_prg = board_cpu_read(board, 0x8000, 0);
    uint8_t expected_chr = board_ppu_read(board, 0x0000, 0);
    uint8_t expected_ram = board_cpu_read(board, 0x6123, 0);
    float expected_audio = board_audio(board);
    BOARD_CHECK(expected_prg == 6);
    BOARD_CHECK(expected_chr == 5);
    BOARD_CHECK(expected_ram == 0x5A);
    BOARD_CHECK(!board_irq_pending(board));

    uint8_t *saved = NULL;
    size_t saved_size = 0;
    BOARD_CHECK(state_capture(board, &saved, &saved_size) == 0);

    board_cpu_write(board, 0x8000, 9);
    board_cpu_write(board, 0xA000, 7);
    board_cpu_write(board, 0x8000, 0);
    board_cpu_write(board, 0xA000, 1);
    board_cpu_write(board, 0x8000, 8);
    board_cpu_write(board, 0xA000, 0xC0);
    board_cpu_write(board, 0x6123, 0xA5);
    board_cpu_write(board, 0xC000, 8);
    board_cpu_write(board, 0xE000, 0);
    for (unsigned i = 0; i < 8; ++i) board_clock_cpu(board, false);

    BOARD_CHECK(state_apply(board, saved, saved_size) == 0);
    BOARD_CHECK(board_cpu_read(board, 0x8000, 0) == expected_prg);
    BOARD_CHECK(board_ppu_read(board, 0x0000, 0) == expected_chr);
    BOARD_CHECK(board_cpu_read(board, 0x6123, 0) == expected_ram);
    BOARD_CHECK(memcmp(&expected_audio, &(float){board_audio(board)}, sizeof(float)) == 0);
    BOARD_CHECK(!board_irq_pending(board));
    board_clock_cpu(board, false);
    board_clock_cpu(board, false);
    BOARD_CHECK(!board_irq_pending(board));
    board_clock_cpu(board, false);
    BOARD_CHECK(board_irq_pending(board));

    free(saved);
    board_destroy(board);
    return 0;
}

static int test_board_state_rejection_is_atomic(void) {
    state_fill_media(0);
    iNESHeader header = state_header(208);
    CartridgeBoard *board = board_create(&header, state_prg, sizeof(state_prg),
                                         state_chr, sizeof(state_chr));
    BOARD_CHECK(board != NULL);
    state_seed(board);

    uint8_t *saved = NULL;
    size_t saved_size = 0;
    BOARD_CHECK(state_capture(board, &saved, &saved_size) == 0);
    uint8_t *before = NULL;
    size_t before_size = 0;
    BOARD_CHECK(state_capture(board, &before, &before_size) == 0);

    NesStateReader reader;
    BoardStateRestore *restore = NULL;
    nes_state_reader_init(&reader, saved, saved_size - 1);
    BOARD_CHECK(board_state_validate(board, &reader, &restore) == NES_STATE_ERROR_CORRUPT);
    BOARD_CHECK(restore == NULL);

    uint8_t *with_tail = (uint8_t *)malloc(saved_size + 1);
    BOARD_CHECK(with_tail != NULL);
    memcpy(with_tail, saved, saved_size);
    with_tail[saved_size] = 0xA5;
    nes_state_reader_init(&reader, with_tail, saved_size + 1);
    BOARD_CHECK(board_state_validate(board, &reader, &restore) == NES_STATE_ERROR_CORRUPT);
    BOARD_CHECK(restore == NULL);
    free(with_tail);

    uint8_t *after = NULL;
    size_t after_size = 0;
    BOARD_CHECK(state_capture(board, &after, &after_size) == 0);
    BOARD_CHECK(before_size == after_size && memcmp(before, after, before_size) == 0);
    free(before);
    free(after);

    state_fill_media(1);
    CartridgeBoard *wrong_rom = board_create(&header, state_prg, sizeof(state_prg),
                                             state_chr, sizeof(state_chr));
    BOARD_CHECK(wrong_rom != NULL);
    nes_state_reader_init(&reader, saved, saved_size);
    BOARD_CHECK(board_state_validate(wrong_rom, &reader, &restore) == NES_STATE_ERROR_INCOMPATIBLE);
    BOARD_CHECK(restore == NULL);
    board_destroy(wrong_rom);

    state_fill_media(0);
    iNESHeader other_header = state_header(217);
    CartridgeBoard *wrong_mapper = board_create(&other_header, state_prg, sizeof(state_prg),
                                                state_chr, sizeof(state_chr));
    BOARD_CHECK(wrong_mapper != NULL);
    nes_state_reader_init(&reader, saved, saved_size);
    BOARD_CHECK(board_state_validate(wrong_mapper, &reader, &restore) == NES_STATE_ERROR_INCOMPATIBLE);
    BOARD_CHECK(restore == NULL);
    board_destroy(wrong_mapper);

    NesStateWriter tiny;
    nes_state_writer_init(&tiny, 16);
    BOARD_CHECK(board_state_capture(board, &tiny) == NES_STATE_ERROR_OUT_OF_MEMORY);
    nes_state_writer_destroy(&tiny);

    free(saved);
    board_destroy(board);
    return 0;
}

int test_board_state_accuracy(void) {
    int failures = 0;
    failures += test_all_mapper_board_states();
    failures += test_unif_board_states();
    failures += test_fcns_board_state();
    failures += test_fme7_restoration();
    failures += test_board_state_rejection_is_atomic();
    return failures;
}
