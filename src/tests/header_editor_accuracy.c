/*
 * header_editor_accuracy.c - Cartridge header editor
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "board_tests.h"
#include "../media/header_editor.h"
#include "../ui/header_editor_frontend.h"
#include "../ui/frontend_panels.h"
#include "../ui/frontend_execution.h"
#include "../util/file_io.h"

int run_header_editor_accuracy_tests(void) {
    const char *source = "build/header-editor-source.nes", *output = "build/header-editor-output.nes";
    const size_t length = 16 + 512 + 16384 + 8192 + 32;
    uint8_t *bytes = calloc(1, length);
    BOARD_CHECK(bytes);
    memcpy(bytes, "NES\032", 4);
    bytes[4] = 1;
    bytes[5] = 1;
    bytes[6] = 4;
    for (size_t i = 16; i < length; ++i) {
        bytes[i] = (uint8_t)(i * 17);
    }
    BOARD_CHECK(nes_file_write_atomic(source, bytes, length) == NES_FILE_OK);
    HeaderEditor *editor = header_editor_create();
    BOARD_CHECK(editor);
    char error[256];
    iNESHeader h;
    BOARD_CHECK(header_editor_open(editor, source, error, sizeof(error)));
    HeaderEditorMetadata m, original;
    BOARD_CHECK(header_editor_metadata(editor, &m));
    original = m;
    BOARD_CHECK(header_editor_validate(editor, &m, &h, error, sizeof(error)));
    m.value[HEADER_FORMAT] = 2;
    m.value[HEADER_BATTERY] = 1;
    m.value[HEADER_PRG_RAM] = 8192;
    m.value[HEADER_PRG_NVRAM] = 8192;
    m.value[HEADER_CHR_RAM] = 128;
    m.value[HEADER_CHR_NVRAM] = 256;
    m.value[HEADER_MAPPER] = 0;
    m.value[HEADER_SUBMAPPER] = 0;
    m.value[HEADER_MIRROR] = 1;
    m.value[HEADER_TIMING] = 1;
    m.value[HEADER_MISC_ROMS] = 1;
    m.value[HEADER_INPUT] = 1;
    BOARD_CHECK(header_editor_validate(editor, &m, &h, error, sizeof(error)));
    BOARD_CHECK(!header_editor_save_copy(editor, &m, source, NULL, error, sizeof(error)));
    FrontendExecutionRuntime execution = {0};
    execution.rom_path = output;
    BOARD_CHECK(!header_editor_save_copy(editor, &m, output, &execution, error, sizeof(error)));
    uint8_t *active = prg_rom;
    BOARD_CHECK(header_editor_save_copy(editor, &m, output, NULL, error, sizeof(error)));
    BOARD_CHECK(prg_rom == active);
    uint8_t *saved = NULL;
    size_t saved_size = 0;
    BOARD_CHECK(nes_file_read_all(output, length, &saved, &saved_size) == NES_FILE_OK);
    BOARD_CHECK(saved_size == length && !memcmp(saved + 16, bytes + 16, length - 16));
    HeaderEditor *reopened = header_editor_create();
    HeaderEditorMetadata roundtrip;
    BOARD_CHECK(header_editor_open(reopened, output, error, sizeof(error)));
    BOARD_CHECK(header_editor_metadata(reopened, &roundtrip));
    BOARD_CHECK(!memcmp(&roundtrip, &m, sizeof(m)));
    BOARD_CHECK(load_rom(output) == 0);
    BOARD_CHECK(prg_size == 16384 && !memcmp(prg_rom, bytes + 528, 16384));
    BOARD_CHECK(unload_rom());
    HeaderEditorMetadata bad = m;
    bad.value[HEADER_PRG] = UINT64_MAX;
    BOARD_CHECK(!header_editor_validate(editor, &bad, &h, error, sizeof(error)));
    bad = m;
    bad.value[HEADER_PRG] = 32768;
    BOARD_CHECK(!header_editor_validate(editor, &bad, &h, error, sizeof(error)));
    bad = m;
    bad.value[HEADER_TRAINER] = 0;
    BOARD_CHECK(!header_editor_validate(editor, &bad, &h, error, sizeof(error)));
    bad = m;
    bad.value[HEADER_PRG_RAM] = 129;
    BOARD_CHECK(!header_editor_validate(editor, &bad, &h, error, sizeof(error)));
    bad = m;
    bad.value[HEADER_BATTERY] = 0;
    BOARD_CHECK(!header_editor_validate(editor, &bad, &h, error, sizeof(error)));
    bad = m;
    bad.value[HEADER_FORMAT] = 1;
    BOARD_CHECK(!header_editor_validate(editor, &bad, &h, error, sizeof(error)));
    bad = m;
    bad.value[HEADER_CONSOLE] = 3;
    bad.value[HEADER_EXTENDED_CONSOLE] = 4;
    BOARD_CHECK(header_editor_encode(&bad, &h, error, sizeof(error)) && h.zero[2] == 4);
    bad = m;
    bad.value[HEADER_CONSOLE] = 1;
    bad.value[HEADER_VS_PPU] = 2;
    bad.value[HEADER_VS_HARDWARE] = 3;
    BOARD_CHECK(header_editor_encode(&bad, &h, error, sizeof(error)) && h.zero[2] == 0x32);
    bad = m;
    bad.value[HEADER_MAPPER] = 4095;
    bad.value[HEADER_SUBMAPPER] = 15;
    BOARD_CHECK(header_editor_encode(&bad, &h, error, sizeof(error)) && rom_mapper_number(&h) == 4095 &&
                h.prg_ram_size == 255);
    bad = m;
    bad.value[HEADER_PRG] = 24576;
    BOARD_CHECK(header_editor_encode(&bad, &h, error, sizeof(error)) && (h.flags9 & 15) == 15);
    /* Round-trip every metadata family through files and the public decoder. */
    for (unsigned variant = 0; variant < 5; ++variant) {
        HeaderEditorMetadata candidate = m;
        candidate.value[HEADER_TIMING] = 0;
        candidate.value[HEADER_INPUT] = 4;
        if (variant == 0) {
            candidate.value[HEADER_CONSOLE] = 1;
            candidate.value[HEADER_VS_PPU] = 2;
            candidate.value[HEADER_VS_HARDWARE] = 3;
        } else if (variant == 1) {
            candidate.value[HEADER_CONSOLE] = 3;
            candidate.value[HEADER_EXTENDED_CONSOLE] = 1;
            candidate.value[HEADER_VS_HARDWARE] = 3;
        } else if (variant == 2) {
            candidate.value[HEADER_MAPPER] = 4095;
            candidate.value[HEADER_SUBMAPPER] = 15;
            candidate.value[HEADER_INPUT] = 63;
            candidate.value[HEADER_TIMING] = 3;
        } else if (variant == 3) {
            candidate = original;
            candidate.value[HEADER_LEGACY_RAM] = 255;
            candidate.value[HEADER_MIRROR] = 2;
        } else {
            candidate.value[HEADER_CONSOLE] = 3;
            candidate.value[HEADER_EXTENDED_CONSOLE] = 4;
        }
        BOARD_CHECK(header_editor_save_copy(editor, &candidate, output, NULL, error, sizeof(error)));
        BOARD_CHECK(header_editor_open(reopened, output, error, sizeof(error)));
        BOARD_CHECK(header_editor_metadata(reopened, &roundtrip));
        BOARD_CHECK(!memcmp(&candidate, &roundtrip, sizeof(candidate)));
    }
    /* A rejected draft must leave an existing output byte-for-byte intact. */
    BOARD_CHECK(nes_file_write_atomic(output, bytes, length) == NES_FILE_OK);
    bad = m;
    bad.value[HEADER_CHR] = 16384;
    BOARD_CHECK(!header_editor_save_copy(editor, &bad, output, NULL, error, sizeof(error)));
    free(saved);
    saved = NULL;
    BOARD_CHECK(nes_file_read_all(output, length, &saved, &saved_size) == NES_FILE_OK);
    BOARD_CHECK(saved_size == length && !memcmp(saved, bytes, length));
    BOARD_CHECK(nes_file_write_atomic(output, bytes, 15) == NES_FILE_OK);
    BOARD_CHECK(!header_editor_open(editor, output, error, sizeof(error)));
    BOARD_CHECK(header_editor_metadata(editor, &roundtrip) && !memcmp(&roundtrip, &original, sizeof(original)));
    BOARD_CHECK(nes_file_write_atomic(output, bytes, 20) == NES_FILE_OK);
    BOARD_CHECK(header_editor_open(reopened, output, error, sizeof(error)));
    BOARD_CHECK(!header_editor_validate(reopened, &original, &h, error, sizeof(error)));
    frontend_panels_reset();
    frontend_panel_set_session_active(false);
    BOARD_CHECK(header_editor_frontend_register(NULL));
    FrontendPanelInfo info;
    BOARD_CHECK(frontend_panel_get(HEADER_EDITOR_PANEL, &info) && info.enabled && !info.flags);
    BOARD_CHECK(frontend_panel_action(HEADER_EDITOR_PANEL, HEADER_EDITOR_OPEN, source, 0, error, sizeof(error)));
    FrontendPanelControl controls[32];
    FrontendPanelModel model = {controls, 32, 0, NULL};
    BOARD_CHECK(frontend_panel_snapshot(HEADER_EDITOR_PANEL, &model, error, sizeof(error)) && model.count == 23);
    BOARD_CHECK(!frontend_panel_action(HEADER_EDITOR_PANEL, HEADER_EDITOR_FIELD_BASE + HEADER_PRG,
                                       "18446744073709551616", 0, error, sizeof(error)));
    header_editor_frontend_unregister();
    header_editor_destroy(editor);
    header_editor_destroy(reopened);
    free(bytes);
    free(saved);
    nes_file_remove(source);
    nes_file_remove(output);
    nes_file_remove("build/header-editor-output.sav");
    nes_file_remove("build/header-editor-output.chr.sav");
    return 0;
}
