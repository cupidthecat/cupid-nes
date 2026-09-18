/*
 * studybox_accuracy.c - StudyBox media, tape, IRQ and audio regressions
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../system/hardware.h"
#include <math.h>
#include <time.h>

typedef struct {
    uint8_t *data;
    size_t size;
} StudyBoxBlob;

static unsigned studybox_checks;

#define STUDY_CHECK(condition) do { \
    ++studybox_checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static void put16(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static StudyBoxBlob make_wav(void) {
    enum { sample_count = 64, header_size = 44, data_size = sample_count * 2 };
    StudyBoxBlob blob = {0};
    blob.size = header_size + data_size;
    blob.data = (uint8_t *)calloc(1, blob.size);
    if (!blob.data) return blob;
    memcpy(blob.data, "RIFF", 4);
    put32(blob.data + 4, (uint32_t)blob.size - 8);
    memcpy(blob.data + 8, "WAVEfmt ", 8);
    put32(blob.data + 16, 16);
    put16(blob.data + 20, 1);
    put16(blob.data + 22, 1);
    put32(blob.data + 24, 1789773);
    put32(blob.data + 28, 1789773u * 2u);
    put16(blob.data + 32, 2);
    put16(blob.data + 34, 16);
    memcpy(blob.data + 36, "data", 4);
    put32(blob.data + 40, data_size);
    int16_t samples[sample_count] = {0};
    samples[4] = 16384;
    samples[5] = 8192;
    samples[12] = -16384;
    samples[13] = -8192;
    for (unsigned i = 0; i < sample_count; ++i)
        put16(blob.data + header_size + i * 2, (uint16_t)samples[i]);
    return blob;
}

static StudyBoxBlob make_stbx(void) {
    StudyBoxBlob wav = make_wav();
    StudyBoxBlob blob = {0};
    if (!wav.data) return blob;
    blob.size = 72 + wav.size;
    blob.data = (uint8_t *)calloc(1, blob.size);
    if (!blob.data) { free(wav.data); return blob; }
    memcpy(blob.data, "STBX", 4);
    put32(blob.data + 4, 4);
    put32(blob.data + 8, 0x100);

    memcpy(blob.data + 12, "PAGE", 4);
    put32(blob.data + 16, 16);
    put32(blob.data + 20, 0);
    put32(blob.data + 24, 4);
    const uint8_t page0[8] = {0xC5, 0x10, 0x11, 0x12, 0x13, 0, 0x15, 0x16};
    memcpy(blob.data + 28, page0, sizeof(page0));

    memcpy(blob.data + 36, "PAGE", 4);
    put32(blob.data + 40, 16);
    put32(blob.data + 44, 8);
    put32(blob.data + 48, 12);
    const uint8_t page1[8] = {0xC5, 0x20, 0x21, 0x22, 0x23, 1, 0x25, 0x26};
    memcpy(blob.data + 52, page1, sizeof(page1));

    memcpy(blob.data + 60, "AUDI", 4);
    put32(blob.data + 64, (uint32_t)wav.size + 4);
    put32(blob.data + 68, 0);
    memcpy(blob.data + 72, wav.data, wav.size);
    free(wav.data);
    return blob;
}

static bool write_blob(const char *path, const uint8_t *data, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(data, 1, size, file) == size;
    return fclose(file) == 0 && ok;
}

static bool send_command(uint8_t command, uint8_t control) {
    write_mem(0x4202, (uint8_t)(control | 0x20));
    write_mem(0x4202, control);
    if (!(cart_cpu_read(0x4202) & 0x40)) return false;
    for (unsigned bit = 0; bit < 8; ++bit) {
        uint8_t data = (command & (0x80u >> bit)) ? 0x80 : 0;
        write_mem(0x4202, (uint8_t)(control | data | 0x10));
        if (!cart || !cart->clock) return false;
        cart->clock(100);
        if (!(cart_cpu_read(0x4202) & 0x40)) return false;
        write_mem(0x4202, (uint8_t)(control | data));
    }
    return true;
}

static int test_studybox_device(void) {
    uint8_t *bios = (uint8_t *)malloc(0x40000);
    STUDY_CHECK(bios != NULL);
    for (size_t i = 0; i < 0x40000; ++i) bios[i] = (uint8_t)(i / 0x4000);
    StudyBoxBlob media = make_stbx();
    STUDY_CHECK(media.data != NULL);

    char media_path[128], bios_path[128];
    unsigned long stamp = (unsigned long)time(NULL);
    snprintf(media_path, sizeof(media_path), "build/studybox-%lu-%lu.stbx",
             stamp, (unsigned long)clock());
    snprintf(bios_path, sizeof(bios_path), "build/studybox-%lu-%lu.bin",
             stamp, (unsigned long)clock());
    STUDY_CHECK(write_blob(media_path, media.data, media.size));
    STUDY_CHECK(write_blob(bios_path, bios, 0x40000));
    STUDY_CHECK(load_studybox(media_path, bios_path) == 0);
    STUDY_CHECK(rom_is_studybox() && !rom_is_fds());
    ppu_power_on(&ppu);
    STUDY_CHECK(cpu_power_on(&cpu));
    STUDY_CHECK(cart_get_mirroring() == MIRROR_FOUR);

    STUDY_CHECK(cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    STUDY_CHECK(read_mem(0xC000) == 0);
    write_mem(0x4201, 2);
    STUDY_CHECK(read_mem(0x8000) == 2 && read_mem(0xC000) == 0);

    write_mem(0x4400, 0x88);
    write_mem(0x4200, 0x00);
    write_mem(0x5000, 0x55);
    write_mem(0x6000, 0x66);
    write_mem(0x7000, 0x77);
    write_mem(0x4200, 0xC7);
    write_mem(0x5000, 0xA5);
    write_mem(0x6000, 0xB6);
    write_mem(0x7000, 0xC7);
    write_mem(0x4200, 0x00);
    STUDY_CHECK(read_mem(0x4400) == 0x88 && read_mem(0x5000) == 0x55);
    STUDY_CHECK(read_mem(0x6000) == 0x66 && read_mem(0x7000) == 0x77);
    write_mem(0x4200, 0xC7);
    STUDY_CHECK(read_mem(0x5000) == 0xA5 && read_mem(0x6000) == 0xB6 && read_mem(0x7000) == 0xC7);
    STUDY_CHECK(cart_cpu_read_bus(0x4000, 0xD4) == 0xD4);

    ppu_write(0x2000, 0x10);
    ppu_write(0x2400, 0x20);
    ppu_write(0x2800, 0x30);
    ppu_write(0x2C00, 0x40);
    STUDY_CHECK(ppu_read(0x2000) == 0x10 && ppu_read(0x2400) == 0x20);
    STUDY_CHECK(ppu_read(0x2800) == 0x30 && ppu_read(0x2C00) == 0x40);

    STUDY_CHECK(send_command(0x01, 0x03));
    cart->clock(2999999);
    STUDY_CHECK((cart_cpu_read(0x4201) & 0x40) == 0 && !cart_irq_pending());
    cart->clock(1);
    STUDY_CHECK((cart_cpu_read(0x4201) & 0xC0) == 0xC0);
    cart->clock(3);
    STUDY_CHECK(!cart_irq_pending() && (cart_cpu_read(0x4201) & 0xA0) == 0xA0);
    cart->clock(1);
    STUDY_CHECK(cart_irq_pending());
    float sample = cart_expansion_audio();
    STUDY_CHECK(sample > 0.49f && sample < 0.51f);
    STUDY_CHECK(cart_cpu_read(0x4200) == 0xAA && !cart_irq_pending());
    cart->clock(7820);
    STUDY_CHECK(cart_irq_pending() && cart_cpu_read(0x4200) == 0xC5);
    STUDY_CHECK(!cart_irq_pending());
    cart->clock(3355);
    STUDY_CHECK(cart_irq_pending() && cart_cpu_read(0x4200) == 0x10);

    STUDY_CHECK(send_command(0x86, 0x03));
    STUDY_CHECK((cart_cpu_read(0x4201) & 0xC0) == 0xC0);
    cart->clock(4);
    STUDY_CHECK(cart_irq_pending());
    sample = cart_expansion_audio();
    STUDY_CHECK(sample < -0.49f && sample > -0.51f);
    (void)cart_cpu_read(0x4200);

    cpu_soft_reset(&cpu);
    STUDY_CHECK(read_mem(0x8000) == 2);
    STUDY_CHECK(cart_cpu_read(0x4201) == 0 && cart_cpu_read(0x4202) == 0);
    STUDY_CHECK(cart_expansion_audio() == 0.0f && !cart_irq_pending());

    uint8_t *bad = (uint8_t *)malloc(media.size);
    STUDY_CHECK(bad != NULL);
    memcpy(bad, media.data, media.size);
    put32(bad + 48, 2);
    STUDY_CHECK(load_studybox_memory(bad, media.size, bios, 0x40000) == -1);
    STUDY_CHECK(rom_is_studybox() && read_mem(0x8000) == 2);
    STUDY_CHECK(load_studybox_memory(media.data, media.size, bios, 0x3FFFF) == -1);
    STUDY_CHECK(rom_is_studybox() && read_mem(0x8000) == 2);
    memcpy(bad, media.data, media.size);
    put32(bad + 68, 1);
    STUDY_CHECK(load_studybox_memory(bad, media.size, bios, 0x40000) == -1);
    STUDY_CHECK(load_studybox_memory(media.data, media.size - 1, bios, 0x40000) == -1);
    STUDY_CHECK(rom_is_studybox() && read_mem(0x8000) == 2);

    memcpy(bad, media.data, media.size);
    bad[72] = 'X';
    STUDY_CHECK(load_studybox_memory(bad, media.size, bios, 0x40000) == 0);
    ppu_power_on(&ppu);
    STUDY_CHECK(cpu_power_on(&cpu));
    STUDY_CHECK(send_command(0x86, 0x03));
    cart->clock(4);
    STUDY_CHECK(cart_expansion_audio() == 0.0f);

    BoardImage ordinary;
    STUDY_CHECK(board_image_create(&ordinary, 0, 0x8000, 0x2000, true));
    STUDY_CHECK(board_image_load(&ordinary) == 0);
    STUDY_CHECK(!rom_is_studybox() && read_mem(0x8000) == 0);
    board_image_free(&ordinary);

    free(bad);
    free(media.data);
    free(bios);
    STUDY_CHECK(remove(media_path) == 0 && remove(bios_path) == 0);
    return 0;
}

int test_studybox_accuracy(void) {
    studybox_checks = 0;
    int failures = test_studybox_device();
    unload_rom();
    printf("StudyBox: %u checks, %d failures\n", studybox_checks, failures);
    return failures;
}
