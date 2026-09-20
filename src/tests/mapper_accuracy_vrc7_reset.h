/*
 * mapper_accuracy_vrc7_reset.h - VRC7 console reset regression tests
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 */

static int test_vrc7_console_reset(void) {
    for (unsigned submapper = 0; submapper <= 2; ++submapper) {
        iNESHeader h = vrc7_header((uint8_t)submapper, false, true);
        CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
        fixture_prg[0x7FFFC] = 0;
        fixture_prg[0x7FFFD] = 0xE0;
        nes_set_region(NES_REGION_NTSC);
        cpu_use_default_startup_alignment();
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        cart->reset();

        cart_cpu_write(0x8000, 3);
        cart_cpu_write(0xA000, 5);
        cart_cpu_write(0xE000, 0x83);
        cart_cpu_write(0x6123, 0xA7);
        vrc7_program_custom_patch();
        vrc7_key_channel0(0, true);
        cart->clock(36 * 8);
        float held_sample = cart_expansion_audio();
        CHECK(held_sample != 0.0f);

        uint16_t secondary = submapper == 2 ? 0x10u : 0x08u;
        cart_cpu_write((uint16_t)(0xE000u + secondary), 0xF6);
        cart_cpu_write(0xF000, 0x07);
        uint64_t reset_start = cpu_total_cycles;
        cpu_soft_reset(&cpu);
        CHECK(cpu_total_cycles - reset_start == 7);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_ppu_read(0) == 5);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
        CHECK(cart_cpu_read(0x6123) == 0xA7);
        CHECK(!cart_irq_pending() && cart_expansion_audio() == held_sample);
        cart->clock(2);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());

        /* The chip reset takes effect at the existing sample boundary. */
        cart->clock(25);
        CHECK(cart_expansion_audio() == held_sample);
        cart->clock(1);
        CHECK(cart_expansion_audio() == 0.0f);
        cart->clock(36 * 32);
        CHECK(cart_expansion_audio() == 0.0f);

        /* Reset preserves mute, including its register-write gate. */
        cart_cpu_write(0xE000, 0xC3);
        cpu_soft_reset(&cpu);
        vrc7_key_channel0(1, true);
        cart_cpu_write(0xE000, 0x83);
        cart->clock(36 * 256);
        CHECK(cart_expansion_audio() == 0.0f);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
        CHECK(cart_cpu_read(0x6123) == 0xA7);
    }
    return 0;
}

static int test_vrc7_reset_address_latch(void) {
    iNESHeader h = vrc7_header(2, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    nes_set_region(NES_REGION_NTSC);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    float explicit_address[128];
    bool audible = false;

    for (unsigned pass = 0; pass < 2; ++pass) {
        cart->reset();
        cart_cpu_write(0x9010, 0x10);
        cpu_soft_reset(&cpu);
        if (pass == 0) cart_cpu_write(0x9010, 0x10);
        cart_cpu_write(0x9030, 0x80);
        vrc7_audio_write(0x30, 0x10);
        vrc7_audio_write(0x20, 0x18);
        for (unsigned sample = 0; sample < 128; ++sample) {
            cart->clock(36);
            float output = cart_expansion_audio();
            if (pass == 0) {
                explicit_address[sample] = output;
                if (output != 0.0f) audible = true;
            } else {
                CHECK(output == explicit_address[sample]);
            }
        }
    }
    CHECK(audible);
    return 0;
}
