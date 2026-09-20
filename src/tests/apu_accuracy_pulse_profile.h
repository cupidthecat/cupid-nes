/*
 * apu_accuracy_pulse_profile.h - Pulse hardware profile regression tests
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

static uint8_t pulse_profile_dac(unsigned channel) {
    uint8_t value = read_mem(0x4018);
    return channel ? (uint8_t)(value >> 4) : (uint8_t)(value & 0x0F);
}

static uint8_t capture_pulse_profile_sequence(unsigned channel, unsigned duty, bool envelope, uint8_t output[8]) {
    reset_audio();
    cpu_set_test_mode(true);

    uint16_t control = (uint16_t)(0x4000u + channel * 4u);
    Pulse *pulse = channel ? &apu.pulse2 : &apu.pulse1;
    uint8_t value = (uint8_t)((duty << 6) | (envelope ? 0x03u : 0x1Fu));

    write_mem(0x4015, (uint8_t)(1u << channel));
    write_mem(control, value);
    write_mem((uint16_t)(control + 2), 8);
    write_mem((uint16_t)(control + 3), 0);

    pulse->timer = 1;
    apu_step(&apu, 1);
    if (envelope) {
        apu.cycle_in_seq = 7456;
        apu.frame_clock_block = 0;
        pulse->timer = 100;
        apu_step(&apu, 1);
    }

    write_mem(control, value);
    output[0] = pulse_profile_dac(channel);
    for (unsigned step = 1; step < 8; ++step) {
        pulse->timer = 0;
        apu_step(&apu, 2);
        output[step] = pulse_profile_dac(channel);
    }
    return pulse->duty;
}

static bool pulse_profile_sequence_matches(const uint8_t actual[8], const uint8_t expected[8]) {
    for (unsigned step = 0; step < 8; ++step) {
        if (actual[step] != (uint8_t)(expected[step] * 15u)) {
            return false;
        }
    }
    return true;
}

static void test_apu_pulse_profile(void) {
    static const uint8_t duty_sequences[4][8] = {
        {0, 1, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 1, 0, 0, 0},
        {1, 0, 0, 1, 1, 1, 1, 1},
    };

    CHECK("pulse duty profile defaults to standard behavior", !apu_swap_duty_cycles_enabled());

    apu_set_swap_duty_cycles(true);
    CHECK("clone pulse duty profile can be selected", apu_swap_duty_cycles_enabled());
    apu_power_on(&apu);
    CHECK("pulse duty profile survives power-on", apu_swap_duty_cycles_enabled());
    apu_soft_reset(&apu);
    CHECK("pulse duty profile survives soft reset", apu_swap_duty_cycles_enabled());
    apu_reset(&apu);
    CHECK("pulse duty profile survives reset", apu_swap_duty_cycles_enabled());

    CHECK("later DMC revision remains independently selectable", apu_set_cpu_revision(APU_CPU_REVISION_LATE_2A03));
    CHECK("pulse duty profile does not replace DMC revision selection",
          apu_swap_duty_cycles_enabled() && apu_get_cpu_revision() == APU_CPU_REVISION_LATE_2A03);
    apu_set_swap_duty_cycles(false);
    CHECK("changing pulse duty profile leaves DMC revision unchanged",
          apu_get_cpu_revision() == APU_CPU_REVISION_LATE_2A03);
    CHECK("early DMC revision restored after profile independence test",
          apu_set_cpu_revision(APU_CPU_REVISION_EARLY_2A03));

    apu_set_swap_duty_cycles(true);
    reset_audio();
    write_mem(0x4015, 1);
    write_mem(0x4000, 0x5F);
    CHECK("clone profile maps duty one to duty two on register write", apu.pulse1.duty == 2);
    apu_set_swap_duty_cycles(false);
    CHECK("changing profile does not rewrite a running pulse duty", apu.pulse1.duty == 2);
    write_mem(0x4000, 0x5F);
    CHECK("standard profile applies on the next pulse register write", apu.pulse1.duty == 1);

    for (unsigned swapped = 0; swapped < 2; ++swapped) {
        apu_set_swap_duty_cycles(swapped != 0);
        for (unsigned channel = 0; channel < 2; ++channel) {
            for (unsigned duty = 0; duty < 4; ++duty) {
                unsigned mapped = swapped ? ((duty & 2u) >> 1) | ((duty & 1u) << 1) : duty;
                uint8_t constant_output[8];
                uint8_t stored = capture_pulse_profile_sequence(channel, duty, false, constant_output);
                CHECK("pulse duty profile selects every constant-volume waveform",
                      stored == mapped && pulse_profile_sequence_matches(constant_output, duty_sequences[mapped]));

                uint8_t envelope_output[8];
                stored = capture_pulse_profile_sequence(channel, duty, true, envelope_output);
                CHECK("pulse duty profile selects every envelope waveform",
                      stored == mapped && pulse_profile_sequence_matches(envelope_output, duty_sequences[mapped]));
            }
        }
    }

    apu_set_swap_duty_cycles(false);
    cpu_set_test_mode(false);
    reset_audio();
}
