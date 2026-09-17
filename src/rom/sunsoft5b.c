/*
 * sunsoft5b.c - Sunsoft 5B expansion audio
 *
 * Author: @frankischilling
 *
 * The Sunsoft 5B contains a YM2149F-compatible three-channel PSG. The chip is
 * clocked from the CPU clock and internally divides that clock by sixteen for
 * its tone, noise, and envelope counters.
 *
 * This file is part of Cupid NES Emulator.
 */

#include <string.h>
#include "sunsoft5b.h"

/* YM2149F envelope levels are 1.5 dB apart. Manual 4-bit volume values use
 * odd entries from this 5-bit curve; envelope levels use the whole table. */
static const float level_table[32] = {
    0.0f, 0.0f, 0.00668344f, 0.00794328f, 0.00944061f, 0.01122018f,
    0.01333521f, 0.01584893f, 0.01883649f, 0.02238721f, 0.02660725f,
    0.03162278f, 0.03758374f, 0.04466836f, 0.05308844f, 0.06309573f,
    0.07498942f, 0.08912509f, 0.10592537f, 0.12589254f, 0.14962357f,
    0.17782794f, 0.21134890f, 0.25118864f, 0.29853826f, 0.35481339f,
    0.42169650f, 0.50118723f, 0.59566214f, 0.70794578f, 0.84139514f, 1.0f
};

static uint16_t tone_period(const Sunsoft5B *audio, unsigned channel) {
    uint16_t period = (uint16_t)(audio->registers[channel * 2]
        | ((uint16_t)(audio->registers[channel * 2 + 1] & 0x0Fu) << 8));
    return period ? period : 1;
}

static uint8_t noise_period(const Sunsoft5B *audio) {
    uint8_t period = audio->registers[6] & 0x1Fu;
    return period ? period : 1;
}

static uint16_t envelope_period(const Sunsoft5B *audio) {
    uint16_t period = (uint16_t)(audio->registers[0x0B]
        | ((uint16_t)audio->registers[0x0C] << 8));
    return period ? period : 1;
}

static void envelope_restart(Sunsoft5B *audio) {
    uint8_t shape = audio->registers[0x0D] & 0x0Fu;
    audio->envelope_attack = (shape & 0x04u) != 0;
    audio->envelope_level = audio->envelope_attack ? 0 : 31;
    audio->envelope_counter = 0;
    audio->envelope_holding = false;
}

static void envelope_step(Sunsoft5B *audio) {
    if (audio->envelope_holding) return;

    if (audio->envelope_attack) {
        if (audio->envelope_level < 31) {
            audio->envelope_level++;
            return;
        }
    } else if (audio->envelope_level > 0) {
        audio->envelope_level--;
        return;
    }

    uint8_t shape = audio->registers[0x0D] & 0x0Fu;
    if (!(shape & 0x08u)) {
        audio->envelope_level = 0;
        audio->envelope_holding = true;
        return;
    }
    if (shape & 0x01u) {
        if (shape & 0x02u) audio->envelope_level = (uint8_t)(31u - audio->envelope_level);
        audio->envelope_holding = true;
        return;
    }
    if (shape & 0x02u) {
        audio->envelope_attack = !audio->envelope_attack;
    } else {
        audio->envelope_level = audio->envelope_attack ? 0 : 31;
    }
}

static void psg_tick(Sunsoft5B *audio) {
    for (unsigned channel = 0; channel < 3; ++channel) {
        uint16_t period = tone_period(audio, channel);
        if (++audio->tone_counter[channel] >= period) {
            audio->tone_counter[channel] = 0;
            audio->tone_output[channel] = !audio->tone_output[channel];
        }
    }

    if (++audio->noise_counter >= noise_period(audio)) {
        audio->noise_counter = 0;
        audio->noise_phase = !audio->noise_phase;
        if (audio->noise_phase) {
            uint32_t feedback = ((audio->noise_lfsr >> 16) ^ (audio->noise_lfsr >> 13)) & 1u;
            audio->noise_lfsr = ((audio->noise_lfsr << 1) | feedback) & 0x1FFFFu;
            if (!audio->noise_lfsr) audio->noise_lfsr = 1;
        }
    }

    if (++audio->envelope_counter >= envelope_period(audio)) {
        audio->envelope_counter = 0;
        envelope_step(audio);
    }
}

void sunsoft5b_reset(Sunsoft5B *audio) {
    if (!audio) return;
    memset(audio, 0, sizeof(*audio));
    audio->noise_lfsr = 0x1FFFFu;
    audio->noise_phase = true;
    envelope_restart(audio);
}

void sunsoft5b_write(Sunsoft5B *audio, uint16_t address, uint8_t value) {
    if (!audio) return;
    switch (address & 0xE000u) {
        case 0xC000:
            audio->selected_register = value;
            break;
        case 0xE000:
            if (audio->selected_register < 16) {
                uint8_t reg = audio->selected_register;
                if (reg == 1 || reg == 3 || reg == 5) value &= 0x0F;
                else if (reg == 6) value &= 0x1F;
                else if (reg >= 8 && reg <= 10) value &= 0x1F;
                else if (reg == 0x0D) value &= 0x0F;
                else if (reg >= 0x0E) value = 0;
                audio->registers[reg] = value;
                if (reg == 0x0D) envelope_restart(audio);
            }
            break;
    }
}

void sunsoft5b_clock(Sunsoft5B *audio, int cpu_cycles) {
    if (!audio || cpu_cycles <= 0) return;
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        audio->master_divider++;
        if (audio->master_divider == 16) {
            audio->master_divider = 0;
            psg_tick(audio);
        }
    }
}

float sunsoft5b_output(const Sunsoft5B *audio) {
    if (!audio) return 0.0f;
    float output = 0.0f;
    uint8_t mixer = audio->registers[7];
    bool noise_output = (audio->noise_lfsr & 1u) != 0;

    for (unsigned channel = 0; channel < 3; ++channel) {
        bool tone_gate = (mixer & (1u << channel)) || audio->tone_output[channel];
        bool noise_gate = (mixer & (1u << (channel + 3))) || noise_output;
        if (!tone_gate || !noise_gate) continue;

        uint8_t volume = audio->registers[8 + channel];
        uint8_t level = (volume & 0x10u)
            ? audio->envelope_level
            : (uint8_t)((volume & 0x0Fu) ? ((volume & 0x0Fu) * 2u + 1u) : 0u);
        output += level_table[level];
    }

    /* Match the cartridge-audio polarity used by the existing mixer. One full
     * 5B channel is close to one maximum MMC5 pulse in the mixed signal. */
    return -output * 0.126f;
}
