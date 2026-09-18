/*
 * epsm.cpp - EPSM YMF288 bus, timing, and audio integration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator, licensed under the GNU General
 * Public License, version 3 or any later version. The chip engine retains
 * its own BSD 3-Clause license in src/third_party/ymfm.
 */
#include "epsm.h"
#include "../third_party/ymfm/ymfm_opn.h"
#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>

static std::array<uint8_t, EPSM_ADPCM_ROM_SIZE> configured_rom{};
static bool configured_rom_present;

struct EpsmDevice final : ymfm::ymfm_interface {
    std::array<uint8_t, EPSM_ADPCM_ROM_SIZE> adpcm_rom;
    bool rom_present;
    std::array<uint32_t, 2> timers{};
    uint32_t busy_clocks = 0;
    bool irq = false;
    ymfm::ymf288 chip;
    std::vector<uint8_t> power_on_state;

    uint64_t total_clocks = 0;
    uint64_t master_remainder = 0;
    uint32_t master_frequency = 0;
    unsigned sample_phase = 0;
    uint8_t previous_out = 0;
    uint8_t protocol_address = 0;
    uint8_t protocol_data = 0;
    std::array<std::array<double, 4>, 2> history{};

    EpsmDevice() : adpcm_rom(configured_rom), rom_present(configured_rom_present), chip(*this) {
        chip.set_fidelity(ymfm::OPN_FIDELITY_MED);
        chip.reset();
        ymfm::ymfm_saved_state initial(power_on_state, true);
        chip.save_restore(initial);
    }

    void ymfm_set_timer(uint32_t timer, int32_t duration) override {
        if (timer < timers.size()) timers[timer] = duration > 0 ? (uint32_t)duration : 0;
    }

    void ymfm_set_busy_end(uint32_t clocks) override { busy_clocks = clocks; }
    bool ymfm_is_busy() override { return busy_clocks != 0; }
    void ymfm_update_irq(bool asserted) override { irq = asserted; }

    uint8_t ymfm_external_read(ymfm::access_class type, uint32_t address) override {
        return type == ymfm::ACCESS_ADPCM_A && address < adpcm_rom.size()
             ? adpcm_rom[address] : 0;
    }

    void power_on() {
        timers.fill(0);
        busy_clocks = 0;
        irq = false;
        total_clocks = 0;
        master_remainder = 0;
        master_frequency = 0;
        sample_phase = 0;
        previous_out = protocol_address = protocol_data = 0;
        history = {};
        // A power cycle also resets the chip's address latch and resampler history.
        ymfm::ymfm_saved_state initial(power_on_state, false);
        chip.save_restore(initial);
    }

    void clock(unsigned master_clocks, uint32_t master_hz) {
        if (!master_hz) return;
        if (master_frequency != master_hz) {
            // Preserve chip time and discard only the old divider's fraction.
            master_frequency = master_hz;
            master_remainder = 0;
        }
        uint64_t elapsed = master_remainder + (uint64_t)master_clocks * EPSM_CLOCK_RATE;
        uint64_t clocks = elapsed / master_hz;
        master_remainder = elapsed % master_hz;
        while (clocks--) {
            ++total_clocks;
            if (busy_clocks) --busy_clocks;
            for (unsigned timer = 0; timer < timers.size(); ++timer)
                if (timers[timer] && --timers[timer] == 0)
                    m_engine->engine_timer_expired(timer);

            if (++sample_phase == 144) {
                sample_phase = 0;
                ymfm::ymf288::output_data output;
                chip.generate(&output, 1);
                for (unsigned side = 0; side < history.size(); ++side) {
                    int32_t mixed = output.data[side] + (output.data[2] >> 2);
                    uint16_t sample_bits = (uint16_t)mixed;
                    int32_t sample = sample_bits < 0x8000 ? sample_bits : (int32_t)sample_bits - 0x10000;
                    auto &values = history[side];
                    values[0] = values[1];
                    values[1] = values[2];
                    values[2] = values[3];
                    values[3] = sample / 32768.0;
                }
            }
        }
    }

    float sample(unsigned side) const {
        const auto &v = history[side];
        double t = sample_phase / 144.0;
        // Four-point cubic interpolation, with two source samples of history.
        double a = -0.5 * v[0] + 1.5 * v[1] - 1.5 * v[2] + 0.5 * v[3];
        double b = v[0] - 2.5 * v[1] + 2.0 * v[2] - 0.5 * v[3];
        double c = -0.5 * v[0] + 0.5 * v[2];
        return (float)(((a * t + b) * t + c) * t + v[1]);
    }
};

static std::unique_ptr<EpsmDevice> active_epsm;

bool epsm_set_adpcm_rom(const uint8_t *data, size_t size) {
    if (!data && !size) {
        configured_rom.fill(0);
        configured_rom_present = false;
        return true;
    }
    if (!data || size != configured_rom.size()) return false;
    std::memcpy(configured_rom.data(), data, size);
    configured_rom_present = true;
    return true;
}

bool epsm_load_adpcm_file(const char *path) {
    if (!path || !*path) return false;
    std::FILE *file = std::fopen(path, "rb");
    if (!file) return false;
    std::array<uint8_t, EPSM_ADPCM_ROM_SIZE> contents;
    bool ok = std::fread(contents.data(), 1, contents.size(), file) == contents.size();
    if (ok) ok = std::fgetc(file) == EOF && !std::ferror(file);
    if (std::fclose(file) != 0) ok = false;
    return ok && epsm_set_adpcm_rom(contents.data(), contents.size());
}

EpsmDevice *epsm_create(void) {
    try {
        return new EpsmDevice();
    } catch (const std::bad_alloc &) {
        return nullptr;
    }
}

void epsm_destroy(EpsmDevice *device) { delete device; }
void epsm_activate(EpsmDevice *device) { active_epsm.reset(device); }
bool epsm_enabled(void) { return active_epsm != nullptr; }
bool epsm_has_adpcm_rom(void) { return active_epsm && active_epsm->rom_present; }
void epsm_power_on(void) { if (active_epsm) active_epsm->power_on(); }
void epsm_clock_master(unsigned clocks, uint32_t master_hz) {
    if (active_epsm) active_epsm->clock(clocks, master_hz);
}
uint64_t epsm_clock_count(void) { return active_epsm ? active_epsm->total_clocks : 0; }
bool epsm_irq_pending(void) { return active_epsm && active_epsm->irq; }

void epsm_write_4016(uint8_t data_bus, uint8_t out_pins) {
    if (!active_epsm) return;
    EpsmDevice &device = *active_epsm;
    bool high = (out_pins & 2u) != 0;
    bool previous_high = (device.previous_out & 2u) != 0;
    if (high && !previous_high) {
        device.protocol_data = (uint8_t)((data_bus & 0xF0u) | (device.protocol_data & 0x0Fu));
        device.protocol_address = (uint8_t)(((data_bus & 4u) >> 1) | ((data_bus & 8u) >> 3));
    } else if (!high && previous_high) {
        device.protocol_data = (uint8_t)((device.protocol_data & 0xF0u) | (data_bus >> 4));
        device.chip.write(device.protocol_address, device.protocol_data);
    }
    device.previous_out = out_pins;
}

void epsm_write_port(uint16_t address, uint8_t value) {
    if (active_epsm && address >= 0x401C && address <= 0x401F)
        active_epsm->chip.write(address & 3u, value);
}

void epsm_sample_stereo(float *left, float *right) {
    if (left) *left = active_epsm ? active_epsm->sample(0) : 0;
    if (right) *right = active_epsm ? active_epsm->sample(1) : 0;
}
