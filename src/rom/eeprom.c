/*
 * eeprom.c - 24C01 and 24C02 serial EEPROM transactions
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
#include "eeprom.h"
#include <string.h>

void eeprom24_init(Eeprom24 *chip, unsigned capacity) {
    if (!chip) return;
    memset(chip, 0, sizeof(*chip));
    if (capacity == 128 || capacity == 256) chip->capacity = capacity;
}

static void receive_bit(Eeprom24 *chip, uint8_t *value, bool sda) {
    if (chip->bit_count >= 8) return;
    unsigned shift = chip->capacity == 128 ? chip->bit_count : 7u - chip->bit_count;
    uint8_t mask = (uint8_t)(1u << shift);
    *value = (uint8_t)((*value & (uint8_t)~mask) | (sda ? mask : 0));
    chip->bit_count++;
}

static void next_address(Eeprom24 *chip) {
    chip->address = (uint8_t)((chip->address + 1u) & (chip->capacity - 1u));
}

static void rising_edge(Eeprom24 *chip, bool sda) {
    switch (chip->mode) {
        case EEPROM_CHIP_ADDRESS:
            receive_bit(chip, &chip->chip_address, sda);
            break;
        case EEPROM_ADDRESS:
            if (chip->capacity == 128 && chip->bit_count == 7) {
                chip->bit_count = 8;
                chip->next_mode = sda ? EEPROM_READ : EEPROM_WRITE;
                if (sda) chip->data = chip->bytes[chip->address & 0x7F];
            } else {
                receive_bit(chip, &chip->address, sda);
            }
            break;
        case EEPROM_READ:
            if (chip->bit_count < 8) {
                unsigned shift = chip->capacity == 128 ? chip->bit_count : 7u - chip->bit_count;
                chip->output = ((chip->data >> shift) & 1u) != 0;
                chip->bit_count++;
            }
            break;
        case EEPROM_WRITE:
            receive_bit(chip, &chip->data, sda);
            break;
        case EEPROM_SEND_ACK:
            chip->output = false;
            break;
        case EEPROM_WAIT_ACK:
            if (!sda) {
                chip->next_mode = chip->capacity == 128 ? EEPROM_IDLE : EEPROM_READ;
                if (chip->capacity == 256) chip->data = chip->bytes[chip->address];
            }
            break;
        case EEPROM_IDLE:
            break;
    }
}

static void falling_edge(Eeprom24 *chip) {
    switch (chip->mode) {
        case EEPROM_CHIP_ADDRESS:
            if (chip->bit_count == 8) {
                chip->bit_count = 0;
                chip->output = true;
                if ((chip->chip_address & 0xA0) != 0xA0) {
                    chip->mode = EEPROM_IDLE;
                } else {
                    chip->mode = EEPROM_SEND_ACK;
                    chip->next_mode = (chip->chip_address & 1) ? EEPROM_READ : EEPROM_ADDRESS;
                    if (chip->next_mode == EEPROM_READ) chip->data = chip->bytes[chip->address];
                }
            }
            break;
        case EEPROM_ADDRESS:
            if (chip->bit_count == 8) {
                chip->mode = EEPROM_SEND_ACK;
                chip->output = true;
                if (chip->capacity == 256) {
                    chip->bit_count = 0;
                    chip->next_mode = EEPROM_WRITE;
                }
            }
            break;
        case EEPROM_READ:
            if (chip->bit_count == 8) {
                chip->mode = EEPROM_WAIT_ACK;
                next_address(chip);
            }
            break;
        case EEPROM_WRITE:
            if (chip->bit_count == 8) {
                chip->mode = EEPROM_SEND_ACK;
                chip->next_mode = chip->capacity == 128 ? EEPROM_IDLE : EEPROM_WRITE;
                if (chip->bytes[chip->address] != chip->data) {
                    chip->bytes[chip->address] = chip->data;
                    chip->dirty = true;
                }
                next_address(chip);
                if (chip->capacity == 256) chip->bit_count = 0;
            }
            break;
        case EEPROM_SEND_ACK:
            chip->mode = chip->next_mode;
            chip->bit_count = 0;
            chip->output = true;
            break;
        case EEPROM_WAIT_ACK:
            if (chip->capacity == 256) {
                chip->mode = chip->next_mode;
                chip->bit_count = 0;
                chip->output = true;
            }
            break;
        case EEPROM_IDLE:
            break;
    }
}

void eeprom24_write(Eeprom24 *chip, bool scl, bool sda) {
    if (!chip || !chip->capacity) return;
    if (chip->scl && scl && chip->sda != sda) {
        // START and STOP need a stable high clock, not two simultaneous pin edges.
        chip->mode = sda ? EEPROM_IDLE
                        : chip->capacity == 128 ? EEPROM_ADDRESS : EEPROM_CHIP_ADDRESS;
        chip->output = true;
        if (!sda) {
            chip->bit_count = 0;
            if (chip->capacity == 128) chip->address = 0;
        }
    } else if (scl && !chip->scl) {
        rising_edge(chip, sda);
    } else if (!scl && chip->scl) {
        falling_edge(chip);
    }
    chip->scl = scl;
    chip->sda = sda;
}
