/*
 * eeprom.h - Serial cartridge EEPROM pins and storage
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
#ifndef EEPROM_H
#define EEPROM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    EEPROM_IDLE,
    EEPROM_ADDRESS,
    EEPROM_READ,
    EEPROM_WRITE,
    EEPROM_SEND_ACK,
    EEPROM_WAIT_ACK,
    EEPROM_CHIP_ADDRESS
} EepromMode;

typedef struct {
    uint8_t bytes[256];
    unsigned capacity;
    EepromMode mode, next_mode;
    uint8_t chip_address, address, data, bit_count;
    bool output, scl, sda, dirty;
} Eeprom24;

// Capacity zero disconnects the chip; supported chips have 128 or 256 bytes.
void eeprom24_init(Eeprom24 *chip, unsigned capacity);
void eeprom24_write(Eeprom24 *chip, bool scl, bool sda);

#endif
