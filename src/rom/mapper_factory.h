/*
 * mapper_factory.h - Cartridge mapper selection and setup
 *
 * Author: @frankischilling
 *
 * This private header selects the mapper implementation and initializes cartridge
 * board state.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef MAPPER_FACTORY_H
#define MAPPER_FACTORY_H

// Mapper selection and initialization.
static bool vrc24_submapper_supported(int mapper_no, uint8_t submapper) {
    switch (mapper_no) {
        case 21: return submapper <= 2;
        case 22: return submapper == 0;
        case 23: case 25: return submapper <= 3;
        case 27: case 183: return submapper == 0;
        default: return false;
    }
}

static bool bandai_layout(int mapper, uint8_t submapper, bool nes2,
                          size_t chr_bytes,
                          RomRamSizes *ram, unsigned eeprom_sizes[2]) {
    if (!chr_bytes) return false;
    if (mapper == 153)
        return (!ram->prg_ram || ram->prg_ram == PRG_BANK_8K)
            && (!ram->prg_nvram || ram->prg_nvram == PRG_BANK_8K);

    unsigned serial_bytes = mapper == 159 || mapper == 157 ? 128 : submapper == 4 ? 0 : 256;
    if (nes2) {
        if (ram->prg_ram || (ram->prg_nvram && ram->prg_nvram != serial_bytes)) return false;
    } else if (ram->prg_ram + ram->prg_nvram > PRG_BANK_8K) {
        return false;
    }
    if (mapper == 157) {
        eeprom_sizes[0] = 256;
        eeprom_sizes[1] = !nes2 || ram->prg_nvram == 128 ? 128 : 0;
    } else if (mapper == 159) {
        eeprom_sizes[0] = 128;
    } else if (submapper == 0 || (submapper == 5 && ram->prg_nvram == 256)) {
        eeprom_sizes[0] = 256;
    }
    // The header describes the serial device, not an addressable PRG-RAM chip.
    ram->prg_ram = ram->prg_nvram = 0;
    return true;
}

static bool default_prg_ram_geometry_supported(int mapper_no) {
    switch (mapper_no) {
        case 0: case 2: case 3: case 4: case 7: case 9: case 10: case 11:
        case 13: case 15: case 18: case 21: case 22: case 23: case 24:
        case 25: case 26: case 27: case 28: case 30: case 32: case 33: case 48:
        case 64: case 65: case 66: case 67: case 68: case 71: case 72:
        case 73: case 74: case 75: case 76: case 78: case 79: case 85:
        case 87: case 88: case 89: case 92: case 93: case 94: case 95:
        case 96: case 97: case 101: case 105: case 111: case 113: case 118:
        case 119: case 140: case 144: case 146: case 151: case 154:
        case 158: case 180: case 184: case 185: case 191: case 192:
        case 194: case 195: case 206: case 232:
            return true;
        default:
            return false;
    }
}

static bool ram_geometry_supported(int mapper_no, bool nes2, const RomRamSizes *ram,
                                   bool chr_is_ram, size_t chr_sz) {
    size_t prg_total = ram->prg_ram + ram->prg_nvram;
    size_t chr_total = ram->chr_ram + ram->chr_nvram;
    bool default_prg_layout = default_prg_ram_geometry_supported(mapper_no);
    if (!default_prg_layout && mapper_no != 1 && mapper_no != 5 && mapper_no != 82 && mapper_no != 155
        && prg_total && (prg_total & (prg_total - 1))) return false;

    bool split_prg = ram->prg_ram && ram->prg_nvram;
    bool split_prg_supported = default_prg_layout || mapper_no == 1 || mapper_no == 4 || mapper_no == 5
        || mapper_no == 19 || mapper_no == 24 || mapper_no == 26 || mapper_no == 68
        || mapper_no == 76 || mapper_no == 85
        || mapper_no == 88 || mapper_no == 95 || mapper_no == 105 || mapper_no == 118
        || mapper_no == 154 || mapper_no == 155 || mapper_no == 206 || mapper_no == 210;
    if (split_prg && !split_prg_supported) return false;
    if (mapper_no == 1 || mapper_no == 105 || mapper_no == 155) {
        if (prg_total > 0x8000) return false;
    } else if (mapper_no == 5) {
        if (nes2) {
            const size_t supported[] = {0, 0x2000, 0x4000, 0x8000, 0x10000, 0x20000};
            bool work_ok = false, save_ok = false;
            for (size_t i = 0; i < sizeof(supported) / sizeof(supported[0]); ++i) {
                if (ram->prg_ram == supported[i]) work_ok = true;
                if (ram->prg_nvram == supported[i]) save_ok = true;
            }
            if (!work_ok || !save_ok) return false;
        } else if (prg_total > 0x10000) {
            return false; // Legacy bank registers address at most eight 8KB pages.
        }
    } else if (mapper_no == 19 || mapper_no == 210) {
        // These boards select only the pages their registers can address.
        // Larger declared chips remain partially unreachable, as on hardware.
    } else if (mapper_no == 90 || mapper_no == 209 || mapper_no == 211) {
        if (prg_total != 0) return false;
    } else if (!default_prg_layout && prg_total > 0x2000) {
        return false;
    }

    // Explicit CHR storage may coexist with CHR ROM even when this board
    // has no register path that selects it. Keep those chips independent
    // for persistence without replacing the ROM-backed PPU mapping.
    bool unmapped_chr_storage = !chr_is_ram && chr_total;
    if (chr_is_ram && ram->chr_ram && ram->chr_nvram) return false;
    if (nes2 && chr_is_ram && chr_total != chr_sz) return false;
    if (unmapped_chr_storage) return true;
    size_t chr_limit;
    switch (mapper_no) {
        case 1: case 9: case 10: case 11: case 89: case 105: case 113: case 144: case 155:
            chr_limit = 0x20000; break;
        case 79: case 146: chr_limit = 0x10000; break;
        case 3: chr_limit = 0x200000; break;
        case 4: case 24: case 26: case 118: chr_limit = 0x40000; break;
        case 33: case 48: case 67: case 68: chr_limit = 0x80000; break;
        case 80: case 82: case 207: chr_limit = 0x40000; break;
        case 72: case 78: case 92: case 140: chr_limit = 0x20000; break;
        case 87: chr_limit = 0x8000; break;
        case 101: chr_limit = 0x200000; break;
        case 64: case 158: chr_limit = 0x40000; break;
        case 5: chr_limit = 0x100000; break;
        case 93: chr_limit = CHR_BANK_8K; break;
        case 184: chr_limit = 0x8000; break;
        // These fixed windows and masked bank registers can leave part of a
        // declared chip unreachable without changing its physical allocation.
        case 0: case 2: case 7: case 13: case 15: case 66: case 71:
        case 73: case 94: case 97: case 180: case 185: case 232:
        case 28: case 30: case 96: case 111: chr_limit = SIZE_MAX; break;
        case 18: case 32: case 65: chr_limit = 0x40000; break;
        case 75: case 151: chr_limit = 0x20000; break;
        case 21: case 23: case 25: case 27: case 183: chr_limit = 0x80000; break;
        case 22: chr_limit = 0x40000; break;
        case 19: case 95: case 206: case 210: chr_limit = 0x40000; break;
        case 76: chr_limit = 0x80000; break;
        case 88: case 154: chr_limit = 0x20000; break;
        case 90: case 209: case 211: chr_limit = 0x200000; break;
        case 85: chr_limit = 0x40000; break;
        default: chr_limit = 0x2000; break;
    }
    return chr_total <= chr_limit;
}

static void build_mapper(Mapper *m,
    uint8_t(*cr)(uint16_t), void(*cw)(uint16_t,uint8_t),
    uint8_t(*pr)(uint16_t), void(*pw)(uint16_t,uint8_t),
    void(*rst)(void), Mirroring(*gm)(void))
{
    m->cpu_read = cr; m->cpu_write = cw;
    m->ppu_read = pr; m->ppu_write = pw;
    m->reset = rst; m->clock = NULL;
    m->get_mirroring = gm;
}

static uint8_t fds_mapper_cpu_read(uint16_t addr) { return fds_cpu_read_bus(addr, 0xFF); }
static void fds_mapper_cpu_write(uint16_t addr, uint8_t value) { fds_cpu_write(addr, value); }
static uint8_t fds_mapper_ppu_read(uint16_t addr) { return fds_ppu_read(addr); }
static void fds_mapper_ppu_write(uint16_t addr, uint8_t value) { fds_ppu_write(addr, value); }
static Mirroring fds_mapper_mirroring(void) { return fds_mirroring(); }

int mapper_init_fds(FdsImage *image) {
    if (!image) return -1;
    mapper_shutdown();
    memset(&C, 0, sizeof(C));
    build_mapper(&mapper_fds, fds_mapper_cpu_read, fds_mapper_cpu_write,
                 fds_mapper_ppu_read, fds_mapper_ppu_write, fds_reset, fds_mapper_mirroring);
    mapper_fds.clock = fds_clock_cpu;
    cart = &mapper_fds;
    fds_activate(image);
    fds_reset();
    return 0;
}

static uint8_t board_mapper_cpu_read(uint16_t addr) {
    return board_cpu_read(active_board, addr, cart_cpu_bus_input);
}
static void board_mapper_cpu_write(uint16_t addr, uint8_t value) {
    board_cpu_write(active_board, addr, value);
}
static uint8_t board_mapper_ppu_read(uint16_t addr) {
    return board_ppu_read(active_board, addr, cart_ppu_fetch_source);
}
static void board_mapper_ppu_write(uint16_t addr, uint8_t value) {
    board_ppu_write(active_board, addr, value);
}
static void board_mapper_reset(void) { board_reset(active_board, true); }
static void board_mapper_clock(int cycles) {
    for (int i = 0; i < cycles; ++i) board_clock_cpu(active_board, cart_cpu_cycle_is_write);
}
static Mirroring board_mapper_mirroring(void) { return board_mirroring(active_board); }

static int activate_prepared_board(CartridgeBoard *prepared, uint16_t mapper_no,
                                   uint8_t *prg, size_t prg_sz,
                                   uint8_t *chr, size_t chr_sz) {
    if (!prepared) return -1;
    mapper_shutdown();
    active_board = prepared;
    C.mapper_no = mapper_no;
    C.prg = prg; C.prg_sz = prg_sz;
    C.chr = chr; C.chr_sz = chr_sz;
    C.mirr_base = board_mirroring(active_board);
    build_mapper(&mapper_board, board_mapper_cpu_read, board_mapper_cpu_write,
                 board_mapper_ppu_read, board_mapper_ppu_write,
                 board_mapper_reset, board_mapper_mirroring);
    mapper_board.clock = board_mapper_clock;
    cart = &mapper_board;
    return C.mapper_no;
}

int mapper_init_studybox(CartridgeBoard *prepared) {
    return activate_prepared_board(prepared, BOARD_STUDYBOX_MAPPER_ID, NULL, 0, NULL, 0) < 0 ? -1 : 0;
}

int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz)
{
    return mapper_init_from_header_metadata(h, prg, prg_sz, chr, chr_sz, NULL);
}

int mapper_init_from_header_metadata(const iNESHeader *h,
                                     uint8_t *prg, size_t prg_sz,
                                     uint8_t *chr, size_t chr_sz,
                                     const RomDatabaseInfo *database)
{
    if ((database && database->present && board_handles_mapper(database->mapper))
        || (h && board_handles_header(h))) {
        CartridgeBoard *prepared = board_create_with_metadata(h, prg, prg_sz, chr, chr_sz, database);
        if (!prepared) return -1;
        uint16_t mapper_no = board_is_fcns_header(h) ? BOARD_FCNS_MAPPER_ID
            : database && database->present ? database->mapper
                                            : (uint16_t)rom_mapper_number(h);
        return activate_prepared_board(prepared, mapper_no, prg, prg_sz, chr, chr_sz);
    }
    if (!h || !prg || !prg_sz || !chr || !chr_sz) return -1;
    int mapper_no = database && database->present ? database->mapper : rom_mapper_number(h);
    /* Database corrections apply only to legacy/headerless images.  The loader
       uses a synthesized NES 2.0-shaped header to carry extended mapper and
       console metadata, but mapper behavior must retain the source image's
       legacy semantics. */
    bool nes2 = !(database && database->present) && (h->flags7 & 0x0C) == 0x08;
    uint8_t submapper = database && database->present && database->submapper_present
                      ? database->submapper : nes2 ? h->prg_ram_size >> 4 : 0;
    switch (mapper_no) {
        case 0: case 1: case 2: case 3: case 4: case 5:
        case 7: case 9: case 10: case 11: case 13: case 15: case 28: case 30: case 111: case 118: case 155:
        case 16: case 153: case 157: case 159:
        case 18: case 32: case 33: case 48: case 64: case 65: case 158:
        case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 183:
        case 19: case 66: case 67: case 68: case 71: case 72: case 73: case 75: case 76: case 78:
        case 80: case 82: case 85: case 87: case 88: case 89: case 92: case 93: case 95: case 96: case 97: case 99: case 101:
        case 140: case 151: case 154: case 184: case 185: case 206: case 207: case 210:
        case 90: case 105: case 209: case 211: case 232:
        case 79: case 94: case 113: case 144: case 146: case 180:
            break;
        default:
            fprintf(stderr, "Unsupported mapper: %d\n", mapper_no);
            return -1;
    }
    // A mapper can use its normal bank path as soon as the image contains one
    // complete native PRG page. Smaller images require an explicit reduced-page
    // path that repeats whole physical copies through the CPU window.
    size_t prg_page_size = mapper_prg_page_size((uint16_t)mapper_no);
    bool has_native_prg_page = prg_page_size && prg_sz >= prg_page_size;
    bool reduced_prg_supported = prg_page_size != 0;
    if (prg_sz < PRG_BANK_16K && !has_native_prg_page && !reduced_prg_supported) {
        fprintf(stderr, "Unsupported PRG size for mapper %d\n", mapper_no);
        return -1;
    }
    bool ignores_submapper = mapper_no == 79 || mapper_no == 94 || mapper_no == 113
        || mapper_no == 144 || mapper_no == 146 || mapper_no == 180;
    if (submapper && !(ignores_submapper
        || (mapper_no == 1 && submapper == 5)
        || vrc24_submapper_supported(mapper_no, submapper)
        || (mapper_no == 85 && submapper <= 2)
        || (mapper_no == 4 && (submapper == 1 || submapper == 3))
        || (mapper_no == 16 && (submapper == 4 || submapper == 5))
        || (mapper_no == 48 && submapper == 1)
        || (mapper_no == 32 && submapper == 1)
        || (mapper_no == 71 && submapper == 1)
        || (mapper_no == 232 && submapper == 1)
        || (mapper_no == 78 && (submapper == 1 || submapper == 3))
        || (mapper_no == 185 && submapper >= 4 && submapper <= 7)
        || (mapper_no == 206 && submapper == 1)
        || (mapper_no == 210 && submapper <= 2)
        || ((mapper_no == 2 || mapper_no == 3 || mapper_no == 7) && submapper <= 2)
        || (mapper_no == 30 && submapper <= 4))) {
        fprintf(stderr, "Unsupported mapper/submapper: %d/%u\n", mapper_no, submapper);
        return -1;
    }
    RomRamSizes ram;
    rom_ram_sizes_with_metadata(h, database, &ram);
    bool is_jy = mapper_no == 90 || mapper_no == 209 || mapper_no == 211;
    if (is_jy && !nes2) ram.prg_ram = ram.prg_nvram = 0;
    if (mapper_no == 99 && !nes2 && !(h->flags6 & 2)) {
        ram.prg_ram = 0x800;
    }
    if ((ram.prg_nvram || ram.chr_nvram) && !(h->flags6 & 2)) {
        fprintf(stderr, "Nonvolatile RAM declared without the battery flag\n");
        return -1;
    }
    bool chr_is_ram = database && database->present
                    ? database->chr_rom_size == 0
                    : h->chr_rom_chunks == 0 && (!nes2 || (h->flags9 & 0xF0) == 0);
    size_t chr_page_size = mapper_chr_page_size((uint16_t)mapper_no);
    if (!chr_is_ram && chr_page_size && chr_sz < chr_page_size
        && !mapper_has_shrinking_chr_window((uint16_t)mapper_no)) {
        fprintf(stderr, "Unsupported CHR-ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    unsigned eeprom_sizes[2] = {0, 0};
    bool is_bandai = mapper_no == 16 || mapper_no == 153 || mapper_no == 157 || mapper_no == 159;
    if (is_bandai && !bandai_layout(mapper_no, submapper, nes2, chr_sz,
                                    &ram, eeprom_sizes)) {
        fprintf(stderr, "Unsupported cartridge layout for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 30 && !chr_is_ram && (h->flags6 & 9u) == 9u
        && ram.chr_ram && ram.chr_nvram) {
        fprintf(stderr, "Unsupported mixed CHR storage for four-screen mapper 30\n");
        return -1;
    }
    if (!ram_geometry_supported(mapper_no, nes2, &ram, chr_is_ram, chr_sz)
        || (mapper_no == 4 && submapper == 1 && ram.prg_ram + ram.prg_nvram != 0x400)) {
        fprintf(stderr, "Unsupported RAM layout for mapper %d (PRG %zu+%zu, CHR %zu+%zu)\n",
                mapper_no, ram.prg_ram, ram.prg_nvram, ram.chr_ram, ram.chr_nvram);
        return -1;
    }
    RamBlock new_work = {NULL, ram.prg_ram};
    RamBlock new_save = {NULL, ram.prg_nvram};
    RamBlock new_chr_work = {NULL, !chr_is_ram ? ram.chr_ram : 0};
    RamBlock new_chr_save = {NULL, !chr_is_ram ? ram.chr_nvram : 0};
    if (new_work.size) new_work.data = (uint8_t *)calloc(1, new_work.size);
    if (new_save.size) new_save.data = (uint8_t *)calloc(1, new_save.size);
    if (new_chr_work.size) new_chr_work.data = (uint8_t *)calloc(1, new_chr_work.size);
    if (new_chr_save.size) new_chr_save.data = (uint8_t *)calloc(1, new_chr_save.size);
    if ((new_work.size && !new_work.data) || (new_save.size && !new_save.data)
        || (new_chr_work.size && !new_chr_work.data)
        || (new_chr_save.size && !new_chr_save.data)) {
        free(new_work.data);
        free(new_save.data);
        free(new_chr_work.data);
        free(new_chr_save.data);
        fprintf(stderr, "Cartridge RAM allocation failed\n");
        return -1;
    }

    Vrc7Fm new_fm = {0};
    if (mapper_no == 85 && !vrc7_fm_init(&new_fm)) {
        free(new_work.data);
        free(new_save.data);
        free(new_chr_work.data);
        free(new_chr_save.data);
        fprintf(stderr, "VRC7 audio allocation failed\n");
        return -1;
    }

    nes_initialize_power_on_ram(new_work.data, new_work.size, 0);
    nes_initialize_power_on_ram(new_save.data, new_save.size, 0);
    nes_initialize_power_on_ram(new_chr_work.data, new_chr_work.size, 0);
    nes_initialize_power_on_ram(new_chr_save.data, new_chr_save.size, 0);

    // Finish all fallible setup before releasing the previous cartridge's RAM.
    mapper_shutdown();
    prg_work_ram = new_work;
    prg_save_ram = new_save;
    chr_work_ram = new_chr_work;
    chr_save_ram = new_chr_save;
    C.prg = prg; C.prg_sz = prg_sz;
    C.chr = chr; C.chr_sz = chr_sz;
    C.chr_is_ram = chr_is_ram;
    C.mapper_no = (uint16_t)mapper_no;
    C.ram = ram;
    C.nes2 = nes2;
    C.submapper = submapper;
    C.mmc1a = mapper_no == 155;
    C.bus_conflicts = mapper_no == 11 || mapper_no == 144
        || mapper_no == 72 || mapper_no == 78 || mapper_no == 92
        || mapper_no == 96 || mapper_no == 185
        || (submapper == 2 && (mapper_no == 2 || mapper_no == 3 || mapper_no == 7 || mapper_no == 30))
        || (mapper_no == 30 && submapper == 0 && !(h->flags6 & 2));
    if (database && database->present && database->bus_conflicts >= 0)
        C.bus_conflicts = database->bus_conflicts != 0;
    
    // iNES flags6:
    // bit 0 = 1 -> VERTICAL mirroring, 0 -> HORIZONTAL mirroring
    // bit 3 = 1 -> four-screen (overrides bit 0)
    Mirroring mir;
    if (h->flags6 & 0x08) {
        mir = MIRROR_FOUR;
    } else {
        mir = (h->flags6 & 0x01) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
    }
    if (database && database->present && database->mirroring_override)
        mir = database->mirroring;
    cart_set_mirroring(mir);

    switch(mapper_no) {
        case 0:
            build_mapper(&mapper_nrom, nrom_cpu_read, nrom_cpu_write,
                        nrom_ppu_read, nrom_ppu_write, NULL, nrom_mirr);
            cart = &mapper_nrom;
            break;
        case 1: case 155:
            build_mapper(&mapper_mmc1, mmc1_cpu_read, mmc1_cpu_write,
                        mmc1_ppu_read, mmc1_ppu_write, mmc1_reset, mmc1_mirr);
            cart = &mapper_mmc1;
            break;
        case 2: case 94: case 180:
            build_mapper(&mapper_uxrom, uxrom_cpu_read, uxrom_cpu_write,
                        uxrom_ppu_read, uxrom_ppu_write, uxrom_reset, uxrom_mirr);
            cart = &mapper_uxrom;
            break;
        case 3:
            build_mapper(&mapper_cnrom, cnrom_cpu_read, cnrom_cpu_write,
                        cnrom_ppu_read, cnrom_ppu_write, cnrom_reset, cnrom_mirr);
            cart = &mapper_cnrom;
            break;
        case 185:
            build_mapper(&mapper_cnrom_protect, cnrom185_cpu_read, cnrom185_cpu_write,
                         cnrom185_ppu_read, cnrom185_ppu_write,
                         cnrom185_reset, cnrom185_mirr);
            cart = &mapper_cnrom_protect;
            break;
        case 4:
            build_mapper(&mapper_mmc3, mmc3_cpu_read, mmc3_cpu_write,
                        mmc3_ppu_read, mmc3_ppu_write, mmc3_reset, mmc3_mirr);
            cart = &mapper_mmc3;
            break;
        case 5:
            build_mapper(&mapper_mmc5, mmc5_cpu_read, mmc5_cpu_write,
                        mmc5_ppu_read, mmc5_ppu_write, mmc5_reset, mmc5_mirr);
            mapper_mmc5.clock = mmc5_clock;
            cart = &mapper_mmc5;
            break;
        case 7:
            build_mapper(&mapper_aorom, aorom_cpu_read, aorom_cpu_write,
                        aorom_ppu_read, aorom_ppu_write, aorom_reset, aorom_mirr);
            cart = &mapper_aorom;
            break;
        case 9:
            build_mapper(&mapper_mmc2, mmc2_cpu_read, mmc2_cpu_write,
                        mmc2_ppu_read, mmc2_ppu_write, mmc2_reset, mmc2_mirr);
            cart = &mapper_mmc2;
            break;
        case 10:
            build_mapper(&mapper_mmc4, mmc4_cpu_read, mmc4_cpu_write,
                        mmc4_ppu_read, mmc4_ppu_write, mmc4_reset, mmc4_mirr);
            cart = &mapper_mmc4;
            break;
        case 11: case 144:
            build_mapper(&mapper_colordreams, colordreams_cpu_read, colordreams_cpu_write,
                        colordreams_ppu_read, colordreams_ppu_write, colordreams_reset, colordreams_mirr);
            cart = &mapper_colordreams;
            break;
        case 79: case 113: case 146:
            build_mapper(&mapper_nina, nina_cpu_read, nina_cpu_write,
                         nina_ppu_read, nina_ppu_write, nina_reset, nina_mirr);
            cart = &mapper_nina;
            break;
        case 13:
            build_mapper(&mapper_cprom, cprom_cpu_read, cprom_cpu_write,
                        cprom_ppu_read, cprom_ppu_write, cprom_reset, cprom_mirr);
            cart = &mapper_cprom;
            break;
        case 15:
            build_mapper(&mapper_100in1, m15_cpu_read, m15_cpu_write,
                        m15_ppu_read, m15_ppu_write, m15_reset, m15_mirr);
            cart = &mapper_100in1;
            break;
        case 24: case 26:
            memset(&vrc6, 0, sizeof(vrc6));
            vrc6.variant_b = mapper_no == 26;
            build_mapper(&mapper_vrc6, vrc6_cpu_read, vrc6_cpu_write,
                        vrc6_ppu_read, vrc6_ppu_write, vrc6_reset, vrc6_mirr);
            mapper_vrc6.clock = vrc6_clock;
            cart = &mapper_vrc6;
            break;
        case 21: case 22: case 23: case 25: case 27: case 183:
            memset(&vrc24, 0, sizeof(vrc24));
            if (!vrc24_select_variant()) return -1;
            build_mapper(&mapper_vrc24, vrc24_cpu_read, vrc24_cpu_write,
                        vrc24_ppu_read, vrc24_ppu_write, vrc24_reset, vrc24_mirr);
            if (vrc24_has_irq()) mapper_vrc24.clock = vrc24_clock;
            cart = &mapper_vrc24;
            break;
        case 28:
            build_mapper(&mapper_action53, m28_cpu_read, m28_cpu_write,
                        m28_ppu_read, m28_ppu_write, NULL, m28_mirr);
            cart = &mapper_action53;
            m28_power_on();
            break;
        case 33:
            build_mapper(&mapper_taito33, taito33_cpu_read, taito33_cpu_write,
                        taito33_ppu_read, taito33_ppu_write, taito33_reset, taito33_mirr);
            cart = &mapper_taito33;
            break;
        case 66:
            build_mapper(&mapper_gxrom, gxrom_cpu_read, gxrom_cpu_write,
                         gxrom_ppu_read, gxrom_ppu_write, gxrom_reset, gxrom_mirr);
            cart = &mapper_gxrom;
            break;
        case 71:
            m71.force_bf9097 = submapper == 1;
            build_mapper(&mapper_m71, m71_cpu_read, m71_cpu_write,
                         m71_ppu_read, m71_ppu_write, m71_reset, m71_mirr);
            cart = &mapper_m71;
            break;
        case 72: case 78: case 87: case 92: case 101: case 140:
            build_mapper(&mapper_jaleco_discrete, jaleco_discrete_cpu_read, jaleco_discrete_cpu_write,
                         jaleco_discrete_ppu_read, jaleco_discrete_ppu_write,
                         jaleco_discrete_reset, jaleco_discrete_mirr);
            cart = &mapper_jaleco_discrete;
            break;
        case 67:
            build_mapper(&mapper_sunsoft3, sunsoft3_cpu_read, sunsoft3_cpu_write,
                         sunsoft3_ppu_read, sunsoft3_ppu_write, sunsoft3_reset, sunsoft3_mirr);
            mapper_sunsoft3.clock = sunsoft3_clock;
            cart = &mapper_sunsoft3;
            sunsoft3_reset();
            break;
        case 68:
            build_mapper(&mapper_sunsoft4, sunsoft4_cpu_read, sunsoft4_cpu_write,
                         sunsoft4_ppu_read, sunsoft4_ppu_write, sunsoft4_reset, sunsoft4_mirr);
            mapper_sunsoft4.clock = sunsoft4_clock;
            cart = &mapper_sunsoft4;
            sunsoft4_reset();
            break;
        case 89:
            build_mapper(&mapper_sunsoft89, sunsoft89_cpu_read, sunsoft89_cpu_write,
                         sunsoft89_ppu_read, sunsoft89_ppu_write, sunsoft89_reset, sunsoft89_mirr);
            cart = &mapper_sunsoft89;
            sunsoft89_reset();
            break;
        case 93:
            build_mapper(&mapper_sunsoft93, sunsoft93_cpu_read, sunsoft93_cpu_write,
                         sunsoft93_ppu_read, sunsoft93_ppu_write, sunsoft93_reset, sunsoft93_mirr);
            cart = &mapper_sunsoft93;
            sunsoft93_reset();
            break;
        case 184:
            build_mapper(&mapper_sunsoft184, sunsoft184_cpu_read, sunsoft184_cpu_write,
                         sunsoft184_ppu_read, sunsoft184_ppu_write, sunsoft184_reset, sunsoft184_mirr);
            cart = &mapper_sunsoft184;
            sunsoft184_reset();
            break;
        case 73:
            build_mapper(&mapper_vrc3, vrc3_cpu_read, vrc3_cpu_write,
                         vrc3_ppu_read, vrc3_ppu_write, vrc3_reset, vrc3_mirr);
            mapper_vrc3.clock = vrc3_clock;
            cart = &mapper_vrc3;
            vrc3_reset();
            break;
        case 75: case 151:
            build_mapper(&mapper_vrc1, vrc1_cpu_read, vrc1_cpu_write,
                         vrc1_ppu_read, vrc1_ppu_write, vrc1_reset, vrc1_mirr);
            cart = &mapper_vrc1;
            vrc1_reset();
            break;
        case 76: case 88: case 95: case 154: case 206:
            namco108.fixed_prg = mapper_no == 206 && submapper == 1;
            build_mapper(&mapper_namco108, namco108_cpu_read, namco108_cpu_write,
                         namco108_ppu_read, namco108_ppu_write, namco108_reset, namco108_mirr);
            cart = &mapper_namco108;
            namco108_reset();
            break;
        case 232:
            build_mapper(&mapper_m232, m232_cpu_read, m232_cpu_write,
                         m232_ppu_read, m232_ppu_write, m232_reset, m232_mirr);
            cart = &mapper_m232;
            break;
        case 48:
            build_mapper(&mapper_taito48, taito48_cpu_read, taito48_cpu_write,
                        taito48_ppu_read, taito48_ppu_write, taito48_reset, taito48_mirr);
            mapper_taito48.clock = taito48_clock;
            cart = &mapper_taito48;
            break;
        case 30:
            build_mapper(&mapper_unrom512, m30_cpu_read, m30_cpu_write,
                        m30_ppu_read, m30_ppu_write, NULL, m30_mirr);
            cart = &mapper_unrom512;
            m30_power_on(h);
            break;
        case 18:
            build_mapper(&mapper_jaleco18, jaleco18_cpu_read, jaleco18_cpu_write,
                        jaleco18_ppu_read, jaleco18_ppu_write, jaleco18_reset, jaleco18_mirr);
            mapper_jaleco18.clock = jaleco18_clock;
            cart = &mapper_jaleco18;
            break;
        case 97:
            build_mapper(&mapper_irem97, irem97_cpu_read, irem97_cpu_write,
                         irem97_ppu_read, irem97_ppu_write, NULL, irem97_mirr);
            cart = &mapper_irem97;
            irem97_power_on();
            break;
        case 32:
            build_mapper(&mapper_irem32, irem32_cpu_read, irem32_cpu_write,
                        irem32_ppu_read, irem32_ppu_write, irem32_reset, irem32_mirr);
            cart = &mapper_irem32;
            break;
        case 65:
            build_mapper(&mapper_irem65, irem65_cpu_read, irem65_cpu_write,
                        irem65_ppu_read, irem65_ppu_write, irem65_reset, irem65_mirr);
            mapper_irem65.clock = irem65_clock;
            cart = &mapper_irem65;
            break;
        case 64:
            build_mapper(&mapper_rambo1, rambo1_cpu_read, rambo1_cpu_write,
                        rambo1_ppu_read, rambo1_ppu_write, rambo1_reset, rambo1_mirr);
            mapper_rambo1.clock = rambo1_clock;
            cart = &mapper_rambo1;
            break;
        case 118:
            build_mapper(&mapper_txsrom, mmc3_cpu_read, txsrom_cpu_write,
                         mmc3_ppu_read, mmc3_ppu_write, txsrom_reset, mmc3_mirr);
            cart = &mapper_txsrom;
            break;
        case 19: case 210:
            if (mapper_no == 19) namco.variant = NAMCO_VARIANT_163;
            else if (submapper == 1) namco.variant = NAMCO_VARIANT_175;
            else if (submapper == 2) namco.variant = NAMCO_VARIANT_340;
            else namco.variant = NAMCO_VARIANT_UNKNOWN;
            namco.auto_detect = (mapper_no == 210 && submapper == 0)
                             || (mapper_no == 19 && !nes2);
            build_mapper(&mapper_namco, namco_cpu_read, namco_cpu_write,
                         namco_ppu_read, namco_ppu_write, namco_reset, namco_mirr);
            mapper_namco.clock = namco_clock;
            cart = &mapper_namco;
            break;
        case 85:
            vrc7.fm = new_fm;
            build_mapper(&mapper_vrc7, vrc7_cpu_read, vrc7_cpu_write,
                        vrc7_ppu_read, vrc7_ppu_write, vrc7_reset, vrc7_mirr);
            mapper_vrc7.clock = vrc7_clock;
            cart = &mapper_vrc7;
            break;
        case 96:
            build_mapper(&mapper_m96, m96_cpu_read, m96_cpu_write,
                         m96_ppu_read, m96_ppu_write, m96_reset, m96_mirr);
            cart = &mapper_m96;
            break;
        case 90: case 209: case 211:
            build_mapper(&mapper_jy, jy_cpu_read, jy_cpu_write,
                         jy_ppu_read, jy_ppu_write, jy_reset, jy_mirr);
            mapper_jy.clock = jy_clock;
            cart = &mapper_jy;
            break;
        case 99:
            build_mapper(&mapper_vs99, m99_cpu_read, m99_cpu_write,
                         m99_ppu_read, m99_ppu_write, NULL, m99_mirr);
            cart = &mapper_vs99;
            break;
        case 105:
            build_mapper(&mapper_m105, m105_cpu_read, m105_cpu_write,
                         m105_ppu_read, nrom_ppu_write, m105_reset, mmc1_mirr);
            mapper_m105.clock = m105_clock;
            cart = &mapper_m105;
            break;
        case 111:
            build_mapper(&mapper_m111, m111_cpu_read, m111_cpu_write,
                         m111_ppu_read, m111_ppu_write, m111_reset, m111_mirr);
            cart = &mapper_m111;
            break;
        case 16: case 153: case 157: case 159:
            build_mapper(&mapper_bandai, bandai_cpu_read, bandai_cpu_write,
                         bandai_ppu_read, bandai_ppu_write, NULL, bandai_mirroring);
            mapper_bandai.clock = bandai_clock;
            bandai_init(mapper_no, eeprom_sizes[0], eeprom_sizes[1]);
            cart = &mapper_bandai;
            break;
        case 158:
            build_mapper(&mapper_rambo158, rambo1_cpu_read, rambo158_cpu_write,
                        rambo1_ppu_read, rambo1_ppu_write, rambo1_reset, rambo1_mirr);
            mapper_rambo158.clock = rambo1_clock;
            cart = &mapper_rambo158;
            break;
    }
    
    if (cart && cart->reset) cart->reset();
    return mapper_no;
}

#endif // MAPPER_FACTORY_H
