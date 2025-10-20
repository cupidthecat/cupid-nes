// rom/mapper.c - Extended implementation with mappers 0-15
#include <string.h>
#include <stdlib.h>
#include "mapper.h"

#define PRG_BANK_8K  0x2000
#define PRG_BANK_16K 0x4000
#define PRG_BANK_32K 0x8000
#define CHR_BANK_1K  0x0400
#define CHR_BANK_2K  0x0800
#define CHR_BANK_4K  0x1000
#define CHR_BANK_8K  0x2000

// ----------------- Cart wiring/state -----------------
typedef struct {
    uint8_t *prg; size_t prg_sz;
    uint8_t *chr; size_t chr_sz;
    bool chr_is_ram;
    Mirroring mirr_base;
} CartCommon;

static CartCommon C;
static Mapper mapper_nrom, mapper_mmc1, mapper_uxrom, mapper_cnrom, mapper_mmc3;
static Mapper mapper_mmc5, mapper_aorom, mapper_mmc2, mapper_mmc4, mapper_colordreams;
static Mapper mapper_cprom, mapper_100in1;
Mapper *cart = NULL;

static uint8_t prg_ram[0x2000];

// Helpers
static inline Mirroring base_mirr(void) { return C.mirr_base; }
Mirroring cart_get_mirroring(void) { return cart && cart->get_mirroring ? cart->get_mirroring() : base_mirr(); }
uint8_t cart_cpu_read(uint16_t a) { return cart ? cart->cpu_read(a) : 0xFF; }
void cart_cpu_write(uint16_t a, uint8_t v) { if (cart) cart->cpu_write(a, v); }
uint8_t cart_ppu_read(uint16_t a) { return cart ? cart->ppu_read(a) : 0x00; }
void cart_ppu_write(uint16_t a, uint8_t v) { if (cart) cart->ppu_write(a, v); }

// ----------------- Mapper 0 (NROM) -------------------
static uint8_t nrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        if (C.prg_sz == PRG_BANK_16K) return C.prg[(a - 0x8000) & 0x3FFF];
        return C.prg[(a - 0x8000) & 0x7FFF];
    }
    return 0xFF;
}
static void nrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) prg_ram[a - 0x6000] = v;
}
static uint8_t nrom_ppu_read(uint16_t a) { return C.chr[a & 0x1FFF]; }
static void nrom_ppu_write(uint16_t a, uint8_t v) { if (C.chr_is_ram) C.chr[a & 0x1FFF] = v; }
static Mirroring nrom_mirr(void) { return C.mirr_base; }

// ----------------- Mapper 1 (MMC1/SxROM) -----------------
static struct {
    uint8_t shift_reg;
    uint8_t shift_count;
    uint8_t control;
    uint8_t chr_bank0, chr_bank1;
    uint8_t prg_bank;
    Mirroring mirr;
} mmc1;

static void mmc1_write_control(uint8_t v) {
    mmc1.control = v & 0x1F;
    switch (v & 3) {
        case 0: mmc1.mirr = MIRROR_SINGLE0; break;
        case 1: mmc1.mirr = MIRROR_SINGLE1; break;
        case 2: mmc1.mirr = MIRROR_VERTICAL; break;
        case 3: mmc1.mirr = MIRROR_HORIZONTAL; break;
    }
}

static uint8_t mmc1_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        size_t prg_banks = C.prg_sz / PRG_BANK_16K;
        uint8_t prg_mode = (mmc1.control >> 2) & 3;
        size_t bank = 0;
        
        if (prg_mode == 0 || prg_mode == 1) {
            // 32KB mode
            bank = (mmc1.prg_bank >> 1) % (prg_banks / 2);
            return C.prg[bank * PRG_BANK_32K + (a - 0x8000)];
        } else if (prg_mode == 2) {
            // Fix first bank, switch $C000
            if (a < 0xC000) bank = 0;
            else bank = mmc1.prg_bank % prg_banks;
            return C.prg[bank * PRG_BANK_16K + ((a - 0x8000) & 0x3FFF)];
        } else {
            // Fix last bank, switch $8000
            if (a < 0xC000) bank = mmc1.prg_bank % prg_banks;
            else bank = (prg_banks > 0) ? prg_banks - 1 : 0;
            return C.prg[bank * PRG_BANK_16K + ((a - 0x8000) & 0x3FFF)];
        }
    }
    return 0xFF;
}

static void mmc1_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) {
        if (v & 0x80) {
            mmc1.shift_reg = 0;
            mmc1.shift_count = 0;
            mmc1_write_control(mmc1.control | 0x0C);
        } else {
            mmc1.shift_reg = ((v & 1) << 4) | (mmc1.shift_reg >> 1);
            mmc1.shift_count++;
            if (mmc1.shift_count == 5) {
                uint8_t reg = (a >> 13) & 3;
                if (reg == 0) mmc1_write_control(mmc1.shift_reg);
                else if (reg == 1) mmc1.chr_bank0 = mmc1.shift_reg;
                else if (reg == 2) mmc1.chr_bank1 = mmc1.shift_reg;
                else mmc1.prg_bank = mmc1.shift_reg & 0x0F;
                mmc1.shift_reg = 0;
                mmc1.shift_count = 0;
            }
        }
    }
}

static uint8_t mmc1_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_banks == 0) return C.chr[a];
    
    uint8_t chr_mode = (mmc1.control >> 4) & 1;
    if (chr_mode == 0) {
        // 8KB mode
        size_t bank = (mmc1.chr_bank0 >> 1) % (chr_banks / 2);
        return C.chr[bank * CHR_BANK_8K + a];
    } else {
        // 4KB mode
        size_t bank = (a < 0x1000) ? (mmc1.chr_bank0 % chr_banks) : (mmc1.chr_bank1 % chr_banks);
        return C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)];
    }
}

static void mmc1_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_banks == 0) { C.chr[a] = v; return; }
    
    uint8_t chr_mode = (mmc1.control >> 4) & 1;
    if (chr_mode == 0) {
        size_t bank = (mmc1.chr_bank0 >> 1) % (chr_banks / 2);
        C.chr[bank * CHR_BANK_8K + a] = v;
    } else {
        size_t bank = (a < 0x1000) ? (mmc1.chr_bank0 % chr_banks) : (mmc1.chr_bank1 % chr_banks);
        C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)] = v;
    }
}

static Mirroring mmc1_mirr(void) { return mmc1.mirr; }
static void mmc1_reset(void) {
    mmc1.shift_reg = 0;
    mmc1.shift_count = 0;
    mmc1_write_control(0x0C);
    mmc1.chr_bank0 = 0;
    mmc1.chr_bank1 = 0;
    mmc1.prg_bank = 0;
}

// ----------------- Mapper 2 (UxROM) -----------------
static struct { uint8_t bank; } ux;
static uint8_t uxrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000 && a <= 0xBFFF) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t b = (banks == 0) ? 0 : (ux.bank % banks);
        return C.prg[b * PRG_BANK_16K + (a - 0x8000)];
    }
    if (a >= 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t last = (banks ? banks : 1) - 1;
        return C.prg[last * PRG_BANK_16K + (a - 0xC000)];
    }
    return 0xFF;
}
static void uxrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) ux.bank = v & 0x1F;
}
static uint8_t uxrom_ppu_read(uint16_t a) { return nrom_ppu_read(a); }
static void uxrom_ppu_write(uint16_t a, uint8_t v) { nrom_ppu_write(a, v); }
static Mirroring uxrom_mirr(void) { return C.mirr_base; }
static void uxrom_reset(void) { ux.bank = 0; }

// ----------------- Mapper 3 (CNROM) -----------------
static struct { uint8_t chr_bank; } cn;
static uint8_t cnrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) return C.prg[(a - 0x8000) & 0x7FFF];
    return 0xFF;
}
static void cnrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) {
        size_t banks = C.chr_sz / CHR_BANK_8K;
        if (banks) cn.chr_bank = v % banks;
    }
}
static uint8_t cnrom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t off = (size_t)cn.chr_bank * CHR_BANK_8K + a;
    return C.chr[off % C.chr_sz];
}
static void cnrom_ppu_write(uint16_t a, uint8_t v) {
    if (C.chr_is_ram) {
        size_t off = (size_t)cn.chr_bank * CHR_BANK_8K + (a & 0x1FFF);
        C.chr[off % C.chr_sz] = v;
    }
}
static Mirroring cnrom_mirr(void) { return C.mirr_base; }
static void cnrom_reset(void) { cn.chr_bank = 0; }

// ----------------- Mapper 4 (MMC3/TxROM) -----------------
static struct {
    uint8_t bank_select;
    uint8_t banks[8];
    uint8_t prg_mode, chr_mode;
    Mirroring mirr;
    uint8_t irq_latch, irq_counter;
    bool irq_enabled, irq_reload;
} mmc3;

static uint8_t mmc3_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        size_t prg_8k_banks = C.prg_sz / PRG_BANK_8K;
        size_t bank = 0;
        if (a < 0xA000) {
            bank = mmc3.prg_mode ? (prg_8k_banks - 2) : mmc3.banks[6];
        } else if (a < 0xC000) {
            bank = mmc3.banks[7];
        } else if (a < 0xE000) {
            bank = mmc3.prg_mode ? mmc3.banks[6] : (prg_8k_banks - 2);
        } else {
            bank = prg_8k_banks - 1;
        }
        bank %= prg_8k_banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFF)];
    }
    return 0xFF;
}

static void mmc3_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) {
        if ((a & 0xE001) == 0x8000) {
            mmc3.bank_select = v & 7;
            mmc3.prg_mode = (v >> 6) & 1;
            mmc3.chr_mode = (v >> 7) & 1;
        } else if ((a & 0xE001) == 0x8001) {
            mmc3.banks[mmc3.bank_select] = v;
        } else if ((a & 0xE001) == 0xA000) {
            mmc3.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
        } else if ((a & 0xE001) == 0xC000) {
            mmc3.irq_latch = v;
        } else if ((a & 0xE001) == 0xC001) {
            mmc3.irq_reload = true;
        } else if ((a & 0xE001) == 0xE000) {
            mmc3.irq_enabled = false;
        } else if ((a & 0xE001) == 0xE001) {
            mmc3.irq_enabled = true;
        }
    }
}

static uint8_t mmc3_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return C.chr[a];
    
    size_t bank = 0;
    if (mmc3.chr_mode == 0) {
        if (a < 0x0800) bank = mmc3.banks[0] & 0xFE;
        else if (a < 0x1000) bank = mmc3.banks[1] & 0xFE;
        else if (a < 0x1400) bank = mmc3.banks[2];
        else if (a < 0x1800) bank = mmc3.banks[3];
        else if (a < 0x1C00) bank = mmc3.banks[4];
        else bank = mmc3.banks[5];
    } else {
        if (a < 0x0400) bank = mmc3.banks[2];
        else if (a < 0x0800) bank = mmc3.banks[3];
        else if (a < 0x0C00) bank = mmc3.banks[4];
        else if (a < 0x1000) bank = mmc3.banks[5];
        else if (a < 0x1800) bank = mmc3.banks[0] & 0xFE;
        else bank = mmc3.banks[1] & 0xFE;
    }
    bank %= chr_1k_banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void mmc3_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) { C.chr[a] = v; return; }
    
    size_t bank = 0;
    if (mmc3.chr_mode == 0) {
        if (a < 0x0800) bank = mmc3.banks[0] & 0xFE;
        else if (a < 0x1000) bank = mmc3.banks[1] & 0xFE;
        else if (a < 0x1400) bank = mmc3.banks[2];
        else if (a < 0x1800) bank = mmc3.banks[3];
        else if (a < 0x1C00) bank = mmc3.banks[4];
        else bank = mmc3.banks[5];
    } else {
        if (a < 0x0400) bank = mmc3.banks[2];
        else if (a < 0x0800) bank = mmc3.banks[3];
        else if (a < 0x0C00) bank = mmc3.banks[4];
        else if (a < 0x1000) bank = mmc3.banks[5];
        else if (a < 0x1800) bank = mmc3.banks[0] & 0xFE;
        else bank = mmc3.banks[1] & 0xFE;
    }
    bank %= chr_1k_banks;
    C.chr[bank * CHR_BANK_1K + (a & 0x03FF)] = v;
}

static Mirroring mmc3_mirr(void) { return mmc3.mirr; }
static void mmc3_reset(void) {
    memset(&mmc3, 0, sizeof(mmc3));
    mmc3.mirr = C.mirr_base;
}

// ----------------- Mapper 5 (MMC5/ExROM) - Simplified -----------------
static struct {
    uint8_t prg_mode;
    uint8_t prg_banks[4];
    uint8_t chr_banks[8];
    Mirroring mirr;
} mmc5;

static uint8_t mmc5_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        size_t prg_8k_banks = C.prg_sz / PRG_BANK_8K;
        size_t bank = 0;
        if (a < 0xA000) bank = mmc5.prg_banks[0];
        else if (a < 0xC000) bank = mmc5.prg_banks[1];
        else if (a < 0xE000) bank = mmc5.prg_banks[2];
        else bank = mmc5.prg_banks[3];
        bank %= prg_8k_banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFF)];
    }
    return 0xFF;
}

static void mmc5_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a == 0x5100) mmc5.prg_mode = v & 3;
    else if (a == 0x5101) {} // CHR mode (ignored for simplicity)
    else if (a >= 0x5113 && a <= 0x5117) {
        mmc5.prg_banks[a - 0x5113] = v;
    } else if (a >= 0x5120 && a <= 0x5127) {
        mmc5.chr_banks[a - 0x5120] = v;
    } else if (a == 0x5105) {
        // Nametable control
        mmc5.mirr = C.mirr_base;
    }
}

static uint8_t mmc5_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return C.chr[a];
    size_t bank = mmc5.chr_banks[a / CHR_BANK_1K] % chr_1k_banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void mmc5_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) { C.chr[a] = v; return; }
    size_t bank = mmc5.chr_banks[a / CHR_BANK_1K] % chr_1k_banks;
    C.chr[bank * CHR_BANK_1K + (a & 0x03FF)] = v;
}

static Mirroring mmc5_mirr(void) { return mmc5.mirr; }
static void mmc5_reset(void) {
    memset(&mmc5, 0, sizeof(mmc5));
    mmc5.mirr = C.mirr_base;
    size_t prg_8k_banks = C.prg_sz / PRG_BANK_8K;
    for (int i = 0; i < 4; i++) mmc5.prg_banks[i] = (prg_8k_banks > 0) ? prg_8k_banks - 1 : 0;
}

// ----------------- Mapper 7 (AOROM) -----------------
static struct { uint8_t prg_bank; Mirroring mirr; } ao;
static uint8_t aorom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        size_t b = (banks ? ao.prg_bank % banks : 0);
        return C.prg[b * PRG_BANK_32K + (a - 0x8000)];
    }
    return 0xFF;
}
static void aorom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) {
        ao.prg_bank = v & 0x07;
        ao.mirr = (v & 0x10) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
    }
}
static uint8_t aorom_ppu_read(uint16_t a) { return nrom_ppu_read(a); }
static void aorom_ppu_write(uint16_t a, uint8_t v) { nrom_ppu_write(a, v); }
static Mirroring aorom_mirr(void) { return ao.mirr; }
static void aorom_reset(void) { ao.prg_bank = 0; ao.mirr = C.mirr_base; }

// ----------------- Mapper 9 (MMC2/PxROM) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t chr_banks[4];
    uint8_t latch[2];
    Mirroring mirr;
} mmc2;

static uint8_t mmc2_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000 && a < 0xA000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        size_t bank = mmc2.prg_bank % banks;
        return C.prg[bank * PRG_BANK_8K + (a - 0x8000)];
    }
    if (a >= 0xA000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        size_t bank = (banks >= 3) ? (banks - 3) : 0;
        return C.prg[bank * PRG_BANK_8K + ((a - 0xA000) & 0x1FFF)];
    }
    return 0xFF;
}

static void mmc2_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0xA000 && a <= 0xAFFF) mmc2.prg_bank = v & 0x0F;
    else if (a >= 0xB000 && a <= 0xBFFF) mmc2.chr_banks[0] = v & 0x1F;
    else if (a >= 0xC000 && a <= 0xCFFF) mmc2.chr_banks[1] = v & 0x1F;
    else if (a >= 0xD000 && a <= 0xDFFF) mmc2.chr_banks[2] = v & 0x1F;
    else if (a >= 0xE000 && a <= 0xEFFF) mmc2.chr_banks[3] = v & 0x1F;
    else if (a >= 0xF000) mmc2.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
}

static uint8_t mmc2_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) return C.chr[a];
    
    uint8_t bank_idx = (a < 0x1000) ? (mmc2.latch[0] * 2) : (2 + mmc2.latch[1] * 2);
    size_t bank = mmc2.chr_banks[bank_idx] % chr_4k_banks;
    uint8_t val = C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)];
    
    // Latch update
    if (a == 0x0FD8) mmc2.latch[0] = 0;
    else if (a == 0x0FE8) mmc2.latch[0] = 1;
    else if (a >= 0x1FD8 && a <= 0x1FDF) mmc2.latch[1] = 0;
    else if (a >= 0x1FE8 && a <= 0x1FEF) mmc2.latch[1] = 1;
    
    return val;
}

static void mmc2_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) { C.chr[a] = v; return; }
    uint8_t bank_idx = (a < 0x1000) ? (mmc2.latch[0] * 2) : (2 + mmc2.latch[1] * 2);
    size_t bank = mmc2.chr_banks[bank_idx] % chr_4k_banks;
    C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)] = v;
}

static Mirroring mmc2_mirr(void) { return mmc2.mirr; }
static void mmc2_reset(void) {
    memset(&mmc2, 0, sizeof(mmc2));
    mmc2.mirr = C.mirr_base;
}

// ----------------- Mapper 10 (MMC4/FxROM) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t chr_banks[4];
    uint8_t latch[2];
    Mirroring mirr;
} mmc4;

static uint8_t mmc4_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000 && a < 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t bank = mmc4.prg_bank % banks;
        return C.prg[bank * PRG_BANK_16K + (a - 0x8000)];
    }
    if (a >= 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t bank = (banks > 0) ? banks - 1 : 0;
        return C.prg[bank * PRG_BANK_16K + (a - 0xC000)];
    }
    return 0xFF;
}

static void mmc4_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0xA000 && a <= 0xAFFF) mmc4.prg_bank = v & 0x0F;
    else if (a >= 0xB000 && a <= 0xBFFF) mmc4.chr_banks[0] = v & 0x1F;
    else if (a >= 0xC000 && a <= 0xCFFF) mmc4.chr_banks[1] = v & 0x1F;
    else if (a >= 0xD000 && a <= 0xDFFF) mmc4.chr_banks[2] = v & 0x1F;
    else if (a >= 0xE000 && a <= 0xEFFF) mmc4.chr_banks[3] = v & 0x1F;
    else if (a >= 0xF000) mmc4.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
}

static uint8_t mmc4_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) return C.chr[a];
    
    uint8_t bank_idx = (a < 0x1000) ? (mmc4.latch[0] * 2) : (2 + mmc4.latch[1] * 2);
    size_t bank = mmc4.chr_banks[bank_idx] % chr_4k_banks;
    uint8_t val = C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)];
    
    // Latch update (same as MMC2)
    if (a == 0x0FD8) mmc4.latch[0] = 0;
    else if (a == 0x0FE8) mmc4.latch[0] = 1;
    else if (a >= 0x1FD8 && a <= 0x1FDF) mmc4.latch[1] = 0;
    else if (a >= 0x1FE8 && a <= 0x1FEF) mmc4.latch[1] = 1;
    
    return val;
}

static void mmc4_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) { C.chr[a] = v; return; }
    uint8_t bank_idx = (a < 0x1000) ? (mmc4.latch[0] * 2) : (2 + mmc4.latch[1] * 2);
    size_t bank = mmc4.chr_banks[bank_idx] % chr_4k_banks;
    C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)] = v;
}

static Mirroring mmc4_mirr(void) { return mmc4.mirr; }
static void mmc4_reset(void) {
    memset(&mmc4, 0, sizeof(mmc4));
    mmc4.mirr = C.mirr_base;
}

// ----------------- Mapper 11 (Color Dreams) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
} colordreams;

static uint8_t colordreams_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        size_t bank = (banks > 0) ? (colordreams.prg_bank % banks) : 0;
        return C.prg[bank * PRG_BANK_32K + (a - 0x8000)];
    }
    return 0xFF;
}

static void colordreams_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) {
        colordreams.prg_bank = v & 0x03;
        colordreams.chr_bank = (v >> 4) & 0x0F;
    }
}

static uint8_t colordreams_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (banks == 0) return C.chr[a];
    size_t bank = colordreams.chr_bank % banks;
    return C.chr[bank * CHR_BANK_8K + a];
}

static void colordreams_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (banks == 0) { C.chr[a] = v; return; }
    size_t bank = colordreams.chr_bank % banks;
    C.chr[bank * CHR_BANK_8K + a] = v;
}

static Mirroring colordreams_mirr(void) { return C.mirr_base; }
static void colordreams_reset(void) {
    colordreams.prg_bank = 0;
    colordreams.chr_bank = 0;
}

// ----------------- Mapper 13 (CPROM) -----------------
static struct {
    uint8_t chr_bank;
} cprom;

static uint8_t cprom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) return C.prg[(a - 0x8000) & 0x7FFF];
    return 0xFF;
}

static void cprom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) cprom.chr_bank = v & 0x03;
}

static uint8_t cprom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    if (a < 0x1000) {
        // Fixed first 4KB
        return C.chr[a];
    } else {
        // Switchable second 4KB
        size_t off = (size_t)cprom.chr_bank * CHR_BANK_4K + (a - 0x1000);
        return C.chr[off % C.chr_sz];
    }
}

static void cprom_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    if (a < 0x1000) {
        C.chr[a] = v;
    } else {
        size_t off = (size_t)cprom.chr_bank * CHR_BANK_4K + (a - 0x1000);
        C.chr[off % C.chr_sz] = v;
    }
}

static Mirroring cprom_mirr(void) { return C.mirr_base; }
static void cprom_reset(void) { cprom.chr_bank = 0; }

// ----------------- Mapper 15 (100-in-1 Contra Function 16) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t mode;
    Mirroring mirr;
} m15;

static uint8_t m15_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram[a - 0x6000];
    if (a >= 0x8000) {
        size_t banks_16k = C.prg_sz / PRG_BANK_16K;
        size_t banks_32k = C.prg_sz / PRG_BANK_32K;
        
        switch (m15.mode) {
            case 0: // 32KB mode
            case 1: {
                size_t bank = (banks_32k > 0) ? (m15.prg_bank % banks_32k) : 0;
                return C.prg[bank * PRG_BANK_32K + (a - 0x8000)];
            }
            case 2: // 16KB mode, fixed second bank
                if (a < 0xC000) {
                    size_t bank = (banks_16k > 0) ? (m15.prg_bank % banks_16k) : 0;
                    return C.prg[bank * PRG_BANK_16K + (a - 0x8000)];
                } else {
                    size_t bank = (banks_16k > 0) ? ((m15.prg_bank + 1) % banks_16k) : 0;
                    return C.prg[bank * PRG_BANK_16K + (a - 0xC000)];
                }
            case 3: // 16KB mode, mirrored
            default: {
                size_t bank = (banks_16k > 0) ? (m15.prg_bank % banks_16k) : 0;
                return C.prg[bank * PRG_BANK_16K + ((a - 0x8000) & 0x3FFF)];
            }
        }
    }
    return 0xFF;
}

static void m15_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram[a - 0x6000] = v; return; }
    if (a >= 0x8000) {
        m15.prg_bank = v & 0x3F;
        m15.mode = (v >> 6) & 0x03;
        m15.mirr = (v & 0x80) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
    }
}

static uint8_t m15_ppu_read(uint16_t a) { return nrom_ppu_read(a); }
static void m15_ppu_write(uint16_t a, uint8_t v) { nrom_ppu_write(a, v); }
static Mirroring m15_mirr(void) { return m15.mirr; }
static void m15_reset(void) {
    m15.prg_bank = 0;
    m15.mode = 0;
    m15.mirr = C.mirr_base;
}

// ----------------- Init/Factory ----------------------
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

int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz)
{
    memset(&C, 0, sizeof(C));
    C.prg = prg; C.prg_sz = prg_sz;
    C.chr = chr; C.chr_sz = chr_sz ? chr_sz : CHR_BANK_8K;
    C.chr_is_ram = (h->chr_rom_chunks == 0);
    C.mirr_base = (h->flags6 & 0x01) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
    if (h->flags6 & 0x08) C.mirr_base = MIRROR_FOUR;

    int mapper_no = ((h->flags7 & 0xF0) | ((h->flags6 & 0xF0) >> 4));

    switch(mapper_no) {
        case 0:
            build_mapper(&mapper_nrom, nrom_cpu_read, nrom_cpu_write,
                        nrom_ppu_read, nrom_ppu_write, NULL, nrom_mirr);
            cart = &mapper_nrom;
            break;
        case 1:
            build_mapper(&mapper_mmc1, mmc1_cpu_read, mmc1_cpu_write,
                        mmc1_ppu_read, mmc1_ppu_write, mmc1_reset, mmc1_mirr);
            cart = &mapper_mmc1;
            break;
        case 2:
            build_mapper(&mapper_uxrom, uxrom_cpu_read, uxrom_cpu_write,
                        uxrom_ppu_read, uxrom_ppu_write, uxrom_reset, uxrom_mirr);
            cart = &mapper_uxrom;
            break;
        case 3:
            build_mapper(&mapper_cnrom, cnrom_cpu_read, cnrom_cpu_write,
                        cnrom_ppu_read, cnrom_ppu_write, cnrom_reset, cnrom_mirr);
            cart = &mapper_cnrom;
            break;
        case 4:
            build_mapper(&mapper_mmc3, mmc3_cpu_read, mmc3_cpu_write,
                        mmc3_ppu_read, mmc3_ppu_write, mmc3_reset, mmc3_mirr);
            cart = &mapper_mmc3;
            break;
        case 5:
            build_mapper(&mapper_mmc5, mmc5_cpu_read, mmc5_cpu_write,
                        mmc5_ppu_read, mmc5_ppu_write, mmc5_reset, mmc5_mirr);
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
        case 11:
            build_mapper(&mapper_colordreams, colordreams_cpu_read, colordreams_cpu_write,
                        colordreams_ppu_read, colordreams_ppu_write, colordreams_reset, colordreams_mirr);
            cart = &mapper_colordreams;
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
        default:
            // Fallback to NROM
            build_mapper(&mapper_nrom, nrom_cpu_read, nrom_cpu_write,
                        nrom_ppu_read, nrom_ppu_write, NULL, nrom_mirr);
            cart = &mapper_nrom;
            break;
    }
    
    if (cart && cart->reset) cart->reset();
    return mapper_no;
}