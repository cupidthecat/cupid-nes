#!/usr/bin/env python3
"""Run the canonical CPU trace and a fixed set of diagnostic ROMs."""

import argparse
from pathlib import Path
import subprocess
import sys


GROUPS = (
    ("instr_test-v5/rom_singles", 16),
    ("instr_misc/rom_singles", 4),
    ("instr_timing/rom_singles", 2),
    ("cpu_interrupts_v2/rom_singles", 5),
    ("apu_test/rom_singles", 8),
    ("ppu_vbl_nmi/rom_singles", 10),
    ("sprdma_and_dmc_dma", 2),
)
INDIVIDUAL_ROMS = (
    "ppu_open_bus/ppu_open_bus.nes",
    "ppu_read_buffer/test_ppu_read_buffer.nes",
    "oam_read/oam_read.nes",
    "cpu_dummy_writes/cpu_dummy_writes_ppumem.nes",
    "cpu_dummy_writes/cpu_dummy_writes_oam.nes",
)
MMC3_ROMS = (
    "1-clocking.nes",
    "2-details.nes",
    "3-A12_clocking.nes",
    "4-scanline_timing.nes",
    "5-MMC3.nes",
)


def run_checks(binary: Path, root: Path) -> int:
    binary = binary.resolve(strict=True)
    root = root.resolve(strict=True)
    roms = []
    for directory, expected in GROUPS:
        group = sorted((root / directory).glob("*.nes"))
        if len(group) != expected:
            raise ValueError(f"{directory}: expected {expected} ROMs, found {len(group)}")
        roms.extend(group)
    roms.extend(root / name for name in INDIVIDUAL_ROMS)
    trace_rom = root / "other/nestest.nes"
    trace_log = root / "other/nestest.log"
    mmc3_roms = [root / "mmc3_test_2/rom_singles" / name for name in MMC3_ROMS]
    for path in [trace_rom, trace_log, *roms, *mmc3_roms]:
        if not path.is_file():
            raise FileNotFoundError(path)

    trace = subprocess.run([str(binary), "--trace", str(trace_rom), str(trace_log)], timeout=120)
    if trace.returncode:
        return 1
    # The read-buffer ROM needs more than 1,200 frames to finish its DMA checks.
    diagnostics = subprocess.run(
        [str(binary), "--rom", "7200", *(str(path) for path in roms)], timeout=900
    )
    # These five ROMs assume writable result RAM without setting $A001.
    # The runner logs the explicit setup and uses ordinary cartridge writes.
    mapper_tests = subprocess.run(
        [str(binary), "--mmc3-rom", "1200", *(str(path) for path in mmc3_roms)], timeout=180
    )
    return 1 if diagnostics.returncode or mapper_tests.returncode else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="Path to the built accuracy-tests executable")
    parser.add_argument("rom_root", type=Path, help="Root of the nes-test-roms checkout")
    arguments = parser.parse_args()
    try:
        return run_checks(arguments.binary, arguments.rom_root)
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"Diagnostic run failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
