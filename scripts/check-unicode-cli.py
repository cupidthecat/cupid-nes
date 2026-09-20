#!/usr/bin/env python3
"""Check production Windows launches with UTF-8 data and image paths."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


def make_nrom(path: Path) -> None:
    header = bytearray(b"NES\x1a" + bytes(12))
    header[4] = 1
    header[5] = 1
    header[7] = 8
    program = bytearray([0xEA]) * 0x4000
    program[0:3] = b"\x4c\x00\x80"
    program[-6:-4] = b"\x00\x80"
    program[-4:-2] = b"\x00\x80"
    program[-2:] = b"\x00\x80"
    path.write_bytes(header + program + bytes(0x2000))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="Built production emulator")
    arguments = parser.parse_args()
    binary = arguments.binary.resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f"Emulator is not executable: {binary}")

    project = Path(__file__).resolve().parent.parent
    root = project / "build" / "unicode-cli-é-日本-🎮"
    if root.exists():
        shutil.rmtree(root)
    data_dir = root / "設定-é-🎮"
    rom_dir = root / "ゲーム-é-🎮"
    data_dir.mkdir(parents=True)
    rom_dir.mkdir(parents=True)
    fixture = rom_dir / "テスト-é-🎮.nes"
    make_nrom(fixture)

    try:
        result = subprocess.run(
            [str(binary), "--data-dir", str(data_dir), "--barcode", "12345678", str(fixture)],
            cwd=project, capture_output=True, text=True, encoding="utf-8", errors="strict",
            timeout=15,
        )
        expected_load = f"Loading ROM: {fixture}"
        if (result.returncode != 1
                or expected_load not in result.stdout
                or "Barcode input requires a Datach cartridge" not in result.stderr
                or "Sanitizer" in result.stderr or "runtime error:" in result.stderr):
            raise RuntimeError(f"{result.stdout}\n{result.stderr}")
    finally:
        shutil.rmtree(root, ignore_errors=True)

    print("Unicode CLI: UTF-8 data-dir and ROM path passed")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.TimeoutExpired, UnicodeError) as error:
        print(f"Unicode CLI check failed: {error}", file=sys.stderr)
        sys.exit(1)
