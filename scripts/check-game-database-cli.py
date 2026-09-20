#!/usr/bin/env python3
"""Check default database discovery through the production application."""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zlib


def launch(binary: Path, project: Path, data_dir: Path, image: Path,
           arguments: list[str]) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        [str(binary), "--data-dir", str(data_dir), *arguments,
         "--barcode", "12345678", str(image)],
        cwd=project, capture_output=True, text=True, encoding="utf-8",
        errors="replace", timeout=15,
    )
    combined = result.stdout + result.stderr
    if result.returncode != 1 or "Sanitizer" in combined or "runtime error:" in combined:
        raise RuntimeError(f"Unexpected application result for {arguments}:\n{combined}")
    return result


def check_loaded(result: subprocess.CompletedProcess[str], path: Path, outcome: str,
                 image_bytes: bytes, region: str, metadata: str) -> None:
    # A NROM cartridge cannot scan a Datach barcode. That checked error exits
    # after the real image loader but before an SDL window or audio device opens.
    expected = (
        f"Game database: {path}: {outcome}",
        f"Timing region: {region} (selection: auto)",
        f"Metadata source: {metadata}",
        f"File CRC32: {zlib.crc32(image_bytes):08X}",
    )
    if (any(text not in result.stdout for text in expected)
            or "Barcode input requires a Datach cartridge" not in result.stderr):
        raise RuntimeError(f"Missing expected loader output {expected}:\n{result.stdout}\n{result.stderr}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="Built production emulator")
    arguments = parser.parse_args()
    binary = arguments.binary.resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f"Emulator is not executable: {binary}")

    project = Path(__file__).resolve().parent.parent
    build = project / "build"
    build.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="database-cli-", dir=build) as directory:
        data_dir = Path(directory)
        fixture = data_dir / "legacy.nes"
        default_db = data_dir / "NesDB.txt"
        explicit_db = data_dir / "explicit.txt"
        header = bytearray(b"NES\x1a" + bytes(12))
        header[4] = 2
        header[5] = 1
        program = bytearray([0xEA]) * 0x8000
        program[0:3] = b"\x4c\x00\x80"
        program[-4:-2] = b"\x00\x80"
        payload = bytes(program) + bytes(0x2000)
        legacy = bytes(header) + payload
        fixture.write_bytes(legacy)
        crc = zlib.crc32(payload)

        def row(system: str) -> str:
            return f"{crc:08X},{system},TEST,,,0,b32768,b8192,0,8,0,0,h,8,N,,0,0\n"

        result = launch(binary, project, data_dir, fixture, [])
        check_loaded(result, default_db, "Optional default was not found", legacy, "NTSC", "iNES")
        print("PASS: absent optional default uses image metadata")

        default_db.write_text(row("NesPal"), encoding="utf-8")
        result = launch(binary, project, data_dir, fixture, [])
        check_loaded(result, default_db, "Loaded 1 entry", legacy, "PAL", "game database")
        if f"PRG+CHR CRC32: {crc:08X}" not in result.stdout:
            raise RuntimeError("Default lookup used the wrong cartridge CRC")
        print("PASS: default database corrects the production cartridge load")

        explicit_db.write_text(row("Dendy"), encoding="utf-8")
        result = launch(binary, project, data_dir, fixture, ["--game-db", str(explicit_db)])
        check_loaded(result, explicit_db, "Loaded 1 entry", legacy, "Dendy", "game database")
        print("PASS: explicit database takes precedence over the default")

        result = launch(binary, project, data_dir, fixture, ["--no-game-db-overrides"])
        check_loaded(result, default_db, "Loaded 1 entry", legacy, "NTSC", "iNES")
        print("PASS: disabled overrides preserve legacy image metadata")

        headerless = data_dir / "headerless.bin"
        headerless.write_bytes(payload)
        result = launch(binary, project, data_dir, headerless, ["--no-game-db-overrides"])
        check_loaded(result, default_db, "Loaded 1 entry", payload, "PAL", "game database (headerless)")
        print("PASS: headerless lookup remains available with overrides disabled")

        header[7] = 8
        header[15] = 1
        nes20 = bytes(header) + payload
        fixture.write_bytes(nes20)
        result = launch(binary, project, data_dir, fixture, [])
        check_loaded(result, default_db, "Loaded 1 entry", nes20, "NTSC", "NES 2.0")
        print("PASS: NES 2.0 metadata takes precedence over database corrections")
        fixture.write_bytes(legacy)

        for label, selected, options in (
            ("malformed default", default_db, []),
            ("malformed explicit", explicit_db, ["--game-db", str(explicit_db)]),
        ):
            selected.write_text("invalid,row\n", encoding="utf-8")
            result = launch(binary, project, data_dir, fixture, options)
            if (f"Game database: {selected}: Could not parse the game database" not in result.stderr
                    or "Loading ROM:" in result.stdout or "Mapper:" in result.stdout):
                raise RuntimeError(f"{label}:\n{result.stdout}\n{result.stderr}")
            print(f"PASS: {label} is rejected before loading an image")
            selected.write_text(row("NesPal"), encoding="utf-8")

        missing = data_dir / "missing.txt"
        result = launch(binary, project, data_dir, fixture, ["--game-db", str(missing)])
        if (f"Game database: {missing}: File not found" not in result.stderr
                or "Loading ROM:" in result.stdout or "Mapper:" in result.stdout):
            raise RuntimeError(f"Missing explicit path:\n{result.stdout}\n{result.stderr}")
        print("PASS: a missing explicit database never falls back silently")

        default_db.write_text("invalid,row\n", encoding="utf-8")
        explicit_db.write_text(row("Dendy"), encoding="utf-8")
        result = launch(binary, project, data_dir, fixture, ["--game-db", str(explicit_db)])
        check_loaded(result, explicit_db, "Loaded 1 entry", legacy, "Dendy", "game database")
        print("PASS: an explicit database does not consult a malformed default")

    print("Game database CLI: 10 cases passed")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"Game database CLI check failed: {error}", file=sys.stderr)
        sys.exit(1)
