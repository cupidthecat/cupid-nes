#!/usr/bin/env python3
"""Check region selection through the production command-line loader."""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def check_launch(binary: Path, project: Path, fixture: Path) -> None:
    cases = (
        ([], "PAL", "auto", "nes-001"),
        (["--region", "auto"], "PAL", "auto", "nes-001"),
        (["--region", "ntsc"], "NTSC", "ntsc", "nes-001"),
        (["--region", "pal"], "PAL", "pal", "nes-001"),
        (["--region", "dendy"], "Dendy", "dendy", "nes-001"),
        (["--console", "famicom"], "PAL", "auto", "famicom"),
        (["--region", "dendy", "--console", "famicom"], "Dendy", "dendy", "famicom"),
        (["--region", "ntsc", "--region", "pal"], "PAL", "pal", "nes-001"),
    )
    for arguments, region, mode, console in cases:
        # The NROM fixture cannot scan a Datach barcode. That checked error path
        # unloads the cartridge after timing selection and before SDL starts.
        result = subprocess.run(
            [str(binary), *arguments, "--barcode", "12345678", str(fixture)],
            cwd=project, capture_output=True, text=True, timeout=15,
        )
        expected = f"Timing region: {region} (selection: {mode})"
        if (result.returncode != 1 or expected not in result.stdout
                or f"Console: {console}" not in result.stdout
                or "Barcode input requires a Datach cartridge" not in result.stderr
                or "Sanitizer" in result.stderr or "runtime error:" in result.stderr):
            raise RuntimeError(f"{arguments}:\n{result.stdout}\n{result.stderr}")
        print(f"PASS: {' '.join(arguments) or 'default'} -> {region}, {console}")

    for arguments in (["--region"], ["--region", "invalid"], ["--region", "PAL"]):
        result = subprocess.run(
            [str(binary), *arguments], cwd=project,
            capture_output=True, text=True, timeout=15,
        )
        if (result.returncode != 1
                or "Region must be auto, ntsc, pal, or dendy" not in result.stderr
                or "Loading ROM:" in result.stdout
                or "Sanitizer" in result.stderr or "runtime error:" in result.stderr):
            raise RuntimeError(f"{arguments}:\n{result.stdout}\n{result.stderr}")
        print(f"PASS: rejected {' '.join(arguments)}")


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
    with tempfile.TemporaryDirectory(prefix="region-cli-", dir=build) as directory:
        fixture = Path(directory) / "region.nes"
        header = bytearray(b"NES\x1a" + bytes(12))
        header[4] = 1
        header[5] = 1
        header[7] = 8
        header[12] = 1  # PAL NES 2.0 image.
        program = bytearray([0xEA]) * 0x4000
        program[0:3] = b"\x4c\x00\x80"
        program[-4:-2] = b"\x00\x80"
        fixture.write_bytes(header + program + bytes(0x2000))
        check_launch(binary, project, fixture)
    print("Region CLI: 11 cases passed")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"Region CLI check failed: {error}", file=sys.stderr)
        sys.exit(1)
