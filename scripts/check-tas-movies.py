#!/usr/bin/env python3
"""Compare full movie playback with a supplied FCEUX reference executable.

Inputs and executables are hashed before execution. No games or movie files are
downloaded. Success requires every internal RAM byte and lag result to match on
every frame, plus the final palette image; CPU endpoint differences are reported
separately because the two cores can stop on different instruction boundaries.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


def digest(path: Path) -> dict[str, Any]:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            checksum.update(block)
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": checksum.hexdigest()}


def run(command: list[str], log: Path, environment: dict[str, str], timeout: float) -> dict[str, Any]:
    started = time.monotonic()
    with log.open("wb") as stream:
        try:
            process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                     env=environment, timeout=timeout, check=False)
            code = process.returncode
        except subprocess.TimeoutExpired:
            stream.write(b"\nValidation process exceeded the time limit.\n")
            code = 124
    return {"command": command, "exit_code": code,
            "elapsed_seconds": time.monotonic() - started, "log": digest(log)}


def suffix(prefix: Path, extension: str) -> Path:
    return Path(str(prefix) + extension)


def compare(reference: Path, candidate: Path, expected_frames: int) -> dict[str, Any]:
    result: dict[str, Any] = {"expected_frames": expected_frames, "frames": 0,
                              "ram_mismatch_frames": 0, "lag_mismatch_frames": 0,
                              "pc_mismatch_frames": 0, "first_ram_difference": None,
                              "first_lag_difference": None, "first_pc_differences": []}
    with suffix(reference, ".csv").open(newline="") as ref_csv, \
            suffix(candidate, ".csv").open(newline="") as test_csv, \
            suffix(reference, ".ram").open("rb") as ref_ram, \
            suffix(candidate, ".ram").open("rb") as test_ram:
        ref_rows, test_rows = csv.DictReader(ref_csv), csv.DictReader(test_csv)
        previous = [0, 0]
        for frame in range(1, expected_frames + 1):
            left, right = next(ref_rows, None), next(test_rows, None)
            if left is None or right is None:
                raise ValueError(f"A trace ends before frame {frame}")
            for index, row in enumerate((left, right)):
                if int(row["frame"]) != frame or int(row["lagged"]) not in (0, 1):
                    raise ValueError(f"Invalid cursor or lag flag at frame {frame}")
                lag = int(row["lag_count"])
                if lag != previous[index] + int(row["lagged"]):
                    raise ValueError(f"Invalid cumulative lag at frame {frame}")
                previous[index] = lag
            if (left["lagged"], left["lag_count"]) != (right["lagged"], right["lag_count"]):
                result["lag_mismatch_frames"] += 1
                if result["first_lag_difference"] is None:
                    result["first_lag_difference"] = frame
            if left["pc"] != right["pc"]:
                result["pc_mismatch_frames"] += 1
                if len(result["first_pc_differences"]) < 16:
                    result["first_pc_differences"].append({"frame": frame,
                        "reference": int(left["pc"]), "candidate": int(right["pc"])})
            expected, actual = ref_ram.read(2048), test_ram.read(2048)
            if len(expected) != 2048 or len(actual) != 2048:
                raise ValueError(f"Truncated RAM trace at frame {frame}")
            if expected != actual:
                result["ram_mismatch_frames"] += 1
                if result["first_ram_difference"] is None:
                    address = next(i for i, values in enumerate(zip(expected, actual)) if values[0] != values[1])
                    result["first_ram_difference"] = {"frame": frame, "address": address,
                                                       "reference": expected[address], "candidate": actual[address]}
            result["frames"] = frame
        if next(ref_rows, None) is not None or next(test_rows, None) is not None or ref_ram.read(1) or test_ram.read(1):
            raise ValueError("Trace has extra rows or RAM bytes after the final frame")
    expected = suffix(reference, ".pixels").read_bytes()
    actual = suffix(candidate, ".pixels").read_bytes()
    if len(expected) != 256 * 240 or len(actual) != 256 * 240:
        raise ValueError("Final palette images must contain 256 by 240 pixels")
    result["final_palette_mismatch_pixels"] = sum((a & 63) != (b & 63) for a, b in zip(expected, actual))
    result["reference_lag_count"], result["candidate_lag_count"] = previous
    result["pass"] = not (result["ram_mismatch_frames"] or result["lag_mismatch_frames"]
                            or result["final_palette_mismatch_pixels"])
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--rom", type=Path, required=True)
    parser.add_argument("--movie", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="A new output directory")
    parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    script = Path(__file__).with_name("trace-fceux-movie.lua").resolve(strict=True)
    inputs = {name: getattr(args, name).resolve(strict=True)
              for name in ("runner", "reference", "rom", "movie")}
    if any(not path.is_file() for path in inputs.values()):
        parser.error("All input paths must be files")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    reference, candidate = output / "reference", output / "candidate"
    environment.update({"QT_QPA_PLATFORM": "offscreen", "SDL_AUDIODRIVER": "dummy",
                        "CUPID_REFERENCE_MOVIE": str(inputs["movie"]),
                        "CUPID_REFERENCE_PREFIX": str(reference)})
    report: dict[str, Any] = {"inputs": {name: digest(path) for name, path in inputs.items()},
                              "reference_script": digest(script), "pass": False}
    code = 1
    try:
        report["reference_run"] = run([str(inputs["reference"]), "--no-config", "1", "--sound", "0",
                                        "--loadlua", str(script), str(inputs["rom"])],
                                       output / "reference.log", environment, args.timeout)
        if report["reference_run"]["exit_code"] != 0:
            raise ValueError("Reference execution failed; inspect reference.log")
        frames = int(suffix(reference, ".result").read_text().strip())
        if frames < 1:
            raise ValueError("A playback acceptance movie must contain at least one frame")
        report["candidate_run"] = run([str(inputs["runner"]), "--movie-trace", "0", str(inputs["rom"]),
                                       str(inputs["movie"]), str(candidate)],
                                      output / "candidate.log", environment, args.timeout)
        if report["candidate_run"]["exit_code"] != 0:
            raise ValueError("Candidate playback/restoration failed; inspect candidate.log")
        report["comparison"] = compare(reference, candidate, frames)
        for name, path in inputs.items():
            if report["inputs"][name] != digest(path):
                raise ValueError(f"Input changed during validation: {name}")
        if report["reference_script"] != digest(script):
            raise ValueError("Reference Lua script changed during validation")
        report["pass"] = report["comparison"]["pass"]
        code = 0 if report["pass"] else 1
    except (OSError, ValueError, KeyError) as error:
        report["error"] = str(error)
    report["artifacts"] = [digest(path) for path in sorted(output.iterdir()) if path.is_file()]
    destination = output / "report.json"
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(destination), "pass": report["pass"],
                       "comparison": report.get("comparison"), "error": report.get("error")}, indent=2))
    return code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f"TAS validation failed: {error}", file=sys.stderr)
        sys.exit(1)
