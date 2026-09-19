#!/usr/bin/env python3
from pathlib import Path
import sys


MAX_LINES = 1500
SOURCE_SUFFIXES = {".c", ".h", ".cc", ".hh", ".cpp", ".hpp"}
IMPORTED_FILES = {Path("src/rom/emu2413.c")}
IMPORTED_DIRECTORIES = {Path("src/third_party")}


def is_imported(path: Path) -> bool:
    return path in IMPORTED_FILES or any(directory in path.parents for directory in IMPORTED_DIRECTORIES)


def source_files(root: Path):
    for path in (root / "src").rglob("*"):
        relative = path.relative_to(root)
        if path.is_file() and path.suffix in SOURCE_SUFFIXES and not is_imported(relative):
            yield relative


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    oversized = []
    for relative in source_files(root):
        line_count = len((root / relative).read_bytes().splitlines())
        if line_count > MAX_LINES:
            oversized.append((relative, line_count))

    if oversized:
        for path, line_count in sorted(oversized):
            print(f"{path}: {line_count} lines (limit {MAX_LINES})", file=sys.stderr)
        return 1

    print(f"Source size: PASS (all project-owned files are at most {MAX_LINES} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
