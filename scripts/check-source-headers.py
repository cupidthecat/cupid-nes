#!/usr/bin/env python3
# check-source-headers.py - Check project attribution and license headers.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check owned C/C++ files without rewriting imported copyright notices."""

from pathlib import Path
import sys


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    failures = []
    count = 0
    for directory in ("src", "include"):
        for path in sorted((root / directory).rglob("*")):
            if path.suffix not in {".c", ".h", ".cc", ".hh", ".cpp", ".hpp"}:
                continue
            relative = path.relative_to(root)
            if "third_party" in relative.parts or path.name in {"emu2413.c", "emu2413.h"}:
                continue
            text = path.read_text(encoding="utf-8-sig")
            header = text.split("*/", 1)[0]
            # Adapted board files retain the component's author and license.
            credited = "Author:" in header or "Copyright" in header
            licensed = "GPL-3.0-or-later" in header or "GNU General Public License" in header
            if not text.startswith("/*") or not credited or not licensed:
                failures.append(str(relative))
            count += 1
    if failures:
        for path in failures:
            print(f"{path}: missing attribution or license in opening header", file=sys.stderr)
        return 1
    print(f"Source headers: PASS ({count} project files; imported notices preserved)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
