# Contributing to Cupid

Cupid development covers NES hardware accuracy, emulator features, and the desktop tools used to run and inspect games. Start with the [architecture guide](docs/architecture.md) to locate the relevant subsystem, and [development and testing](docs/development.md) to build the production test runner.

## Reporting a bug

Search the existing issues and open pull requests before creating a report. Include enough information to repeat the failure:

- The commit or build, operating system, compiler when relevant, and complete launch command.
- The image's SHA-256, mapper/submapper and region where known, and any header modifications.
- Steps from launch or reset to the first failure, with expected and observed behavior.
- A useful log, assertion, screenshot, or minimal test fixture. For a regression, include the last known passing commit when available.

Use the bug-report template for crashes, incorrect output, loading failures, or build errors. Use the hardware-accuracy template for a missing device or a specific timing/register discrepancy. Distinguish a verified hardware observation from a hypothesis about the cause.

Provide synthetic or redistributable fixtures when possible. A game name and hash can identify an affected image without uploading commercial ROM or BIOS data. Save problems should be reproduced with a copy of the media and its save files.

## Preparing a patch

Inspect the current implementation, its callers, and relevant tests before changing it. Work on a branch or isolated worktree, keep unrelated edits intact, and make commits that describe the behavior changed.

Fix the device, bus, or application behavior that causes the failure. Do not add per-game success paths, replace actual output with an expected result, or silently select a different board when metadata is unsupported. Preserve loader validation, save ownership, reset distinctions, and existing copyright notices.

Keep the hardware change and the evidence for it together. A new helper, declaration, or test-only path does not implement a device in the application. When a feature needs normal frontend access, verify that inputs and outputs can reach it, as with both screens and both APUs of a dual VS system.

A new C or C++ source file must be included in both [Makefile](Makefile) and [the Windows build script](scripts/test-windows.ps1). The C core is built as C11, while the EPSM wrapper and bundled ymfm sources are built as C++17. Use the surrounding source/header organization and language style. Run a strict build before treating a code patch as ready for review.

Project-owned source files should stay at or below 1,500 lines. Split a file at subsystem or test-group boundaries before it grows past that limit. Private implementation headers keep helpers and state in one translation unit without turning them into public interfaces. Imported components keep their upstream layout and formatting markers. Run `python scripts/check-source-size.py` to check the limit locally; CI runs the same command.

The root `.clang-format` keeps function definitions in separate blocks, expands short functions and control statements, and adds braces to unbraced control statements. Keep one blank line between global declaration groups, including adjacent scalar and array definitions. Leave one blank line after a completed `if`, loop, or `switch` block when the next statement begins a separate step. ClangFormat preserves those lines but does not add them in every case, so check them during review.

## Regression coverage

Choose a test that would fail for the original defect and exercise the real implementation. Use production loader tests for metadata and replacement behavior, CPU bus operations for timing and DMA, and save/reload tests for persistent memory. Include boundary and rejected-input cases when they are part of the change.

After each implemented issue, require the pinned AccuracyCoin suite to pass **144/144 with zero failed, skipped, or unfinished tests**. This also applies to frontend and presentation changes, which must preserve the hardware behavior beneath them. Record the exact tested commit and command. Later integration changes must retain that result.

Before an implementation patch is ready for review, the final revision needs the production hardware suite, canonical 8,991-state trace, all 91 pinned diagnostic ROMs, full AccuracyCoin, and the strict GCC and Clang-with-sanitizers CI jobs. Add acceptance checks for the behavior the issue requests: a hardware pass alone does not prove that a menu works, a recording can be replayed, or a network peer can connect. Preserve ROM revisions, hashes, pass thresholds, result protocols, and explicit setup for legacy diagnostics. The diagnostic script checks files and group counts, not the external checkout's Git revision, and the AccuracyCoin runner does not verify the ROM hash; those pins must be checked separately as shown in the test guide and CI workflow.

Report a failed or unfinished check accurately. Do not reduce coverage to make CI pass. The [test guide](docs/development.md) contains commands and the [accuracy notes](docs/accuracy.md) explain what each result establishes.

## Documentation changes

Write about behavior present in the code. Use exact option names, defaults, paths, and examples. Distinguish application options from diagnostic modes, and identify limitations beside the relevant feature.

Keep common setup and navigation in the README, practical tasks in the user guides, and device timing details in the accuracy notes. Add new pages to the [documentation index](docs/README.md) and link them from the relevant existing page. Check relative links, heading anchors, code fences, and test pins.

A prose edit does not need a C unit test. Validate the facts against source and check any changed commands. The existing hardware CI still runs on documentation pushes. Historical checkpoint records identify actual runs; do not replace their commit hashes with an untested revision.

## Pull requests

Explain the concrete problem, the resulting behavior, and the validation performed. Link related issues and include the tested commit. For a timing change, describe the triggering access sequence or boundary. For a save change, describe replacement and failure behavior.

Use the pull-request template to separate completed checks from checks that were not run. Keep known limitations visible. Review the final diff for unrelated files, generated binaries, temporary saves, and personal paths before pushing.

Related changes already under review should be added to their existing pull request when that is the agreed scope. New work should have a reviewable scope. Passing CI supplies test evidence; merging remains a separate maintainer decision.

The project license is [GPL-3.0-or-later](LICENSE). Retain component notices in existing source files, including the [emu2413 MIT license](src/rom/emu2413.LICENSE), and identify any new dependency or imported code in the review. The [credits page](docs/credits.md) links component and diagnostic sources.
