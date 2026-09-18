---
name: Hardware accuracy
about: Describe a missing NES device or a specific hardware behavior discrepancy
title: ''
---

## Hardware and behavior

Identify the device, mapper/submapper, console or chip revision, and expected behavior. Link primary hardware documentation or provide a measured access sequence. Mark hypotheses as hypotheses.

## Current implementation

Include the tested Cupid commit, relevant source paths, and the behavior observed. State whether the device is rejected as unsupported or accepted with incorrect behavior.

## Minimal reproduction

Describe the CPU/PPU register sequence, bus timing, memory layout, or image metadata needed to trigger the difference. Include a synthetic or redistributable fixture where possible.

## Acceptance checks

Describe a focused regression that distinguishes the correct behavior, including boundary, reset, loader, or persistence cases when relevant. Explain how the hardware is reached through normal application execution.

The implementation must retain the existing hardware suite, canonical CPU trace, all 91 pinned diagnostic ROMs, and AccuracyCoin 144/144 with zero skipped or unfinished results. Record the tested commit and final GCC/Clang sanitizer CI results when submitting the fix.
