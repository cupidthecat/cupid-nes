# Cartridge and media checkpoints

[Documentation index](README.md) | [Earlier checkpoints](accuracy-checkpoints.md) | [Hardware guide](hardware.md)

The records below identify implementation and integration commits for the cartridge, media, and device accuracy work in PR #123. Each listed checkpoint passed the production hardware suite and AccuracyCoin 144/144, with zero skipped or unfinished tests and the cartridge's own tally at 144/144. The build column records the configuration actually tested at that commit. A later combined sanitizer run does not retroactively validate an earlier commit under sanitizers.

AccuracyCoin uses revision `9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e` and ROM SHA-256 `7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c`. These runs completed 4,182 frames. The ROM, result protocol, and expected tally were retained throughout the changes.

## Issue checkpoints

Normal Windows builds use strict Clang warnings and optimization. Sanitizer builds use AddressSanitizer and UndefinedBehaviorSanitizer with the same production sources. "Both" means that separate normal and sanitizer runs passed at the named commit. The final Linux CI also checks GCC, Clang, and leak detection.

| Issue | Implemented behavior | Tested commit | Build |
| --- | --- | --- | --- |
| #77 | Native cartridge page geometry and loader safety | `0ffadbc9fb3c5f71fc6f25c4f367127a9489bf41` | Sanitizers |
| #78 | Independent cartridge RAM and persistent storage | `8478d5be8c5df163d8f278ae614785a31eed79c4` | Sanitizers |
| #79 | Power-on RAM profiles and independent startup VBL selection | `58d832c9ea0ed51fe570ffa6f7d1c7444cb6118b` | Normal |
| #80 | PPU reset suppression | `647329915cfc0e1596e00c85471124979ee93086` | Normal |
| #81 | Audio reconstruction from timed APU transitions | `33dc379c59b12455285ff51861b06caa372c501d` | Sanitizers |
| #82 | NTSC composite signal reconstruction | `d3119860edb5dd705aee2c45da2d2d446738b568` | Sanitizers |
| #83 | Four-frame VS coin-input pulses | `55876db4d0d73d88fbe8b22cf87fd458e6a497e7` | Normal |
| #84 | Database defaults, legacy corrections, and headerless cartridges | `9406f912110a3421b3251016e2e79f76a65a4e37` | Both |
| #85 | Controller selection from cartridge metadata | `d8d4377256afb7a524b3d95b0c6d4622d67cf8e6` | Normal |
| #86 | Named UNIF boards, metadata, 8237A banking, and FamicomBox CPU RAM | `0b452a173fea7252eb3948eebecbae213a8cb2a9` | Both |
| #87 | Bandai 70/152 banking and mirroring | `adb39877c2b30b0172d09d829d4440c6c2c99050` | Sanitizers |
| #88 | Golden Five mapper 104 | `6fd2ab7d1d48e9e8b7b019fb562e3fe5c291a316` | Normal |
| #89 | Farid mappers 323/324 | `b6fbcca7239c165744a9b982fee773cce5ae3ccc` | Normal |
| #90 | Front Fareast mappers 6/8/17 | `a6c2e254aaf176bd23925b867055e98ce7d13aa8` | Normal |
| #91 | Jaleco JF-13 mapper 86 | `f7f60ebac6fcaf474f0672769b88f6f8d86fe1bc` | Normal |
| #92 | JY mappers 35/91 | `0dc0045028423100d9ce66ff1daa290a90121812` | Normal |
| #93 | Kaiser cartridge families | `b74e0fa632aab3f66890dceb8a28770c421dcde6` | Normal |
| #94 | Magic Floor mapper 218 | `961b1abb734f68bf7d8290756264bf4e2ee5b04a` | Normal |
| #95 | MMC3-derived boards 12 through 123 | `c8a1010d5be99292c18a73b9c76f6abf2d37954c` | Normal |
| #96 | MMC3-derived boards 126 through 215 | `5c7d73d7e36460a3317d5a80b475a328a2710cb5` | Normal |
| #97 | MMC3-derived boards 217 through 262 | `4d54fb9a85470ed60cf757411dd6491cdb6f85f9` | Normal |
| #98 | MMC3-derived boards 263 through 366 | `89729fc139962e45e9e372192cd272fde0357757` | Normal |
| #99 | Mapper 31 cartridge bank switching | `cfd3b5f5a0520068c0bb8b52588a1b47ebb7c89d` | Normal |
| #100 | NTDEC boards and protection reads | `ca243bd371cf5a39e462e5737cfd013ab7bbfe04` | Normal |
| #101 | Racermate mapper 168 | `a6ea0c7e0fd62c0392faca325fe77d4ad768c1a4` | Normal |
| #102 | Rainbow mapper 682 | `9f8770cd1652d6613481542659c39ddb11afb02b` | Both |
| #103 | Sachen discrete boards | `131b6b84d7c313b4bf72b39a9a54d4fc8b42398f` | Normal |
| #104 | Sachen mappers 243/513 | `1e7700f9a26c33bf6906718af9aee5525461c3ab` | Normal |
| #105 | Sealie Computing mapper 29 | `03bc295fd5a2287c94b743c44db1d070a1495a02` | Normal |
| #106 | Taito X1-017 mapper 552 | `3d3219cb2f6698af6a3dfa10f4df70367b5d44e6` | Sanitizers |
| #107 | TXC cartridge registers, banking, and protection | `a0b2610d710d4807e8828980249a8df8ddfa3c8a` | Both |
| #108 | Drip Game mapper 284 | `26b01e3a4a03affd59289f86efbfe5aa7fa73a3f` | Normal |
| #109 | Discrete boards 38 through 59 | `587897fce2e33f85314037cffa13baa51198b75e` | Normal |
| #110 | Unlicensed boards 60 through 163, including shared IRQ behavior | `f77ad00679a7adb0e724c09ff73273579190f207` | Both |
| #111 | Discrete boards 166 through 212 | `1fc321133c2984a8cc853805ddf3bb38e7118d0c` | Both |
| #112 | Discrete boards 213 through 233 | `c11053c276d20ea8afd7a92ae4002d577ffbb507` | Both |
| #113 | Discrete boards 234 through 266 | `75825eb1eb5529d6429bfe612682c4285ba235f3` | Both |
| #114 | Discrete boards 274 through 329 | `ebe9d28fec0540d63d610dc181c39fe5a5a568e0` | Both |
| #115 | Discrete boards 331 through 530 | `78320e20a3fa7b91cef53b6e1a231ea77567a1f6` | Both |
| #116 | Waixing cartridge families | `d5d97adff2e73aaecfc82513640a32c25823b18e` | Both |
| #117 | Whirlwind boards and CPU-driven IRQ tests | `1233feba3a004bcf9cb5014a6a04fb525af9aac3` | Both |
| #118 | NSF/NSFe execution, PPU timing, expansion combinations, metadata, and frontend track controls | `b3e6462c89da3e0c8c147e9aa5c5c238f62465a0` | Both |
| #119 | Famicom Network System board, character ROM, and controller | `a587de263b7a7003e13685ed6784e2eb951e0f2c` | Both |
| #120 | StudyBox media, tape transport, and audio | `9cfce84e45a8daf7d60e5279cfbe164f73518e0b` | Both |
| #121 | SNES controllers and mice, NTT Data keypad, and Virtual Boy controller | `f695c7c63ac7ea80756ea69b2f963539cb8e4264` | Both |
| #122 | Karaoke mapper 188 and application microphone controls | `ceb7f61e75882aba6e9af4f63d16612915babcaf` | Both |

The issue numbers identify specific board lists; a numeric range in this table does not imply support for every intervening mapper. The [hardware table](hardware.md#cartridge-mappers) lists the actual variants.

## Memory and review checkpoints

The geometry work continued after the initial #77 and #78 commits. These later tests exercise physical source selection, bank writes, uncovered addresses, trainer placement, persistence, and failed replacement through the production CPU and PPU paths.

| Correction | Tested commit | Build |
| --- | --- | --- |
| Nintendo page geometry | `1be12b1b67e23dae7e31a73f7b2e6babd7ec644d` | Normal |
| Native 8 KiB CHR-window geometry | `c8267efa48602fcbc0afbe8ef4d58adf5041d5a2` | Normal |
| Small native CHR images | `6c5f05efc496c12c90cb19548a7a730dc979c2f1` | Normal |
| Native 1 KiB page geometry | `2d032a803d86760b349228f482e7cdedbc3b4a16` | Normal |
| Native mappings with separate memory sources | `a9264791d4e662285b2bc1736bc6bc61a65fca92` | Normal |
| MMC3 CHR ROM/RAM coverage and mapping replacement | `1842fa552a81c1f44da1ad1199ffa96fdeff80bb` | Both |
| Namco memory-source ownership | `c0c4ff92c122a873d27a9a4036ff8cdd22a7a721` | Both |
| Separate work/save RAM and trainer precedence | `01e26b818a1bb4735a6944e96e18890b1c931e75` | Both |
| Declared CHR storage beside CHR ROM | `8d58a5b07c564546d9c98bfe2349c255b412eb42` | Both |
| Reduced PRG images on mappers 94 and 180 | `252b5ed8370f343cc4759a9a18fe6b285537c91f` | Both |
| MMC5 extended pattern-address masking | `c6e94bdfa0a214e69d3030842314fc015f1cd085` | Both |
| Complete native 8 KiB PRG pages below the former loader minimum | `72a805464f8ba2eb90319d9e7a6a881432a52fa5` | Both |
| Reduced NROM, MMC1, CNROM, and MMC5 PRG pages with uncovered CPU addresses | `ee8ad005d4d51e6ffede8ddf2630802fcceb8ca0` | Both |
| All eight FamicomBox DIP inputs, register aliases, and reset behavior | `1627ef5539650e890434f789e524754649ff861b` | Both |
| Independent RAM in fixed cartridge windows, complete-page coverage, and save-file preservation | `3103dbb4d17eedf7fc62ae1551b41b98fc655f3b` | Both |
| Remaining native PRG/CHR page geometry and source-specific reads | `1cbaa21814b95539fce8abaf09025a4269141064` | Both |
| Irem 77 CHR source ownership, RAM-only aliases, and declared storage | `0622f41ba8e4c4474bc0cf1b335de80450eed2ba` | Both |
| Default PRG-RAM windows with independent chips and register-owned writes | `4321e1bc4e0abb2577025ef37fc272b9f1fbd319` | Both |
| Declared CHR chips beyond fixed bank windows and unreachable save bytes | `6185fe54a45e8e949c945252e29ae2233eab1092` | Both |
| CHR ROM startup visibility before bank selection | `1f6fa07f8268aea92a68d5f91b2e9d6ed7d4d43f` | Both |
| Native CHR sources, protected RAM writes, and UNROM 512 nametable storage | `940e66c9fbabfd6fb1cabf79aae4f9ee54bd2f89` | Both |
| Small Action 53 and Oeka Kids CHR ROMs with CPU-driven PPU latches | `e3fe8675faa458b739c9f205c8234f88cddb74f9` | Both |
| Mixed-CHR MMC3 source selection, explicit RAM sizes, IRQs, and persistence | `3193b5d3d24dca8420273940e21e2c3f8ebfc285` | Both |
| Taito work/save RAM selection, declared sizes, and permission windows | `8c7d5bf92ec9e60611fda4c2bfbfcfb427d4fc84` | Both |
| NINA-001/BNROM and FME-7 memory routing, register writes, and retained state | `cc6991c04e1b6137156babbcf308d127ca719906` | Both |
| UNROM 512/GTROM CPU RAM and independent PRG-RAM/flash save files | `1a19f5817d89bd173bcb81684419ebfd5a022cfe` | Both |
| CPU reset clears the latched mapper IRQ while retained counters can raise a new interrupt during reset | `e61606ba859a97a5d2669b0c4037e834687763ce` | Both |

Passing AccuracyCoin does not establish the correctness of every cartridge. Focused tests cover bank selection, CPU instructions that access registers and RAM, PPU reads and writes, IRQ timing, open bus, reset, audio output, and storage round trips. NSF tests also check that cartridge rendering and APU IRQ behavior return after music playback ends.

## Reproducing the checks

Use a clean worktree at the listed commit. Record `git rev-parse HEAD`, verify the pinned ROM hash, and run the [normal and sanitizer commands](development.md). Both the application and hardware runner must build; the runner must report `Hardware regressions: PASS (0 failures)` before the AccuracyCoin invocation.

The final combined revision also requires the 8,991-state CPU trace, all 91 pinned diagnostic ROMs, and both hosted compiler jobs. See [the workflow](../.github/workflows/accuracy.yml) for the exact compiler flags, test pins, and failure checks. Historical results remain attached to their named commits; consult the PR checks for a later revision.

## Combined validation

Commit `e61606ba859a97a5d2669b0c4037e834687763ce` includes all issue checkpoints above and the subsequent memory-source, save-ownership, and CPU-reset fixes. Its strict Windows normal and sanitizer builds both pass the production hardware suite and AccuracyCoin 144/144 with zero skipped or unfinished tests and a matching cartridge tally. The local records are `build/checks/irq-reset-integrated-result.json` and `build/checks/irq-reset-integrated-sanitized-result.json`, with their associated build and cartridge logs.

Both local builds also match all 8,991 canonical CPU states and pass all 91 diagnostic ROMs. Those results are recorded in `build/checks/final-cartridge-normal-diagnostics-result.json` and `build/checks/final-cartridge-sanitized-diagnostics-result.json`, with their corresponding diagnostic logs.

The [push workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35441690837) and [pull-request workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35441692889) also pass at that revision. Both the strict GCC job and Clang sanitizer job build the application, run the hardware suite, match all 8,991 canonical CPU states, pass all 91 pinned diagnostic ROMs, and finish AccuracyCoin at 144/144 with no skipped or unfinished tests. The Linux sanitizer job enables leak detection as well as address and undefined-behavior checks.

These records identify the revision actually tested. Subsequent geometry or documentation changes receive their own PR checks; a historical green result does not establish a later revision's result.
