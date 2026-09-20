# Accuracy implementation checkpoints

[Documentation index](README.md)

The [hardware revision and CHR checkpoints](#hardware-revision-and-chr-checkpoints) record issues #125 through #130 and #139. The [partial CHR window](#partial-chr-window-checkpoint) and [regional timing](#regional-timing-checkpoint) checkpoints record the remaining #139 cases and #140. Earlier cartridge and media work is recorded in [cartridge and media checkpoints](cartridge-checkpoints.md). The earlier core implementation and validation history remains below.

Each issue or review checkpoint below passed the production hardware regressions and the full pinned AccuracyCoin cartridge: 144/144 passed, zero skipped, and zero unfinished. These records identify the commits tested after integration. Later fixes require their own checks, and the final pull-request commit must pass the complete CI workflow.

AccuracyCoin is pinned to revision `9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e`. The ROM SHA-256 is `7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c`. Each recorded run completed 4,182 frames and agreed with the cartridge's own 144/144 tally. The result checks and ROM pins were retained throughout the work.

| Issue | Implemented behavior | Tested commit |
| --- | --- | --- |
| #5 | Frame IRQ acknowledgment | `244b9e67ee6e8e6cc94e20bcb777198444e82617` |
| #6 | Color Dreams bus conflicts | `466604784efb6ff28b226704ac0be3709b7e2dfd` |
| #7 | Dot-257 PPU scroll writes | `3a4d5ee1554bb80cc0bfe3428aac9a9253295773` |
| #8 | OAM row corruption profiles | `b011026a30e9d77fcc3da6d55e1a7f6f983b13ca` |
| #9 | PPU startup write restriction | `7dd8e35cd45f759e37efee23615c8f9ef8bdb234` |
| #10 | Later-CPU DMC reload collisions | `83f78a6cf8bd809e3d352bc0c41d40086c1d0d7a` |
| #11 | OAM row decay and refresh | `99af1554d7ba0983b57a27ad854ac473941b6d91` |
| #12 | NES and Famicom controller-port wiring | `dfe61d85f38a08f20aca557bf81971e333be62fb` |
| #13 | TQROM mixed CHR memory | `a5ab29003209e62f33b754fbc4528c165e0c523d` |
| #14 | TKSROM/TLSROM nametable routing | `315d036674d9fac31ebee9afb1f2c6801be6bc4b` |
| #15 | BNROM and NINA-001 | `17c4a928c9f543c8aaef51e34ac9213b8f4bc58a` |
| #16 | GxROM | `0f6d4962d2710a2e73375daa5302bbcce7f12534` |
| #17 | Codemasters board variants | `1f19d4fe2ab07328e1d3fe0c80ab2b7350dd8e29` |
| #18 | Namco 108 | `e72123dda888653ad6240405a198bc7434cf280c` |
| #19 | VRC2 and VRC4 | `aa7c07dbb3e9603cb6aa2a2a0d91130b43ba44e0` |
| #20 | VRC6 banking and audio | `b37dd405cd883496edb02da0ec074baa2da14c3c` |
| #21 | VRC7 banking and FM audio | `911884abe7be864a8e470e701a9f6f345e1fb509` |
| #22 | Namco 163/175/340 | `e9af7e3fd3c5984a5ba57a17fefc84cd1ac38922` |
| #23 | FME-7 and Sunsoft 5B | `1330acb538462bdcb7e512083e70451aa8c05dc0` |
| #24 | Bandai FCG/LZ93D50 and Datach | `34c01c992d0dfcf29daeb1a6ae4554db6170f5ad` |
| #25 | Jaleco SS88006 | `9785b9775d1101c61ffa8c4c62e7622a088e42eb` |
| #26 | Irem G-101 and H-3001 | `05a355f0257ae8ba3cc44d7b0755217d73e6c6df` |
| #27 | Taito 33/48 | `980c9bff42d3db1512979ca807a7b78922a1ee76` |
| #28 | RAMBO-1 | `7521eab83288f9e5e456318cbe22b838b67b0413` |
| #29 | MMC1A | `e7104b7887e1acfe79fe1154bdfedc85f23f5348` |
| #30 | MC-ACC | `7fe5101bbd805a9606d57a8b141f0eaf88519f68` |
| #31 | Action 53 | `7505c20c6083765397e3858f17228cb87788bdd3` |
| #32 | UNROM 512 and flash | `2fe2481c0c5d259e880cbb090316163f49796655` |
| #33 | NES and Famicom light guns | `e444e4b15f436ad0ebe969b43df85d176a11d7c7` |
| #34 | Multiplayer adapters | `7e08a6f30199be89454c02af2d25dc7fc0852362` |
| #35 | Arkanoid controllers | `0aa620b4d64f9d45bb34cc3f01c3d67cd0154400` |
| #36 | Power Pad and Family Trainer | `b03f7c931b066cbf4f142f26989bd086fc6bd17e` |
| #37 | Family BASIC keyboard and tape | `75e2f0bbfd94cd70939b278588a9a72dd1e3b288` |
| #38 | Disk-system memory, controller, and audio | `97678f1c9df2772b74d8d555d04f62ee1b30d108` |
| #39 | VS System CPU/PPU/APU, input, DMA, and cartridge hardware | `9d67e89a7ed704c75702ba78eb8d36a47cffebdc` |

The VRC7 implementation also passed the Windows sanitizer gate at `911884abe7be864a8e470e701a9f6f345e1fb509`. Arithmetic and register-boundary corrections passed ordinary and sanitizer gates at `5c62d5f1c10f236b24e7e92e7524bbcce3d29599`. The reset correction passed the ordinary gate at `3f1ec191ea34644871b89ab0cde384db9ace4ef2`. The GCC register-indexing correction passed the integrated gate at `b4a6d15f631d9c261c6f28976f0e9ef7b5a4edd3` and the Linux GCC and Clang sanitizer CI jobs.

The following review corrections also passed the hardware suite and AccuracyCoin 144/144 with zero skipped or unfinished tests:

| Correction | Tested commit |
| --- | --- |
| Preserve the disk controller's state and pending IRQ when media is ejected | `dabbafa08d0a7c5070a7d0ef3e45b60d1393850e` |
| Use board RAM defaults for legacy headers and accept their 4 MiB zero-count PRG encoding | `c97f47dcf1e9b331740d86844fa8af0256a4e90a` |
| Present both VS cabinets, mix secondary audio, reset single-system protection, honor declared RAM, and clear VS state on disk loading | `7eb5620bfb21f98bed72240af2d4052304547432` |

The complete hardware suite and AccuracyCoin also passed under Windows AddressSanitizer and UndefinedBehaviorSanitizer at `7eb5620bfb21f98bed72240af2d4052304547432`. The CI workflow separately runs strict GCC and Clang builds, enables Linux leak detection, checks the canonical CPU trace and all 91 diagnostic ROMs, and repeats the full AccuracyCoin gate on the pull-request commit.

## Further cartridge, input and timing work

The following issue integrations each passed the hardware suite and the same complete 144/144 AccuracyCoin check. Mapper and device tests use production loading, CPU/PPU bus accesses, reset paths and persistence where those are part of the issue.

| Issue | Implemented behavior | Tested commit |
| --- | --- | --- |
| #41 | VS RGB frame lengths on both frame parities and both cabinets | `8ba741074b44b640b9ad30b2a3ca8f017abca2a1` |
| #42 | Selectable MMC3 revision-A IRQ qualification | `f4bbfa0e88a455a1b330a39de8a772569aed45e2` |
| #43 | Serial VS Zapper input and beam timing | `1f1aed9355d0a2b4e4db22cb48de19aca3c7a2b4` |
| #44 | Extended VS console descriptors with the existing 2C03 fallback | `163c9ad97bf3800cf5c14de13cccc9b8af1590f5` |
| #45 | VRC1 mappers 75 and 151 | `d1a7517ecf152767afc4b0d5bd9192cc481c0bd5` |
| #46 | VRC3 mapper 73 | `507af2113e5efe5c314b3b9602e4619ca5f70afd` |
| #47 | Sunsoft 3 mapper 67 | `264eea41866bd601a56cb1e24fad03507d35b69d` |
| #48 | Sunsoft 4 mapper 68 | `edbaa7f6744387b918752c3dacc5691e594d2fc3` |
| #49 | Sunsoft mappers 89, 93 and 184 | `b5b44c35c19cea11da776effe50c13f3f4e8dfc3` |
| #50 | Namco 108 variants 76, 88, 95 and 154 | `e35160a62cb2d0cb90a90c9956c4106171340dcb` |
| #51 | Taito X1 mappers 80, 82 and 207 | `7ff809c7e135b4cf9302be129cff8147d8ebe350` |
| #52 | Irem 77 and 97, including declared mapper 77 CHR RAM | `ad00a284935a11e37642291cf4d93dacd5186b32` |
| #53 | Jaleco 72, 78, 87, 92, 101 and 140, including 16 KiB PRG mirroring | `00b7cbb98096d21a36860b556f709743f3024a1b` |
| #54 | Protected CNROM mapper 185 and submappers 4 through 7 | `e953b1818604b83dc47c95963c31cc13f4383076` |
| #55 | Mapper 105 banking, fixed CHR RAM and competition timer | `0d689b188be25b40757baca27ea16d40d8bf1a1d` |
| #56 | Mapper 232 outer/inner banks and submapper-1 wiring | `6dd42ab9b777154b4a578350eeb09df9054e7281` |
| #57 | Mixed-CHR MMC3 boards 74, 191, 192, 194 and 195 | `a7ebec3fb4f657b5e23c44be34d42907342c4764` |
| #58 | JY 90, 209 and 211 banking, nametable routing and IRQ sources | `0d6bbb4ee4283a2cf7180201b9eed633b334c142` |
| #59 | GTROM mapper 111, including persistent PRG flash | `dfa65961738ab304bcc4092985f92aeb33c59766` |
| #60 | Mapper 96 and Oeka Kids tablet through a combined cartridge/CPU input fixture | `d2a37395160e1e538eb2ab982e2403d77ce1e01d` |
| #61 | ASCII Turbo File serial storage | `d70535fcd285c36dc190577e81ce96b9c8e7e187` |
| #62 | BattleBox commands, word storage and persistence | `391f227c2f5005d3553fea12949cd45a92103ef2` |
| #63 | Subor keyboard matrix and mouse packets | `a25ac274b45a28fb46c52ef9b1aff16ae91d58e6` |
| #64 | Hori Track reports | `9b428016ccc1f54115ed71a30076e313b708e0a7` |
| #65 | Konami Hyper Shot latch behavior | `cf08c709e19fefbb59aa1eaaeb4c918caf4ae119` |
| #66 | Bandai Hyper Shot serial controls and light sensing | `bf7ad97474925c426776dff03c4fa5c63f5bad4e` |
| #67 | Party Tap switch groups | `2d9c343412564e0e79f4f822a158f4a7051a4743` |
| #68 | Pachinko buttons and plunger report | `365e6b1364050af03967c50dd5f03cf0eef3f8ca` |
| #69 | Exciting Boxing input selection | `feecf5c2e5258c3e33839cc71d187f34fc20f4bd` |
| #70 | Jissen Mahjong matrix reads | `a8b5c9ce1d585b4b79e6e1e892a6bbe3401fbb60` |
| #71 | Barcode Battler framing and regional timing | `7c26380b0925489963398bcb5687266965e71a58` |
| #72 | Explicit and seeded CPU/PPU startup alignment | `4eb5d46c1a0eb985ab9d08c6d01c0bc001d081f7` |
| #73 | Optional 2A03 channel-output diagnostic reads | `8cb13babadec67bfe2c68c30fca5d4acb993ca8d` |
| #74 | EPSM YMF288 sound, bus protocols, stereo output and timer IRQs | `b2239f83c0c87992713a672bdf6f2e41a9a5acfb` |

Additional review checkpoints passed the same hardware and AccuracyCoin requirements:

| Correction or added regression | Tested commit |
| --- | --- |
| Repeatable seeded startup choices, including dual VS cabinets | `de77fa3b54d6591bf92a3208b2ab7b77fc4b0050` |
| Nonzero-address unmapped CHR reads on VRC1, Sunsoft 3 and Sunsoft 4 | `46a370d042ced9c35d10c82c56d6f1e6e41ab6cf` |
| GTROM bank and flash-command state across actual CPU soft reset | `32e5be9ae9fe0a25b34fa58b51c823764c304167` |
| Subor left/right modifier keys remain pressed until both are released | `8bcbbefd6c0558ea370d508a12aba8f7a979f509` |
| Oeka Kids pointer clicks assert contact in the upper area and offscreen | `e67908051946afc9c7f79fed3a13f22f5750bfa4` |
| Mapper 96 legacy PRG RAM, CPU access and battery persistence | `1b46cfbaf4ac8c70ddd73f457d5df12b5f007b4f` |
| Irem 77/97 inherited RAM, explicit RAM layouts and save round trips | `28636a4f7f856f7653064b2fa3195d0e2e28b3fe` |
| Jaleco discrete PRG RAM reads and board-specific write interception | `4b66328521126a38df21b110777158142ac655c1` |
| Nonzero Jaleco trainer RAM survives register writes and CPU soft reset | `bc1dbc0617e89154d5db1f43d9164db4bbbb4189` |
| Mapper 184 inherited RAM reads with CHR-register writes in the same window | `b021c90723c54d1807fc9e9410ba6fa710333fd9` |
| Pulse and noise DAC latches remain visible after channel disable until the next timer edge | `922c4da75c24e2c32344e5e31a19439222fd49a4` |

The RAM regressions execute CPU loads and stores across legacy defaults, declared RAM and explicit no-RAM layouts. Nonzero trainer and save data distinguish readable RAM from open bus or a constant return value. Writes to Jaleco 87/101/140 and Sunsoft 184 continue selecting banks without changing that RAM. The APU regression executes channel-disable and diagnostic-read instructions on both sides of a DAC timer edge.

The JY integration also prevents advanced nametable reads from exposing CHR RAM through a ROM-only path. The Subor integration uses unsigned keyboard shifts to avoid undefined behavior. Both corrections are included in their issue checkpoints above. Every combined revision must pass the GCC and Clang sanitizer CI runs, the 8,991-state trace, all 91 diagnostic ROMs, and AccuracyCoin; earlier checkpoints do not replace those checks.

## Combined validation

Implementation revision `b214e3b3c41757086773dbeca46ce27becc0aef7` includes issues #41 through #74 and the review corrections above. Its [push workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35349695056) and [pull-request workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35349723644) both passed. Each workflow's GCC and Clang sanitizer jobs passed the production hardware suite, 8,991-state CPU trace, all 91 diagnostic ROMs, and AccuracyCoin 144/144 with zero skipped or unfinished tests. The Clang jobs also enabled Linux leak detection.

The same revision passed local strict Windows builds, Windows AddressSanitizer/UndefinedBehaviorSanitizer, and Linux GCC and Clang sanitizer runs. Separate frontend harnesses exercised Oeka click/contact handling, paired Subor modifiers, and the application's default/enabled CPU test mode across a frontend reset.

These are historical results for the named revision. A later documentation or source commit needs its own CI result; consult the pull request's checks for that revision. The frontend harnesses were separate review tools, while the tracked CI suite is defined in [the workflow](../.github/workflows/accuracy.yml).

## Hardware revision and CHR checkpoints

Each integration commit below passed a strict Windows build, production hardware regressions, and AccuracyCoin in both normal and AddressSanitizer/UndefinedBehaviorSanitizer builds. Every run reported 144/144 passed, zero skipped, zero unfinished, and 4,182 frames, matching the cartridge's own tally. The ROM pin and result requirements above were unchanged.

| Issue | Implemented behavior | Tested integration commit |
| --- | --- | --- |
| #125 | Optional disabled OAMDATA reads return decaying PPU open bus without reading OAM or refreshing the latch | `dbfc8b7304badc35787246cecf26adb4d7968ee8` |
| #126 | Optional disabled palette readback uses buffered PPUDATA reads with the ordinary external transfer and address increment | `350780df92200dae50cc538bd6439ac3dbd6ee30` |
| #127 | Early sprite-evaluation wrap behavior feeds the ordinary sprite pipeline, including the possible X=255 pixel | `0213e74415d4d5a61e2c68f249a518191734f327` |
| #128 | Oldest-Famicom noise profile retains the written mode flag and selects the long-sequence feedback tap | `bed43aed297ebe0da1ace4f0a65724be802fa553` |
| #129 | Clone pulse duty profile swaps selections 1 and 2 at base-APU register writes while preserving MMC5 behavior | `4f97d11bf7c1658b30d33dc2a261a1d512cdd00e` |
| #130 | VS light sensing uses the selected hardware palette independently of display settings | `c0012ca2fb9763cd8e5568604e398cfcba5efd2d` |
| #139 | Native no-ROM CHR storage accepts a volatile prefix and NVRAM tail, with banking and NVRAM-relative save offsets | `a3c8afd8d5cd155d297917079072aad7ee430151` |

The PPU regressions cover register recovery, bus decay, sprite counts and heights, wrapped Y coordinates, fetch addresses, flips, palette selection, and reset behavior. The APU regressions cover all regional noise rates, both pulse channels and volume modes, write-time selection, reset persistence, and MMC5 independence. VS tests cover palette variants, sensor thresholds, beam timing, and separation from display edits.

The CHR regressions cover a fixed 4 KiB + 4 KiB allocation, unequal CPROM banks, and UNROM 512 nametable and pattern aliases. They compare complete save files, reload volatile and persistent data, retain short-save initialization, and verify the active cartridge after a rejected replacement. With 16 KiB of volatile CHR followed by 16 KiB of NVRAM, physical offset `$6000` persists at `.chr.sav` offset `$2000`. The unsupported ROM-plus-two-sidecar four-screen layout remains rejected.

The combined implementation is `a3c8afd8d5cd155d297917079072aad7ee430151`. Its Linux GCC and Clang AddressSanitizer/UndefinedBehaviorSanitizer builds also passed the production hardware suite, 8,991-state CPU trace, all 91 pinned diagnostic ROMs, and AccuracyCoin 144/144. The Linux sanitizer run enabled leak detection. These results belong to that source revision; the final pull-request commit must pass its own GCC and Clang sanitizer CI jobs.

Revision `6cbf43b6ddb4ad03c09675f63ae732eff3bbfb88` updates the README and five technical guides while retaining that implementation, build scripts, and test pins. Its [push workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35484461117) and [pull-request workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35484472930) both passed the GCC and Clang sanitizer jobs. Each job passed the production hardware suite, 8,991-state CPU trace, all 91 diagnostic ROMs, and AccuracyCoin 144/144 with zero skipped or unfinished tests. These are results for the named revision; later commits require their own CI checks.

## NSF multiplier reset checkpoint

Revision `265f7d2919940db3dac83faf3617cb65dddb11eb` preserves the NSF/NSFe MMC5 multiplier operands across soft reset and track changes. A fresh music-image load initializes both operands to zero. The regression runs a synthetic program through the production loader and CPU: its initialization routine reads `$5205/$5206` and stores both product bytes in RAM. It checks `$FE * $FD` across soft reset, a partial operand write followed by a track change, and separate operand initialization after reload. The expansion-audio combination tests also check retained operands while continuing to require cleared audio output.

Before the implementation change, the new reset regression and corrected audio-combination check both failed because the multiplier returned zero. The committed fix passed the strict Windows build, production hardware suite, and AccuracyCoin in normal and AddressSanitizer/UndefinedBehaviorSanitizer builds. Both AccuracyCoin runs reported 144/144 passed, zero skipped, zero unfinished, and 4,182 frames, matching the cartridge's tally. The ROM revision, SHA-256, and result requirements above were unchanged.

Both Windows builds also passed the canonical CPU trace's 8,991 states and all 91 pinned diagnostic ROMs through `scripts/run-diagnostics.py`.

## VRC7 console reset checkpoint

Revision `1dd63fd97a7228f58f14fc13812bc21ba3a3a055` connects mapper 85 console reset to the FM chip's reset operation. Banking, control, RAM, IRQ registers and counter state survive, along with the audio address latch, mute state and sample-clock phase. CPU reset clears an already-pending mapper IRQ before its bus cycles; the retained counter can raise a new IRQ during those cycles. The regression checks all three supported submapper values, an IRQ counter that keeps advancing through the seven CPU reset cycles, the next FM sample boundary, muted writes and a data write through the retained address latch. Before the fix, the sample-boundary assertion failed because the synthesizer kept playing.

The strict Windows build and production hardware suite passed in normal and AddressSanitizer/UndefinedBehaviorSanitizer builds. Each build then passed AccuracyCoin 144/144 with zero skipped and zero unfinished in 4,182 frames, matching the cartridge's tally. The test ROM pin and SHA-256 were unchanged.

## NMI during CPU reset checkpoint

Revision `6177689eed717b5c7ed32842226b866c54d92111` clears earlier NMI requests before the CPU reset bus cycles, preserving new edges detected during those cycles. With PPU reset suppression enabled, the running PPU can raise its vblank NMI inside that window. The regression advances the real PPU to five offsets in each of NTSC, PAL and Dendy, then verifies the first instruction, NMI handler and return. Paired cases use ordinary PPU reset, and the handler count checks that a held NMI line does not cause repeated delivery. Before the fix, all 15 cases with PPU reset suppression enabled missed the interrupt; the paired controls passed.

This revision includes the NSF multiplier and VRC7 reset fixes above. Its strict Windows build and production hardware suite passed in normal and AddressSanitizer/UndefinedBehaviorSanitizer builds. Both builds passed the canonical CPU trace's 8,991 states, all 91 pinned diagnostic ROMs, and AccuracyCoin 144/144 with zero skipped or unfinished tests. Each AccuracyCoin run completed in 4,182 frames and matched the cartridge's tally. The ROM revisions, SHA-256 and result requirements were unchanged.

These local results belong to the named implementation revision. Later documentation commits retain that source, and the final pull-request revision must pass its own GCC and Clang sanitizer CI jobs.

Revision `f26b7ee6aa3f298368f646983b389f932763869b` retains that implementation and passed both the [push workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35502276983) and [pull-request workflow](https://github.com/cupidthecat/cupid-nes/actions/runs/35502278770). Each workflow's GCC and Clang sanitizer jobs passed the production hardware suite, 8,991-state CPU trace, all 91 diagnostic ROMs and AccuracyCoin 144/144 with zero skipped or unfinished tests. The Clang jobs enabled address, undefined-behavior and leak checks. These CI results belong to the named revision.

## Disk-adapter RAM checkpoint

Revision `ecbd355d3d1d4e0dd9217df4877260fe45950cbe` applies the RAM power-on profile to the disk adapter's 32 KiB work RAM and 8 KiB CHR RAM. Previously both areas were always zeroed. The regression checks every byte for the fixed profiles, repeatable seeded initialization of both areas, preservation through CPU startup and soft reset, and fresh initialization on reload. A rejected image must preserve the active RAM and leave the random source unchanged.

The new regression failed on the preceding implementation and passed with the fix. The production hardware suite and AccuracyCoin passed in normal and AddressSanitizer/UndefinedBehaviorSanitizer builds. Both AccuracyCoin runs reported 144/144 passed, zero skipped, zero unfinished, and 4,182 frames, matching the cartridge's tally. The test image revision and SHA-256 above were unchanged.

## Partial CHR window checkpoint

Revision `dcdac463aac1d426c7c419b2b59e2b5214e11b03` corrects the remaining partial-window cases in #139. Native CHR mapping repeats complete banks and leaves an incomplete final window on open bus. Writes to uncovered addresses no longer modify an aliased byte or dirty a CHR save. VRC6 retains the preceding nametable mapping in chunks that a short replacement cannot cover; Sunsoft 4 exposes only complete 256-byte chunks from its selected CHR offset.

Before the fix, the added regressions produced 19 failures across pattern reads, save persistence, VRC6 nametables, and Sunsoft 4 nametables. They check every pattern-table address for the selected layouts, verify that unmapped writes leave the backing allocation unchanged, compare complete save files, and exercise nametable writes, mirrors, bank changes and CPU reset. Aligned 768-byte RAM and unaligned 384-byte RAM have separate expected mappings; the latter cannot establish a native bank mapping.

The committed revision passed strict Windows builds and the complete production hardware suite with normal and AddressSanitizer/UndefinedBehaviorSanitizer settings. The native mixed-CHR suite passed all nine groups. Both builds passed AccuracyCoin 144/144 with zero skipped or unfinished tests in 4,182 frames, matching the cartridge's tally. This revision includes the disk-adapter RAM initialization above. The ROM pins, SHA-256 and result requirements were unchanged.

## Regional timing checkpoint

Revision `7e2ab28275e3e81f41efef3077554b6206005058` adds `--region auto|ntsc|pal|dendy` through the production image loaders. Auto keeps image and database timing rules; explicit choices take precedence without rewriting that metadata. The selection affects the next successful load. Reset retains the loaded timing, and rejected replacements preserve the active machine. VS, FDS and StudyBox require effective NTSC timing.

The region suite passes 511 checks across iNES, NES 2.0, UNIF, database-corrected and recognized headerless images, startup-alignment failures, rejected replacements and fixed-timing hardware. Loaded-machine tests execute CPU instructions to verify the PPU divider ratio, check frame lengths and vblank boundaries, distinguish PAL APU periods from Dendy's NTSC periods, and check audio sample conversion through reset. Eleven launch cases exercise the production argument parser and loader; CI runs them with both compiler configurations.

NSF and NSFe INIT receive `X=1` only in PAL mode; NTSC and Dendy receive `X=0`. A synthetic music program records INIT registers and increments RAM on each PLAY call. Fixed expected cycle intervals check successive calls at the selected clock, alongside reset, track changes, metadata preservation and image replacement. With the preceding INIT-X expression restored, the classic NSF Dendy case failed while the remaining hardware groups and all 511 region checks passed.

The committed source passed strict Windows builds with normal and AddressSanitizer/UndefinedBehaviorSanitizer settings. Both builds passed the full hardware suite, eleven launch cases, the canonical CPU trace's 8,991 states, all 91 pinned diagnostic ROMs, and AccuracyCoin 144/144. Each AccuracyCoin run reported zero skipped and zero unfinished tests in 4,182 frames, matching the cartridge's tally. The test ROM revisions, SHA-256 and pass requirements were unchanged.

This revision includes the partial CHR, disk-adapter RAM, NSF multiplier, VRC7 reset and NMI during reset corrections above. Their separate checkpoints retain the results for each fix and the seven earlier hardware issues. These local results belong to the named implementation revision. Later documentation commits retain that source; the final pull-request revision must pass its own GCC and Clang sanitizer CI jobs.

## Cartridge save failure checkpoint

The production loaders now refuse an image replacement or unload when cartridge storage cannot be saved. The regression runs a CPU store on a native cartridge and a C++ board, then places a nonempty directory at the save destination to force the final atomic replacement to fail. It checks that both a valid replacement image and an unload are rejected, while the current ROM, program counter, RAM, and destination contents remain intact. Removing the obstruction allows a retry; reloading the original cartridge verifies the saved byte.

Both groups passed with the complete hardware suite in strict Windows normal and AddressSanitizer/UndefinedBehaviorSanitizer builds. The region and database launch checks passed in both builds. Each AccuracyCoin run passed 144/144 with zero skipped or unfinished tests in 4,182 frames, matching the cartridge's tally. These local results cover the persistence changes on the combined integration branch; they do not establish results for later frontend or archive changes. The existing ROM pins and pass requirements were unchanged.

## Archive, patch, and disk-overlay checkpoint

The prepared-image loader passed six patch-format groups, seven archive and image groups, and four FDS save-option groups in the complete Windows hardware suite. The archive fixtures contain owned synthetic cartridges and cover ZIP, LZMA 7z, solid LZMA2 7z, Unicode names, member selection, corruption, entry and output limits, separate save identities, and database lookup after patching. The disk tests exercise headered and headerless FDS and QD images, multiple sides, read-only sources, reloads with pending writes, and failed overlay replacement.

The same implementation passed strict normal and AddressSanitizer/UndefinedBehaviorSanitizer builds, the region and database launch suites, and AccuracyCoin 144/144 with zero skipped or unfinished tests in 4,182 frames. Sanitizer testing found and fixed unaligned integer access in the bundled archive decoder and a freed-buffer read in the recent-image parser. Malformed recent lists now leave the previous list intact. These local checkpoints cover the prepared-image and disk-overlay APIs; later application controls and automatic disk operations require their own integration checks. Existing diagnostic ROM pins and pass requirements were unchanged.

## Reproducing a checkpoint

Check out the listed commit in a separate worktree, prepare SDL2 and the pinned ROM as described in [development and testing](development.md), then run:

```powershell
.\scripts\test-windows.ps1 -SdlRoot C:\path\to\SDL2-2.32.10
.\build\windows\accuracy-tests.exe --accuracycoin 12000 C:\path\to\AccuracyCoin.nes
.\scripts\test-windows.ps1 -SdlRoot C:\path\to\SDL2-2.32.10 -Sanitize
.\build\windows-sanitized\accuracy-tests.exe --accuracycoin 12000 C:\path\to\AccuracyCoin.nes
```

The Linux equivalents, canonical CPU trace, 91-ROM collection, and sanitizer commands are in [the accuracy notes](accuracy.md). AccuracyCoin is a regression baseline for CPU/PPU/APU interactions; passing it does not substitute for the focused mapper, disk, audio, and input-device tests.

## Feature integration checkpoints

The hardware checkpoints above cover #125 through #130, #139, and #140. The
following feature work is integrated into the production desktop application.
The listed regression files are under `src/tests` unless a script path is given.

| Issue | Integrated behavior | Focused coverage |
| --- | --- | --- |
| #131 | Transactional machine states, slots, files, and board/peripheral state | state_accuracy.c, board_state_accuracy.c, state_ui_accuracy.c |
| #132 | Pause, stepping, reset, power, reload, and speed controls | frontend_accuracy.c |
| #133 | Saved configuration, profiles, keyboard/gamepad bindings, and launch precedence | frontend_accuracy.c, desktop_accuracy.c |
| #134 | Debugger, inspection panels, breakpoints, trace, and bounded Lua callbacks | debugger_accuracy.c |
| #135 | Rewind history and run-ahead with isolated host output | rewind_accuracy.c |
| #136 | Open, recent images, archive selection, and transactional switching | frontend_accuracy.c |
| #137 | ZIP/7z loading and IPS/UPS/BPS patches | media_accuracy.c, patch_accuracy.c |
| #138 | Cheat parsing, persistence, memory matching, and frontend controls | cheat_accuracy.c |
| #141 | Database discovery, explicit paths, corrections, and desktop selection | game_database_discovery_accuracy.c, desktop_accuracy.c |
| #142 | Disk save modes, overlays, write protection, and automatic loading | fds_options_accuracy.c, fds_automation_accuracy.c |
| #143 | Versioned input movies and deterministic session ownership | movie_accuracy.c, movie_frontend_accuracy.c |
| #144 | TCP sessions, compatibility handshake, slot ownership, frame hashes, and recovery | netplay_accuracy.c, scripts/check-netplay.py |
| #145 | Music transport, repeat, shuffle, timing, fade, and silence progression | nsf_player_accuracy.c |
| #146 | PNG screenshots, WAV audio, and AVI video recording | capture_container_accuracy.c, capture_session_accuracy.c |
| #147 | Regional overscan, layers, channel mixing, and stereo preservation | video_presentation_accuracy.c, audio_mix_accuracy.c |
| #148 | Format-109 replacement assets, audio, discovery, install, capture, and export | hd_pack_accuracy.cpp, hd_renderer_accuracy.cpp, hd_runtime_accuracy.cpp |
| #149 | Desktop menus, settings, storage, device panels, navigation, and scaled layouts | desktop_accuracy.c; manual acceptance remains open |

These clean source revisions passed AccuracyCoin **144/144**, with zero skipped
or unfinished tests in 4,182 frames. Each used ROM revision
`9bc42d1e3acbeeaea215b1011d58f4ce72a8a49e` and SHA-256
`7e25ac08d2e7ed14c9b1f16bd853148fef09a824452164f8e0d69fd2bd96176c`.

| Revision | Checkpoint | Windows configuration |
| --- | --- | --- |
| `0f22c06bacc00e80a1e436c829686cf9a98ec395` | Database and application baseline | Strict normal |
| `8af8e5ddcae209ba242fff0a278239c3dc78fc3d` | Disk automation and execution policy | Strict normal and ASan/UBSan |
| `f8186f2f92a5863166dc764dd46f0d83b5adc6f3` | Music player | Strict normal and ASan/UBSan |
| `db4758089567f437353b1c8549014c73c5ff7473` | Complete machine states | Strict normal and ASan/UBSan |
| `10cdfb64918b855e60d9c91c9b7532ddfeefa56a` | Capture and state integration | Strict normal |
| `4455b2b2e54a644abe6c1b294e4c5dd412942630` | Debugger and trace integration | Strict normal |
| `1eb1d1fcea02bb7bb68e34a9e604d27623423209` | Movies and combined state/session guards | Strict normal and ASan/UBSan |
| `ee44f496c15f71fa2120706745d30a3490954ef7` | Desktop runtime and settings | Strict normal |
| `d1929f3b8b67766fb42168416970608d15a618f5` | HD rendering and audio integration | Strict normal |
| `b830ed9` | Network sessions | Strict normal |
| `7f89580` | Desktop navigation and saved pack selection | Strict normal |
| `cb722fe` | Live settings, storage controls, nested audio locks, and session guards | Strict normal and ASan/UBSan |

The disk, music, state, capture, debugger, and combined movie checkpoints also
passed the complete production hardware suite and region/database launch checks.
The combined movie checkpoint passed the canonical 8,991-state CPU trace and all
91 pinned diagnostic ROMs in both Windows configurations. The network and later
Windows checkpoints include seven separate-process connection tests.

Revision `f231786` fixes fractional-scale font rendering. Its strict Linux build
passed AccuracyCoin 144/144 with the same zero-skip, zero-unfinished result.
The desktop renderer produces the [documented screenshots](desktop.md) from
synthetic fixtures. Manual native-window acceptance for #149 is still pending;
these automated checks do not establish physical-controller or native-dialog
behavior on both operating systems.

Use the commands in the reproduction section for each listed revision. The
final pull-request head must pass its own CI runs; earlier checkpoint results
are not substituted for those runs.

Revision `368e6a9` passed the strict Linux hardware suite and AccuracyCoin
144/144 with zero skipped or unfinished tests after binding-label and panel
navigation updates.
