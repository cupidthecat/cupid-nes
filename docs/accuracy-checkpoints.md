# Accuracy implementation checkpoints

Each commit below passed the production hardware regressions and the full pinned AccuracyCoin cartridge: **144/144 passed, zero skipped, and zero unfinished**. These are the commits tested after integrating each issue. Later fixes require their own checks, and the final pull-request commit must pass the complete CI workflow.

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

## Reproducing a checkpoint

Check out the listed commit in a separate worktree, prepare SDL2 and the pinned ROM as described in the README, then run:

```powershell
.\scripts\test-windows.ps1 -SdlRoot C:\path\to\SDL2-2.32.10
.\build\windows\accuracy-tests.exe --accuracycoin 12000 C:\path\to\AccuracyCoin.nes
```

The Linux equivalents, canonical CPU trace, 91-ROM collection, and sanitizer commands are in [the accuracy notes](accuracy.md). AccuracyCoin is a regression baseline for CPU/PPU/APU interactions; passing it does not substitute for the focused mapper, disk, audio, and input-device tests.
