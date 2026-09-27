# Debugger tools

Open the debugger panels from the desktop's Debug category. Captures belong to the current host session. They are not stored in save states, movies, or deterministic hashes. Capture is available during live execution; movie, netplay, rewind, and speculative execution do not add samples.

Start capture, run or step the game, then stop capture to inspect the results. Previous page and Next page browse 24 rows at a time. Copy sends the selected row to the clipboard. Text Hooker copies all collected text. Export writes through an atomic file replacement and rejects paths reserved by the active media/session.

## Code/data logger

The logger identifies bytes by their physical PRG offsets. Bank switching therefore keeps distinct cartridge locations separate. `C` means an executed opcode, `o` an instruction operand, `D` a data read, `B` both code and data, and `.` an untouched byte. The coverage summary counts these categories independently; a byte can appear in more than one category.

Debugger inspection does not add coverage. Read samples include actual dummy and DMA reads when those reads reach the CPU observation hook. Opcode fetches use the byte fetched by the CPU. Capture never fetches an instruction a second time.

Clear results erases coverage. Save coverage writes the binary format below; an export path ending in `.csv` writes a row for each physical byte. Import replaces coverage only after checking the entire file, PRG size, image hash, and flag bits.

Only storage that the cartridge resolver identifies within the loaded PRG buffer receives physical coverage. Custom read handlers, decompressed data, disk buffers, and other unresolved mappings remain outside PRG coverage. Writable PRG/flash coverage is historical for that backing byte: changing its contents does not erase the flags. Reload the image to begin a new identity, or clear coverage to begin a new measurement.

The PRG coverage limit is 64 MiB. The file begins with this ASCII line:

```text
CUPID-CDL 1 <decimal PRG byte count> <40 lowercase SHA-1 hex digits>
```

The following bytes are one flag byte per physical PRG byte: bit 0 is opcode, bit 1 is data, and bit 2 is operand. Other bits and trailing bytes are rejected. The hash identifies the PRG contents at capture initialization, before any later flash changes. CSV columns are `prg_offset,code,data,operand`.

## CPU profiler

The profiler counts completed instructions and their actual CPU cycles, including stalls within the instruction. A paused breakpoint probe adds neither an instruction nor cycles. Hardware interrupt entry is excluded from instruction cycle totals; BRK is an instruction and includes its entry sequence.

Results show counts, cycles, percentage of total captured instruction cycles, and an available symbol. Sort by instructions changes the default cycle ordering. Group by function spans combines locations covered by function symbols; other locations keep their own rows. Export writes every result in the current grouping and sort order.

Counters are 64-bit and saturate at their maximum. At most 131,072 distinct instruction locations are retained. The summary reports samples dropped after the location table fills. A physical PRG location has its own key; other locations use `100000000 + CPU address` in hexadecimal. Unresolved banked storage uses this CPU scope and cannot distinguish banks.

## NES event viewer

The event list shows the last captured completed frame. Each row includes frame number, scanline, dot, CPU cycle, category mask, instruction PC, bus address/value, and the bus address's physical/CPU key. Capture keeps up to 131,072 events per frame and reports overflow. Stopping capture retains the last completed frame.

Apply capture settings changes the category filter without recapturing. Category bits are hexadecimal:

| Bit | Event |
| --- | --- |
| `001` | CPU read |
| `002` | CPU write |
| `004` | PPU register access |
| `008` | APU register access |
| `010` | OAM DMA request |
| `020` | NMI |
| `040` | IRQ |
| `080` | Vblank status transition |
| `100` | Sprite-zero hit |
| `200` | Cartridge write or mapper IRQ |

A row can have several category bits. For example, `006` includes PPU writes because the row carries both write and PPU bits. The filter includes rows matching any selected bit. Inspect selected instruction shows nearby disassembly. Export retains timestamps and addresses as text.

## Call stack

Start capture before the calls you want to inspect. The stack records observed JSR, BRK, and hardware interrupt frames, with caller, target, return address, and return stack pointer. Inspect selected instruction opens an eight-instruction preview; Inspect return address previews the return site.

RTS and RTI remove matching frames. Stack-pointer replacement, overwritten return bytes, unmatched returns, and overflow mark affected frames as uncertain. The stack holds 128 frames. It does not invent callers that ran before capture began. Reset, a state discontinuity, or a register edit clears the model. Names are resolved using saved physical keys so a later bank switch cannot rename a saved frame as a different physical function.

## Symbols, functions, and source

Symbols and functions accepts a CPU address, a name, a byte span, and an optional comment. Leave Physical PRG offset blank to resolve the current mapping. Enter a hexadecimal offset to describe another bank. Functions only filters the list and marks newly saved labels as functions. Names can contain letters, digits, `_`, `.`, and `@`; each name and physical start key must be unique.

Symbols appear in disassembly, trace output, profiler rows, and stack rows. The debugger's existing address fields accept exact symbol names. Adding a breakpoint by a symbol name preserves its physical bank. An unlabeled watch in the debugger's viewer uses an available symbol name.

To associate source, enter a relative file path and a one-based line number. Source view follows the PC by default. Choose Source root to locate those relative files. Turn Follow PC off to inspect an address or symbol. The source pane reads the mapped line and can set an execute breakpoint for that mapping. Missing files or metadata show disassembly. Files are limited to 4 MiB; a displayed source line must fit in the pane's 1,536-byte buffer. Source roots are host settings and are not written to the symbol file.

Import accepts the documented Cupid symbol format. Failed imports preserve all active symbols. Export begins with:

```text
CUPID-SYMBOLS 1 <40 lowercase SHA-1 hex digits>
```

Each following line has nine tab-separated fields:

```text
key_hex  cpu_address_hex  span_decimal  function_0_or_1  name  relative_file  line_decimal  -  comment
```

The spacing above represents tabs. An empty file field and line `0` mean no source mapping. Physical keys are PRG offsets; CPU-scoped keys are `100000000 + address`, in hexadecimal. Source paths use `/`, cannot be absolute, and cannot contain `.` or `..` components. Tabs, control characters, duplicate keys/names, and invalid spans are rejected. Imports are limited to 8 MiB and 8,192 symbols. Source mappings describe the supplied span; the tool does not infer missing line metadata.

## Find references

Choose a CPU range, enter the target address or constant, then Find static references. By default, the search decodes the currently mapped bytes. To inspect another bank, supply a physical PRG start offset; First CPU address supplies the virtual address used for relative branches in that slice.

Results are static linear-disassembly matches, not evidence that an instruction executed. Indexed operands match their base address; indirect operands match the pointer address, not its runtime destination. Find constants searches immediate operands. Rows include the source CPU address, physical/CPU key, instruction, and target. Inspecting an unmapped saved bank reports that it is unavailable instead of showing another bank's bytes.

## Text hooker

Configure an inclusive CPU address range and select reads, writes, or both. Start capture applies those settings. Without a table, printable ASCII bytes are collected; zero and newline end a line. Stop capture flushes the unfinished line. Clear results empties output. Suppress repeated lines removes consecutive duplicate completed lines.

An encoding table uses one entry per line:

```text
41=A
4243=BC
00=\n
```

Keys contain one to eight hex byte pairs without spaces. Values are UTF-8 text, up to 63 bytes. The exact value `\n` ends a line; an empty value consumes bytes without producing text. A line beginning with `;` is a comment. Duplicate keys and keys that are prefixes of other keys are rejected, so decoding needs no speculative reads. Tables are limited to 128 KiB and 1,024 entries. A rejected table leaves the active table intact.

Output holds at most 1 MiB. Individual lines hold 1,023 bytes and split at a UTF-8 character boundary. When output fills, further lines are counted as dropped. Capturing a range that includes instruction fetches or dummy reads also observes those accesses. Choose a narrow data range to avoid that noise.

## Trace logger

Trace history defaults to 65,536 entries and can be set from 1 to 1,048,576. Once full, new entries replace the oldest entries; the summary reports how many were overwritten. Resizing retains the newest entries that fit. Stop capture retains the history; Clear results erases it.

The address range filters the instruction PC. An optional bounded debugger expression captures when its value is nonzero; an empty condition captures every address match. For example, `A & $01` captures when the low bit of A is set. The expression language supports the existing arithmetic/bitwise register expressions, not comparison syntax.

The hexadecimal Columns mask selects registers (`1`), CPU cycles (`2`), fetched instruction bytes (`4`), and symbols (`8`). PC, location key, and mnemonic are always shown. Follow last stays at the newest rows; paging turns it off. Export writes all retained entries with the current columns. Each entry keeps the actual fetched bytes, so later bank changes or memory edits do not replace its recorded instruction bytes.

## Regression coverage

`src/tests/debug_tools_accuracy.c` runs against the production CPU, cartridge mappings, debugger hooks, file transactions, and panel registry. It covers bank separation, cycle totals, stack returns and uncertainty, ring retention, rejected imports, source mapping, static references, text decoding, bounded events, panel actions, and extra bus accesses. The integration run also exercises all CPU/joypad bus-order groups with tools disabled and enabled. Full hardware, diagnostic-ROM, AccuracyCoin, and sanitizer results belong to the combined build.
