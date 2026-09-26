# Cartridge header editor

Open **Tools > iNES / NES 2.0 Header Editor** and choose an uncompressed `.nes` file. A game does not need to be running. The editor keeps a private snapshot of the chosen file; editing or saving does not replace the active cartridge.

Fields show decimal values. ROM and RAM sizes are bytes. The labels explain format, mirroring, console and timing codes. NES 2.0 also provides mapper/submapper, four RAM/NVRAM sizes, VS PPU and hardware codes, extended console subtype, miscellaneous ROM count, and the default input/expansion device code. Battery and trainer use 0 or 1. The iNES byte 8 field is preserved as metadata; Cupid uses the cartridge board's RAM defaults when loading iNES.

Change fields, check the validation message, then select **Save edited copy (.nes)**. Choose a `.nes` filename different from the source and active media. **Revert draft** restores the selected file's original metadata. Selecting another file replaces the draft only after the file is read successfully.

NES 2.0 RAM sizes must be zero or `64 << shift`, with shift from 1 through 15. NVRAM requires the battery flag. Unsupported fields must be cleared before converting to iNES; conversion never silently discards them. ROM sizes must have a valid encoding and fit inside the existing file. The iNES PRG size range is 1 through 256 banks of 16 KiB; CHR supports 0 through 255 banks of 8 KiB. NES 2.0 also supports exponent/multiplier ROM sizes.

Saving replaces only the 16-byte header. Trainer, PRG, CHR and trailing bytes remain in their original order and are copied exactly. Trainer presence cannot change because that would reinterpret the first 512 payload bytes. The editor neither inserts nor removes bytes. It retains trailing data even if the declared PRG/CHR boundary changes. A nonzero miscellaneous ROM count requires trailing data.

Writes use a flushed temporary sibling and atomic replacement. The source, active image, firmware, save files and other runtime-protected paths cannot be used as output destinations. The editor accepts files up to 512 MiB, including malformed payload declarations that can be repaired by changing the header. It rejects missing signatures and incomplete headers.

Validation uses Cupid's mapper, RAM and VS decoders, plus the loader's overflow, payload, console and CHR-storage rules. It does not instantiate a cartridge board or apply game-database overrides. A valid header therefore does not promise that every mapper/submapper or board geometry is implemented. Open the saved copy through the normal image command to run the complete production loader, including board validation, database policy and input-configuration checks. Reserved header bits are cleared when encoding the edited header.
