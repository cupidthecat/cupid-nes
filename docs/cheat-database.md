# Cheat database

Open a cartridge, then choose **Tools > Cheats > Cheat Database**. The browser
matches the SHA-1 of the loaded PRG ROM and shows named cheats for that revision.
It includes a small catalog for Super Mario Bros. (World), Contra (USA), and
Mega Man 2 (USA). A different revision can have a different checksum and no match.

Select an entry to preview its codes, then choose **Add selected cheat**. Codes
are added to the normal cheat list, disabled by default. Check **Enable added
cheats** to enable them on import, or enable them later in the cheat editor.
A multi-code entry is added as one validated group: an invalid code or a full
cheat list leaves the active list unchanged.

You can load an additional UTF-8 tab-separated file. Each non-comment row has
four fields, separated by actual tab characters:

```text
PRG SHA-1<TAB>Game name<TAB>Cheat description<TAB>Code;Code
```

Use a 40-digit hexadecimal SHA-1 and codes accepted by the normal cheat editor.
Separate codes in a group with semicolons. Lines beginning with `#` are comments.
Names and descriptions are limited to 95 bytes; a group contains at most 16
codes. A file may contain up to 32,768 entries and 16 MiB of text.

Loading a file replaces the previous custom catalog and keeps the built-in
entries. A malformed file leaves both the previous catalog and active cheats
unchanged. Catalogs do not affect deterministic compatibility; enabled active
codes do. Adding or enabling cheats is unavailable during movies, netplay,
rewind, and speculative execution.
