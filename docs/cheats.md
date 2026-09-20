# Cheat codes

Cupid accepts three NES cheat formats:

| Format | Syntax | Behavior |
| --- | --- | --- |
| Game Genie | six or eight letters from `APZLGITYEOXUKSVN` | Six-letter codes replace matching reads. Eight-letter codes also require the decoded compare byte. |
| PAR | eight hexadecimal digits | Decodes an address, value, and compare byte. |
| Raw | `AAAA:VV` or `AAAA:VV:CC` | Uses a 16-bit CPU address, replacement byte, and optional compare byte. |

Cheats are applied after the real CPU read. Device side effects happen once through the ordinary bus path. A compare code tests the original byte returned by the currently mapped device or ROM bank. A bank switch can therefore cause the same code to begin or stop matching without changing the cheat entry. Cheats do not intercept writes or modify the source image.

The cheat list supports add, edit, remove, enable/disable, and reorder operations. Saved lists contain the loaded game's CRC identity and use UTF-8 text. Saving uses the same atomic sibling-file replacement as other application data. Loading is transactional: a malformed file, invalid UTF-8, wrong game identity, or read failure leaves the active list unchanged.

Interactive cheat changes are rejected during movie recording, movie playback, and netplay. The active enabled-code list exposes a compatibility hash so deterministic sessions can include the selected cheats in compatibility checks. Resetting the emulated console does not change the list. Loading a different game clears it only after the new image has been accepted; a failed image replacement keeps the current game's cheats.

The public cheat management interface is in [`src/cheats/cheats.h`](../src/cheats/cheats.h).
