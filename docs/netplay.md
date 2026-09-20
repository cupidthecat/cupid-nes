# Netplay

[Documentation index](README.md)

Open the same game on both computers, then choose Tools > Netplay. On the host,
assign each player slot to Host, Guest, or Unused, choose a TCP port, and select
Host game. The default assigns player 1 to the host and player 2 to the guest.
On the other computer, enter the host's address and port and select Join game.
The default port is 8964. Both computers must be able to reach that TCP port;
Cupid does not provide a matchmaking or relay service.

The connection checks the protocol version, loaded image checksums, effective
region, controller wiring, hardware profiles, and enabled cheats. The host then
supplies the initial machine state. Each computer submits only its assigned
player slots. Global peripheral inputs belong to the host. Both computers use
the same frame inputs and compare hardware hashes before and after execution.
Audio consumption, window size, and presentation settings do not enter those
hashes.

The host controls shared pause and resume. The guest cannot independently reset,
load a state, change the game, or alter deterministic settings during a session.
The status bar and Netplay panel show the role and current frame. Networking
uses frame lockstep: a slow or delayed peer makes the other peer wait. There is
no prediction or rollback mode.

Disconnect / cancel restores and pauses the preceding local session. A timeout,
malformed message, or hash disagreement also ends the shared session and restores
local state. Network play does not write shared progress into the offline game's
battery or disk files. Clear a failed connection with Disconnect before starting
another one.

The protocol is specific to Cupid and is not compatible with other netplay
clients. It is direct TCP without transport encryption or account authentication.

## Regression checks

Run `python scripts/check-netplay.py build/accuracy-tests` after a Linux build,
or use `build/windows/accuracy-tests.exe` on Windows. The Windows test script
and CI run this automatically. Separate host and guest processes cover NTSC,
PAL, delayed input, different audio-consumption rates, slot ownership, shared
pause, image and hardware-profile mismatch, malformed messages, desynchronization,
and abrupt peer termination. Successful sessions compare 24 frame hashes from
each process. Failure cases check that local state and live storage ownership
are restored.
