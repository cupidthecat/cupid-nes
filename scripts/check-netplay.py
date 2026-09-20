#!/usr/bin/env python3
"""Exercise the production netplay protocol in separate emulator processes."""
import socket
import subprocess
import sys


def run_pair(binary, scenario):
    host = subprocess.Popen([binary, "--netplay-peer", "host", "0", scenario],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    client = None
    try:
        prelude = []
        for line in host.stdout:
            prelude.append(line)
            if line.startswith("LISTENING "):
                port = int(line.split()[1])
                break
        else:
            raise RuntimeError("Host did not start: " + "".join(prelude) + host.stderr.read())
        if scenario == "malformed":
            with socket.create_connection(("127.0.0.1", port), timeout=3) as peer:
                peer.sendall(b"invalid-header")
            client_out = ""
        else:
            client = subprocess.Popen([binary, "--netplay-peer", "client", str(port), scenario],
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            client_out, client_err = client.communicate(timeout=90)
            if client.returncode:
                raise RuntimeError("Client failed:\n" + client_out + client_err)
        host_out, host_err = host.communicate(timeout=90)
        if host.returncode:
            raise RuntimeError("Host failed:\n" + host_out + host_err)
        if scenario in ("normal", "pal"):
            host_frames = [line for line in host_out.splitlines() if line.startswith("FRAME ")]
            client_frames = [line for line in client_out.splitlines() if line.startswith("FRAME ")]
            if len(host_frames) != 24 or host_frames != client_frames:
                raise RuntimeError("The peers did not produce 24 identical hardware checkpoints")
        print(f"PASS: netplay {scenario}", flush=True)
    finally:
        for process in (host, client):
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate()


if __name__ == "__main__":
    for case in ("normal", "pal", "mismatch", "profile", "malformed", "desync", "interrupted"):
        run_pair(sys.argv[1], case)
