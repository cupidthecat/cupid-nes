#!/usr/bin/env python3
# package-gtk-linux.py - Package the Linux GTK executable and component licenses.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
"""Create a deterministic x86-64 Linux archive using system runtime libraries."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile


def package(args):
    root = Path(__file__).resolve().parents[1]
    executable = Path(args.executable).resolve()
    binary = executable.read_bytes()
    # Reject a Windows binary, another architecture, or the headless test build.
    if binary[:6] != b'\x7fELF\x02\x01' or binary[18:20] != b'\x3e\x00':
        raise ValueError('Expected a little-endian x86-64 ELF executable')
    dependencies = subprocess.check_output(['ldd', str(executable)], text=True)
    if 'not found' in dependencies or 'libgtk-4.so.1' not in dependencies:
        raise ValueError('GTK runtime dependencies are missing or this is a headless build')
    version = (root / 'VERSION').read_text().strip()
    if not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.-]+)?', version):
        raise ValueError('VERSION must contain a release SemVer')
    output = Path(args.output_directory).resolve()
    output.mkdir(parents=True, exist_ok=True)
    files = {'cupid-nes': binary,
             'VERSION': (root / 'VERSION').read_bytes(),
             'LICENSE': (root / 'LICENSE').read_bytes()}
    files['README.md'] = f'''# Cupid NES {version} for Linux x64

Built for Ubuntu 24.04 (x86-64). Uses the system GTK4, SDL2 and curl libraries.
On Ubuntu 24.04, install the runtime dependencies:

```sh
sudo apt install libgtk-4-1 libsdl2-2.0-0 libcurl4t64
```

Extract the archive, enter the cupid-linux-x64 directory, and launch:

```sh
./cupid-nes
./cupid-nes /path/to/game.nes
```

A graphical desktop session is required. ROMs and BIOS files are not included.
Documentation: https://github.com/cupidthecat/cupid-nes/tree/main/docs
Source revision: {args.revision}
'''.encode()
    third_party = root / 'src/third_party'
    for source in sorted(third_party.rglob('*')):
        if source.is_file() and ('LICENSE' in source.name.upper() or source.suffix == '.md'):
            files['licenses/' + source.relative_to(third_party).as_posix()] = source.read_bytes()
    for relative in ('src/third_party/stb/stb_truetype.h', 'src/third_party/stb/stb_vorbis.h',
                     'src/rom/emu2413.h', 'src/rom/emu2413.LICENSE'):
        source = root / relative
        files['licenses/' + source.name] = source.read_bytes()
    manifest = {'version': version, 'revision': args.revision, 'platform': 'Ubuntu 24.04 x86-64',
                'files': {name: hashlib.sha256(data).hexdigest() for name, data in sorted(files.items())}}
    files['manifest.json'] = (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode()
    archive = output / 'cupid-linux-x64.tar.gz'
    with archive.open('wb') as raw:
        with gzip.GzipFile(filename='', fileobj=raw, mode='wb', mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as bundle:
                for name, data in sorted(files.items()):
                    info = tarfile.TarInfo('cupid-linux-x64/' + name)
                    info.size = len(data)
                    info.mode = 0o755 if name == 'cupid-nes' else 0o644
                    bundle.addfile(info, io.BytesIO(data))
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_suffix('.gz.sha256').write_text(f'{digest}  {archive.name}\n', encoding='ascii')
    print(f'{archive}: {digest}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--revision', required=True)
    parser.add_argument('--output-directory', default='build/release')
    package(parser.parse_args())
