#!/usr/bin/env python3
# package-gtk-windows.py - Bundle a relocatable GTK runtime with verified imports.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run using UCRT64 Python; archive order and timestamps are deterministic."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import zipfile


def run(*args):
    return subprocess.check_output([str(arg) for arg in args], text=True).strip()


def package(args):
    root = Path(__file__).resolve().parents[1]
    prefix = Path(args.prefix).resolve()
    executable = Path(args.executable).resolve()
    version = (root / 'VERSION').read_text().strip()
    if not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.-]+)?', version):
        raise ValueError('VERSION must contain a release SemVer')
    output = Path(args.output_directory).resolve() if args.output_directory else root / 'build' / 'release'
    output.mkdir(parents=True, exist_ok=True)
    archive = output / 'cupid-windows-x64.zip'
    objdump = prefix / 'bin' / 'objdump.exe'
    system = Path(os.environ['SystemRoot']) / 'System32'
    available = {p.name.lower(): p for p in (prefix / 'bin').glob('*.dll')}
    with tempfile.TemporaryDirectory(prefix='gtk-package-', dir=output) as temporary:
        stage = Path(temporary)
        (stage / 'bin').mkdir()
        shutil.copy2(executable, stage / 'bin' / 'cupid-nes.exe')
        for helper in sorted((prefix / 'bin').glob('gspawn-win64-helper*.exe')):
            shutil.copy2(helper, stage / 'bin' / helper.name)
        for relative in ('share/glib-2.0/schemas', 'share/icons', 'share/locale',
                         'share/licenses', 'share/fontconfig', 'share/mime', 'etc/fonts',
                         'lib/gdk-pixbuf-2.0', 'lib/gio/modules', 'lib/gtk-4.0'):
            source = prefix / relative
            if source.is_dir():
                shutil.copytree(source, stage / relative)
        # Compiled schema indexes and module caches must describe this bundle.
        schemas = stage / 'share/glib-2.0/schemas'
        run(prefix / 'bin/glib-compile-schemas.exe', schemas)
        for cache in stage.rglob('loaders.cache'):
            cache.unlink()
        pending = list(stage.rglob('*.dll')) + list((stage / 'bin').glob('*.exe'))
        inspected = set()
        system_imports = set()
        while pending:
            binary = pending.pop()
            if binary in inspected:
                continue
            inspected.add(binary)
            imports = re.findall(r'DLL Name:\s*(\S+)', run(objdump, '-p', binary))
            for name in imports:
                key = name.lower()
                if key in available:
                    destination = stage / 'bin' / available[key].name
                    if not destination.exists():
                        shutil.copy2(available[key], destination)
                        pending.append(destination)
                elif key.startswith(('api-ms-win-', 'ext-ms-win-')) or (system / name).is_file():
                    system_imports.add(name)
                else:
                    raise RuntimeError(f'Unresolved runtime import: {binary.name}: {name}')
        # Keep loader paths relocatable, independent of the packaging machine.
        loaders = sorted(stage.glob('lib/gdk-pixbuf-2.0/*/loaders/*.dll'))
        if loaders:
            query = prefix / 'bin/gdk-pixbuf-query-loaders.exe'
            cache_text = run(query, *loaders)
            cache_text = cache_text.replace(str(stage).replace('\\', '/'), '.')
            cache_text = cache_text.replace(str(stage).replace('\\', '\\\\'), '.')
            cache_text = '\n'.join(line for line in cache_text.splitlines()
                                   if not line.startswith('# LoaderDir ='))
            (loaders[0].parent.parent / 'loaders.cache').write_text(cache_text + '\n', encoding='utf-8')
        shutil.copy2(root / 'LICENSE', stage / 'LICENSE')
        shutil.copy2(root / 'VERSION', stage / 'VERSION')
        third_party = root / 'src/third_party'
        for source in sorted(third_party.rglob('*')):
            if source.is_file() and ('LICENSE' in source.name.upper() or source.suffix == '.md'):
                destination = stage / 'share/licenses/cupid-nes' / source.relative_to(third_party)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
        # These dependencies keep their license text inside the imported header.
        for relative in ('src/third_party/stb/stb_truetype.h', 'src/third_party/stb/stb_vorbis.h',
                         'src/rom/emu2413.h'):
            source = root / relative
            shutil.copy2(source, stage / 'share/licenses/cupid-nes' / source.name)
        (stage / 'cupid-nes.cmd').write_text(
            '@echo off\nsetlocal\ncd /d "%~dp0"\n'
            'set "PATH=%~dp0bin;%SystemRoot%\\System32;%SystemRoot%"\n'
            'set "XDG_DATA_DIRS=%~dp0share"\n'
            'set "GSETTINGS_SCHEMA_DIR=%~dp0share\\glib-2.0\\schemas"\n'
            'set "GDK_PIXBUF_MODULE_FILE=%~dp0lib\\gdk-pixbuf-2.0\\2.10.0\\loaders.cache"\n'
            'set "GIO_MODULE_DIR=%~dp0lib\\gio\\modules"\n'
            '"%~dp0bin\\cupid-nes.exe" %*\n', encoding='utf-8')
        manifest = {'version': version, 'revision': args.revision,
                    'gtk': run(prefix / 'bin/pkg-config.exe', '--modversion', 'gtk4'),
                    'compiler': run(prefix / 'bin/gcc.exe', '-dumpfullversion'),
                    'system_imports': sorted(system_imports), 'files': {}}
        manifest['packages'] = run(prefix.parent / 'usr/bin/pacman.exe', '-Q').splitlines()
        for path in sorted(stage.rglob('*')):
            if path.is_file():
                manifest['files'][path.relative_to(stage).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
        (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as bundle:
            for path in sorted(stage.rglob('*')):
                if path.is_file():
                    info = zipfile.ZipInfo(path.relative_to(stage).as_posix(), date_time=(1980, 1, 1, 0, 0, 0))
                    info.compress_type = zipfile.ZIP_DEFLATED
                    info.external_attr = 0o644 << 16
                    bundle.writestr(info, path.read_bytes())
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_suffix('.zip.sha256').write_text(f'{digest}  {archive.name}\n', encoding='ascii')
    print(f'{archive}: {digest}')
    return digest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--revision', default='unknown')
    parser.add_argument('--output-directory')
    parser.add_argument('--verify-reproducible', action='store_true')
    arguments = parser.parse_args()
    first_digest = package(arguments)
    if arguments.verify_reproducible:
        if package(arguments) != first_digest:
            raise RuntimeError('Packaging identical inputs produced different archives')
        print('Archive reproducibility: PASS')
