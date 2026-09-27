#!/usr/bin/env python3
# check-gtk-launch.py - Smoke-test GTK startup and event-loop survival.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
"""Start the desktop on a real display (or Xvfb); fail on early exit or GTK errors."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    executable = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix='cupid-gtk-smoke-') as temporary:
        environment = dict(os.environ, SDL_AUDIODRIVER='dummy')
        environment.pop('SDL_VIDEODRIVER', None)
        if sys.platform != 'win32':
            environment['GDK_BACKEND'] = 'x11'
            environment['SDL_VIDEODRIVER'] = 'x11'
        # Isolate all preferences and prevent the smoke test changing user data.
        for name in ('HOME', 'USERPROFILE', 'APPDATA', 'LOCALAPPDATA', 'XDG_CONFIG_HOME',
                     'XDG_DATA_HOME', 'XDG_CACHE_HOME'):
            environment[name] = temporary
        with tempfile.TemporaryFile() as log:
            process = subprocess.Popen([str(executable)], env=environment, stdout=log, stderr=log)
            try:
                time.sleep(5)
                if process.poll() is not None:
                    raise RuntimeError(f'Desktop exited during startup: {process.returncode}')
            finally:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                log.seek(0)
                output = log.read().decode('utf-8', errors='replace')
                if output:
                    print(output)
            if any(marker in output for marker in ('Gtk-CRITICAL', 'GLib-GObject-CRITICAL',
                                                    'Gtk-ERROR', 'GLib-ERROR', 'Gdk-ERROR')):
                raise RuntimeError('GTK startup reported a critical error')
    print('GTK launch smoke: PASS (desktop survived five seconds)')


if __name__ == '__main__':
    main()
