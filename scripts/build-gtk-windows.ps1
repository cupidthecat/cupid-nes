# build-gtk-windows.ps1 - Build and package the UCRT64 GTK desktop.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
param(
    [string]$MsysRoot,
    [string]$BuildRevision = 'unknown',
    [int]$Jobs = 4,
    [switch]$Package,
    [switch]$Test
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (-not $MsysRoot) { $MsysRoot = Join-Path $projectRoot 'build/dependencies/msys64' }
$MsysRoot = (Resolve-Path -LiteralPath $MsysRoot).Path
$bash = Join-Path $MsysRoot 'usr/bin/bash.exe'
if (-not (Test-Path -LiteralPath $bash)) { throw "Missing MSYS2 shell: $bash" }
if ($BuildRevision -notmatch '^[A-Za-z0-9._-]+$') { throw 'Invalid build revision' }
if ($Jobs -lt 1) { throw 'Jobs must be positive' }
function Quote-Bash([string]$Value) { return "'" + $Value.Replace("'", "'\''") + "'" }
$savedSystem = $env:MSYSTEM
$savedChere = $env:CHERE_INVOKING
try {
    $env:MSYSTEM = 'UCRT64'
    $env:CHERE_INVOKING = '1'
    $rootArgument = Quote-Bash ($projectRoot.Replace('\', '/'))
    $command = "cd $rootArgument && pkg-config --modversion gtk4 sdl2 && make -j$Jobs GTK=1 BUILD_REVISION=$BuildRevision TARGET=build/windows-gtk1/cupid-nes.exe all"
    & $bash -lc $command
    if ($LASTEXITCODE -ne 0) { throw 'GTK desktop build failed' }
    if ($Test) {
        & $bash -lc "cd $rootArgument && make -j$Jobs GTK=1 BUILD_REVISION=$BuildRevision gtk-smoke && GSK_RENDERER=cairo G_DEBUG=fatal-criticals SDL_AUDIODRIVER=dummy ./build/gtk-settings-test.exe && GSK_RENDERER=cairo G_DEBUG=fatal-criticals SDL_AUDIODRIVER=dummy ./build/gtk-ui-smoke.exe build/gtk-smoke-windows"
        if ($LASTEXITCODE -ne 0) { throw 'GTK desktop regressions failed' }
        & $bash -lc "cd $rootArgument && env -u GSK_RENDERER -u GDK_DEBUG G_DEBUG=fatal-criticals SDL_AUDIODRIVER=dummy ./build/gtk-ui-smoke.exe --startup-check"
        if ($LASTEXITCODE -ne 0) { throw 'GTK default compositor startup failed' }
        & $bash -lc "cd $rootArgument && SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy make -j$Jobs GTK=0 TARGET=build/windows-gtk0/cupid-nes.exe TEST_TARGET=build/windows-gtk0/accuracy-tests.exe all test"
        if ($LASTEXITCODE -ne 0) { throw 'Hardware regressions failed' }
    }
    if ($Package) {
        & $bash -lc "cd $rootArgument && python scripts/package-gtk-windows.py --prefix /ucrt64 --executable build/windows-gtk1/cupid-nes.exe --revision $BuildRevision --verify-reproducible"
        if ($LASTEXITCODE -ne 0) { throw 'GTK runtime packaging failed' }
    }
} finally {
    $env:MSYSTEM = $savedSystem
    $env:CHERE_INVOKING = $savedChere
}
