# setup-gtk-windows.ps1 - Install an isolated UCRT64 toolchain in ignored build data.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
param()
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$dependencies = Join-Path $projectRoot 'build/dependencies'
$msysRoot = Join-Path $dependencies 'msys64'
$archive = Join-Path $dependencies 'msys2-base-x86_64-20260611.tar.xz'
New-Item -ItemType Directory -Force -Path $dependencies | Out-Null
if (-not (Test-Path -LiteralPath (Join-Path $msysRoot 'usr/bin/bash.exe'))) {
    Invoke-WebRequest 'https://repo.msys2.org/distrib/x86_64/msys2-base-x86_64-20260611.tar.xz' -OutFile $archive
    $expected = 'A2D047E8EE213C3C6A49A8DE427EB1069DF12207C0422FF1B3CBB5C905C34221'
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $expected) {
        throw 'MSYS2 archive checksum mismatch'
    }
    & tar -xf $archive -C $dependencies
    if ($LASTEXITCODE -ne 0) { throw 'MSYS2 extraction failed' }
}
$savedSystem = $env:MSYSTEM
try {
    $env:MSYSTEM = 'UCRT64'
    # Runtime upgrades close the first shell; the second completes the upgrade.
    & (Join-Path $msysRoot 'usr/bin/bash.exe') -lc 'pacman -Syu --noconfirm'
    if ($LASTEXITCODE -ne 0) { throw 'MSYS2 core update failed' }
    & (Join-Path $msysRoot 'usr/bin/bash.exe') -lc 'pacman -Syu --noconfirm && pacman -S --needed --noconfirm make diffutils mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-pkgconf mingw-w64-ucrt-x86_64-gtk4 mingw-w64-ucrt-x86_64-SDL2 mingw-w64-ucrt-x86_64-python'
    if ($LASTEXITCODE -ne 0) { throw 'GTK toolchain installation failed' }
} finally { $env:MSYSTEM = $savedSystem }
Write-Output "GTK UCRT64 toolchain: $msysRoot"
