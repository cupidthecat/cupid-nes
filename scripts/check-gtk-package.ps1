# check-gtk-package.ps1 - Validate archive hashes and launch with a clean runtime PATH.
# Author: @frankischilling
# SPDX-License-Identifier: GPL-3.0-or-later
param([string]$Archive)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (-not $Archive) { $Archive = Join-Path $projectRoot 'build/release/cupid-windows-x64.zip' }
$checksum = (Get-Content -LiteralPath "$Archive.sha256" -Raw).Split(' ')[0].Trim()
if ((Get-FileHash -LiteralPath $Archive -Algorithm SHA256).Hash -ne $checksum) {
    throw 'Archive checksum mismatch'
}
$python = (Get-Command python -ErrorAction Stop).Source
$destination = Join-Path $projectRoot ('build/package-check-' + [guid]::NewGuid().ToString('N'))
Expand-Archive -LiteralPath $Archive -DestinationPath $destination
$manifest = Get-Content -LiteralPath (Join-Path $destination 'manifest.json') -Raw | ConvertFrom-Json
foreach ($entry in $manifest.files.PSObject.Properties) {
    $actual = (Get-FileHash -LiteralPath (Join-Path $destination $entry.Name) -Algorithm SHA256).Hash
    if ($actual -ne $entry.Value) { throw "Package hash mismatch: $($entry.Name)" }
}
$saved = @{}
foreach ($name in @('PATH', 'XDG_DATA_DIRS', 'GSETTINGS_SCHEMA_DIR', 'GDK_PIXBUF_MODULE_FILE', 'GIO_MODULE_DIR')) {
    $saved[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
Push-Location $destination
try {
    $env:PATH = "$destination/bin;$env:SystemRoot/System32;$env:SystemRoot"
    $env:XDG_DATA_DIRS = "$destination/share"
    $env:GSETTINGS_SCHEMA_DIR = "$destination/share/glib-2.0/schemas"
    $env:GDK_PIXBUF_MODULE_FILE = "$destination/lib/gdk-pixbuf-2.0/2.10.0/loaders.cache"
    $env:GIO_MODULE_DIR = "$destination/lib/gio/modules"
    & $python (Join-Path $projectRoot 'scripts/check-gtk-launch.py') (Join-Path $destination 'bin/cupid-nes.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Packaged GTK startup failed' }
    Write-Output "Verified package: $destination"
} finally {
    Pop-Location
    foreach ($name in $saved.Keys) { [Environment]::SetEnvironmentVariable($name, $saved[$name], 'Process') }
}
