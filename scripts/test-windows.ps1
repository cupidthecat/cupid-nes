param(
    [Parameter(Mandatory = $true)]
    [string]$SdlRoot,
    [string]$Compiler = 'clang',
    [switch]$Sanitize
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$sdkRoot = (Resolve-Path -LiteralPath $SdlRoot).Path
$sdkInclude = Join-Path $sdkRoot 'include'
$sdkLibrary = Join-Path $sdkRoot 'lib/x64/SDL2.lib'
$sdkRuntime = Join-Path $sdkRoot 'lib/x64/SDL2.dll'
foreach ($required in @((Join-Path $sdkInclude 'SDL.h'), $sdkLibrary, $sdkRuntime)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Missing SDL2 VC SDK file: $required"
    }
}

$outputDirectory = Join-Path $projectRoot $(if ($Sanitize) { 'build/windows-sanitized' } else { 'build/windows' })
$includeDirectory = Join-Path $outputDirectory 'include'
$sdlIncludeDirectory = Join-Path $includeDirectory 'SDL2'
New-Item -ItemType Directory -Force -Path $sdlIncludeDirectory | Out-Null
Get-ChildItem -LiteralPath $sdkInclude -Filter '*.h' | Copy-Item -Destination $sdlIncludeDirectory -Force
Copy-Item -LiteralPath $sdkRuntime -Destination (Join-Path $outputDirectory 'SDL2.dll') -Force

$flags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-D_CRT_SECURE_NO_WARNINGS', '-DSDL_MAIN_HANDLED', "-I$includeDirectory")
if ($Sanitize) {
    $flags += @('-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer')
    $resourceDirectory = & $Compiler '-print-resource-dir'
    if ($LASTEXITCODE -ne 0) { throw 'Cannot locate the compiler runtime directory' }
    $asanRuntime = Join-Path $resourceDirectory.Trim() 'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
    if (-not (Test-Path -LiteralPath $asanRuntime -PathType Leaf)) {
        throw "Missing Clang AddressSanitizer runtime: $asanRuntime"
    }
    Copy-Item -LiteralPath $asanRuntime -Destination $outputDirectory -Force
} else {
    $flags += '-O2'
}

$coreSources = @('src/system/timing.c', 'src/system/hardware.c', 'src/cpu/cpu.c', 'src/ppu/ppu.c', 'src/rom/rom.c', 'src/rom/mapper.c',
                 'src/rom/eeprom.c', 'src/rom/sunsoft5b.c', 'src/joypad/joypad.c', 'src/joypad/family_basic.c',
                 'src/apu/apu.c', 'src/ui/palette_tool.c')
$testSources = @('src/tests/accuracy_test.c', 'src/tests/cpu_accuracy.c', 'src/tests/cpu_trace.c',
                 'src/tests/apu_accuracy.c', 'src/tests/ppu_accuracy.c', 'src/tests/mapper_accuracy.c',
                 'src/tests/rom_runner.c', 'src/tests/input_accuracy.c', 'src/tests/bandai_accuracy.c')
$application = Join-Path $outputDirectory 'cupid-nes.exe'
$testProgram = Join-Path $outputDirectory 'accuracy-tests.exe'

Push-Location $projectRoot
try {
    & $Compiler @flags @coreSources 'src/main.c' $sdkLibrary '-o' $application
    if ($LASTEXITCODE -ne 0) { throw 'Emulator build failed' }
    & $Compiler @flags @coreSources @testSources $sdkLibrary '-o' $testProgram
    if ($LASTEXITCODE -ne 0) { throw 'Hardware test build failed' }
    & $testProgram
    if ($LASTEXITCODE -ne 0) { throw 'Hardware regressions failed' }
    Write-Output "Emulator: $application"
    Write-Output "Diagnostic runner: $testProgram"
} finally {
    Pop-Location
}
