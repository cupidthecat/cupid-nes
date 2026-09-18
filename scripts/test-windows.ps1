param(
    [Parameter(Mandatory = $true)]
    [string]$SdlRoot,
    [string]$Compiler = 'clang',
    [string]$CxxCompiler,
    [switch]$Sanitize
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (-not $CxxCompiler) {
    $compilerName = [System.IO.Path]::GetFileNameWithoutExtension($Compiler)
    $compilerDirectory = Split-Path -Parent $Compiler
    $cppName = switch ($compilerName) {
        'clang' { 'clang++' }
        'gcc' { 'g++' }
        default { throw 'Specify -CxxCompiler for this C compiler' }
    }
    if ([System.IO.Path]::GetExtension($Compiler) -eq '.exe') { $cppName += '.exe' }
    $CxxCompiler = if ($compilerDirectory) { Join-Path $compilerDirectory $cppName } else { $cppName }
}
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
$cppFlags = @($flags | Where-Object { $_ -ne '-std=c11' }) + @('-std=c++17')

$coreSources = @('src/system/timing.c', 'src/system/hardware.c', 'src/system/vs_system.c', 'src/cpu/cpu.c', 'src/ppu/ppu.c', 'src/rom/rom.c', 'src/rom/mapper.c',
                 'src/rom/fds.c',
                 'src/rom/vrc7_audio.c', 'src/rom/emu2413.c',
                 'src/rom/eeprom.c', 'src/rom/namco163.c', 'src/rom/sunsoft5b.c',
                 'src/joypad/joypad.c', 'src/joypad/family_basic.c', 'src/joypad/special_peripherals.c',
                 'src/apu/apu.c', 'src/third_party/blip_buf.c', 'src/video/ntsc_composite.c', 'src/ui/palette_tool.c')
$cppSources = @('src/apu/epsm.cpp', 'src/third_party/ymfm/ymfm_opn.cpp',
                'src/third_party/ymfm/ymfm_ssg.cpp', 'src/third_party/ymfm/ymfm_adpcm.cpp',
                'src/rom/boards/runtime.cpp', 'src/rom/boards/factory.cpp')
$testSources = @('src/tests/accuracy_test.c', 'src/tests/cpu_accuracy.c', 'src/tests/cpu_trace.c',
                 'src/tests/apu_accuracy.c', 'src/tests/ppu_accuracy.c', 'src/tests/mapper_accuracy.c',
                 'src/tests/fds_accuracy.c',
                 'src/tests/rom_runner.c', 'src/tests/input_accuracy.c', 'src/tests/bandai_accuracy.c',
                 'src/tests/vs_accuracy.c', 'src/tests/epsm_accuracy.c', 'src/tests/board_accuracy.c',
                 'src/tests/board_codemasters_accuracy.c', 'src/tests/board_magic_floor_accuracy.c',
                 'src/tests/board_jaleco_accuracy.c', 'src/tests/board_nsf_cart_accuracy.c')
$application = Join-Path $outputDirectory 'cupid-nes.exe'
$testProgram = Join-Path $outputDirectory 'accuracy-tests.exe'
$objectDirectory = Join-Path $outputDirectory 'objects'
New-Item -ItemType Directory -Force -Path $objectDirectory | Out-Null

function Compile-Source([string]$Source, [string]$Driver, [string[]]$BuildFlags) {
    $object = Join-Path $objectDirectory ($Source.Replace('/', '_') + '.obj')
    & $Driver @BuildFlags '-c' $Source '-o' $object
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $Source" }
    return $object
}

Push-Location $projectRoot
try {
    $coreObjects = @(
        foreach ($source in $coreSources) { Compile-Source $source $Compiler $flags }
        foreach ($source in $cppSources) { Compile-Source $source $CxxCompiler $cppFlags }
    )
    $mainObject = Compile-Source 'src/main.c' $Compiler $flags
    $testObjects = @(foreach ($source in $testSources) { Compile-Source $source $Compiler $flags })
    & $CxxCompiler @cppFlags @coreObjects $mainObject $sdkLibrary '-o' $application
    if ($LASTEXITCODE -ne 0) { throw 'Emulator build failed' }
    & $CxxCompiler @cppFlags @coreObjects @testObjects $sdkLibrary '-o' $testProgram
    if ($LASTEXITCODE -ne 0) { throw 'Hardware test build failed' }
    & $testProgram
    if ($LASTEXITCODE -ne 0) { throw 'Hardware regressions failed' }
    Write-Output "Emulator: $application"
    Write-Output "Diagnostic runner: $testProgram"
} finally {
    Pop-Location
}
