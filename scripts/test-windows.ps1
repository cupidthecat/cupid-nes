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

$flags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-D_CRT_SECURE_NO_WARNINGS', '-DSDL_MAIN_HANDLED',
           '-DZ7_PPMD_SUPPORT', '-DZ7_EXTRACT_ONLY', '-DMINIZ_NO_ZLIB_COMPATIBLE_NAMES', "-I$includeDirectory")
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

$coreSources = @('src/system/timing.c', 'src/system/hardware.c', 'src/system/vs_system.c', 'src/state/state.c', 'src/state/state_io.c', 'src/state/state_alloc.c', 'src/cpu/cpu.c', 'src/cpu/cpu_observer.c', 'src/ppu/ppu.c', 'src/rom/rom.c', 'src/rom/mapper.c',
                 'src/rom/fds.c', 'src/rom/nsf.c', 'src/util/file_io.c', 'src/util/sha1.c',
                 'src/debugger/ppu_inspector.c', 'src/debugger/debugger.c', 'src/debugger/disassembly.c', 'src/debugger/lua_runtime.c', 'src/cheats/cheats.c',
                 'src/rom/vrc7_audio.c', 'src/rom/emu2413.c',
                 'src/rom/eeprom.c', 'src/rom/namco163.c', 'src/rom/sunsoft5b.c',
                 'src/joypad/joypad.c', 'src/joypad/family_basic.c', 'src/joypad/special_peripherals.c',
                 'src/apu/apu.c', 'src/third_party/blip_buf.c', 'src/video/ntsc_composite.c', 'src/video/video_trace.c', 'src/ui/palette_tool.c',
                 'src/ui/nsf_frontend.c', 'src/ui/frontend_commands.c', 'src/ui/execution_control.c',
                 'src/ui/machine_actions.c', 'src/ui/app_paths.c', 'src/ui/frontend_execution.c', 'src/ui/replay_frontend.c', 'src/ui/netplay_frontend.c',
                 'src/ui/frontend_panels.c', 'src/ui/frontend_session.c', 'src/ui/platform_frontend.c', 'src/ui/platform_paths.c', 'src/ui/device_frontend.c', 'src/ui/storage_frontend.c', 'src/ui/device_panels.c',
                 'src/ui/settings.c', 'src/ui/settings_runtime.c', 'src/ui/game_database.c', 'src/ui/hd_pack_frontend.c', 'src/ui/idle_frontend.c',
                 'src/ui/image_open.c', 'src/ui/session_actions.c',
                 'src/ui/clay_backend.c', 'src/ui/font_atlas.c', 'src/ui/desktop_ppu.c', 'src/ui/desktop_layout.c', 'src/ui/desktop_events.c','src/ui/desktop_windows.c','src/ui/desktop_settings_model.c','src/ui/desktop_menus.c', 'src/third_party/clay/clay.c', 'src/third_party/stb/stb_truetype.c', 'src/ui/desktop_ui.c', 'src/ui/desktop_idle.c', 'src/ui/desktop_features.c', 'src/ui/state_frontend.c', 'src/ui/state_runtime.c',
                 'src/ui/debug_frontend.c', 'src/ui/output_guard.c', 'src/ui/host_input.c', 'src/ui/peripheral_input.c', 'src/ui/cheat_frontend.c',
                 'src/ui/video_runtime.c', 'src/ui/audio_runtime.c')
$coreSources += @('src/system/execution_policy.c', 'src/replay/rewind.c', 'src/video/frame_snapshot.c',
                  'src/replay/input_event.c', 'src/replay/movie.c',
                  'src/replay/netplay.c', 'src/replay/netplay_hash.c', 'src/replay/netplay_transport.c',
                  'src/audio/audio_observer.c', 'src/audio/audio_mix.c', 'src/video/presentation.c',
                  'src/ui/nsf_player.c', 'src/ui/nsf_player_ui.c',
                  'src/ui/nsf_player_runtime.c',
                  'src/capture/capture_writer.c', 'src/capture/capture_png.c', 'src/capture/capture_session.c',
                  'src/ui/capture_frontend.c', 'src/ui/capture_runtime.c',
                  'src/media/patch.c', 'src/media/patch_create.c', 'src/media/image_source.c',
                  'src/media/archive_common.c', 'src/media/archive_zip.c', 'src/media/archive_7z.c',
                  'src/third_party/miniz/miniz.c', 'src/third_party/spng/spng.c', 'src/third_party/lzma/7zArcIn.c',
                  'src/third_party/lzma/7zBuf.c', 'src/third_party/lzma/7zBuf2.c',
                  'src/third_party/lzma/7zCrc.c', 'src/third_party/lzma/7zCrcOpt.c',
                  'src/third_party/lzma/7zDec.c', 'src/third_party/lzma/7zStream.c',
                  'src/third_party/lzma/Bcj2.c', 'src/third_party/lzma/Bra.c',
                  'src/third_party/lzma/Bra86.c', 'src/third_party/lzma/BraIA64.c',
                  'src/third_party/lzma/CpuArch.c', 'src/third_party/lzma/Delta.c',
                  'src/third_party/lzma/Lzma2Dec.c', 'src/third_party/lzma/LzmaDec.c',
                  'src/third_party/lzma/Ppmd7.c', 'src/third_party/lzma/Ppmd7Dec.c')
$coreSources += @('src/third_party/lua/lapi.c', 'src/third_party/lua/lauxlib.c', 'src/third_party/lua/lbaselib.c',
                  'src/third_party/lua/lcode.c', 'src/third_party/lua/lcorolib.c', 'src/third_party/lua/lctype.c',
                  'src/third_party/lua/ldebug.c', 'src/third_party/lua/ldo.c', 'src/third_party/lua/ldump.c',
                  'src/third_party/lua/lfunc.c', 'src/third_party/lua/lgc.c', 'src/third_party/lua/llex.c',
                  'src/third_party/lua/lmathlib.c', 'src/third_party/lua/lmem.c', 'src/third_party/lua/lobject.c',
                  'src/third_party/lua/lopcodes.c', 'src/third_party/lua/lparser.c', 'src/third_party/lua/lstate.c',
                  'src/third_party/lua/lstring.c', 'src/third_party/lua/lstrlib.c', 'src/third_party/lua/ltable.c',
                  'src/third_party/lua/ltablib.c', 'src/third_party/lua/ltm.c', 'src/third_party/lua/lundump.c',
                  'src/third_party/lua/lutf8lib.c', 'src/third_party/lua/lvm.c', 'src/third_party/lua/lzio.c')
$cppSources = @('src/video/pixel_filter.cpp', 'src/apu/epsm.cpp', 'src/third_party/ymfm/ymfm_opn.cpp',
                'src/third_party/ymfm/ymfm_ssg.cpp', 'src/third_party/ymfm/ymfm_adpcm.cpp',
                'src/rom/game_db.cpp', 'src/rom/boards/runtime.cpp', 'src/rom/boards/factory.cpp',
                'src/rom/boards/state.cpp', 'src/hd/hd_assets.cpp', 'src/hd/hd_pack_loader.cpp',
                'src/hd/hd_conditions.cpp', 'src/hd/hd_renderer.cpp', 'src/hd/hd_runtime.cpp',
                'src/third_party/stb/stb_vorbis.cpp')
$cppSources += @('src/video/pixel_scalers.cpp', 'src/third_party/xbrz/xbrz.cpp',
                 'src/third_party/hqx/hq2x.cpp', 'src/third_party/hqx/hq3x.cpp',
                 'src/third_party/hqx/hq4x.cpp', 'src/third_party/hqx/init.cpp',
                 'src/third_party/scale2x/scale2x.cpp', 'src/third_party/scale2x/scale3x.cpp',
                 'src/third_party/sai/2xSai.cpp', 'src/third_party/sai/Super2xSai.cpp', 'src/third_party/sai/SuperEagle.cpp')
$coreSources += @('src/util/md5.c', 'src/replay/tas_startup.c', 'src/replay/tas_session.c',
                  'src/replay/tas_timeline.c', 'src/replay/tas_input.c', 'src/replay/tas_state.c', 'src/ui/tas_frontend.c')
$coreSources += @('src/replay/fm2.c', 'src/replay/fm2_parse.c', 'src/replay/fm2_write.c', 'src/replay/tas_project.c',
                  'src/replay/tas_project_edit.c', 'src/replay/tas_project_meta.c', 'src/ui/desktop_tas.c')
$testSources = @('src/tests/frontend_benchmark.c', 'src/tests/netplay_accuracy.c', 'src/tests/accuracy_test.c', 'src/tests/cpu_accuracy.c', 'src/tests/cpu_trace.c',
                 'src/tests/apu_accuracy.c', 'src/tests/ppu_accuracy.c', 'src/tests/mapper_accuracy.c',
                 'src/tests/region_accuracy.c', 'src/tests/file_io_accuracy.c', 'src/tests/persistence_accuracy.c',
                 'src/tests/patch_accuracy.c', 'src/tests/media_accuracy.c', 'src/tests/fds_options_accuracy.c',
                 'src/tests/fds_automation_accuracy.c', 'src/tests/execution_policy_accuracy.c',
                 'src/tests/nsf_player_accuracy.c', 'src/tests/capture_container_accuracy.c',
                 'src/tests/capture_session_accuracy.c', 'src/tests/video_trace_accuracy.c',
                 'src/tests/video_presentation_accuracy.c', 'src/tests/audio_mix_accuracy.c',
                 'src/tests/native_flash_geometry_accuracy.c', 'src/tests/mapper30_111_prg_ram_accuracy.c',
                 'src/tests/fds_accuracy.c',
                 'src/tests/studybox_accuracy.c', 'src/tests/nsf_accuracy.c',
                 'src/tests/rom_runner.c', 'src/tests/input_accuracy.c', 'src/tests/bandai_accuracy.c',
                 'src/tests/vs_accuracy.c', 'src/tests/epsm_accuracy.c', 'src/tests/board_accuracy.c',
                 'src/tests/board_codemasters_accuracy.c', 'src/tests/board_magic_floor_accuracy.c',
                 'src/tests/board_jaleco_accuracy.c', 'src/tests/board_nsf_cart_accuracy.c',
                 'src/tests/board_ffe_accuracy.c', 'src/tests/board_farid_accuracy.c',
                 'src/tests/board_sealie_accuracy.c', 'src/tests/board_ntdec_accuracy.c',
                 'src/tests/board_racermate_accuracy.c', 'src/tests/board_taito_accuracy.c',
                 'src/tests/board_sachen_accuracy.c', 'src/tests/board_kaiser_accuracy.c',
                 'src/tests/board_mmc3_accuracy.c', 'src/tests/board_mmc3_mixed_chr_accuracy.c',
                 'src/tests/board_sachen_late_accuracy.c',
                 'src/tests/board_jy_small_accuracy.c', 'src/tests/board_drip_accuracy.c',
                 'src/tests/board_mmc3_96_accuracy.c', 'src/tests/board_rainbow_accuracy.c',
                 'src/tests/board_mmc3_97_accuracy.c', 'src/tests/board_mmc3_98_accuracy.c',
                 'src/tests/board_unlicensed_109_accuracy.c', 'src/tests/board_unlicensed_111_accuracy.c',
                 'src/tests/board_unlicensed_112_accuracy.c', 'src/tests/board_txc_107_accuracy.c',
                 'src/tests/board_unlicensed_113_accuracy.c', 'src/tests/board_unlicensed_114_accuracy.c',
                 'src/tests/board_unlicensed_110_accuracy.c', 'src/tests/board_unlicensed_115_accuracy.c',
                 'src/tests/rom_database_defaults_accuracy.c', 'src/tests/game_database_discovery_accuracy.c', 'src/tests/unif_accuracy.c',
                 'src/tests/board_waixing_116_accuracy.c', 'src/tests/board_whirlwind_117_accuracy.c',
                 'src/tests/mmc5_extended_geometry_accuracy.c', 'src/tests/native_ram_accuracy.c',
                 'src/tests/board_irem77_accuracy.c', 'src/tests/default_prg_ram_geometry_accuracy.c',
                 'src/tests/native_chr_capacity_accuracy.c', 'src/tests/native_mixed_chr_accuracy.c',
                 'src/tests/board_nina_fme7_accuracy.c', 'src/tests/board_state_accuracy.c', 'src/tests/state_accuracy.c',
                 'src/tests/state_ui_accuracy.c',
                 'src/tests/debugger_accuracy.c', 'src/tests/ppu_inspector_accuracy.c',
                 'src/tests/cheat_accuracy.c', 'src/tests/rewind_accuracy.c', 'src/tests/movie_accuracy.c',
                 'src/tests/movie_frontend_accuracy.c',
                 'src/tests/frontend_accuracy.c', 'src/tests/desktop_accuracy.c')
$cppTestSources = @('src/tests/hd_pack_accuracy.cpp', 'src/tests/hd_renderer_accuracy.cpp', 'src/tests/hd_runtime_accuracy.cpp')
$testSources += @('src/tests/tas_session_accuracy.c', 'src/tests/movie_runner.c')
$coreSources += @('src/ui/desktop_tas_edit.c', 'src/ui/desktop_tas_layout.c')
$coreSources += @('src/replay/tas_project_io.c', 'src/replay/tas_script.c')
$coreSources += @('src/replay/tas_history.c')
$coreSources += @('src/replay/tas_navigation.c', 'src/ui/desktop_tas_navigation.c')
$coreSources += @('src/replay/tas_splice.c', 'src/replay/tas_splice_load.c')
$coreSources += @('src/ui/desktop_tas_splice.c', 'src/ui/desktop_tas_splice_layout.c')
$coreSources += @('src/replay/tas_project_fm3.c', 'src/replay/tas_project_fm3_read.c')
$testSources += @('src/tests/tas_editor_input_accuracy.c')
$testSources += @('src/tests/fm2_accuracy.c')
$testSources += @('src/tests/tas_project_accuracy.c', 'src/tests/tas_script_accuracy.c')
$testSources += @('src/tests/tas_history_accuracy.c')
$testSources += @('src/tests/tas_navigation_accuracy.c')
$testSources += @('src/tests/tas_splice_accuracy.c')
$testSources += @('src/tests/video_runtime_accuracy.c', 'src/tests/pixel_filter_accuracy.c')
$coreSources += @('src/video/ntsc_settings.c')
$coreSources += @('src/replay/fcm.c', 'src/ui/fcm_frontend.c')
$coreSources += @('src/ui/desktop_keyboard.c')
$coreSources += @('src/ui/settings_core.c')
$coreSources += @('src/ui/presentation_host.c')
$coreSources += @('src/cheats/cheat_database.c', 'src/ui/cheat_database_frontend.c')
$coreSources += @('src/media/header_editor.c', 'src/ui/header_editor_frontend.c')
$coreSources += @('src/capture/capture_codec.c')
$coreSources += @('src/capture/capture_gif.c', 'src/capture/capture_riff.c', 'src/capture/capture_overlay.c',
                  'src/capture/movie_subtitles.c', 'src/capture/movie_backup.c', 'src/capture/movie_preferences.c')
$coreSources += @('src/debugger/debug_analysis.c', 'src/debugger/debug_capture.c', 'src/debugger/debug_catalog.c')
$coreSources += @('src/ui/debug_tools_frontend.c')
$testSources += @('src/tests/fcm_accuracy.c')
$coreSources += @('src/cheats/game_genie.c', 'src/ui/game_genie_frontend.c')
$coreSources += @('src/debugger/memory_view.c', 'src/debugger/memory_search.c',
                   'src/ui/memory_search_frontend.c', 'src/ui/desktop_memory.c')
$coreSources += @('src/debugger/expression.c', 'src/debugger/memory_watch.c')
$coreSources += @('src/debugger/memory_editor.c')
$coreSources += @('src/ui/hex_frontend.c', 'src/ui/desktop_hex.c')
$testSources += @('src/tests/memory_editor_accuracy.c', 'src/tests/hex_frontend_accuracy.c')
$coreSources += @('src/debugger/assembler.c', 'src/ui/assembler_frontend.c')
$testSources += @('src/tests/assembler_accuracy.c')
$testSources += @('src/tests/ntsc_settings_accuracy.c')
$testSources += @('src/tests/game_genie_accuracy.c')
$testSources += @('src/tests/memory_search_accuracy.c', 'src/tests/memory_tools_accuracy.c')
$testSources += @('src/tests/memory_watch_accuracy.c', 'src/tests/watch_frontend_accuracy.c')
$coreSources += @('src/ui/watch_frontend.c')
$application = Join-Path $outputDirectory 'cupid-nes.exe'
$coreSources += @('src/ui/capture_tools.c', 'src/ui/recovery_store.c', 'src/ui/state_recorder.c',
                  'src/ui/lifecycle_frontend.c', 'src/ui/game_config.c', 'src/ui/cli_options.c',
                  'src/ui/cli_parse.c', 'src/ui/cli_help.c', 'src/ui/update_metadata.c',
                  'src/ui/update_transport.c', 'src/ui/update_checker.c',
                  'src/video/frame_timing.c', 'src/video/history_view.c', 'src/ui/timing_frontend.c',
                  'src/ui/history_frontend.c', 'src/ui/presentation_tools.c')
$coreSources += @('src/ui/overclock_frontend.c')
$cppSources += @('src/hd/hd_builder.cpp', 'src/video/shader_parser.cpp', 'src/video/shader_preset.cpp')
$testSources += @('src/tests/capture_extensions_accuracy.c', 'src/tests/frame_timing_accuracy.c')
$testSources += @('src/tests/capture_frontend_extensions_accuracy.c')
$testSources += @('src/tests/keyboard_accuracy.c')
$testSources += @('src/tests/cheat_database_accuracy.c')
$testSources += @('src/tests/lifecycle_accuracy.c', 'src/tests/update_cli_accuracy.c',
                 'src/tests/debug_tools_accuracy.c', 'src/tests/presentation_frontend_accuracy.c')
$testSources += @('src/tests/header_editor_accuracy.c')
$testSources += @('src/tests/overclock_accuracy.c', 'src/tests/desktop_menu_accuracy.c')
$testSources += @('src/tests/presentation_history_accuracy.c', 'src/tests/presentation_audio_accuracy.c')
$cppTestSources += @('src/tests/hd_builder_accuracy.cpp', 'src/tests/shader_preset_accuracy.cpp')
$testProgram = Join-Path $outputDirectory 'accuracy-tests.exe'
$objectDirectory = Join-Path $outputDirectory 'objects'
New-Item -ItemType Directory -Force -Path $objectDirectory | Out-Null

function Compile-Source([string]$Source, [string]$Driver, [string[]]$BuildFlags) {
    $object = Join-Path $objectDirectory ($Source.Replace('/', '_') + '.obj')
    & $Driver @BuildFlags '-c' $Source '-o' $object
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $Source" }
    return $object
}

function Link-Program([string[]]$Objects, [string]$Output) {
    # Object paths can exceed the Windows command-line limit in long checkouts.
    $responseFile = "$Output.rsp"
    $linkArguments = @($cppFlags) + @($Objects) + @($sdkLibrary, '-lshell32', '-lcomdlg32',
                                                  '-lws2_32', '-lole32', '-lwinhttp', '-o', $Output)
    $responseLines = foreach ($argument in $linkArguments) {
        '"' + $argument.Replace('\', '/').Replace('"', '\"') + '"'
    }
    [System.IO.File]::WriteAllLines($responseFile, [string[]]$responseLines,
                                   [System.Text.UTF8Encoding]::new($false))
    & $CxxCompiler ("@" + $responseFile)
    if ($LASTEXITCODE -ne 0) { throw "Link failed: $Output" }
}

Push-Location $projectRoot
try {
    $coreObjects = @(
        foreach ($source in $coreSources) { Compile-Source $source $Compiler $flags }
        foreach ($source in $cppSources) { Compile-Source $source $CxxCompiler $cppFlags }
    )
    $mainObject = Compile-Source 'src/main.c' $Compiler $flags
    $testObjects = @(
        foreach ($source in $testSources) { Compile-Source $source $Compiler $flags }
        foreach ($source in $cppTestSources) { Compile-Source $source $CxxCompiler $cppFlags }
    )
    Link-Program ($coreObjects + @($mainObject)) $application
    Link-Program ($coreObjects + $testObjects) $testProgram
    & $testProgram
    if ($LASTEXITCODE -ne 0) { throw 'Hardware regressions failed' }
    & python (Join-Path $projectRoot 'scripts/check-region-cli.py') $application
    if ($LASTEXITCODE -ne 0) { throw 'Region CLI regressions failed' }
    & python (Join-Path $projectRoot 'scripts/check-unicode-cli.py') $application
    if ($LASTEXITCODE -ne 0) { throw 'Unicode CLI regressions failed' }
    & python (Join-Path $projectRoot 'scripts/check-netplay.py') $testProgram
    if ($LASTEXITCODE -ne 0) { throw 'Netplay regressions failed' }
    Write-Output "Emulator: $application"
    Write-Output "Diagnostic runner: $testProgram"
} finally {
    Pop-Location
}
