CC = gcc
CFLAGS ?= -std=c11 -Wall -Wextra -O2
ifeq ($(origin CXX),default)
CXX = $(if $(findstring clang,$(CC)),clang++,g++)
endif
CXXFLAGS ?= $(filter-out -std=c%,$(CFLAGS)) -std=c++17
CPPFLAGS += -DSDL_MAIN_HANDLED -DZ7_PPMD_SUPPORT -DZ7_EXTRACT_ONLY -DMINIZ_NO_ZLIB_COMPATIBLE_NAMES
ifneq ($(OS),Windows_NT)
CPPFLAGS += -D_POSIX_C_SOURCE=200809L
endif
LDLIBS ?= -lSDL2 -lm

TARGET = cupid-nes
TEST_TARGET = build/accuracy-tests
CORE_SRC = src/system/timing.c src/system/hardware.c src/system/vs_system.c src/state/state.c src/state/state_io.c src/state/state_alloc.c src/cpu/cpu.c src/cpu/cpu_observer.c src/ppu/ppu.c src/rom/rom.c src/rom/mapper.c \
           src/rom/fds.c src/rom/nsf.c src/util/file_io.c src/util/sha1.c \
           src/debugger/debugger.c src/debugger/disassembly.c src/debugger/lua_runtime.c src/cheats/cheats.c \
           src/rom/vrc7_audio.c src/rom/emu2413.c \
           src/rom/eeprom.c src/rom/namco163.c src/rom/sunsoft5b.c \
           src/joypad/joypad.c src/joypad/family_basic.c src/joypad/special_peripherals.c \
           src/apu/apu.c src/third_party/blip_buf.c src/video/ntsc_composite.c src/video/video_trace.c src/ui/palette_tool.c src/ui/nsf_frontend.c \
           src/ui/frontend_commands.c src/ui/execution_control.c src/ui/machine_actions.c src/ui/app_paths.c \
           src/ui/frontend_execution.c src/ui/replay_frontend.c src/ui/netplay_frontend.c src/ui/frontend_panels.c src/ui/frontend_session.c \
           src/ui/platform_frontend.c src/ui/platform_paths.c src/ui/device_frontend.c src/ui/storage_frontend.c src/ui/device_panels.c src/ui/settings.c src/ui/settings_runtime.c src/ui/game_database.c src/ui/hd_pack_frontend.c \
           src/ui/idle_frontend.c src/ui/image_open.c src/ui/session_actions.c \
           src/ui/ui_font.c src/ui/desktop_ui.c src/ui/desktop_idle.c src/ui/desktop_features.c src/ui/state_frontend.c src/ui/state_runtime.c \
           src/ui/debug_frontend.c src/ui/output_guard.c src/ui/host_input.c src/ui/peripheral_input.c src/ui/cheat_frontend.c \
           src/ui/video_runtime.c src/ui/audio_runtime.c
CORE_SRC += src/system/execution_policy.c src/replay/rewind.c src/video/frame_snapshot.c \
            src/replay/input_event.c src/replay/movie.c \
            src/replay/netplay.c src/replay/netplay_hash.c src/replay/netplay_transport.c \
            src/audio/audio_observer.c src/audio/audio_mix.c src/video/presentation.c \
            src/ui/nsf_player.c src/ui/nsf_player_ui.c \
            src/ui/nsf_player_runtime.c \
            src/capture/capture_writer.c src/capture/capture_png.c src/capture/capture_session.c \
            src/ui/capture_frontend.c src/ui/capture_runtime.c \
            src/media/patch.c src/media/patch_create.c src/media/image_source.c \
            src/media/archive_common.c src/media/archive_zip.c src/media/archive_7z.c \
            src/third_party/miniz/miniz.c src/third_party/spng/spng.c src/third_party/lzma/7zArcIn.c \
            src/third_party/lzma/7zBuf.c src/third_party/lzma/7zBuf2.c \
            src/third_party/lzma/7zCrc.c src/third_party/lzma/7zCrcOpt.c \
            src/third_party/lzma/7zDec.c src/third_party/lzma/7zStream.c \
            src/third_party/lzma/Bcj2.c src/third_party/lzma/Bra.c \
            src/third_party/lzma/Bra86.c src/third_party/lzma/BraIA64.c \
            src/third_party/lzma/CpuArch.c src/third_party/lzma/Delta.c \
            src/third_party/lzma/Lzma2Dec.c src/third_party/lzma/LzmaDec.c \
            src/third_party/lzma/Ppmd7.c src/third_party/lzma/Ppmd7Dec.c
CORE_SRC += src/third_party/lua/lapi.c src/third_party/lua/lauxlib.c src/third_party/lua/lbaselib.c \
            src/third_party/lua/lcode.c src/third_party/lua/lcorolib.c src/third_party/lua/lctype.c \
            src/third_party/lua/ldebug.c src/third_party/lua/ldo.c src/third_party/lua/ldump.c \
            src/third_party/lua/lfunc.c src/third_party/lua/lgc.c src/third_party/lua/llex.c \
            src/third_party/lua/lmathlib.c src/third_party/lua/lmem.c src/third_party/lua/lobject.c \
            src/third_party/lua/lopcodes.c src/third_party/lua/lparser.c src/third_party/lua/lstate.c \
            src/third_party/lua/lstring.c src/third_party/lua/lstrlib.c src/third_party/lua/ltable.c \
            src/third_party/lua/ltablib.c src/third_party/lua/ltm.c src/third_party/lua/lundump.c \
            src/third_party/lua/lutf8lib.c src/third_party/lua/lvm.c src/third_party/lua/lzio.c
CORE_CXX_SRC = src/apu/epsm.cpp src/third_party/ymfm/ymfm_opn.cpp \
               src/third_party/ymfm/ymfm_ssg.cpp src/third_party/ymfm/ymfm_adpcm.cpp \
               src/rom/game_db.cpp src/rom/boards/runtime.cpp src/rom/boards/factory.cpp src/rom/boards/state.cpp \
               src/hd/hd_assets.cpp src/hd/hd_pack_loader.cpp src/hd/hd_conditions.cpp \
               src/hd/hd_renderer.cpp src/hd/hd_runtime.cpp src/third_party/stb/stb_vorbis.cpp
TEST_SRC = src/tests/netplay_accuracy.c src/tests/accuracy_test.c src/tests/cpu_accuracy.c \
           src/tests/cpu_trace.c src/tests/apu_accuracy.c src/tests/file_io_accuracy.c src/tests/persistence_accuracy.c \
           src/tests/ppu_accuracy.c src/tests/mapper_accuracy.c src/tests/region_accuracy.c \
           src/tests/native_flash_geometry_accuracy.c src/tests/mapper30_111_prg_ram_accuracy.c \
           src/tests/fds_accuracy.c src/tests/studybox_accuracy.c src/tests/nsf_accuracy.c \
           src/tests/rom_runner.c src/tests/input_accuracy.c src/tests/bandai_accuracy.c \
           src/tests/vs_accuracy.c src/tests/epsm_accuracy.c src/tests/board_accuracy.c \
           src/tests/board_codemasters_accuracy.c src/tests/board_magic_floor_accuracy.c \
           src/tests/board_jaleco_accuracy.c src/tests/board_nsf_cart_accuracy.c \
           src/tests/board_ffe_accuracy.c src/tests/board_farid_accuracy.c \
           src/tests/board_sealie_accuracy.c src/tests/board_ntdec_accuracy.c \
           src/tests/board_racermate_accuracy.c src/tests/board_taito_accuracy.c \
           src/tests/board_sachen_accuracy.c src/tests/board_kaiser_accuracy.c \
           src/tests/board_mmc3_accuracy.c src/tests/board_mmc3_mixed_chr_accuracy.c \
           src/tests/board_sachen_late_accuracy.c \
           src/tests/board_jy_small_accuracy.c src/tests/board_drip_accuracy.c \
           src/tests/board_mmc3_96_accuracy.c src/tests/board_rainbow_accuracy.c \
           src/tests/board_mmc3_97_accuracy.c src/tests/board_mmc3_98_accuracy.c \
           src/tests/board_unlicensed_109_accuracy.c src/tests/board_unlicensed_111_accuracy.c \
           src/tests/board_unlicensed_112_accuracy.c src/tests/board_txc_107_accuracy.c \
           src/tests/board_unlicensed_113_accuracy.c src/tests/board_unlicensed_114_accuracy.c \
           src/tests/board_unlicensed_110_accuracy.c src/tests/board_unlicensed_115_accuracy.c \
           src/tests/rom_database_defaults_accuracy.c src/tests/game_database_discovery_accuracy.c src/tests/unif_accuracy.c \
           src/tests/board_waixing_116_accuracy.c src/tests/board_whirlwind_117_accuracy.c \
           src/tests/mmc5_extended_geometry_accuracy.c src/tests/native_ram_accuracy.c \
           src/tests/board_irem77_accuracy.c src/tests/default_prg_ram_geometry_accuracy.c \
           src/tests/native_chr_capacity_accuracy.c src/tests/native_mixed_chr_accuracy.c \
           src/tests/board_nina_fme7_accuracy.c src/tests/board_state_accuracy.c src/tests/state_accuracy.c \
           src/tests/state_ui_accuracy.c \
           src/tests/debugger_accuracy.c src/tests/cheat_accuracy.c \
           src/tests/rewind_accuracy.c src/tests/movie_accuracy.c src/tests/movie_frontend_accuracy.c \
           src/tests/frontend_accuracy.c src/tests/desktop_accuracy.c
TEST_SRC += src/tests/patch_accuracy.c src/tests/media_accuracy.c src/tests/fds_options_accuracy.c \
            src/tests/fds_automation_accuracy.c src/tests/execution_policy_accuracy.c \
            src/tests/nsf_player_accuracy.c src/tests/capture_container_accuracy.c \
            src/tests/capture_session_accuracy.c src/tests/video_trace_accuracy.c \
            src/tests/video_presentation_accuracy.c src/tests/audio_mix_accuracy.c
TEST_CXX_SRC = src/tests/hd_pack_accuracy.cpp src/tests/hd_renderer_accuracy.cpp src/tests/hd_runtime_accuracy.cpp
CORE_OBJ = $(CORE_SRC:.c=.o) $(CORE_CXX_SRC:.cpp=.o)
TEST_OBJ = $(TEST_SRC:.c=.o) $(TEST_CXX_SRC:.cpp=.o)
OBJ = $(CORE_OBJ) $(TEST_OBJ) src/main.o

all: $(TARGET)

$(TARGET): $(CORE_OBJ) src/main.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(TEST_TARGET): $(CORE_OBJ) $(TEST_OBJ) | build
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

build:
	mkdir -p $@

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(OBJ) $(OBJ:.o=.d) $(TARGET) $(TEST_TARGET)

-include $(OBJ:.o=.d)

.PHONY: all test clean
