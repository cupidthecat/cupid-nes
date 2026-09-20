CC = gcc
CFLAGS ?= -std=c11 -Wall -Wextra -O2
ifeq ($(origin CXX),default)
CXX = $(if $(findstring clang,$(CC)),clang++,g++)
endif
CXXFLAGS ?= $(filter-out -std=c%,$(CFLAGS)) -std=c++17
CPPFLAGS += -DSDL_MAIN_HANDLED
LDLIBS ?= -lSDL2 -lm

TARGET = cupid-nes
TEST_TARGET = build/accuracy-tests
CORE_SRC = src/system/timing.c src/system/hardware.c src/system/vs_system.c src/cpu/cpu.c src/ppu/ppu.c src/rom/rom.c src/rom/mapper.c \
           src/rom/fds.c src/rom/nsf.c \
           src/rom/vrc7_audio.c src/rom/emu2413.c \
           src/rom/eeprom.c src/rom/namco163.c src/rom/sunsoft5b.c \
           src/joypad/joypad.c src/joypad/family_basic.c src/joypad/special_peripherals.c \
           src/apu/apu.c src/third_party/blip_buf.c src/video/ntsc_composite.c src/ui/palette_tool.c src/ui/nsf_frontend.c
CORE_CXX_SRC = src/apu/epsm.cpp src/third_party/ymfm/ymfm_opn.cpp \
               src/third_party/ymfm/ymfm_ssg.cpp src/third_party/ymfm/ymfm_adpcm.cpp \
               src/rom/game_db.cpp src/rom/boards/runtime.cpp src/rom/boards/factory.cpp
TEST_SRC = src/tests/accuracy_test.c src/tests/cpu_accuracy.c \
           src/tests/cpu_trace.c src/tests/apu_accuracy.c \
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
           src/tests/rom_database_defaults_accuracy.c src/tests/unif_accuracy.c \
           src/tests/board_waixing_116_accuracy.c src/tests/board_whirlwind_117_accuracy.c \
           src/tests/mmc5_extended_geometry_accuracy.c src/tests/native_ram_accuracy.c \
           src/tests/board_irem77_accuracy.c src/tests/default_prg_ram_geometry_accuracy.c \
           src/tests/native_chr_capacity_accuracy.c src/tests/native_mixed_chr_accuracy.c \
           src/tests/board_nina_fme7_accuracy.c
CORE_OBJ = $(CORE_SRC:.c=.o) $(CORE_CXX_SRC:.cpp=.o)
TEST_OBJ = $(TEST_SRC:.c=.o)
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
