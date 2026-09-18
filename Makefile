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
           src/rom/fds.c \
           src/rom/vrc7_audio.c src/rom/emu2413.c \
           src/rom/eeprom.c src/rom/namco163.c src/rom/sunsoft5b.c \
           src/joypad/joypad.c src/joypad/family_basic.c src/joypad/special_peripherals.c \
           src/apu/apu.c src/third_party/blip_buf.c src/video/ntsc_composite.c src/ui/palette_tool.c
CORE_CXX_SRC = src/apu/epsm.cpp src/third_party/ymfm/ymfm_opn.cpp \
               src/third_party/ymfm/ymfm_ssg.cpp src/third_party/ymfm/ymfm_adpcm.cpp \
               src/rom/boards/runtime.cpp src/rom/boards/factory.cpp
TEST_SRC = src/tests/accuracy_test.c src/tests/cpu_accuracy.c \
           src/tests/cpu_trace.c src/tests/apu_accuracy.c \
           src/tests/ppu_accuracy.c src/tests/mapper_accuracy.c \
           src/tests/fds_accuracy.c \
           src/tests/rom_runner.c src/tests/input_accuracy.c src/tests/bandai_accuracy.c \
           src/tests/vs_accuracy.c src/tests/epsm_accuracy.c src/tests/board_accuracy.c \
           src/tests/board_codemasters_accuracy.c src/tests/board_magic_floor_accuracy.c \
           src/tests/board_jaleco_accuracy.c src/tests/board_nsf_cart_accuracy.c \
           src/tests/board_ffe_accuracy.c src/tests/board_farid_accuracy.c
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
