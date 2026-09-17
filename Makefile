CC = gcc
CFLAGS ?= -std=c11 -Wall -Wextra -O2
CPPFLAGS += -DSDL_MAIN_HANDLED
LDLIBS ?= -lSDL2 -lm

TARGET = cupid-nes
TEST_TARGET = build/accuracy-tests
CORE_SRC = src/system/timing.c src/system/hardware.c src/cpu/cpu.c src/ppu/ppu.c src/rom/rom.c src/rom/mapper.c \
           src/rom/eeprom.c src/joypad/joypad.c src/apu/apu.c src/ui/palette_tool.c
TEST_SRC = src/tests/accuracy_test.c src/tests/cpu_accuracy.c \
           src/tests/cpu_trace.c src/tests/apu_accuracy.c \
           src/tests/ppu_accuracy.c src/tests/mapper_accuracy.c \
           src/tests/rom_runner.c src/tests/input_accuracy.c src/tests/bandai_accuracy.c
CORE_OBJ = $(CORE_SRC:.c=.o)
TEST_OBJ = $(TEST_SRC:.c=.o)
OBJ = $(CORE_OBJ) $(TEST_OBJ) src/main.o

all: $(TARGET)

$(TARGET): $(CORE_OBJ) src/main.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(TEST_TARGET): $(CORE_OBJ) $(TEST_OBJ) | build
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

build:
	mkdir -p $@

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(OBJ) $(OBJ:.o=.d) $(TARGET) $(TEST_TARGET)

-include $(OBJ:.o=.d)

.PHONY: all test clean
