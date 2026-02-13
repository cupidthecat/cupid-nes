# Compiler and flags
CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -DPPU_DEBUG_LOG=1

LDFLAGS = -lSDL2 -lm

# Directories and files
CPUDIR    = src/cpu
PPUDIR    = src/ppu
ROMDIR    = src/rom
JOYPADDIR = src/joypad 
APUDIR    = src/apu
UIDIR     = src/ui
MAINDIR   = src
TARGET    = cupid-nes

# Source files
SRC     = $(wildcard $(CPUDIR)/*.c) \
          $(wildcard $(PPUDIR)/*.c) \
          $(wildcard $(ROMDIR)/*.c) \
          $(wildcard $(JOYPADDIR)/*.c) \
          $(wildcard $(UIDIR)/*.c) \
          $(MAINDIR)/main.c
OBJ = src/cpu/cpu.o \
      src/ppu/ppu.o \
      src/rom/rom.o \
      src/rom/mapper.o \
      src/joypad/joypad.o \
      src/apu/apu.o \
      src/ui/palette_tool.o \
      src/main.o


# Default target builds the executable
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compile .c files into .o files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
