# Compiler and flags
CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -lSDL2

# Directories and files
CPUDIR    = src/cpu
PPUDIR    = src/ppu
ROMDIR    = src/rom
JOYPADDIR = src/joypad
MAINDIR   = src
TARGET    = cupid-nes

# Source files
SRC     = $(wildcard $(CPUDIR)/*.c) \
          $(wildcard $(PPUDIR)/*.c) \
          $(wildcard $(ROMDIR)/*.c) \
          $(wildcard $(JOYPADDIR)/*.c) \
          $(MAINDIR)/main.c
OBJ     = $(SRC:.c=.o)

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
