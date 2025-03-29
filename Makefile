# Compiler and flags
CC      = gcc
CFLAGS  = -Wall -Wextra -O2

# Directories and files
SRCDIR  = src
CPUDIR  = $(SRCDIR)/cpu
TARGET  = cupid-nes

# Find all C source files in the cpu folder
SRC     = $(wildcard $(CPUDIR)/*.c) $(SRCDIR)/tests/*.c
OBJ     = $(SRC:.c=.o)

# Default target builds the executable
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ)

# Compile .c files into .o files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
