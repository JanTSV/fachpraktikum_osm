# Compiler
CXX = g++

# Compiler and linker flags
CXXFLAGS = -Wall -Wextra -std=c++17 -Iinclude

# Source and build directories
SRC_DIR = src
INC_DIR = include
OBJ_DIR = build

# Find all .cpp files in src/
SRC = $(wildcard $(SRC_DIR)/*.cpp)

# Generate .o files in build/ with the same base names
OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))

# Target executable name
TARGET = main

# Default target
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files into object files in build/
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

.PHONY: all clean
