CXX = g++
CXXFLAGS_DEBUG = -Wall -Wextra -std=c++17 -Iinclude -g
CXXFLAGS_RELEASE = -Wall -Wextra -std=c++17 -Iinclude -O3 -DNDEBUG
LIBS = -lz -lbz2 -lexpat

SRC_DIR = src
SRC = $(wildcard $(SRC_DIR)/*.cpp)

OBJ_DIR_DEBUG = build/debug
OBJ_DIR_RELEASE = build/release

OBJ_DEBUG = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR_DEBUG)/%.o,$(SRC))
OBJ_RELEASE = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR_RELEASE)/%.o,$(SRC))

TARGET_DEBUG = osm_debug
TARGET_RELEASE = osm_release

# Default target
all: debug

# Debug build
debug: CXXFLAGS = $(CXXFLAGS_DEBUG)
debug: OBJ_DIR = $(OBJ_DIR_DEBUG)
debug: OBJ = $(OBJ_DEBUG)
debug: $(TARGET_DEBUG)

$(TARGET_DEBUG): $(OBJ_DEBUG)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Release build
release: CXXFLAGS = $(CXXFLAGS_RELEASE)
release: OBJ_DIR = $(OBJ_DIR_RELEASE)
release: OBJ = $(OBJ_RELEASE)
release: $(TARGET_RELEASE)

$(TARGET_RELEASE): $(OBJ_RELEASE)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compile rule for debug objects
$(OBJ_DIR_DEBUG)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR_DEBUG)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile rule for release objects
$(OBJ_DIR_RELEASE)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR_RELEASE)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build $(TARGET_DEBUG) $(TARGET_RELEASE)

.PHONY: all clean debug release
