CXX = g++
CXXFLAGS_DEBUG = -Wall -Wextra -std=c++17 -Iinclude -g
CXXFLAGS_RELEASE = -Wall -Wextra -std=c++17 -Iinclude -O3 -DNDEBUG
CXXFLAGS_TEST = $(CXXFLAGS_DEBUG) -DTEST
LIBS = -lz -lbz2 -lexpat -lboost_serialization

SRC_DIR = src
SRC = $(wildcard $(SRC_DIR)/*.cpp)

OBJ_DIR_DEBUG = build/debug
OBJ_DIR_RELEASE = build/release
OBJ_DIR_TEST = build/test

OBJ_DEBUG = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR_DEBUG)/%.o,$(SRC))
OBJ_RELEASE = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR_RELEASE)/%.o,$(SRC))
OBJ_TEST = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR_TEST)/%.o,$(SRC))

TARGET_DEBUG = osm_debug
TARGET_RELEASE = osm_release
TARGET_TEST = osm_test

# Default target
all: debug

# Debug build
debug: CXXFLAGS = $(CXXFLAGS_DEBUG)
debug: OBJ_DIR = $(OBJ_DIR_DEBUG)
debug: OBJ = $(OBJ_DEBUG)
debug: $(TARGET_DEBUG)

$(TARGET_DEBUG): $(OBJ_DEBUG)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compile rule for debug objects
$(OBJ_DIR_DEBUG)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR_DEBUG)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Release build
release: CXXFLAGS = $(CXXFLAGS_RELEASE)
release: OBJ_DIR = $(OBJ_DIR_RELEASE)
release: OBJ = $(OBJ_RELEASE)
release: $(TARGET_RELEASE)

$(TARGET_RELEASE): $(OBJ_RELEASE)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compile rule for release objects
$(OBJ_DIR_RELEASE)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR_RELEASE)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Test build
test: $(TARGET_TEST)

$(TARGET_TEST): $(OBJ_TEST)
	$(CXX) $(CXXFLAGS_TEST) -o $@ $^ $(LIBS)

$(OBJ_DIR_TEST)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR_TEST)
	$(CXX) $(CXXFLAGS_TEST) -c $< -o $@

clean:
	rm -rf build $(TARGET_DEBUG) $(TARGET_RELEASE) $(TARGET_TEST)

.PHONY: all clean debug release test
