# Build directory
BUILD_DIR := build
TARGET := igvc_vectornav_exec

# Default target
all: build

# Configure + build
build:
	mkdir -p $(BUILD_DIR)
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)

# Run the executable
run: build
	./$(BUILD_DIR)/$(TARGET)

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all build run clean
