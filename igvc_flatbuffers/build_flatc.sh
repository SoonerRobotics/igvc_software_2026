#!/bin/bash
set -e

# Hold the current directory
CURRENT_DIR=$(pwd)

# The current flatc version we want
FLATC_VERSION="25.2.10"
FLATC_GIT_REPO="https://github.com/google/flatbuffers.git"

# The installation directory for flatc
INSTALL_DIR="/usr/bin/flatc"

# Check if flatc is already installed and at the correct version
if command -v flatc &> /dev/null; then
    INSTALLED_VERSION=$(flatc --version | awk '{print $3}')
    if [ "$INSTALLED_VERSION" == "$FLATC_VERSION" ]; then
        echo "flatc version $FLATC_VERSION is already installed."
        exit 0
    else
        echo "Different version of flatc installed (found $INSTALLED_VERSION, need $FLATC_VERSION). Reinstalling..."
    fi
else
    echo "flatc not found. Installing version $FLATC_VERSION..."
fi

# Create a tmp directory for building flatc
TMP_DIR=$(mktemp -d)
echo "Building flatc in temporary directory: $TMP_DIR"
cd "$TMP_DIR"

# Clone the flatbuffers repository
git clone --depth 1 --branch "v${FLATC_VERSION}" "$FLATC_GIT_REPO"
cd flatbuffers

# Build flatc
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .
make -j$(nproc)

# Install flatc
sudo cp ./flatc "$INSTALL_DIR"

# Clean up
cd ..
rm -rf "$TMP_DIR"

# Return to the original directory
cd "$CURRENT_DIR"

echo "flatc version $FLATC_VERSION installed successfully at $INSTALL_DIR."