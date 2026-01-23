#!/usr/bin/env bash
set -euo pipefail

# ---- Config ----
REPO_URL="https://github.com/IntelRealSense/librealsense.git"
BRANCH="master"
WORKDIR="${WORKDIR:-$HOME/src}"
SRC_DIR="$WORKDIR/librealsense"
BUILD_DIR="$SRC_DIR/build"

INSTALL_PREFIX="${INSTALL_PREFIX:-/usr/local}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

BUILD_EXAMPLES="${BUILD_EXAMPLES:-ON}"
BUILD_GRAPHICAL_EXAMPLES="${BUILD_GRAPHICAL_EXAMPLES:-ON}"
BUILD_PYTHON_BINDINGS="${BUILD_PYTHON_BINDINGS:-OFF}"

# ---- Dependencies ----
echo "==> Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  libssl-dev \
  libusb-1.0-0-dev \
  libudev-dev

if [[ "$BUILD_GRAPHICAL_EXAMPLES" == "ON" ]]; then
  sudo apt-get install -y \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libgtk-3-dev
fi

# ---- Clone repo ----
mkdir -p "$WORKDIR"

if [[ -d "$SRC_DIR/.git" ]]; then
  echo "==> Updating librealsense..."
  git -C "$SRC_DIR" fetch --all --tags
  git -C "$SRC_DIR" checkout "$BRANCH"
  git -C "$SRC_DIR" pull --ff-only || true
else
  echo "==> Cloning librealsense..."
  git clone "$REPO_URL" "$SRC_DIR"
fi

# ---- UDEV RULES (FIXED FOR REAL) ----
echo "==> Installing RealSense udev rules..."

RULES_INSTALLED=0
RULES_DIR="/etc/udev/rules.d"

for rule in "$SRC_DIR"/config/*realsense*.rules; do
  if [[ -f "$rule" ]]; then
    echo "  Installing $(basename "$rule")"
    sudo cp "$rule" "$RULES_DIR/"
    RULES_INSTALLED=1
  fi
done

if [[ "$RULES_INSTALLED" -eq 1 ]]; then
  sudo udevadm control --reload-rules
  sudo udevadm trigger
else
  echo "WARNING: No RealSense udev rules found in config/"
fi

# ---- Build ----
echo "==> Configuring build..."
mkdir -p "$BUILD_DIR"

cmake -S "$SRC_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -DBUILD_EXAMPLES="$BUILD_EXAMPLES" \
  -DBUILD_GRAPHICAL_EXAMPLES="$BUILD_GRAPHICAL_EXAMPLES" \
  -DBUILD_PYTHON_BINDINGS="$BUILD_PYTHON_BINDINGS"

echo "==> Building..."
cmake --build "$BUILD_DIR" -j"$(nproc)"

# ---- Install ----
echo "==> Installing..."
sudo cmake --install "$BUILD_DIR"
sudo ldconfig

echo
echo "✅ librealsense installed successfully"
echo "Unplug/replug the camera (or reboot) to apply udev rules."
