#!/bin/bash
set -e

ZIP_URL="https://cdn.soonerrobotics.org/scr/vn.zip"
INSTALL_DIR="/usr/local/vectornav"
TMP_DIR="/tmp/vectornav_install"

# check if /usr/local/vectornav exists
if [ -d "$INSTALL_DIR" ]; then
    echo "$INSTALL_DIR already exists. Skipping installation."
    exit 0
fi

# create the directory
sudo mkdir -p "$INSTALL_DIR"

# clone the zip
mkdir -p "$TMP_DIR"
cd "$TMP_DIR"
wget "$ZIP_URL" -O vn.zip

# unzip the contents
unzip vn.zip -d vn_contents
cd vn_contents
sudo cp -r * "$INSTALL_DIR/"
cd ..

# cleanup
rm -rf "$TMP_DIR"