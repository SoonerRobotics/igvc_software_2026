#!/bin/bash
set -e

# Namespace for C# flatbuffers
CS_NAMESPACE="SUS.FlatBuffers"

# Get all .fbs files in the ./src directory
FBS_FILES=(./src/*.fbs)

if [ ${#FBS_FILES[@]} -eq 0 ]; then
    echo "No .fbs files found in ./src"
    exit 1
fi

# Ask user where to place generated C# files
echo "Specify the output directory for generated C# FlatBuffer files."
echo "For WSL users, this may look like: /mnt/c/Users/{user}/Desktop/scr_simulator/Assets/Scripts"
read -rp "Directory: " CS_DIR

if [ -z "$CS_DIR" ]; then
    echo "Output directory cannot be empty"
    exit 1
fi

# Create the output directory if it does not exist
mkdir -p "$CS_DIR"

# Create a tmp directory for intermediate files
TMP_DIR="./src/tmp_flatbuffers"
mkdir -p "$TMP_DIR"

# Loop through each .fbs file and generate C# code
for FBS_FILE in "${FBS_FILES[@]}"; do
    # Load the file and replace {NAMESPACE_PLACEHOLDER}
    FBS_CONTENT=$(<"$FBS_FILE")
    CS_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$CS_NAMESPACE}

    # Write modified schema to temp file
    CS_TMP_FILE="$TMP_DIR/$(basename "${FBS_FILE%.fbs}")_cs.fbs"
    echo "$CS_FBS_CONTENT" > "$CS_TMP_FILE"

    # Generate C# code
    flatc --csharp -o "$CS_DIR" "$CS_TMP_FILE"
done

# Clean up temporary files
rm -rf "$TMP_DIR"

echo "C# FlatBuffer files generated in: $CS_DIR"
