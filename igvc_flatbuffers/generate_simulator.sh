#!/bin/bash
set -e

# Output directories
CSHARP_DIR="/mnt/c/Users/dylan/Desktop/scr_simulator/Assets/Scripts"

# Namespaces
CSHARP_NAMESPACE="Messages"

# FlatBuffer schema files
FBS_FILES=(./src/*.fbs)

# Create output directories
mkdir -p "$CSHARP_DIR"

# Temporary directory
TMP_DIR="./src/tmp_flatbuffers"
mkdir -p "$TMP_DIR"

# Convert first letter to uppercase
pascal() { echo "${1^}"; }

# Generate code for each schema
for FBS_FILE in "${FBS_FILES[@]}"; do
    FBS_CONTENT=$(<"$FBS_FILE")

    CSHARP_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$CSHARP_NAMESPACE}

    BASE="$(basename "${FBS_FILE%.fbs}")"

    CSHARP_TMP="$TMP_DIR/${BASE}_csharp.fbs"

    echo "$CSHARP_FBS_CONTENT" > "$CSHARP_TMP"

    flatc --csharp -o "$CSHARP_DIR" "$CSHARP_TMP"
done

# Root of generated C# files
CSHARP_ROOT="$CSHARP_DIR/$CSHARP_NAMESPACE"

# Normalize folder casing and merge case-insensitive duplicates
find "$CSHARP_ROOT" -depth -type d | while read -r DIR; do
    PARENT="$(dirname "$DIR")"
    BASE="$(basename "$DIR")"
    CANON="$(pascal "$BASE")"
    TARGET="$PARENT/$CANON"

    if [[ "$DIR" != "$TARGET" ]]; then
        if [[ -d "$TARGET" ]]; then
            mv "$DIR"/* "$TARGET"/ 2>/dev/null || true
            rmdir "$DIR" 2>/dev/null || true
        else
            mv "$DIR" "$TARGET"
        fi
    fi
done

# Rewrite C# namespace declarations to match folders
find "$CSHARP_ROOT" -type f -name "*.cs" | while read -r FILE; do
    DIR_PATH="$(dirname "$FILE")"
    REL_PATH="${DIR_PATH#$CSHARP_DIR/}"

    IFS='/' read -ra PARTS <<< "$REL_PATH"
    CANON_PARTS=()

    for PART in "${PARTS[@]}"; do
        CANON_PARTS+=("$(pascal "$PART")")
    done

    NEW_NAMESPACE="$(IFS=.; echo "${CANON_PARTS[*]}")"

    sed -i.bak -E "s/^namespace[[:space:]]+.*/namespace $NEW_NAMESPACE/" "$FILE"
    rm "$FILE.bak"
done

# Fix lingering lowercase namespace references
find "$CSHARP_ROOT" -type f -name "*.cs" -exec \
    sed -i.bak 's/\bMessages\.arc\b/Messages.Arc/g' {} +

# Remove sed backup files
find "$CSHARP_ROOT" -type f -name "*.bak" -delete

# Remove temporary files
rm -rf "$TMP_DIR"
