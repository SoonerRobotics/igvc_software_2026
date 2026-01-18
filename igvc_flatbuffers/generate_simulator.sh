#!/bin/bash
set -e

# Output directories
CSHARP_DIR="/home/dylan/Projects/scr_simulator/Assets/Scripts"
CSHARP_NAMESPACE="Messages"

FBS_FILES=(./src/**/*.fbs)

mkdir -p "$CSHARP_DIR"

TMP_DIR="./src/tmp_flatbuffers"
mkdir -p "$TMP_DIR"

# Convert dot.separated.identifiers to PascalCase.Segments
pascal_dot_namespace() {
    awk '
    /^namespace[[:space:]]+/ {
        ns=$2
        sub(/;$/, "", ns)

        n=split(ns, parts, ".")
        for (i=1; i<=n; i++) {
            part=tolower(parts[i])
            parts[i]=toupper(substr(part,1,1)) substr(part,2)
        }

        printf "namespace "
        for (i=1; i<=n; i++) {
            printf "%s%s", parts[i], (i<n?".":"")
        }
        print ";"
        next
    }
    { print }
    '
}

for FBS_FILE in "${FBS_FILES[@]}"; do
    FBS_CONTENT=$(<"$FBS_FILE")

    CSHARP_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$CSHARP_NAMESPACE}
    CSHARP_FBS_CONTENT=$(echo "$CSHARP_FBS_CONTENT" | pascal_dot_namespace)

    BASE="$(basename "${FBS_FILE%.fbs}")"
    CSHARP_TMP="$TMP_DIR/${BASE}_csharp.fbs"

    echo "$CSHARP_FBS_CONTENT" > "$CSHARP_TMP"

    flatc --csharp -o "$CSHARP_DIR" "$CSHARP_TMP"
done

rm -rf "$TMP_DIR"
