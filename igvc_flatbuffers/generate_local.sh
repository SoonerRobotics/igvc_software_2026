#!/bin/bash
set -e

log() {
    printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$1" >&2
}

trap 'log "ERROR: script failed on line $LINENO"' ERR
trap 'log "Finished generate_local.sh"' EXIT

log "Starting generate_local.sh"

TS_DIR="../igvc_gui/src/lib/arc/messages"
CSHARP_DIR="../igvc_csharp/src"

TS_NAMESPACE="messages"
CSHARP_NAMESPACE="Messages"

FBS_FILES=(./src/*.fbs ./src/**/*.fbs)

log "Ensuring output directories exist: $TS_DIR, $CSHARP_DIR"
mkdir -p "$TS_DIR" "$CSHARP_DIR"

TMP_DIR="./src/tmp_flatbuffers"
log "Preparing temporary directory: $TMP_DIR"
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

if [ "${#FBS_FILES[@]}" -eq 0 ]; then
    log "No .fbs files found, exiting."
    exit 0
fi

for FBS_FILE in "${FBS_FILES[@]}"; do
    log "Processing $FBS_FILE"
    FBS_CONTENT=$(<"$FBS_FILE")

    TS_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$TS_NAMESPACE}
    CSHARP_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$CSHARP_NAMESPACE}
    CSHARP_FBS_CONTENT=$(echo "$CSHARP_FBS_CONTENT" | pascal_dot_namespace)

    BASE="$(basename "${FBS_FILE%.fbs}")"

    TS_TMP="$TMP_DIR/${BASE}_ts.fbs"
    CSHARP_TMP="$TMP_DIR/${BASE}_csharp.fbs"

    log "Writing temp files: $TS_TMP, $CSHARP_TMP"
    echo "$TS_FBS_CONTENT" > "$TS_TMP"
    echo "$CSHARP_FBS_CONTENT" > "$CSHARP_TMP"

    log "Running flatc for TypeScript -> $TS_DIR"
    flatc --ts     -o "$TS_DIR"     "$TS_TMP"

    log "Running flatc for C# -> $CSHARP_DIR"
    flatc --csharp -o "$CSHARP_DIR" "$CSHARP_TMP"
done

log "Removing temporary directory: $TMP_DIR"
rm -rf "$TMP_DIR"
