#!/bin/bash
set -e

# Directories for the Java flatbuffer objects and the Typescript flatbuffer objects
JAVA_DIR="../igvc/src/main/java"
TS_DIR="../igvc_ui/src/lib/flatbuffers"
CPP_DIR="../igvc_vectornav/include/igvc/flatbuffers"

# Namespaces for each language
JAVA_NAMESPACE="com.soonerrobotics.flatbuffers"
TS_NAMESPACE="flatbuffers"
CPP_NAMESPACE="igvc"

# Get all .fbs files in the ./src directory
FBS_FILES=(./src/*.fbs)

# Create the output directories if they do not exist
mkdir -p "$JAVA_DIR"
mkdir -p "$TS_DIR"
mkdir -p "$CPP_DIR"

# Create a tmp directory for intermediate files
TMP_DIR="./src/tmp_flatbuffers"
mkdir -p "$TMP_DIR"

# Loop through each .fbs file and generate Java and Typescript code
for FBS_FILE in "${FBS_FILES[@]}"; do
    # Load the file into a variable and replace {NAMESPACE_PLACEHOLDER} with the per language
    FBS_CONTENT=$(<"$FBS_FILE")
    JAVA_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$JAVA_NAMESPACE}
    TS_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$TS_NAMESPACE}
    CPP_FBS_CONTENT=${FBS_CONTENT//\{NAMESPACE_PLACEHOLDER\}/$CPP_NAMESPACE}

    # Write the modified content to temporary files
    JAVA_TMP_FILE="$TMP_DIR/$(basename "${FBS_FILE%.fbs}")_java.fbs"
    TS_TMP_FILE="$TMP_DIR/$(basename "${FBS_FILE%.fbs}")_ts.fbs"
    CPP_TMP_FILE="$TMP_DIR/$(basename "${FBS_FILE%.fbs}")_cpp.fbs"
    echo "$JAVA_FBS_CONTENT" > "$JAVA_TMP_FILE"
    echo "$TS_FBS_CONTENT" > "$TS_TMP_FILE"
    echo "$CPP_FBS_CONTENT" > "$CPP_TMP_FILE"

    # Generate Java code
    flatc --java -o "$JAVA_DIR" "$JAVA_TMP_FILE"

    # Generate Typescript code
    flatc --ts -o "$TS_DIR" "$TS_TMP_FILE"

    # Generate C++ code
    flatc --cpp -o "$CPP_DIR" "$CPP_TMP_FILE"
done

# Clean up temporary files
rm -rf "$TMP_DIR"