#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="/home/scr/igvc_software_2026/igvc_csharp"
DOTNET_DIR="/home/scr/.dotnet/dotnet"

cd "$PROJECT_DIR"

# clean, build, run
"$DOTNET_DIR" clean
"$DOTNET_DIR" build -c Release
"$DOTNET_DIR" run -c Release --no-build