#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="/home/scr/igvc_software_2026"

cd "$PROJECT_DIR"

dotnet clean
dotnet build
dotnet run