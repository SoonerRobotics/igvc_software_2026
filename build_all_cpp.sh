#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for dir in igvc_cameras igvc_vectornav igvc_zed; do
	echo "Building ${dir}..."
	(
		cd "${SCRIPT_DIR}/${dir}"
		./build.sh
	)
done

echo "All builds completed."
