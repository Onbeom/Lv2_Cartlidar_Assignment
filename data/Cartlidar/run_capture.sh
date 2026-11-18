#!/usr/bin/env bash
set -euo pipefail
capture_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$capture_dir/.capture-venv/bin/python" "$capture_dir/capture_realsense.py" "$@"
