#!/usr/bin/env bash
# Nav2 and docking share the same /burger1 base and TF tree.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /bin/bash "$SCRIPT_DIR/start_base.sh" "$@"
