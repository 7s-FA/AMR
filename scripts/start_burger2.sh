#!/usr/bin/env bash
set -eo pipefail
AMR_REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 "$AMR_REPO/scripts/robot.py" --robot M2 host-ready "$@"
