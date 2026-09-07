#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--wasd" ]]; then
    shift
    exec python3 "$(dirname -- "${BASH_SOURCE[0]}")/manual_drive_wasd.py" "$@"
fi

VTD_ROOT="${VTD_ROOT:-/home/autonav/VIRES/VTD.2025.2}"
DRIVER_TOOL="${VTD_ROOT}/Runtime/Tools/KeyboardMouseDriverControl/KeyboardMouseDriverControl-2025.2.438"
RDB_HOST="${VTD_RDB_HOST:-127.0.0.1}"
RDB_PORT="${VTD_RDB_PORT:-48190}"
INPUT_MODE="${VTD_INPUT_MODE:-window}"

if [[ ! -x "${DRIVER_TOOL}" ]]; then
    echo "[FAIL] VTD keyboard driver not found: ${DRIVER_TOOL}" >&2
    exit 1
fi

echo "VTD keyboard/mouse manual drive"
echo "  Mouse : steering / accelerator-brake"
echo "  D     : drive gear"
echo "  R     : reverse gear"
echo "  N     : neutral gear"
echo "  P     : park gear"
echo "  Q     : quit"
echo
echo "Switch gears only after the vehicle has stopped."

exec "${DRIVER_TOOL}" \
    --host "${RDB_HOST}" \
    -p "${RDB_PORT}" \
    --mode "${INPUT_MODE}" \
    --addOn \
    "$@"
