#!/usr/bin/env bash
set -Eeuo pipefail

# Explicit installation helper. Diagnostics never call this script with --apply.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SOURCE_DIR="${SCRIPT_DIR}/ewellix/ewellix_lift"
PINNED_REVISION="eb41860fbdaefa5fa934551e54e65d384b81d59b"
PATCH_FILE="${SCRIPT_DIR}/patches/ewellix-communication-diagnostics-eb41860.patch"
MODE="check"

usage() {
  cat <<'EOF'
Usage: apply_ewellix_diagnostics_patch.sh [--check|--apply] [--source-dir PATH]

  --check           Verify revision and report ready/applied without changes (default).
  --apply           Apply the pinned patch, or succeed if already applied.
  --source-dir PATH Override the Ewellix checkout for isolated builds/tests.

This command does not build, deploy, start, stop or restart a driver.
EOF
}

fail() {
  printf 'EWELLIX_PATCH: status=error %s\n' "$*" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) MODE="check"; shift ;;
    --apply) MODE="apply"; shift ;;
    --source-dir)
      [[ $# -ge 2 && -n "$2" ]] || fail '--source-dir requires a path'
      SOURCE_DIR="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; fail "unknown argument: $1" ;;
  esac
done

[[ -f "$PATCH_FILE" ]] || fail "patch missing: $PATCH_FILE"
[[ -d "$SOURCE_DIR" ]] || fail "checkout missing: $SOURCE_DIR; initialize the submodule first"
SOURCE_DIR="$(cd -- "$SOURCE_DIR" && pwd -P)"
git_root="$(git -C "$SOURCE_DIR" rev-parse --show-toplevel 2>/dev/null)" || fail 'not a git checkout'
[[ "$git_root" == "$SOURCE_DIR" ]] || fail 'source directory must be the Ewellix repository root'
revision="$(git -C "$SOURCE_DIR" rev-parse HEAD)"
[[ "$revision" == "$PINNED_REVISION" ]] || fail "revision mismatch: expected=$PINNED_REVISION actual=$revision"

if git -C "$SOURCE_DIR" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
  printf 'EWELLIX_PATCH: status=applied revision=%s\n' "$revision"
  exit 0
fi

# Never overwrite local work. Unrelated changes in this checkout are permitted.
patch_paths=(
  ewellix_driver/CMakeLists.txt
  ewellix_driver/package.xml
  ewellix_driver/include/ewellix_driver/ewellix_node/communication_state.hpp
  ewellix_driver/include/ewellix_driver/ewellix_node/ewellix_node.hpp
  ewellix_driver/src/ewellix_node/ewellix_node.cpp
  ewellix_driver/src/ewellix_serial/ewellix_serial.cpp
  ewellix_driver/test/test_communication_state.cpp
  ewellix_driver/test/test_cycle2_data.cpp
)
changes="$(git -C "$SOURCE_DIR" status --porcelain --untracked-files=all -- "${patch_paths[@]}")"
[[ -z "$changes" ]] || fail 'patch target files have local changes; resolve them before installation'
git -C "$SOURCE_DIR" apply --check "$PATCH_FILE" || fail 'patch is incompatible with this checkout'

if [[ "$MODE" == "apply" ]]; then
  git -C "$SOURCE_DIR" apply "$PATCH_FILE"
  printf 'EWELLIX_PATCH: status=applied revision=%s\n' "$revision"
else
  printf 'EWELLIX_PATCH: status=ready revision=%s (use --apply to install)\n' "$revision"
fi
