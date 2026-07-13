#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-lab}"
JETSON_HOST="${JETSON_HOST:-192.168.12.202}"
REMOTE_DIR="${REMOTE_DIR:-/home/lab/object_Detection_shared}"
LOCAL_DIR="${LOCAL_DIR:-bags/jetson_$(date +%Y-%m-%d_%H-%M-%S)}"

usage() {
  cat <<'EOF'
Sync all ROS bag files from the Jetson AGX to this PC.

Defaults match README.md:
  Jetson SSH: lab@192.168.12.202
  Remote dir: /home/lab/object_Detection_shared

Usage:
  scripts/sync_jetson_bags.sh [options]

Options:
  --host HOST         Jetson hostname or IP. Default: 192.168.12.202
  --user USER         SSH user. Default: lab
  --remote-dir DIR    Directory on Jetson to scan. Default: /home/lab/object_Detection_shared
  --local-dir DIR     Local destination. Default: bags/jetson_<timestamp>
  --dry-run           Show what would be copied without copying files.
  --list              List remote .bag files and exit.
  -h, --help          Show this help.

Examples:
  scripts/sync_jetson_bags.sh
  scripts/sync_jetson_bags.sh --dry-run
  scripts/sync_jetson_bags.sh --remote-dir /home/lab/object_Detection_shared --local-dir bags/latest_jetson
EOF
}

DRY_RUN=0
LIST_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      JETSON_HOST="$2"
      shift 2
      ;;
    --user)
      JETSON_USER="$2"
      shift 2
      ;;
    --remote-dir)
      REMOTE_DIR="$2"
      shift 2
      ;;
    --local-dir)
      LOCAL_DIR="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --list)
      LIST_ONLY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

REMOTE="${JETSON_USER}@${JETSON_HOST}"

echo "Jetson: ${REMOTE}"
echo "Remote: ${REMOTE_DIR}"

if [[ "$LIST_ONLY" -eq 1 ]]; then
  ssh "$REMOTE" "find '$REMOTE_DIR' -type f -name '*.bag' -printf '%s bytes  %TY-%Tm-%Td %TH:%TM  %p\n' | sort"
  exit 0
fi

mkdir -p "$LOCAL_DIR"
echo "Local:  ${LOCAL_DIR}"

RSYNC_ARGS=(
  -avh
  --progress
  --partial
  --prune-empty-dirs
  --include='*/'
  --include='*.bag'
  --exclude='*'
)

if [[ "$DRY_RUN" -eq 1 ]]; then
  RSYNC_ARGS=(--dry-run "${RSYNC_ARGS[@]}")
fi

rsync "${RSYNC_ARGS[@]}" "${REMOTE}:${REMOTE_DIR}/" "${LOCAL_DIR}/"

echo
echo "Done. Local bag files:"
find "$LOCAL_DIR" -type f -name '*.bag' -printf '%s bytes  %p\n' | sort
