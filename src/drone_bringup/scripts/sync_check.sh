#!/usr/bin/env bash
set -uo pipefail
CONTAINER="${1:-drone-slam-dev}"
cd "$(dirname "$0")/../../.." || exit 1

files=$(find src models worlds -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.py' \
  -o -name '*.sh' -o -name '*.sdf' -o -name '*.yaml' -o -name '*.rviz' -o -name '*.xml' \
  -o -name '*.txt' -o -name '*.config' \) | sort)

host_sums() { echo "$files" | while read -r f; do echo "$(md5 -q "$f") $f"; done; }
cont_sums() {
  echo "$files" | docker exec -i "$CONTAINER" bash -c \
    'cd /workspace && while read -r f; do echo "$(md5sum "$f" 2>/dev/null | cut -d" " -f1) $f"; done'
}

host=$(host_sums)
if [[ "$host" == "$(cont_sums)" ]]; then
  echo "Files in sync."
  exit 0
fi
echo "Container sees stale files, restarting $CONTAINER..."
docker restart "$CONTAINER" >/dev/null && sleep 2
if [[ "$host" == "$(cont_sums)" ]]; then
  echo "Files in sync."
else
  echo "Still out of sync. Try: Docker Desktop > Restart." >&2
  exit 1
fi
