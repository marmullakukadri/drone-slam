#!/usr/bin/env bash
# Runs on macOS (not in the container). Docker Desktop's file sharing sometimes
# keeps serving an old copy of a file that was just edited on the Mac, which
# makes colcon build stale or truncated code. Compare checksums of the source
# files on both sides and restart the container if they differ.
# Usage: sync_check.sh [container]   (run from the project root)
set -uo pipefail
CONTAINER="${1:-drone-slam-dev}"
cd "$(dirname "$0")/../../.." || exit 1   # project root

files=$(find src models worlds -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.py' \
  -o -name '*.sh' -o -name '*.sdf' -o -name '*.yaml' -o -name '*.rviz' -o -name '*.xml' \
  -o -name '*.txt' -o -name '*.config' \) | sort)

host_sums() { echo "$files" | while read -r f; do echo "$(md5 -q "$f") $f"; done; }
cont_sums() {
  echo "$files" | docker exec -i "$CONTAINER" bash -c \
    'cd /workspace && while read -r f; do echo "$(md5sum "$f" 2>/dev/null | cut -d" " -f1) $f"; done'
}

host=$(host_sums)
cont=$(cont_sums)
if [[ "$host" == "$cont" ]]; then
  echo "Skedarët në kontejner janë të sinkronizuar."
  exit 0
fi
echo "Kontejneri sheh version të vjetër të këtyre skedarëve:"
diff <(echo "$cont") <(echo "$host") | grep '^>' | awk '{print "   " $3}'
echo "Po rinis kontejnerin $CONTAINER për t'i rifreskuar..."
docker restart "$CONTAINER" >/dev/null && sleep 2
if [[ "$host" == "$(cont_sums)" ]]; then
  echo "U sinkronizua."
else
  echo "KUJDES: ende jo i sinkronizuar. Provoni: Docker Desktop > Restart." >&2
  exit 1
fi
