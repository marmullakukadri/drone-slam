#!/usr/bin/env bash
# Start a virtual desktop inside the container and expose it through noVNC.
# Open http://localhost:6080/vnc.html on macOS afterwards.
# Usage: start_vnc.sh [start|stop]   (default: start)
set -euo pipefail

DISPLAY_NUM="${VNC_DISPLAY:-:1}"
GEOMETRY="${VNC_GEOMETRY:-1600x900x24}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT=5900
LOG_DIR=/tmp/vnc
mkdir -p "$LOG_DIR"

stop() {
  pkill -f "websockify.*${NOVNC_PORT}" 2>/dev/null || true
  pkill -x x11vnc 2>/dev/null || true
  pkill -x fluxbox 2>/dev/null || true
  pkill -f "Xvfb ${DISPLAY_NUM}" 2>/dev/null || true
  rm -f "/tmp/.X${DISPLAY_NUM#:}-lock"
  echo "VNC stopped."
}

if [[ "${1:-start}" == "stop" ]]; then stop; exit 0; fi

if pgrep -f "Xvfb ${DISPLAY_NUM}" >/dev/null; then
  echo "Virtual display ${DISPLAY_NUM} already running."
else
  rm -f "/tmp/.X${DISPLAY_NUM#:}-lock"
  Xvfb "$DISPLAY_NUM" -screen 0 "$GEOMETRY" +extension GLX +render -noreset >"$LOG_DIR/xvfb.log" 2>&1 &
  sleep 1.5
fi
export DISPLAY="$DISPLAY_NUM"

# Open every window (RViz, Gazebo) maximized; switch between them in the taskbar.
# Exception: the keyboard window (drone_teleop.py) stays small, bottom right, on top.
mkdir -p "$HOME/.fluxbox"
cat > "$HOME/.fluxbox/apps" <<'APPS'
[app] (title=Tastiera e dronit)
  [Maximized]	{no}
  [Layer]	{2}
  [Position]	(LowerRight)	{10 40}
[end]
[app] (name=.*)
  [Maximized]	{yes}
[end]
APPS
pgrep -x fluxbox >/dev/null || (fluxbox >"$LOG_DIR/fluxbox.log" 2>&1 &)
pgrep -x x11vnc  >/dev/null || x11vnc -display "$DISPLAY_NUM" -forever -shared -nopw -quiet \
    -rfbport "$VNC_PORT" -bg -o "$LOG_DIR/x11vnc.log" >/dev/null 2>&1
pgrep -f "websockify.*${NOVNC_PORT}" >/dev/null || \
  (websockify --web /usr/share/novnc "$NOVNC_PORT" "localhost:${VNC_PORT}" >"$LOG_DIR/novnc.log" 2>&1 &)
sleep 1

echo "Virtual desktop ready on DISPLAY=${DISPLAY_NUM}"
echo "Open on macOS: http://localhost:${NOVNC_PORT}/vnc.html"
