#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${VNC_DISPLAY:-:1}"
GEOMETRY="${VNC_GEOMETRY:-1600x900x24}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT=5900
LOG_DIR=/tmp/vnc
mkdir -p "$LOG_DIR"

if ! pgrep -f "Xvfb ${DISPLAY_NUM}" >/dev/null; then
  rm -f "/tmp/.X${DISPLAY_NUM#:}-lock"
  Xvfb "$DISPLAY_NUM" -screen 0 "$GEOMETRY" +extension GLX +render -noreset >"$LOG_DIR/xvfb.log" 2>&1 &
  sleep 1.5
fi
export DISPLAY="$DISPLAY_NUM"

mkdir -p "$HOME/.fluxbox"
cat > "$HOME/.fluxbox/apps" <<'APPS'
[app] (title=Drone keyboard)
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

echo "Virtual desktop ready on DISPLAY=${DISPLAY_NUM}: http://localhost:${NOVNC_PORT}/vnc.html"
