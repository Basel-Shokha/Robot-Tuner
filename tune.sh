#!/bin/sh
# Starts Robot Tuner on this computer and opens it in Chrome (Linux / macOS). Stop with Ctrl+C.
# Needs Python 3 (preinstalled on macOS and most Linux). Chrome or Edge is needed for the USB measurement page.
cd "$(dirname "$0")/web" || exit 1
PORT=${PORT:-8000}
while (command -v nc >/dev/null && nc -z 127.0.0.1 "$PORT" 2>/dev/null); do PORT=$((PORT + 1)); done
URL="http://localhost:$PORT/"
PY=$(command -v python3 || command -v python) || { echo "Python 3 is needed: https://www.python.org/downloads/"; exit 1; }
echo "Robot Tuner: $URL   (press Ctrl+C to stop)"
(sleep 1
 if [ "$(uname)" = Darwin ]; then open -a "Google Chrome" "$URL" 2>/dev/null || open "$URL"
 else google-chrome "$URL" 2>/dev/null || chromium "$URL" 2>/dev/null || chromium-browser "$URL" 2>/dev/null || xdg-open "$URL" 2>/dev/null || echo "Open $URL in Chrome"; fi
) >/dev/null 2>&1 &
exec "$PY" -m http.server "$PORT" --bind 127.0.0.1
