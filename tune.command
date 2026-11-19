#!/bin/sh
# Starts Robot Tuner on this computer and opens it in Chrome (Linux / macOS). Stop with Ctrl+C.
# Needs Python 3 (preinstalled on macOS and most Linux). Chrome or Edge is needed for the USB measurement page.
cd "$(dirname "$0")/web" || exit 1
PORT=${PORT:-8000}
while (command -v nc >/dev/null && nc -z 127.0.0.1 "$PORT" 2>/dev/null); do PORT=$((PORT + 1)); done
URL="http://localhost:$PORT/"
PY=$(command -v python3 || command -v python)
if [ -z "$PY" ] || ! "$PY" -c 'import sys; sys.exit(sys.version_info < (3,))' 2>/dev/null; then
  echo "Robot Tuner needs Python 3, which is not installed."
  if [ "$(uname)" = Darwin ]; then CMD="xcode-select --install"
  elif command -v apt-get >/dev/null; then CMD="sudo apt-get install -y python3"
  elif command -v dnf >/dev/null; then CMD="sudo dnf install -y python3"
  elif command -v pacman >/dev/null; then CMD="sudo pacman -S --noconfirm python"
  else echo "Install it from https://www.python.org/downloads/ and run this again."; exit 1; fi
  printf "Install it now with: %s ? [y/N] " "$CMD"; read -r a
  case "$a" in y|Y) sh -c "$CMD" || exit 1; echo "Done. Run this again to start Robot Tuner."; exit 0 ;;
    *) echo "Install it from https://www.python.org/downloads/ and run this again."; exit 1 ;; esac
fi
echo "Robot Tuner: $URL   (press Ctrl+C to stop)"
(sleep 1
 if [ "$(uname)" = Darwin ]; then open -a "Google Chrome" "$URL" 2>/dev/null || open "$URL"
 else google-chrome "$URL" 2>/dev/null || chromium "$URL" 2>/dev/null || chromium-browser "$URL" 2>/dev/null || xdg-open "$URL" 2>/dev/null || echo "Open $URL in Chrome"; fi
) >/dev/null 2>&1 &
exec "$PY" -m http.server "$PORT" --bind 127.0.0.1
