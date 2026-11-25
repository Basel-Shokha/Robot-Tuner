#!/usr/bin/env bash
# tools/compile_firmware.sh [out_dir] [jobs]     (BOARD=uno|mega|esp32 to compile one board only)
# Generates every firmware combination (tools/gen_firmware_combos.mjs) and compiles each with arduino-cli.
# Needs arduino-cli with the cores arduino:avr and esp32:esp32 and the libraries listed in
# web/js/firmware-gen.js requiredLibraries (VL53L0X, VL53L1X, PinChangeInterrupt).
set -euo pipefail
OUT=${1:-build/firmware}
JOBS=${2:-$(nproc)}
if [ -n "${BOARD:-}" ] && [ "$BOARD" != all ]; then node tools/gen_firmware_combos.mjs "$OUT" --board "$BOARD"; else node tools/gen_firmware_combos.mjs "$OUT"; fi
compile() {
  fqbn=$1; name=$2; out=$3
  if arduino-cli compile --fqbn "$fqbn" --build-path "$out/.build/$name" "$out/$name" > "$out/$name/compile.log" 2>&1; then
    echo "ok   $name"
  else
    echo "FAIL $name"; sed 's/^/    /' "$out/$name/compile.log" | grep -iE 'error|warning: .*overflow|not enough' | head -20; return 1
  fi
}
export -f compile
# warm the core caches with one sketch per board first (parallel builds of the same core race otherwise)
for fqbn in $(cut -d' ' -f1 "$OUT/combos.txt" | sort -u); do
  first=$(grep "^$fqbn " "$OUT/combos.txt" | head -1 | cut -d' ' -f2)
  compile "$fqbn" "$first" "$OUT"
done
fail=0
grep -v '^$' "$OUT/combos.txt" | xargs -P "$JOBS" -L 1 bash -c 'compile "$0" "$1" "'"$OUT"'"' || fail=1
n=$(grep -c . "$OUT/combos.txt")
if [ $fail -ne 0 ]; then echo "some of the $n sketches did not compile"; exit 1; fi
echo "all $n sketches compiled"
