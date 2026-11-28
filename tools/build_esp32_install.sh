#!/usr/bin/env bash
# tools/build_esp32_install.sh - compiles the ESP32 install firmware (web/js/firmware-esp32.js) and puts the flashable
# image at web/firmware/esp32/robot_tuner_esp32.bin (one file for address 0: bootloader + partitions + app).
# Needs arduino-cli with the esp32:esp32 core and the VL53L0X / VL53L1X libraries. pages.yml runs it before publishing.
set -euo pipefail
OUT=${1:-build/esp32_install}
rm -rf "$OUT"; mkdir -p "$OUT/robot_tuner_esp32"
node --input-type=module -e "import { generateEsp32Install } from './web/js/firmware-esp32.js'; import { writeFileSync } from 'node:fs'; const g = generateEsp32Install(); writeFileSync('$OUT/robot_tuner_esp32/robot_tuner_esp32.ino', g.code); writeFileSync('$OUT/fqbn.txt', g.fqbn);"
arduino-cli compile --fqbn "$(cat "$OUT/fqbn.txt")" --export-binaries --output-dir "$OUT/bin" "$OUT/robot_tuner_esp32"
mkdir -p web/firmware/esp32
cp "$OUT/bin/robot_tuner_esp32.ino.merged.bin" web/firmware/esp32/robot_tuner_esp32.bin
ls -l web/firmware/esp32/robot_tuner_esp32.bin
