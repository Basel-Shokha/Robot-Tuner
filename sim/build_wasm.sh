#!/bin/sh
# Builds web/wasm/rt_sim.mjs + rt_sim.wasm from sim/rt_api.cpp with Emscripten.
# Needs emcc on PATH (source <emsdk>/emsdk_env.sh). The output is committed, so GitHub Pages needs no build.
set -e
cd "$(dirname "$0")/.."
mkdir -p web/wasm
em++ sim/rt_api.cpp -std=c++17 -O3 -ffp-contract=off \
  -s MODULARIZE=1 -s EXPORT_ES6=1 -s EXPORT_NAME=createRtSim \
  -s ALLOW_MEMORY_GROWTH=1 -s ENVIRONMENT=web,worker,node -s FILESYSTEM=0 \
  -s EXPORTED_FUNCTIONS='["_malloc","_free"]' \
  -s EXPORTED_RUNTIME_METHODS='["HEAPF32","HEAPU8","stringToUTF8","lengthBytesUTF8"]' \
  -o web/wasm/rt_sim.mjs
em++ --version | head -1 > web/wasm/BUILD_INFO.txt
echo "source: sim/rt_api.cpp ($(git rev-parse --short HEAD 2>/dev/null || echo unknown))" >> web/wasm/BUILD_INFO.txt
ls -l web/wasm
