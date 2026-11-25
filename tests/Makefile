# make -C tests          native upstream equivalence + native golden check + WASM equivalence (needs node)
# make -C tests firmware  generate every Arduino sketch combination and compile it (needs arduino-cli, see tools/compile_firmware.sh)
# make -C tests golden   regenerate tests/golden/* from the native build (after an intended engine change)
CXX ?= g++
FLAGS = -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-missing-field-initializers
BIN = native/bin
.PHONY: all test-native test-wasm golden firmware
all: test-native test-wasm
$(BIN):
	mkdir -p $(BIN)
$(BIN)/equiv_upstream: native/equiv_upstream.cpp ../sim/*.hpp | $(BIN)
	$(CXX) $(FLAGS) -w $< -o $@
$(BIN)/golden: native/golden.cpp ../sim/*.hpp ../sim/rt_api.cpp | $(BIN)
	$(CXX) $(FLAGS) $< -o $@
test-native: $(BIN)/equiv_upstream $(BIN)/golden
	cd .. && tests/$(BIN)/equiv_upstream && tests/$(BIN)/golden --check
test-wasm:
	cd .. && node --test tests/*.test.mjs tests/js/*.test.mjs
golden: $(BIN)/golden
	cd .. && tests/$(BIN)/golden --write
firmware:
	cd .. && tools/compile_firmware.sh build/firmware
