# make tune    start Robot Tuner on this computer and open it in Chrome (same as ./tune.sh)
# make test    run all tests (C++ + WASM + JS)
# make wasm    rebuild web/wasm from sim/ (needs Emscripten)
.PHONY: tune test wasm
tune:
	./tune.sh
test:
	$(MAKE) -C tests
wasm:
	./sim/build_wasm.sh
