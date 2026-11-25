# make tune    start Robot Tuner on this computer and open it in Chrome (same as ./tune.sh)
# make test    run all tests (C++ + WASM + JS)
# make wasm    rebuild web/wasm from sim/ (needs Emscripten)
# make firmware  compile every sketch the robot builder can generate (needs arduino-cli)
.PHONY: tune test wasm firmware
tune:
	./tune.sh
test:
	$(MAKE) -C tests
wasm:
	./sim/build_wasm.sh
firmware:
	$(MAKE) -C tests firmware
