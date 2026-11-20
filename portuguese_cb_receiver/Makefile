BIN := cb_receiver.exe
ODIN_SYS_ROOT := $(shell odin root)
TMP_ODIN_ROOT := /tmp/odin_root_cb_sdr

all: $(BIN)

$(BIN): *.odin
	@if [ -f "$(ODIN_SYS_ROOT)vendor/miniaudio/lib/miniaudio.a" ]; then \
		odin build . -out:$(BIN) -o:speed; \
	else \
		mkdir -p "$(TMP_ODIN_ROOT)/vendor"; \
		ln -sfn "$(ODIN_SYS_ROOT)base" "$(TMP_ODIN_ROOT)/base"; \
		ln -sfn "$(ODIN_SYS_ROOT)core" "$(TMP_ODIN_ROOT)/core"; \
		if [ ! -f "$(TMP_ODIN_ROOT)/vendor/miniaudio/lib/miniaudio.a" ]; then \
			cp -r "$(ODIN_SYS_ROOT)vendor/miniaudio" "$(TMP_ODIN_ROOT)/vendor/"; \
			cp -r "$(ODIN_SYS_ROOT)vendor/stb" "$(TMP_ODIN_ROOT)/vendor/"; \
			ODIN_ROOT="$(TMP_ODIN_ROOT)" sh "$(TMP_ODIN_ROOT)/vendor/miniaudio/src/build_miniaudio.sh"; \
		fi; \
		ODIN_ROOT="$(TMP_ODIN_ROOT)" odin build . -out:$(BIN) -o:speed -microarch:native -no-bounds-check -lto:thin; \
	fi

test: $(BIN)
	./$(BIN) -selftest

run: $(BIN)
	./$(BIN) -ch 34 -mode ssb -filter 2 -lna quto -gain auto

sim: $(BIN)
	./$(BIN) -sim -simsnr -25 -ch 38 -mode lsb -filter 4

clean:
	rm -f ./$(BIN)

.PHONY: all test run sim clean
