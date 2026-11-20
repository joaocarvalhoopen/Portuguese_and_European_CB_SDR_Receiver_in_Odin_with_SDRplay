all:
	odin build . -out:sdrplay_bindings_test.exe -o:speed

run_tests:
	./sdrplay_bindings_test.exe

run_fm:
	# 97.4 MHz FM Radio Comercial
	./sdrplay_bindings_test.exe -fm 97.4

clean:
	rm -f ./sdrplay_bindings_test.exe
