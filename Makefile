all: base_station rover

bin:
	mkdir bin

network: bin
	make -C src/network

simple_config: bin
	make -C src/simple_config

logger: bin
	make -C src/logger

base_station: bin network simple_config logger
	make -C src/base_station

rover: bin network simple_config logger
	make -C src/rover

clean:
	rm bin/*

format:
	find . -name "*.cpp" -exec clang-format-9 -style=file -i '{}' \;
	find . -name "*.hpp" -exec clang-format-9 -style=file -i '{}' \;

archive:
	git ls-files -z | xargs -0 zip RoverSystem.zip

send: archive
	scp RoverSystem.zip layne@192.168.1.20:/home/layne/RoverSystem.zip

.PHONY: network, base_station, rover, clean, simple_config, format, archive, send, logger
