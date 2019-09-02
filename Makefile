all: bin network simple_config base_station rover

bin:
	mkdir bin

network:
	make -C src/network

network_clean:
	make -C src/network clean

simple_config:
	make -C src/simple_config

base_station:
	make -C src/base_station

rover:
	make -C src/rover

clean:
	rm bin/*

.PHONY: network, network_clean, base_station, rover, clean, simple_config
