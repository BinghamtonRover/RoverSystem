all: network base_station rover

network:
	make -C src/network

network_clean:
	make -C src/network clean

base_station:
	make -C src/base_station

rover:
	make -C src/rover

clean:
	rm bin/*

.PHONY: network, network_clean, base_station, rover, clean
