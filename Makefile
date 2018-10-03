all: network

network:
	make -C src/network

network_clean:
	make -C src/network clean

clean:
	rm bin/*

.PHONY: network, network_clean, clean
