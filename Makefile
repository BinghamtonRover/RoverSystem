all: network

.PHONY: network
network:
	make -C src/network

.PHONY: network_clean
network_clean:
	make -C src/network clean

.PHONY: clean
clean:
	rm bin/*