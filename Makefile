#"rover" is the old rover computer program, will keep it updated regardless 
#all: base_station rover video_computer subsystems_computer
all: base_station video_computer subsystems_computer

bin:
	mkdir bin

network: bin
	make -C src/network

simple_config: bin
	make -C src/simple_config

logger: bin
	make -C src/logger

rocs: bin
	make -C src/rocs

autonomy: bin
	make -C src/autonomy

base_station: bin network simple_config logger
	make -C src/base_station

#rover: bin network simple_config logger rocs
#	make -C src/rover

subsystems_computer: bin network simple_config logger rocs
	make -C src/subsystems_computer

video_computer: bin network simple_config logger
	make -C src/video_computer

clean:
	rm bin/*

format:
	find . -name "*.cpp" -exec clang-format-9 -style=file -i '{}' \;
	find . -name "*.hpp" -exec clang-format-9 -style=file -i '{}' \;

archive:
	git ls-files -z | xargs -0 zip RoverSystem.zip

#send source code to subsystem computer
send_s: archive
	scp RoverSystem.zip pi@192.168.1.20:/home/pi/RoverSystem.zip

#send source code to video computer
send_v: archive
	scp RoverSystem.zip pi@192.168.1.23:/home/pi/RoverSystem.zip

.PHONY: network, base_station, rover, clean, simple_config, format, archive, send, logger, rocs, autonomy, subsystems_computer
