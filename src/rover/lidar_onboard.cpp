#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <endian.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>

#include <string>

#include "lidar.hpp"

namespace lidar {

void send_command(int sock, const char* command) {
	ssize_t bcount = write(sock, command, strlen(command));
	if (bcount <= 0) {
		fprintf(stderr, "[!] LIDAR: Invalid bytes written: %ld\n", bcount);
	}
}

void receive_command(int sock, char* buffer, size_t buffer_size) {
	ssize_t bread = read(sock, buffer, buffer_size);
	if (bread <= 0) {
		fprintf(stderr, "[!] LIDAR: Invalid bytes read: %ld\n", bread);
	}
}

void print_command(const char* command) {
	for (int i = 0; command[i] != 0; i++) {
		if (command[i] == '\x03') break;
		if (command[i] < 13) continue;
		printf("%c", command[i]);
	}

	printf("\n");
}

void parse_and_print_data(const char* command, std::vector<long>& out) {
	// Advance past start code.
	//assert(command[0] == '\x02');
	
	std::vector<std::string> parts;		
	
	parts.push_back("");

	for (int i = 1; command[i] != '\x03'; i++) {
		if (command[i] == ' ') {
			parts.push_back("");
		} else {
			parts[parts.size() - 1].push_back(command[i]);
		}
	}

	// printf("We got data with %lu fields\n", parts.size());

	long num_data_points = strtol(parts[25].c_str(), NULL, 16);
	// printf("%ld data points.\n", num_data_points);

	// printf("Start angle: %s\n", parts[23].c_str());

	for (int i = 0; i < num_data_points; i++) {
		long data_point = strtol(parts[25 + i + 1].c_str(), NULL, 16);
		out.push_back(data_point);
		// printf("\tData Point %d: %ld mm\n", i, data_point);
	}
}

int sock;

Error start(const char* ip) {
	sock = socket(AF_INET, SOCK_STREAM, 0);

	int enable = 1;
	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
	
	struct sockaddr_in local_address{};
	local_address.sin_family = AF_INET; 
	// keep this zero: local_address.sin_port = htobe16(2222);
	local_address.sin_addr.s_addr = INADDR_ANY;

	int bind_res = bind(sock, (const struct sockaddr*)&local_address, sizeof(local_address));
	if (bind_res == -1) {
		fprintf(stderr, "[!] Failed to bind: %d\n", errno);
		return Error::BIND;
	}

	struct sockaddr_in sock_address{};
	sock_address.sin_family = AF_INET;
	sock_address.sin_port = htobe16(2112);
	inet_aton(ip, &sock_address.sin_addr);

	int connect_res = connect(sock, (const struct sockaddr*)&sock_address, sizeof(sock_address));
	if (connect_res == -1) {
		fprintf(stderr, "[!] Failed to connect: %d\n", errno);
		return Error::CONNECT;
	}

	char receive_buffer[BUFFER_SIZE];

	/*
		When logged in, you can't get measurement data.
		You can only change settings.
		To turn measurement (and laser/motor) on or off, you have to be logged in.

			send_command(sock, "\x02sMN SetAccessMode 03 F4724744\x03");
		receive_command(sock, receive_buffer, BUFFER_SIZE);
		print_command(receive_buffer);

		send_command(sock, "\x02sMN LMCstopmeas\x03"); // Stops laser and motor.
		receive_command(sock, receive_buffer, BUFFER_SIZE);
		print_command(receive_buffer);

	*/

	send_command(sock, "\x02sMN LMCstartmeas\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	print_command(receive_buffer);

	send_command(sock, "\x02sRI0\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	print_command(receive_buffer);

	send_command(sock, "\x02sMN Run\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	print_command(receive_buffer);

	send_command(sock, "\x02sRN SCdevicestate\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	print_command(receive_buffer);

	send_command(sock, "\x02sRN LocationName\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	print_command(receive_buffer);

	send_command(sock, "\x02sEN LMDscandata 0\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	print_command(receive_buffer);

	return Error::OK;
}

Error scan(std::vector<long>& data_points) {
	char receive_buffer[BUFFER_SIZE];

	send_command(sock, "\x02sRN LMDscandata\x03");
	receive_command(sock, receive_buffer, BUFFER_SIZE);
	parse_and_print_data(receive_buffer, data_points);

	return Error::OK;
}

}
