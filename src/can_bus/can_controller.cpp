#include "can_controller.hpp"

//can_send a uint32_t
int can_send_velocity(int device_num, uint32_t message) {
	char* can_frame = get_can_velocity_message(device_num, message);
	char can0[] = "can0";
	int ret = can_send(can0, can_frame);
	delete can_frame;
	return ret;
}

//can_send a float
int can_send_velocity(int device_num, float message) {
	union { float f; uint32_t u; } f2u = { .f = message };
	return can_send_velocity(device_num, f2u.u);
}

//send a non-power can_frame (does not have "d" before "#")
int can_send_custom_message(char* custom_message) {
	char can0[] = "can0";
	int ret = can_send(can0, custom_message);
	return ret;
}

//convert uint32_t and device_num to a can_frame (only can frames for sending power, which start with 00d# and 02d#)
char* get_can_velocity_message(int device_num, uint32_t message) {
	char* full_message = new char[12];
	char* vals = new char[8];
	sprintf(vals, "%010x00000000", get_big_endian(message));
	if (device_num < 8) { sprintf(full_message, "0%xd#%s", device_num << 1, (vals + 2)); }
	else { sprintf(full_message, "%xd#%s", device_num << 1, (vals + 2)); }
	delete vals;
	return full_message;
}

//convert uint32_t to big endian
uint32_t get_big_endian(uint32_t u) {
	return ((0x000000FF & u) << 24) | ((0x0000FF00 & u) << 8) | ((0x00FF0000 & u) >> 8)  | ((0xFF000000 & u) >> 24);
}

/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * cansend.c - send CAN-frames via CAN_RAW sockets
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

int can_send(char* argv1, char* argv2)
{
	int s; /* can raw socket */ 
	int required_mtu;
	int mtu;
	int enable_canfd = 1;
	struct sockaddr_can addr;
	struct canfd_frame frame;
	struct ifreq ifr;

	/* parse CAN frame */
	required_mtu = parse_canframe(argv2, &frame);
	if (!required_mtu){
		//fprintf(stderr, "\nWrong CAN-frame format!\n\n");
		return 1;
	}

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		//perror("socket");
		return 1;
	}

	strncpy(ifr.ifr_name, argv1, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		//perror("if_nametoindex");
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (required_mtu > (int)CAN_MTU) {

		/* check if the frame fits into the CAN netdevice */
		if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
			//perror("SIOCGIFMTU");
			return 1;
		}
		mtu = ifr.ifr_mtu;

		if (mtu != CANFD_MTU) {
			//printf("CAN interface is not CAN FD capable - sorry.\n");
			return 1;
		}

		/* interface is ok - try to switch the socket into CAN FD mode */
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
			       &enable_canfd, sizeof(enable_canfd))){
			//printf("error when enabling CAN FD support\n");
			return 1;
		}

		/* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
		frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len));
	}

	/* disable default receive filter on this RAW socket */
	/* This is obsolete as we do not read from the socket at all, but for */
	/* this reason we can remove the receive list in the Kernel to save a */
	/* little (really a very little!) CPU usage.                          */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		//perror("bind");
		return 1;
	}

	/* send frame */
	if (write(s, &frame, required_mtu) != required_mtu) {
		//perror("write");
		return 1;
	}

	close(s);

	return 0;
}
