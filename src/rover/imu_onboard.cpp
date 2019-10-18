#include <assert.h>
#include <endian.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <mutex>
#include <thread>

#include "imu.hpp"

namespace imu {

const size_t MAX_MESSAGE_SIZE = 2048;

const uint8_t PREAMBLE = 0xFA;
const uint8_t BID = 0xFF;
const uint16_t MAX_NORMAL_LEN = 254;

const uint8_t MID_GoToConfig = 48;
const uint8_t MID_GoToMeasurement = 16;
const uint8_t MID_MTData2 = 54;

const uint16_t DID_EulerAngles = 0x2030;
const uint16_t DID_Acceleration = 0x4020;

struct Message {
    uint8_t id;
    uint16_t len;

    uint8_t data[MAX_MESSAGE_SIZE];
};

uint8_t calc_checksum(
    uint8_t bid, uint8_t mid, uint8_t len, uint16_t ext_len, uint16_t data_len, uint8_t* data, uint8_t checksum) {
    uint64_t checksum_sum = 0;

    checksum_sum += (uint64_t) bid;
    checksum_sum += (uint64_t) mid;
    checksum_sum += (uint64_t) len;
    checksum_sum += (uint64_t) ext_len;

    for (size_t i = 0; i < data_len; i++) {
        checksum_sum += (uint64_t) data[i];
    }

    checksum_sum += (uint64_t) checksum;

    return (uint8_t)(checksum_sum & 0xFF);
}

void send_message(int sfd, Message* m) {
    uint8_t preamble = PREAMBLE;
    uint8_t bid = BID;
    uint8_t mid = m->id;
    uint8_t len = 0;
    uint16_t ext_len = 0;

    if (m->len > MAX_NORMAL_LEN) {
        len = 255;
        ext_len = htobe16(m->len);
    } else {
        len = (uint8_t) m->len;
        ext_len = 0;
    }

    uint8_t checksum_bottom = calc_checksum(bid, mid, len, ext_len, m->len, m->data, 0);
    uint8_t checksum = (uint8_t)(255 - checksum_bottom + 1);

    assert(calc_checksum(bid, mid, len, ext_len, m->len, m->data, checksum) == 0);

    write(sfd, &preamble, 1);
    write(sfd, &bid, 1);
    write(sfd, &mid, 1);
    write(sfd, &len, 1);
    if (ext_len != 0) write(sfd, &ext_len, 2);
    write(sfd, m->data, m->len);
    write(sfd, &checksum, 1);
}

bool receive_message(int sfd, Message* m) {
    uint8_t preamble;
    uint8_t bid;
    uint8_t mid;
    uint8_t len;
    uint16_t ext_len = 0;
    uint8_t checksum;

    read(sfd, &preamble, 1);
    while (preamble != PREAMBLE) {
        read(sfd, &preamble, 1);
    }

    read(sfd, &bid, 1);

    read(sfd, &mid, 1);
    m->id = mid;

    read(sfd, &len, 1);

    if (len == 255) {
        read(sfd, &ext_len, 2);
        ext_len = be16toh(ext_len);
    }

    if (len == 255) {
        m->len = ext_len;
    } else {
        m->len = (uint16_t) len;
    }

    read(sfd, m->data, m->len);
    read(sfd, &checksum, 1);

    uint8_t checksum_bottom = calc_checksum(bid, mid, len, ext_len, m->len, m->data, checksum);

    if (checksum_bottom != 0) {
        return false;
    }

    return true;
}

void update_rotation(int sfd);

#define VALUE(type, thing, index) *((type*) &thing[index])

std::mutex rotation_lock;
Rotation rotation = { 0, 0, 0 };
std::thread imu_thread;

Rotation get_rotation() {
    rotation_lock.lock();
    Rotation r = rotation;
    rotation_lock.unlock();

    return r;
}

const char DEVICE_SERIAL_BYID_PATH_TEMPLATE[] = "/dev/serial/by-id/%s";

Error start(const char* device_serial_id) {
    char* device_path = (char*) malloc(sizeof(DEVICE_SERIAL_BYID_PATH_TEMPLATE) + strlen(device_serial_id));
    sprintf(device_path, DEVICE_SERIAL_BYID_PATH_TEMPLATE, device_serial_id);

    int sfd = open(device_path, O_RDWR | O_NOCTTY | O_SYNC);

    free(device_path);

    if (sfd < 0) {
        return Error::OPEN;
    }

    Message m;

    m.id = MID_GoToConfig;
    m.len = 0;
    send_message(sfd, &m);

    sleep(2);
    ioctl(sfd, TCFLSH, 2);

    m.id = MID_GoToMeasurement;
    m.len = 0;
    send_message(sfd, &m);

    imu_thread = std::thread(update_rotation, sfd);

    return Error::OK;
}

void update_rotation(int sfd) {
    while (true) {
        Message m;

        if (!receive_message(sfd, &m)) {
            continue;
        }

        if (m.id != MID_MTData2) {
            continue;
        }

        uint16_t bread = 0;
        while (bread < m.len) {
            uint16_t data_id = VALUE(uint16_t, m.data, bread);
            data_id = be16toh(data_id);

            uint8_t data_len = VALUE(uint8_t, m.data, bread + 2);

            if (data_id == DID_EulerAngles) {
                union {
                    uint32_t raw;
                    float real;
                } swapper;

                swapper.raw = VALUE(uint32_t, m.data, bread + 3);
                swapper.raw = be32toh(swapper.raw);
                float roll = swapper.real;

                swapper.raw = VALUE(uint32_t, m.data, bread + 7);
                swapper.raw = be32toh(swapper.raw);
                float pitch = swapper.real;

                swapper.raw = VALUE(uint32_t, m.data, bread + 11);
                swapper.raw = be32toh(swapper.raw);
                float yaw = swapper.real;

                rotation_lock.lock();
                rotation.pitch = pitch;
                rotation.yaw = yaw;
                rotation.roll = roll;
                rotation_lock.unlock();
                // printf("Roll: %f, Pitch: %f, Yaw %f\n", roll, pitch, yaw);
            }

            bread += 3 + data_len;
        }
    }
}

} // namespace imu
