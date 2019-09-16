#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "../../src/new_network/network.hpp"

void fail() {
    printf("[!] Failed.\n");
    assert(0);
}

const char* MESSAGE = "Hello, world!";

const char* GROUP = "239.255.123.123";

int main() {
    network::Feed in_feed, out_feed;
    
    if (network::init_subscriber(GROUP, 5050, &in_feed) != network::Error::OK) fail();
    if (network::init_publisher(GROUP, 5050, &out_feed) != network::Error::OK) fail();

    network::TestMessage test;
    test.message_size = (uint16_t) strlen(MESSAGE);
    memcpy(test.message, MESSAGE, test.message_size);

    if (network::publish(&out_feed, &test) != network::Error::OK) fail();

    while (true) {
        network::IncomingMessage message;
        auto res = network::receive(&in_feed, &message);
        if (res == network::Error::NOMORE) continue;
        if (res != network::Error::OK) fail();

        if (message.type != network::MessageType::TEST) fail();
        
        network::TestMessage received;
        network::deserialize(&message.buffer, &received);

        received.message[received.message_size] = 0;
        
        printf("> Got message: %s\n", received.message);
    }

    network::close(&in_feed);
    network::close(&out_feed);

    return 0;
}
