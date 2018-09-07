
#include "network.hpp"

namespace network
{

Manager::Manager() :
    incoming_arena(5), outgoing_arena(5)
{

}

OutgoingBuffer* Manager::get_outgoing_buffer()
{
    return outgoing_arena.alloc();
}

void Manager::queue_message(MessageType type, OutgoingBuffer* buffer)
{
    outgoing_queue.push_back();
}

bool Manager::poll_message(IncomingBuffer** buffer)
{

}

} // namespace network