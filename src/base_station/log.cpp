#include "log.hpp"

#include <cassert>

namespace log {

Handler handlers[MAX_HANDLERS];
int num_handlers = 0;
	
void register_handler(Handler handler) {
	assert(num_handlers + 1 < MAX_HANDLERS);

	handlers[num_handlers++] = handler;
}

void log(Level level, std::string message) {
	for (int i = 0; i < num_handlers; i++) {
		handlers[i](level, message);
	}
}

} // namespace log
