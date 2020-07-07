#include "logger.hpp"

#include <stdarg.h>
#include <cassert>

namespace logger {

constexpr int LOG_BUFFER_SIZE = 1024 * 1024;

Handler handlers[MAX_HANDLERS];
int num_handlers = 0;
bool debugMode = true;

void register_handler(Handler handler) {
    assert(num_handlers + 1 < MAX_HANDLERS);

    handlers[num_handlers++] = handler;
}

bool toggleDebugMode() {
    debugMode = !debugMode;
    return debugMode;
}

void log(Level level, const char* format, ...) {
    if (debugMode || level != Level::DEBUG) {
        va_list list;
        va_start(list, format);

        static char log_buffer[LOG_BUFFER_SIZE];

        vsprintf(log_buffer, format, list);

        va_end(list);

        for (int i = 0; i < num_handlers; i++) {
            handlers[i](level, log_buffer);
        }
    }
}

void stderr_handler(logger::Level level, std::string message) {
    fprintf(stderr, "%s\n", message.c_str());
}
} // namespace logger
