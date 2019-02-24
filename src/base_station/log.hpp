#include <string>

namespace log {

constexpr int MAX_HANDLERS = 10;

enum Level {
	DEBUG,
	INFO,
	WARNING,
	ERROR
};

typedef void (*Handler)(Level, std::string);

void register_handler(Handler handler);

void log(Level level, std::string message);

} // namespace log
