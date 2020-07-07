#include <string>

namespace logger {

constexpr int MAX_HANDLERS = 10;

enum Level { DEBUG, INFO, WARNING, ERROR };

typedef void (*Handler)(Level, std::string);

void register_handler(Handler handler);

bool toggleDebugMode();

void log(Level level, const char* format, ...);

void stderr_handler(logger::Level level, std::string message);

} // namespace logger
