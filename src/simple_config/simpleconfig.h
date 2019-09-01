#ifndef SIMPLECONFIG_H
#define SIMPLECONFIG_H

namespace sc {

struct SimpleConfig;

// Sync with get_error_string in simpleconfig.cpp.
enum Error {
    OK,

    FILE_OPEN,
    LINE_TOO_LONG,

    MISSING_KEY,
    MISSING_VALUE,
    DUPLICATE_KEYS,
};

const char* get_error_string(SimpleConfig* simple_config, Error error);
Error parse(const char* maybe_file_name, SimpleConfig** out_config);
void free(SimpleConfig* simple_config);
char* get(SimpleConfig* simple_config, const char* maybe_key);

}

#endif
