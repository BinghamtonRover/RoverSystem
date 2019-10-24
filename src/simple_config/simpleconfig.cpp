#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "simpleconfig.h"

namespace sc {

// This is a macro just for the stringification to work.
#define MAX_LINE_LEN 1000
#define STRINGIFY(X) #X
#define MAX_LINE_LEN_STRING STRINGIFY(MAX_LINE_LEN)

static const int DEFAULT_CAPACITY = 50;

struct SimpleConfig {
    char** keys;
    char** values;
    int count;
    int cap;

    int bad_line;
};

const char* get_error_string(SimpleConfig* simple_config, Error error) {
    static char error_string_buffer[2000];

    switch (error) {
        case Error::OK:
            return "";
        case Error::FILE_OPEN:
            return "failed to open file";
        case Error::LINE_TOO_LONG:
            sprintf(
                error_string_buffer,
                "[line %d]: line is longer than max length (%d characters)",
                simple_config->bad_line,
                MAX_LINE_LEN);
            return error_string_buffer;
        case Error::MISSING_KEY:
            sprintf(error_string_buffer, "[line %d]: expected a key", simple_config->bad_line);
            return error_string_buffer;
        case Error::MISSING_VALUE:
            sprintf(error_string_buffer, "[line %d]: expected a value", simple_config->bad_line);
            return error_string_buffer;
        case Error::DUPLICATE_KEYS:
            sprintf(error_string_buffer, "[line %d]: key was defined twice", simple_config->bad_line);
            return error_string_buffer;
        default:
            return "unknown error";
    }
}

static bool is_space(char c) {
    return c == ' ' || c == '\t';
}

// This assumes line_buffer is null-terminated.
static Error parse_line(char* line_buffer, int buffer_len, int line_number, SimpleConfig* out_config) {
    // Simple: set the bad line number every time. No one cares if there is no error!
    out_config->bad_line = line_number;

    // Ignore this line if it is only comprised of spaces.
    bool is_empty = true;
    for (int i = 0; i < buffer_len; i++) {
        if (!is_space(line_buffer[i])) {
            is_empty = false;
            break;
        }
    }

    if (is_empty) {
        buffer_len = 0;
        return Error::OK;
    }

    int idx = 0;

    // First, we clear any spaces at the beginning.
    while (is_space(line_buffer[idx])) {
        idx++;
    }

    // A comment might be here, in which case the rest of the line is ignored.
    if (line_buffer[idx] == '#') {
        return Error::OK;
    }

    int key_start = idx;
    int key_end = idx;

    // Collect the key.
    // Stop if space or = is found.
    while (true) {
        auto c = line_buffer[idx++];

        if (is_space(c) || c == '=') {
            break;
        }

        key_end++;
    }

    int key_len = key_end - key_start;

    // Check for empty key.
    if (key_len == 0) {
        return Error::MISSING_KEY;
    }

    // Eat spaces, if they exist.
    while (is_space(line_buffer[idx])) {
        idx++;
    }

    // Must be =.
    if (line_buffer[idx] != '=') {
        return Error::MISSING_VALUE;
    }

    idx++;

    // Eat spaces, if they exist.
    while (is_space(line_buffer[idx])) {
        idx++;
    }

    int value_start = idx;
    int value_end = idx;

    // Collect value.
    while (true) {
        char c = line_buffer[idx++];

        // End of the line?
        if (c == '\0') {
            break;
        }

        value_end++;
    }

    int value_len = value_end - value_start;

    // Now we know the ranges of the key and value.
    // We must copy them to their own strings and append them.

    // + 1 for null-terminator.
    char* key_str = (char*) calloc(key_len + 1, sizeof(char));
    strncpy(key_str, line_buffer + key_start, key_len);
    key_str[key_len] = '\0';

    // Check to make sure that this key doesn't already exist.
    for (int i = 0; i < out_config->count; i++) {
        if (strcmp(out_config->keys[i], key_str) == 0) {
            ::free(key_str);
            return Error::DUPLICATE_KEYS;
        }
    }

    char* value_str = (char*) calloc(value_len + 1, sizeof(char));
    strncpy(value_str, line_buffer + value_start, value_len);
    value_str[value_len] = '\0';

    if (out_config->count == out_config->cap) {
        // We need to expand.
        out_config->cap *= 2;
        out_config->keys = (char**) realloc(out_config->keys, out_config->cap * sizeof(char*));
        out_config->values = (char**) realloc(out_config->values, out_config->cap * sizeof(char*));
    }

    out_config->keys[out_config->count] = key_str;
    out_config->values[out_config->count] = value_str;

    out_config->count++;

    return Error::OK;
}

Error parse(const char* maybe_file_name, SimpleConfig** out_config) {
    // Init the config.
    auto config = (SimpleConfig*) malloc(sizeof(SimpleConfig));

    *out_config = config;

    config->count = 0;
    config->cap = DEFAULT_CAPACITY;
    config->keys = (char**) calloc(config->cap, sizeof(char*));
    config->values = (char**) calloc(config->cap, sizeof(char*));

    // Try to open the file.
    auto file = fopen(maybe_file_name, "r");
    if (!file) {
        return Error::FILE_OPEN;
    }

    // Use a stack-allocated buffer for reading.
    // Plus one for null-terminator.
    char line_buffer[MAX_LINE_LEN + 1]{};
    int buffer_len = 0;

    int line_number = 1;

    while (true) {
        // Grab one character.
        auto c = fgetc(file);

        // Prevent buffer overflow.
        if (buffer_len == MAX_LINE_LEN) {
            return Error::LINE_TOO_LONG;
        }

        // Ignore these things.
        if (c == '\r') {
            continue;
        }

        // If we haven't reached EOL, append and continue.
        if (c != '\n' && c != EOF) {
            line_buffer[buffer_len++] = c;
            continue;
        }

        // Make sure to add null-terminator.
        line_buffer[buffer_len] = '\0';

        // At this point, c is either newline or EOF.
        // Both signify the end of a line.

        auto err = parse_line(line_buffer, buffer_len, line_number, config);
        if (err != Error::OK) {
            return err;
        }

        line_number++;

        // Exit if it was EOF.
        if (c == EOF) {
            break;
        }

        // Reset the buffer.
        buffer_len = 0;
    }

    return Error::OK;
}

void free(SimpleConfig* simple_config) {
    if (!simple_config) return;

    for (int i = 0; i < simple_config->count; i++) {
        ::free(simple_config->keys[i]);
        ::free(simple_config->values[i]);
    }

    ::free(simple_config->keys);
    ::free(simple_config->values);

    ::free(simple_config);
}

char* get(SimpleConfig* simple_config, const char* maybe_key) {
    if (!simple_config) return nullptr;

    for (int i = 0; i < simple_config->count; i++) {
        if (strcmp(simple_config->keys[i], maybe_key) == 0) {
            return simple_config->values[i];
        }
    }

    return nullptr;
}

#undef MAX_LINE_LEN
#undef MAX_LINE_LEN_STRING
#undef STRINGIFY

} // namespace sc
