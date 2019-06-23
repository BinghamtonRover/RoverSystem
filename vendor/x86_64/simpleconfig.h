#ifndef SIMPLECONFIG_H
#define SIMPLECONFIG_H

#include <stdint.h>

#ifdef __cplusplus
#define SIMPLECONFIG_EXTERN_C extern "C"
#else
#define SIMPLECONFIG_EXTERN_C
#endif

struct SimpleConfig;

SIMPLECONFIG_EXTERN_C uint8_t * sc_get_error_string(void);
SIMPLECONFIG_EXTERN_C struct SimpleConfig * sc_parse(const uint8_t * maybe_file_name);
SIMPLECONFIG_EXTERN_C void sc_free(struct SimpleConfig * simple_config);
SIMPLECONFIG_EXTERN_C uint8_t * sc_get(struct SimpleConfig * simple_config, uint8_t * maybe_key);

#endif
