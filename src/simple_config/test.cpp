#include <assert.h>
#include <string.h>

#include "simpleconfig.h"

int main() {
    sc::SimpleConfig* config;

    assert(sc::parse("test.sconfig", &config) == sc::Error::OK);

    assert(sc::get(config, "a_key") != nullptr);
    assert(strcmp("a_value", sc::get(config, "a_key")) == 0);

    assert(sc::get(config, "b_key") != nullptr);
    assert(strcmp("b_value ", sc::get(config, "b_key")) == 0);

    sc::free(config);

    return 0;
}
