#ifndef UTIL_H
#define UTIL_H

#include <time.h>
#include <stdint.h>

namespace util {

struct Clock {
    uint32_t start_ms;

    uint32_t get_millis() {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        
        uint32_t ms = (uint32_t)now.tv_sec * 1000;
        ms += (uint32_t)now.tv_nsec / 1000000;

        return ms - start_ms;
    }

    static void init(Clock* clock) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        clock->start_ms = (uint32_t)now.tv_sec * 1000;
        clock->start_ms += (uint32_t)now.tv_nsec / 1000000;
    }
};

struct Timer {
    uint32_t interval;
    uint32_t last;
    Clock* clock;

    bool ready() {
        auto now = clock->get_millis();

        if (now - last >= interval) {
            last = now;
            return true;
        }

        return false;
    }

    static void init(Timer* timer, uint32_t interval, Clock* clock) {
        timer->interval = interval;
        timer->clock = clock;

        timer->last = 0;
    }
};

} // namespace util

#endif
