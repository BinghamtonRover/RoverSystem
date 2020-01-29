/*
    DUMMY IMPLEMENTATION OF AUTONOMY
*/
#include "autonomy.hpp"

AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y) {
    *out_offset_x = 5 - rover_x;
    *out_offset_y = 0 - rover_y;

    return AS_OK;
}

const char* autonomy_get_name() {
    return "DUMMY";
}
