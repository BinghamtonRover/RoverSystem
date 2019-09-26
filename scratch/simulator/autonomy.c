/*
    DUMMY IMPLEMENTATION OF AUTONOMY
*/
#include "autonomy.h"

AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y) {
    *out_offset_x = 0.5;
    *out_offset_y = 0.5;

    return AS_OK;
}

const char* autonomy_get_name() {
    return "DUMMY";
}