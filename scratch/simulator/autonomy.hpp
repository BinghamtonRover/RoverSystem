#ifndef AUTONOMY_H
#define AUTONOMY_H

#include "occupancy_grid.hpp"
#include "world.hpp"

enum AutonomyStatus {
    AS_OK,
    AS_DONE,
    AS_ERROR
};

AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y);

const char* autonomy_get_name();

#endif
