#ifndef WORLD_H
#define WORLD_H

#include "occupancy_grid.hpp"

struct World {
    int grid_size;
    float cell_size;

    float target_x, target_y;

    OccupancyGrid occupancy_grid;
};

void world_to_cell(World* world, float wx, float wy, int* out_cx, int* out_cy);
void cell_to_world(World* world, int cx, int cy, float* out_wx, float* out_wy);

#endif
