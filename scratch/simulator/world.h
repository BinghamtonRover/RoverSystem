#ifndef WORLD_H
#define WORLD_H

#include "occupancy_grid.h"

typedef struct {
    int grid_size;
    float cell_size;

    OccupancyGrid occupancy_grid;
} World;

void world_to_cell(World* world, float wx, float wy, int* out_cx, int* out_cy);
void cell_to_world(World* world, int cx, int cy, float* out_wx, float* out_wy);

#endif
