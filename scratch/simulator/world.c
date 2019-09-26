#include <math.h>

#include "world.h"

void world_to_cell(World* world, float wx, float wy, int* out_cx, int* out_cy) {
    *out_cx = roundf(wx / (float)world->cell_size);
    *out_cy = roundf(wy / (float)world->cell_size);
}

void cell_to_world(World* world, int cx, int cy, float* out_wx, float* out_wy) {
    *out_wx = (float)cx * (float)world->cell_size;
    *out_wy = (float)cy * (float)world->cell_size;
}
