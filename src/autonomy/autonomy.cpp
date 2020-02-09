#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "autonomy.hpp"

namespace autonomy {

OccupancyGrid OccupancyGrid::create(int grid_size) {
    OccupancyGrid og;
    og.grid = (int*)calloc(grid_size * grid_size, sizeof(int));
    og.grid_size = grid_size;
    og.max = 1;

    return og;
}

void OccupancyGrid::set(int q, int r, int val) {
    if (val > this->max) this->max = val;

    q += this->grid_size / 2;
    r += this->grid_size / 2;

    if (q < 0 || q >= this->grid_size) return;
    if (r < 0 || r >= this->grid_size) return;

    this->grid[q * this->grid_size + r] = val;
}

int OccupancyGrid::get(int q, int r) {
    q += this->grid_size / 2;
    r += this->grid_size / 2;

    if (q < 0 || q >= this->grid_size) return 0;
    if (r < 0 || r >= this->grid_size) return 0;

    return this->grid[q * this->grid_size + r];
}

void OccupancyGrid::inc(int q, int r) {
    int val = get(q, r) + 1;
    set(q, r, val);
}

void OccupancyGrid::clear() {
    int gs = this->grid_size;
    
    memset(this->grid, 0, gs * gs * sizeof(int)); 
    
    this->max = 1;
}

void OccupancyGrid::copy_if_max(OccupancyGrid* to) {
    assert(this->grid_size == to->grid_size);

    for (int q = 0; q < this->grid_size; q++) {
        for (int r = 0; r < this->grid_size; r++) {
            int val = this->grid[q * this->grid_size + r];
            if (val > to->grid[q * to->grid_size + r]) {
                if (val > to->max) {
                    to->max = val;
                }
                to->grid[q * this->grid_size + r] = val;
            }
        }
    }
}

Context create_context(float rover_x, float rover_y, float rover_angle, float target_x, float target_y, float cell_size, int grid_size) {
    Context context;
    context.rover_x = rover_x;
    context.rover_y = rover_y;
    context.rover_angle = rover_angle;
    context.target_x = target_x;
    context.target_y = target_y;
    context.cell_size = cell_size;
    context.grid_size = grid_size;
    context.occupancy_grid = OccupancyGrid::create(grid_size);

    return context;
}

void Context::world_to_cell(float wx, float wy, int* out_cx, int* out_cy) {
    *out_cx = roundf(wx / (float)this->cell_size);
    *out_cy = roundf(wy / (float)this->cell_size);
}

void Context::cell_to_world(int cx, int cy, float* out_wx, float* out_wy) {
    *out_wx = (float)cx * (float)this->cell_size;
    *out_wy = (float)cy * (float)this->cell_size;
}

Status step(Context* context, float* out_offset_x, float* out_offset_y) {
    *out_offset_x = 5 - context->rover_x;
    *out_offset_y = 0 - context->rover_y;

    return Status::OK;
}

const char* get_name() {
    return "DUMMY";
}

}
