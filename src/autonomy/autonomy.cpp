/*
    Michael Schutzman GitHub Issue #122
    Implement VFH Algorithm into simulator
    VFH Algorithm:
    http://www-personal.umich.edu/~johannb/Papers/paper16.pdf
*/

/* useful online implimentations of VFH algorithm
    https://clearpathrobotics.com/blog/2014/05/vector-field-histogram/
    https://github.com/agarie/vector-field-histogram/blob/master/src/histogram_grid.c
    https://robotics.stackexchange.com/questions/9925/vfh-vector-field-histogram-obtaining-the-primary-polar-histogram
    http://www-personal.umich.edu/~johannb/vff&vfh.htm
    
*/
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "autonomy.hpp"

#define PI 3.14159265

namespace autonomy {

const int ws = 30;
const float B = 2;
const int alpha = 5;
const int l = 5;
const float threshold = 1;
float cell_certainty = 0;

struct Valley {
    int start, end;
}; 

static void get_polar_obstacle_densities(Context* ctx, float h[], int n, float A) {
    int c_x, c_y;
    float world_x, world_y;
    float cell_magnitude, cell_direction;
    ctx->world_to_cell(ctx->rover_x, ctx->rover_y, &c_x, &c_y);
    for (int i = (c_y - ws/2); i <= (c_y + ws/2); i++)
    {
        for (int j = (c_x - ws/2); j <= (c_x + ws/2); j++)
        {
            cell_direction = atan2(world_y - ctx->rover_y, world_x - ctx->rover_x) * 180.0 / PI;
            if (cell_direction < 0) {
                cell_direction += 360;
            }
            int k = (int)(cell_direction/ alpha);
            ctx->cell_to_world(j, i, &world_x, &world_y);
            // cell coords are world_x, world_y
            // check that obstacle is closer than target bc we don't care abt obstacles farther away than target
            if ( ( pow(ctx->rover_x - world_x, 2) + pow(ctx->rover_y - world_y, 2) ) < ( pow(ctx->rover_x - ctx->target_x, 2) + pow(ctx->rover_y - ctx->target_y, 2) ) ) {
                cell_magnitude = pow(ctx->occupancy_grid.get(j, i), 2) * (A - B * pow(pow(ctx->rover_y - world_y, 2) + pow(ctx->rover_x - world_x, 2), 0.5));
                h[k] += cell_magnitude;
            }
            else {
                h[k] += 0;
            }
        }
    }

    for (int k = 0; k <= (n-1); k++)
    {
        float sum = 0;
        int start = (k-l+1);
        int end = (k+l-1);
        if (start < 0)
        {
            start += n;
        }
        if (end < 0)
        {
            end += n;
        }
        if (start >= n)
        {
            start -= n;
        }
        if (end >= n)
        {
            end -= n;
        }

        for (int w = start; w <= end; w++)
        {
            int c = l-abs(w-k);
            if (w < 0)
            {
                w += n;
            }
            else if (w >= n && end != n)
            {
                w -= n;
            }
            sum += c * h[w];
        }
        h[k] = sum / (2*l-1);
    }
}

static void get_valleys(Valley valleys[], float h[], int n) {
    for (int i = 0; i < n/2; i++) {
        valleys[i].start = -1;
        valleys[i].end = -1;
    }
    int counter = 0;
    bool in_valley = false;
    for (int k = 0; k <= (n-1); k++)
    {
        if (h[k] <= threshold)
        {
            if (in_valley)
            {
                valleys[counter-1].end = k;
            }
            else
            {
                valleys[counter].start = k;
                valleys[counter].end = k;
                counter++;
                in_valley = true;
            }
        }
        else
        {
            in_valley = false;
        }
    }

    if (counter < 2)
    {
        return;
    }

    if (valleys[0].start == 0 && valleys[counter - 1].end == n-1)
    {
        Valley updated_valleys[n/2];
        valleys[0].start = valleys[counter - 1].start;
        valleys[0].end = n + valleys[0].end;
        valleys[counter - 1].start = -1;
        valleys[counter - 1].end = -1;
        for (int i = 0; i <= n/2 - 2; i++)
        {
            updated_valleys[i] = valleys[i];
        }
        valleys = updated_valleys;
    }
}

static int get_best_sector(Valley valleys[], int n, float target_sector) {
    double best_sector = -1;
    for (int i = 0; i < n/2; i++) {
        if (valleys[i].start != -1) {
            if (best_sector == -1) {
                best_sector = valleys[i].start;
            }
            if (valleys[i].end < n) {
                if (valleys[i].start <= target_sector && target_sector <= valleys[i].end) {
                    best_sector = target_sector;
                }
                else if (abs(target_sector - valleys[i].start) < abs(target_sector - valleys[i].end)) {
                    best_sector = valleys[i].start;
                }
                else {
                    best_sector = valleys[i].end;
                }
            }
            else {
                if (target_sector <= valleys[i].end - n) {
                    best_sector = target_sector;
                }
                else if (abs(target_sector - valleys[i].start) < abs(target_sector - (valleys[i].end - n))) {
                    best_sector = valleys[i].start;
                }
                else {
                    best_sector = valleys[i].end - n;
                }
            }
        }
    }
    return (int)best_sector;
}

Status step(Context* ctx, float* out_offset_x, float* out_offset_y) {
    const float dmax = pow(2, 0.5) * (ws - 1) / 2;
    const float A = dmax;
    const int n = 360/alpha;
    float densities[n];
    memset(densities, 0, sizeof(float)*n);
    float target_x = ctx->target_x;
    float target_y = ctx->target_y;
    float target_angle = atan2(target_y - ctx->rover_y, target_x - ctx->rover_x) * 180.0 / PI;
    if (target_angle < 0) {
        target_angle += 360;
    }
    int target_sector = (int)(target_angle / alpha);
    get_polar_obstacle_densities(ctx, densities, n, A);
    Valley valleys[n/2];
    get_valleys(valleys, densities, n);
    int sector = (int)get_best_sector(valleys, n, target_sector);
    double sector_direction = sector * alpha;
    *out_offset_x = cos((sector_direction * PI / 180.0));
    *out_offset_y = sin((sector_direction * PI / 180.0));

    return Status::OK;
}

const char* get_name() {
    return "VFH";
}

// Implementation stuff.

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

}
