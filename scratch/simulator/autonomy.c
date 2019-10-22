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
#include "autonomy.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

const int ws = 10;
const float B = 2;
const int alpha = 5;
const int l = 5;
const float cell_size = 1;
const float threshold = 0.3;
int target_x = 100;
int target_y = 100;
int * c_x = 0;
int * c_y = 0;
float cell_certainty = 0;
float cell_magnitude = 0;
float cell_direction = 0;
float * world_x;
float * world_y;
typedef struct {
    int start, end;
} Valley;

void get_polar_obstacle_densities(World world, float rover_x, float rover_y, float rover_angle, float *h, int n, float A) {
    world_to_cell(&world, rover_x, rover_y, c_x, c_y);
    for (int i = (*c_y - ws/2); i <= (*c_y + ws/2); i++)
    {
        for (int j = (*c_x - ws/2); j <= (*c_x + ws/2); j++)
        {
            cell_to_world(&world, *c_x, *c_y, world_x, world_x);
            cell_direction = atan2(*world_y - rover_y, *world_x - rover_x);
            cell_magnitude = pow(occupancy_grid_get(&world.occupancy_grid, *c_x, *c_y), 2) * (A - B * pow(pow(rover_y - *world_y, 2) + pow(rover_x - *world_x, 2), 0.5));
            int k = (int)(cell_direction/alpha);
            h[k] += cell_magnitude;
        }
    }

    for (int k = 0; k <= (n-1); k++)
    {
        float sum = 0;
        for (int w = (k-l+1); w <= (k+l-1); w++)
        {
            int c = l-abs(w-k);
            if (w < 0)
            {
                w += n;
            }
            else if (w >= n)
            {
                w -= n;
            }
            sum += c * h[w];
        }
        h[k] = sum / (2*l-1);
    }
}

Valley get_valleys(float h[], int n) {
    Valley valleys[n/2];
    int counter = 0;
    bool in_valley = false;
    for (int k = 0; k <= (n-1); k++)
    {
        if (h[k] <= threshold)
        {
            if (in_valley)
            {
                valleys[counter - 1].end = k;
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
        return *valleys;
    }

    if (valleys[0].start == 0 && valleys[counter - 1].end == n-1)
    {
        Valley updated_valleys[counter];
        valleys[0].start = valleys[counter - 1].start;
        valleys[0].end = n + valleys[0].end;
        for (int i = 0; i <= counter - 2; i++)
        {
            updated_valleys[i] = valleys[i];
        }
        return *updated_valleys;
    }

    return *valleys;
}

AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y) {
    const float dmax = pow(2, 0.5) * (ws - 1) / 2;
    const float A = dmax;
    const int n = 360/alpha;
    float densities[n];
    get_polar_obstacle_densities(*world, rover_x, rover_y, rover_angle, densities, n, A);
    Valley valleys = get_valleys(densities, n);
    *out_offset_x = 0.5;
    *out_offset_y = 0.5;

    return AS_OK;
}

const char* autonomy_get_name() {
    return "VFH";
}
