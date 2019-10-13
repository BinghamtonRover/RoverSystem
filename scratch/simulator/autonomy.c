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
#include "main.c"
#include "world.c"

const float dmax = 0;
const float A = dmax;
const float B = 2;
const int ws = 10;
const int alpha = 5;
const int n = 360/alpha;
const int l = 5;
const float cell_size = 1;
const float threshold = 0.3;
int target_x = 100;
int target_y = 100;
int c_x = 0;
int c_y = 0;
float cell_certainty = 0;
float cell_magnitude = 0;
float cell_direction = 0;
float rover_x = 0;
float rover_y = 0;
float world_x = 0;
float world_y = 0;
World world = world; 


AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y) {
    densities = get_polar_obstacle_densities();
    valleys = get_valleys(densities);
    *out_offset_x = 0.5;
    *out_offset_y = 0.5;

    return AS_OK;
}   //loop thru cells in active region, calculate beta and magnitude
//const int A,B
/* cx, cy (i, j)
    cell_certainty = c*
    cell_magnitude
    cell_direction
    rover_x
    rover_y
    world_x
    world_y (xi, xj)
    a = dmax
    b = 2
    ws const, dmax const
    alpha is const at compile time
    h_k is current sector we're looking at
    l hardcoded
    bounds are inclusive
    beta again with x,y target 
    on key press, output best sector

*/

const char* autonomy_get_name() {
    return "VFH";
}

[] get_polar_obstacle_densities() {
    float h[n] = {0};
    world_to_cell(world, rover_x, rover_y, c_x, c_y);
    for (i = (c_y - ws/2); i <= (c_y + ws/2); i++)
    {
        for (j = (c_x - ws/2); i <= (c_x + ws/2); i++)
        {
            cell_to_world(world, c_x, c_y, world_x, world_x);
            cell_direction = atan2(world_y - rover_y, world_x - rover_x);
            cell_magnitude = pow(occupancy_grid_get(grid, c_x, c_y), 2) * (dmax - pow(pow(rover_y - world_y, 2) + pow(rover_x - world_x, 2), 0.5);
            int k = (int)(cell_direction/alpha);
            h[k] += cell_magnitude;
        }
    }

    for (k = 0; k <= (n-1); k++)
    {
        float sum = 0;
        for (w = (k-l+1); w <= (k+l-1); w++)
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
    return h;
}

[] get_valleys(h) {
    valleys = {start, end}[n/2];
    bool in_valley = false;
    for (k = 0; k <= (n-1); k++)
    {
        if (h[k] <= threshold)
        {
            if (in_valley)
            {
                valleys[valleys.length - 1].end = k;
            }
            else
            {
                valleys.append({k, k});
                in_valley = true;
            }
        }
        else
        {
            in_valley = false;
        }
    }

    if (valleys.length < 2)
    {
        return valleys;
    }

    if (valleys[0].start == 0 and valleys[valleys.length - 1].end == n-1)
    {
        valleys[0] = {valleys[valleys.length - 1].start, n + valleys[0].end};
        float updated_valleys[n-1] = {0};
        for (i = 0; i <= valleys.length - 2; i++)
        {
            updated_valleys[i] = valleys[i];
        }
    }

    return updated_valleys;
}
