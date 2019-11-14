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
#include <string.h>

#define PI 3.14159265
const int ws = 10;
const float B = 2;
const int alpha = 5;
const int l = 5;
const float cell_size = 1;
const float threshold = 0.3;
float cell_certainty = 0;
typedef struct {
    int start, end;
} Valley;

void get_polar_obstacle_densities(World world, float rover_x, float rover_y, float rover_angle, float h[], int n, float A) {
    int c_x, c_y;
    float world_x, world_y;
    float cell_magnitude, cell_direction;
    world_to_cell(&world, rover_x, rover_y, &c_x, &c_y);
    for (int i = (c_y - ws/2); i <= (c_y + ws/2); i++)
    {
        for (int j = (c_x - ws/2); j <= (c_x + ws/2); j++)
        {
            
            //cell_to_world(&world, c_x, c_y, &world_x, &world_y);
            cell_to_world(&world, j, i, &world_x, &world_y);
            cell_direction = atan2(world_y - rover_y, world_x - rover_x) * 180 / PI;
            cell_direction += 180; // converts from angle -180-180 to angle 0-360
            cell_magnitude = pow(occupancy_grid_get(&world.occupancy_grid, j, i), 2) * (A - B * pow(pow(rover_y - world_y, 2) + pow(rover_x - world_x, 2), 0.5));
            int k = (int)(cell_direction/alpha);
            printf("index value: %d\n", k);
            printf("cell MAGNITUDE value: %f\n", cell_magnitude);
            h[k] += cell_magnitude;
            //h[k] = k;
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

void get_valleys(Valley valleys[], float h[], int n) {
    //h[24] = 0;
    //h[31] = 0;
    //h[32] = 0;
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

double get_best_sector(Valley valleys[], int n, float target_sector) {
    double error = -1;
    double average = -1;
    int min_error = n + 1;
    double best_sector = -1;
    for (int i = 0; i < n/2; i++) {
        if (valleys[i].start != -1) {
            average = (valleys[i].start + valleys[i].end) / 2.0;
            if (average > n) {
                average -= n;
            }
            error = abs(target_sector - average);
            if (error > -1 && error < min_error) {
                min_error = error;
                best_sector = average;
            }
        }
        error = -1;
    }
    return best_sector;
}

AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y) {
    const float dmax = pow(2, 0.5) * (ws - 1) / 2;
    const float A = dmax;
    const int n = 360/alpha;
    float densities[n];
    memset(densities, 0, sizeof(float)*n);
    float target_x = 3;
    float target_y = 5;
    float target_angle = atan2f(target_y - rover_y, target_x - rover_x) * 180 / PI;
    int target_sector = (int)(target_angle / alpha);
    get_polar_obstacle_densities(*world, rover_x, rover_y, rover_angle, densities, n, A);
    for (int i = 0; i < sizeof(densities)/sizeof(densities[0]); i++)
    {
        printf("THIS IS THE DENSITIES %d: %f\n", i, densities[i]);
    }
    Valley valleys[n/2];
    get_valleys(valleys, densities, n);
    for (int i = 0; i < sizeof(valleys)/sizeof(valleys[0]); i++)
    {
        printf("THIS IS THE VALLEYS  %d: (%d, %d)\n", i, valleys[i].start, valleys[i].end);
    }
    double sector = get_best_sector(valleys, n, target_sector);
    printf("THE BEST SECTOR IS SECTOR %f", sector);
    double sector_direction = sector * alpha - 180;
    printf("sector direction is %f", sector_direction);
    *out_offset_x = cos(sector_direction);
    *out_offset_y = sin(sector_direction);
    //*out_offset_x = -1;
    //*out_offset_y = 0;

    return AS_OK;
}

const char* autonomy_get_name() {
    return "VFH";
}
