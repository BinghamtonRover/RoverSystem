#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

struct OccupancyGrid {
    int* grid;
    int grid_size;
    int max;
};

OccupancyGrid occupancy_grid_create(int grid_size);

void occupancy_grid_set(OccupancyGrid* grid, int q, int r, int val);

int occupancy_grid_get(OccupancyGrid* grid, int q, int r);

void occupancy_grid_inc(OccupancyGrid* grid, int q, int r);

void occupancy_grid_clear(OccupancyGrid* grid);

void occupancy_grid_copy_if_max(OccupancyGrid* from, OccupancyGrid* to);

#endif
