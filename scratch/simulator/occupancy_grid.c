#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "occupancy_grid.h"

OccupancyGrid occupancy_grid_create(int grid_size) {
    return (OccupancyGrid) {
        .grid = (int*) calloc(grid_size * grid_size, sizeof(int)),
        .grid_size = grid_size,
        .max = 1
    };
}

void occupancy_grid_set(OccupancyGrid* grid, int q, int r, int val) {
    if (val > grid->max) grid->max = val;

    q += grid->grid_size / 2;
    r += grid->grid_size / 2;

    grid->grid[q * grid->grid_size + r] = val;
}

int occupancy_grid_get(OccupancyGrid* grid, int q, int r) {
    q += grid->grid_size / 2;
    r += grid->grid_size / 2;

    return grid->grid[q * grid->grid_size + r];
}

void occupancy_grid_inc(OccupancyGrid* grid, int q, int r) {
    occupancy_grid_set(grid, q, r, occupancy_grid_get(grid, q, r) + 1);
}

void occupancy_grid_clear(OccupancyGrid* grid) {
    int gs = grid->grid_size;
    
    memset(grid->grid, 0, gs * gs * sizeof(int)); 
    
    grid->max = 1;
}

void occupancy_grid_copy_if_max(OccupancyGrid* from, OccupancyGrid* to) {
    assert(from->grid_size == to->grid_size);

    for (int q = 0; q < from->grid_size; q++) {
        for (int r = 0; r < from->grid_size; r++) {
            int val = from->grid[q * from->grid_size + r];
            if (val > to->grid[q * to->grid_size + r]) {
                if (val > to->max) {
                    to->max = val;
                }
                to->grid[q * from->grid_size + r] = val;
            }
        }
    }
}
