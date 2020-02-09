#ifndef AUTONOMY_H
#define AUTONOMY_H
namespace autonomy {

enum class Status {
    OK,
    DONE,
    ERROR
};

struct OccupancyGrid {
    int* grid;
    int grid_size;
    int max;

    static OccupancyGrid create(int grid_size);

    void set(int q, int r, int val);
    int get(int q, int r);
    void inc(int q, int r);
    void clear();
    void copy_if_max(OccupancyGrid* to);
};

struct Context {
    float rover_x;
    float rover_y;
    float rover_angle;

    float target_x, target_y;

    OccupancyGrid occupancy_grid;
    float cell_size; // Size of each grid cell in meters.
    int grid_size; // Total size of one grid size in cells.

    // Then have user pass raw LIDAR info.
    // The simulator can then access the context's grid and pass the simulated LIDAR points.

    void world_to_cell(float wx, float wy, int* out_cx, int* out_cy);
    void cell_to_world(int cx, int cy, float* out_wx, float* out_wy);
};

Context create_context(float rover_x, float rover_y, float rover_angle, float target_x, float target_y, float cell_size, int grid_size);

Status step(Context* context, float* out_offset_x, float* out_offset_y);

const char* get_name();

}
#endif
