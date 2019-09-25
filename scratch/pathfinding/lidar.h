#ifndef LIDAR_H
#define LIDAR_H

#include "obstacle.h"

#define LIDAR_NUM_POINTS (271)
#define LIDAR_RANGE (10)

#define LIDAR_SCAN_START (-135)
#define LIDAR_SCAN_END (135)


void lidar_scan(float rover_x, float rover_y, float rover_angle, Obstacle* obstacles, size_t num_obstacles, float* out_points);

#endif
