#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>

#include "lidar.hpp"

const float LIDAR_ZERO_DELTA = 1e-2f;

// Returns true if they are able to intersect, and false if otherwise (parallel).
static bool calc_intersection_point(float s1x, float s1y, float e1x, float e1y, float s2x, float s2y, float e2x, float e2y, float* out_t1, float* out_t2) {
    float neg_det = e1x*e2y - e2x*e1y - e1x*s2y + e2x*s1y + e1y*s2x - e2y*s1x + s1x*s2y - s2x*s1y;
    
    if (fabsf(neg_det) < LIDAR_ZERO_DELTA) return false;

    *out_t1 = (e2x*s1y - e2y*s1x - e2x*s2y + e2y*s2x + s1x*s2y - s2x*s1y) / neg_det;
    *out_t2 = (e1x*s1y - e1y*s1x - e1x*s2y + e1y*s2x + s1x*s2y - s2x*s1y) / neg_det;

    return true;
}

static bool intersect(float startx, float starty, float endx, float endy, float p0x, float p0y, float p1x, float p1y, float* out_distance) {
    float t1, t2;
    if (!calc_intersection_point(startx, starty, endx, endy, p0x, p0y, p1x, p1y, &t1, &t2)) {
        return false;
    }

    if (t1 < 0.0f || t2 < 0.0f || t1 > 1.0f || t2 > 1.0f) {
        return false;
    }

    float ix = (endx - startx) * t1 + startx;
    float iy = (endy - starty) * t1 + starty;
    
    *out_distance = sqrtf((ix - startx)*(ix - startx) + (iy - starty)*(iy - starty));

    return true;
}

void lidar_scan(float rover_x, float rover_y, float rover_angle, Obstacle* obstacles, size_t num_obstacles, float* out_points) {
    int theta_start = LIDAR_SCAN_START;

    for (int theta_idx = 0; theta_idx < LIDAR_NUM_POINTS; theta_idx++) {
        float theta = theta_start + theta_idx + rover_angle;

        float endx = rover_x + LIDAR_RANGE*cosf(theta * M_PI / 180.0f);
        float endy = rover_y + LIDAR_RANGE*sinf(theta * M_PI / 180.0f);

        bool found_intersection = false;
        float min_dist = INFINITY;

        for (size_t i = 0; i < num_obstacles; i++) {
            for (size_t j = 0; j < obstacles[i].num_vertices; j++) {
                size_t index0 = j*2;
                size_t index1 = ((j+1) % obstacles[i].num_vertices) * 2;

                float p0x = obstacles[i].vertices[index0 + 0];
                float p0y = obstacles[i].vertices[index0 + 1];
                float p1x = obstacles[i].vertices[index1 + 0];
                float p1y = obstacles[i].vertices[index1 + 1];

                float dist;
                if (intersect(rover_x, rover_y, endx, endy, p0x, p0y, p1x, p1y, &dist)) {
                    if (dist < min_dist) {
                        min_dist = dist;
                        found_intersection = true;
                    }
                }
            }
        }

        if (found_intersection) {
            out_points[theta_idx] = min_dist;
        } else {
            out_points[theta_idx] = INFINITY;
        }
    }
}
