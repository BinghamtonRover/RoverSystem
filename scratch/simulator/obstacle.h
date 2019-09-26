#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "model.h"
#include "program.h"

typedef struct {
    Model model;

    int i_pos_loc;
} ObstacleModel;

typedef struct {
    float* vertices;
    size_t num_vertices;

    ObstacleModel model;
} Obstacle;

Obstacle obstacle_create(Program* prog, float* vertices, size_t num_vertices);

#endif
