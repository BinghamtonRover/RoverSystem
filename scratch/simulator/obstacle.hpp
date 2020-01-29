#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "model.hpp"
#include "program.hpp"

struct ObstacleModel {
    Model model;

    int i_pos_loc;
};

struct Obstacle {
    float* vertices;
    size_t num_vertices;

    ObstacleModel model;
};

Obstacle obstacle_create(Program* prog, float* vertices, size_t num_vertices);

#endif
