#ifndef LIDAR_POINT_MODEL_H
#define LIDAR_POINT_MODEL_H

#include "model.hpp"
#include "program.hpp"

struct LidarPointModel {
    Model model;

    int i_pos_loc;
};

LidarPointModel lidar_point_model_create(Program* prog, float point_size);

#endif
