#ifndef LIDAR_POINT_MODEL_H
#define LIDAR_POINT_MODEL_H

#include "model.h"
#include "program.h"

typedef struct {
    Model model;

    int i_pos_loc;
} LidarPointModel;

LidarPointModel lidar_point_model_create(Program* prog, float point_size);

#endif
