#ifndef ROVER_MODEL_H
#define ROVER_MODEL_H

#include "model.h"
#include "program.h"

typedef struct {
    Model model;

    int i_pos_loc;
    int i_uv_loc;
} RoverModel;

RoverModel rover_model_create(Program* prog, float rover_size);

#endif
