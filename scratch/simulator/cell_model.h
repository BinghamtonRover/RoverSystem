#ifndef CELL_MODEL_H
#define CELL_MODEL_H

#include "model.h"
#include "program.h"

typedef struct {
    Model model;

    int i_pos_loc;
} CellModel;

CellModel cell_model_create(Program* prog);

#endif
