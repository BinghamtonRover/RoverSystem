#ifndef CELL_MODEL_H
#define CELL_MODEL_H

#include "model.hpp"
#include "program.hpp"

struct CellModel {
    Model model;

    int i_pos_loc;
};

CellModel cell_model_create(Program* prog);

#endif
