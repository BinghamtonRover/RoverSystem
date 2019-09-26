#ifndef HEX_MODEL_H
#define HEX_MODEL_H

#include "model.h"
#include "program.h"

typedef struct {
    Model model;

    int i_pos_loc;
} HexModel;

HexModel hex_model_create(Program* prog);

#endif
