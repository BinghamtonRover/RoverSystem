#ifndef FILL_PROGRAM_H
#define FILL_PROGRAM_H

#include "program.hpp"
#include "mutils.hpp"

struct FillProgram {
    Program prog;

    int u_model_loc;
    int u_view_loc;
    int u_projection_loc;
    int u_color_loc;
};

FillProgram fill_program_create();

void fill_program_set_model(FillProgram* fp, Mat3f model);

void fill_program_set_view(FillProgram* fp, Mat3f view);

void fill_program_set_projection(FillProgram* fp, Mat3f projection);

void fill_program_set_color(FillProgram* fp, float r, float g, float b);

#endif
