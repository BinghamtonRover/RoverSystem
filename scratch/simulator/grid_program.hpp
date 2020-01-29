#ifndef GRID_PROGRAM_H
#define GRID_PROGRAM_H

#include "program.hpp"
#include "mutils.hpp"

typedef struct {
    Program prog;

    int u_model_loc;
    int u_view_loc;
    int u_projection_loc;
    int u_cell_center_loc;
    int u_border_width_loc;
    int u_cell_size_loc;
    int u_background_alpha_loc;
} GridProgram;

GridProgram grid_program_create();

void grid_program_set_model(GridProgram* gp, Mat3f model);
void grid_program_set_view(GridProgram* gp, Mat3f view);
void grid_program_set_projection(GridProgram* gp, Mat3f projection);
void grid_program_set_cell_center(GridProgram* gp, float x, float y);
void grid_program_set_cell_size(GridProgram* gp, float gs);
void grid_program_set_border_width(GridProgram* gp, float bw);
void grid_program_set_background_alpha(GridProgram* gp, float a);

#endif
