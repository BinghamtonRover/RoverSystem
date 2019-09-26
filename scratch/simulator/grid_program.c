#include <GL/glew.h>

#include "grid_program.h"

GridProgram grid_program_create() {
    GridProgram gp;

    gp.prog = program_create_from_files("shaders/grid_vertex.glsl", "shaders/grid_fragment.glsl");

    gp.u_model_loc = glGetUniformLocation(gp.prog.id, "u_model");
    gp.u_view_loc = glGetUniformLocation(gp.prog.id, "u_view");
    gp.u_projection_loc = glGetUniformLocation(gp.prog.id, "u_projection");
    gp.u_hex_center_loc = glGetUniformLocation(gp.prog.id, "u_hex_center");
    gp.u_border_width_loc = glGetUniformLocation(gp.prog.id, "u_border_width");
    gp.u_hex_size_loc = glGetUniformLocation(gp.prog.id, "u_hex_size");
    gp.u_background_alpha_loc = glGetUniformLocation(gp.prog.id, "u_background_alpha");

    return gp;
}

void grid_program_set_model(GridProgram* gp, Mat3f model) {
    program_set_uniform_mat3(&(gp->prog), gp->u_model_loc, model);
}

void grid_program_set_view(GridProgram* gp, Mat3f view) {
    program_set_uniform_mat3(&(gp->prog), gp->u_view_loc, view);
}

void grid_program_set_projection(GridProgram* gp, Mat3f projection) {
    program_set_uniform_mat3(&(gp->prog), gp->u_projection_loc, projection);
}

void grid_program_set_hex_center(GridProgram* gp, float x, float y) {
    program_set_uniform_vec2(&(gp->prog), gp->u_hex_center_loc, x, y);
}

void grid_program_set_hex_size(GridProgram* gp, float gs) {
    program_set_uniform_float(&(gp->prog), gp->u_hex_size_loc, gs);
}

void grid_program_set_border_width(GridProgram* gp, float bw) {
    program_set_uniform_float(&(gp->prog), gp->u_border_width_loc, bw);
}

void grid_program_set_background_alpha(GridProgram* gp, float a) {
    program_set_uniform_float(&(gp->prog), gp->u_background_alpha_loc, a);
}
