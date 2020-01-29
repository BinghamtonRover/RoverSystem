#include <GL/glew.h>

#include "fill_program.hpp"

FillProgram fill_program_create() {
    FillProgram fp;

    fp.prog = program_create_from_files("shaders/fill_vertex.glsl", "shaders/fill_fragment.glsl");

    fp.u_model_loc = glGetUniformLocation(fp.prog.id, "u_model");
    fp.u_view_loc = glGetUniformLocation(fp.prog.id, "u_view");
    fp.u_projection_loc = glGetUniformLocation(fp.prog.id, "u_projection");
    fp.u_color_loc = glGetUniformLocation(fp.prog.id, "u_color");

    return fp;
}

void fill_program_set_model(FillProgram* fp, Mat3f model) {
    program_set_uniform_mat3(&(fp->prog), fp->u_model_loc, model);
}

void fill_program_set_view(FillProgram* fp, Mat3f view) {
    program_set_uniform_mat3(&(fp->prog), fp->u_view_loc, view);
}

void fill_program_set_projection(FillProgram* fp, Mat3f projection) {
    program_set_uniform_mat3(&(fp->prog), fp->u_projection_loc, projection);
}

void fill_program_set_color(FillProgram* fp, float r, float g, float b) {
    program_set_uniform_vec3(&(fp->prog), fp->u_color_loc, r, g, b);
}
