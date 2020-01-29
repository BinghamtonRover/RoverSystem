#include <GL/glew.h>

#include "rover_program.hpp"

RoverProgram rover_program_create() {
    RoverProgram rp;

    rp.prog = program_create_from_files("shaders/rover_vertex.glsl", "shaders/rover_fragment.glsl");

    rp.u_model_loc = glGetUniformLocation(rp.prog.id, "u_model");
    rp.u_view_loc = glGetUniformLocation(rp.prog.id, "u_view");
    rp.u_projection_loc = glGetUniformLocation(rp.prog.id, "u_projection");

    return rp;
}

void rover_program_set_model(RoverProgram* rp, Mat3f model) {
    program_set_uniform_mat3(&(rp->prog), rp->u_model_loc, model);
}

void rover_program_set_view(RoverProgram* rp, Mat3f view) {
    program_set_uniform_mat3(&(rp->prog), rp->u_view_loc, view);
}

void rover_program_set_projection(RoverProgram* rp, Mat3f projection) {
    program_set_uniform_mat3(&(rp->prog), rp->u_projection_loc, projection);
}
