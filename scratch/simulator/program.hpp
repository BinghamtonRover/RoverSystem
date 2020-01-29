#ifndef PROGRAM_H
#define PROGRAM_H

#include "mutils.hpp"

struct Program {
    unsigned int id;
};

Program program_create(const char* vertex_shader_source, const char* fragment_shader_source);
Program program_create_from_files(const char* vertex_file, const char* fragment_file);

void program_set_uniform_float(Program* prog, int location, float value);
void program_set_uniform_vec2(Program* prog, int location, float x, float y);
void program_set_uniform_vec3(Program* prog, int location, float x, float y, float z);
void program_set_uniform_mat3(Program* prog, int location, Mat3f mat);

#endif
