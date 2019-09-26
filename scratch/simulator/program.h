#ifndef PROGRAM_H
#define PROGRAM_H

#include "mutils.h"

typedef struct {
    unsigned int id;
} Program;

Program program_create(char* vertex_shader_source, char* fragment_shader_source);
Program program_create_from_files(char* vertex_file, char* fragment_file);

void program_set_uniform_float(Program* prog, int location, float value);
void program_set_uniform_vec2(Program* prog, int location, float x, float y);
void program_set_uniform_vec3(Program* prog, int location, float x, float y, float z);
void program_set_uniform_mat3(Program* prog, int location, Mat3f mat);

#endif
