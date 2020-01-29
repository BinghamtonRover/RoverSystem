#include <GL/glew.h>
#include <stdio.h>
#include <stdlib.h> 

#include "program.hpp"

static char* read_entire_file(const char* file_name) {
    FILE* file = fopen(file_name, "r");
    if (!file) {
        fprintf(stderr, "[!] Failed to read file %s\n", file_name);
        exit(1);
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    char* file_buffer = (char*)malloc(file_size + 1);
    int bread = fread(file_buffer, 1, file_size, file);
    file_buffer[bread] = 0;

    fclose(file);

    return file_buffer;
}

static unsigned int load_shader(const char* source, unsigned int type) {
    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &source, NULL);
    glCompileShader(id);

    int shader_compiled;
    glGetShaderiv(id, GL_COMPILE_STATUS, &shader_compiled);
    if (shader_compiled != GL_TRUE) {
        char log_buffer[1024];
        glGetShaderInfoLog(id, 1024, NULL, log_buffer);
        const char* shader_str = type == GL_VERTEX_SHADER ? "vertex" : "fragment";
        fprintf(stderr, "[!] Failed to compile %s shader: %s\n", shader_str, log_buffer);
        fprintf(stderr, "%s\n", source);
        exit(1);
    }

    return id;
}

static unsigned int create_program(const char* vertex_shader_source, const char* fragment_shader_source) {
    unsigned int id = glCreateProgram();
    
    unsigned int vertex_id = load_shader(vertex_shader_source, GL_VERTEX_SHADER); 
    unsigned int fragment_id = load_shader(fragment_shader_source, GL_FRAGMENT_SHADER); 

    glAttachShader(id, vertex_id);
    glAttachShader(id, fragment_id);
    glLinkProgram(id);

    int program_linked;
    glGetProgramiv(id, GL_LINK_STATUS, &program_linked);
    if (program_linked != GL_TRUE) {
        char log_buffer[1024];
        glGetProgramInfoLog(id, 1024, NULL, log_buffer);
        fprintf(stderr, "[!] Failed to link program: %s\n", log_buffer);
        exit(1);
    }

    glDeleteShader(vertex_id);
    glDeleteShader(fragment_id);

    return id;
}

Program program_create(const char* vertex_shader_source, const char* fragment_shader_source) {
    return (Program) { create_program(vertex_shader_source, fragment_shader_source) };
}

Program program_create_from_files(const char* vertex_file, const char* fragment_file) {
    char* vertex_shader_source = read_entire_file(vertex_file);
    char* fragment_shader_source = read_entire_file(fragment_file);

    Program p = program_create(vertex_shader_source, fragment_shader_source);

    free(vertex_shader_source);
    free(fragment_shader_source);

    return p;
}

void program_set_uniform_float(Program* prog, int location, float value) {
    glProgramUniform1f(prog->id, location, value);
}

void program_set_uniform_vec2(Program* prog, int location, float x, float y) {
    glProgramUniform2f(prog->id, location, x, y);
}

void program_set_uniform_mat3(Program* prog, int location, Mat3f mat) {
    glProgramUniformMatrix3fv(prog->id, location, 1, GL_TRUE, mat.data);
}

void program_set_uniform_vec3(Program* prog, int location, float x, float y, float z) {
    glProgramUniform3f(prog->id, location, x, y, z);
}
