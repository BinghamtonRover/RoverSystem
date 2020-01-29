#include <stdio.h>
#include <stdlib.h>

#include <GL/glew.h>

#include "model.hpp"

Model model_create(size_t num_vbos) {
    Model m;

    m.vbos = (unsigned int*)calloc(num_vbos, sizeof(unsigned int));
    
    glGenVertexArrays(1, &m.vao);
    glGenBuffers(num_vbos, m.vbos);

    return m;
}

void model_buffer_data(Model* model, size_t vbo_idx, float* data, size_t data_size) {
    glBindBuffer(GL_ARRAY_BUFFER, model->vbos[vbo_idx]);
    glBufferData(GL_ARRAY_BUFFER, data_size, data, GL_STATIC_DRAW);
}

void model_attrib_pointer(Model* model, size_t vbo_idx, int location, int components) {
    glBindVertexArray(model->vao);
    glBindBuffer(GL_ARRAY_BUFFER, model->vbos[vbo_idx]);
    glEnableVertexAttribArray(location);

    glVertexAttribPointer(location, components, GL_FLOAT, GL_FALSE, 0, NULL);
}
