#ifndef MODEL_H
#define MODEL_H

typedef struct {
    unsigned int vao;

    unsigned int* vbos;
    size_t num_vbos;
} Model;

Model model_create(size_t num_vbos);

void model_buffer_data(Model* model, size_t vbo_idx, float* data, size_t data_size);

void model_attrib_pointer(Model* model, size_t vbo_idx, int location, int components);

#endif
