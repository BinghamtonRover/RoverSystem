#include <math.h>

#include <GL/glew.h>

#include "cell_model.hpp"

CellModel cell_model_create(Program* prog) {
    CellModel cm;

    cm.model = model_create(1);
    cm.i_pos_loc = glGetAttribLocation(prog->id, "i_pos");

    float vertices[6 * 2];

    vertices[0] = 0;
    vertices[1] = 0;

    vertices[2] = -0.5;
    vertices[3] = -0.5;

    vertices[4] =  0.5;
    vertices[5] = -0.5;

    vertices[6] =  0.5;
    vertices[7] =  0.5;

    vertices[8] = -0.5;
    vertices[9] =  0.5;

    vertices[10] = -0.5;
    vertices[11] = -0.5;

    model_buffer_data(&cm.model, 0, vertices, sizeof(vertices));
    model_attrib_pointer(&cm.model, 0, cm.i_pos_loc, 2);

    return cm;
}
