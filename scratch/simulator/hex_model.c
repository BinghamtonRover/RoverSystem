#include <math.h>

#include <GL/glew.h>

#include "hex_model.h"

HexModel hex_model_create(Program* prog) {
    HexModel hm;

    hm.model = model_create(1);
    hm.i_pos_loc = glGetAttribLocation(prog->id, "i_pos");

    float vertices[8 * 2];
    vertices[0] = 0;
    vertices[1] = 0;
    for (int i = 0; i < 7; i++) {
        vertices[(i + 1)*2 + 0] = cosf(i * (M_PI / 3.0f));
        vertices[(i + 1)*2 + 1] = sinf(i * (M_PI / 3.0f));
    }
    model_buffer_data(&hm.model, 0, vertices, sizeof(vertices));
    model_attrib_pointer(&hm.model, 0, hm.i_pos_loc, 2);

    return hm;
}
