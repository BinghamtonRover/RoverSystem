#include <GL/glew.h>

#include "rover_model.h"

RoverModel rover_model_create(Program* prog, float rover_size) {
    RoverModel rm;

    rm.model = model_create(2);
    rm.i_pos_loc = glGetAttribLocation(prog->id, "i_pos");
    rm.i_uv_loc = glGetAttribLocation(prog->id, "i_uv");

    float hrs = rover_size / 2.0f;

    float vertices[] = {
        -hrs, -hrs,
        hrs, -hrs,
        -hrs, hrs,
        -hrs, hrs,
        hrs, -hrs,
        hrs, hrs
    };
    model_buffer_data(&rm.model, 0, vertices, sizeof(vertices));
    model_attrib_pointer(&rm.model, 0, rm.i_pos_loc, 2);

    float uvs[] = {
        0, 0,
        1, 0,
        0, 1,
        0, 1,
        1, 0,
        1, 1
    };
    model_buffer_data(&rm.model, 1, uvs, sizeof(uvs));
    model_attrib_pointer(&rm.model, 1, rm.i_uv_loc, 2);

    return rm;
}
