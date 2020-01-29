#include <GL/glew.h>

#include "lidar_point_model.hpp"

LidarPointModel lidar_point_model_create(Program* prog, float point_size) {
    LidarPointModel lpm;

    lpm.model = model_create(1);
    lpm.i_pos_loc = glGetAttribLocation(prog->id, "i_pos");

    float hs = point_size / 2;

    float vertices[] = {
        -hs, -hs,
        hs, -hs,
        -hs, hs,
        -hs, hs,
        hs, -hs,
        hs, hs
    };
    model_buffer_data(&lpm.model, 0, vertices, sizeof(vertices));
    model_attrib_pointer(&lpm.model, 0, lpm.i_pos_loc, 2);

    return lpm;
}
