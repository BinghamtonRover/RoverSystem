#include <GL/glew.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "obstacle.h"

Obstacle obstacle_create(Program* prog, float* vertices, size_t num_vertices) {
    Obstacle obs;

    obs.vertices = vertices;
    obs.num_vertices = num_vertices;
    
    obs.model.model = model_create(1);
    obs.model.i_pos_loc = glGetAttribLocation(prog->id, "i_pos");

    float avg_x = 0;
    float avg_y = 0;
    int n = 0;

    for (size_t i = 0; i < num_vertices; i++) {
        n++;

        float x = vertices[i * 2 + 0];
        float y = vertices[i * 2 + 1];

        avg_x = ((n - 1) * avg_x + x) / n;
        avg_y = ((n - 1) * avg_y + y) / n;
    }

    float* render_vertices = (float*)malloc((num_vertices + 2) * 2 * sizeof(float));
    
    render_vertices[0] = avg_x;
    render_vertices[1] = avg_y;

    memcpy(render_vertices + 2, vertices, num_vertices * 2 * sizeof(float));

    render_vertices[(num_vertices + 1)*2 + 0] = vertices[0];
    render_vertices[(num_vertices + 1)*2 + 1] = vertices[1];

    model_buffer_data(&obs.model.model, 0, render_vertices, (num_vertices + 2) * 2 * sizeof(float));
    model_attrib_pointer(&obs.model.model, 0, obs.model.i_pos_loc, 2);

    return obs;
}
