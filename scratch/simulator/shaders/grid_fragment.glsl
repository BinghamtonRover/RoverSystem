#version 430

const float PI = 3.1415926535897932384626433832795;

in vec2 p_world_pos;

uniform vec2 u_cell_center;
uniform float u_border_width;
uniform float u_cell_size;
uniform float u_background_alpha;

out vec4 out_color;

float dist_to_line(vec2 point, vec2 line_a, vec2 line_b) {
    vec2 line_start = line_a;
    vec2 line_direction = line_b - line_a;

    vec2 line_to_point = point - line_start;
    vec2 line_proj = (dot(line_to_point, line_direction) / dot(line_direction, line_direction)) * line_direction;
    vec2 line_rej = line_to_point - line_proj;

    return length(line_rej);
}

void main() {
    float gs = u_cell_size;
    float bw = u_border_width;

    out_color.rgba = vec4(1, 0, 0, u_background_alpha);
    
    vec2 point_0 = u_cell_center + (gs/2)*vec2(-1, -1);
    vec2 point_1 = u_cell_center + (gs/2)*vec2( 1, -1);
    vec2 point_2 = u_cell_center + (gs/2)*vec2( 1,  1);
    vec2 point_3 = u_cell_center + (gs/2)*vec2(-1,  1);

    if (dist_to_line(p_world_pos, point_0, point_1) < bw/2
        || dist_to_line(p_world_pos, point_1, point_2) < bw/2
        || dist_to_line(p_world_pos, point_2, point_3) < bw/2
        || dist_to_line(p_world_pos, point_3, point_0) < bw/2
    ) {
        out_color.rgba = vec4(0, 0, 0, 1);
    }
}
