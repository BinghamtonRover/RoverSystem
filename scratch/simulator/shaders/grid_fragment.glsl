#version 430

const float PI = 3.1415926535897932384626433832795;

in vec2 p_world_pos;

uniform vec2 u_hex_center;
uniform float u_border_width;
uniform float u_hex_size;
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
    float gs = u_hex_size;
    float bw = u_border_width;

    out_color.rgba = vec4(1, 0, 0, u_background_alpha);

    for (float theta = 0; theta < 2*PI; theta += PI/3) {
        vec2 p1 = u_hex_center + vec2(gs * cos(theta), gs * sin(theta));
        vec2 p2 = u_hex_center + vec2(gs * cos(theta + PI/3), gs * sin(theta + PI/3));

        if (dist_to_line(p_world_pos, p1, p2) < bw/2) {
            out_color.rgba = vec4(0, 0, 0, 1);
        }
    }
}
