#version 430

const float PI = 3.1415926535;

in vec2 p_uv;

out vec4 out_color;

// From https://en.wikipedia.org/wiki/Barycentric_coordinate_system.
bool in_triangle(vec2 p, vec2 p1, vec2 p2, vec2 p3) {
    float det = (p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y);
    float l1 = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / det;
    float l2 = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / det;
    float l3 = 1 - l1 - l2;

    return l1 >= 0 && l2 >= 0 && l3 >= 0;
}

void main() {
    float pss = .60;
    float pse = .90;
    float ts = (pse - pss) / cos(PI/6);

    vec2 p0 = vec2(pss, 0.5 - 0.5*ts);
    vec2 p1 = vec2(pse, 0.5);
    vec2 p2 = vec2(pss, 0.5 + 0.5*ts);

    if (in_triangle(p_uv, p0, p1, p2)) {
        out_color = vec4(0, 1, 0, 1);
    } else {
        out_color = vec4(0, 0, 1, 1);
    }
}
