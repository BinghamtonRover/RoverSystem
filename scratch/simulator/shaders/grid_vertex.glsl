#version 430 

in vec2 i_pos;

uniform mat3 u_model;
uniform mat3 u_view;
uniform mat3 u_projection;

out vec2 p_world_pos;

void main() {
    gl_Position = vec4(u_projection * u_view * u_model * vec3(i_pos, 1), 1);

    p_world_pos = vec3(u_model * vec3(i_pos, 1)).xy;
}
