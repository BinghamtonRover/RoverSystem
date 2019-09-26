#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "mutils.h"
#include "program.h"
#include "cell_model.h"
#include "grid_program.h"
#include "fill_program.h"
#include "rover_model.h"
#include "rover_program.h"
#include "obstacle.h"
#include "map.h"
#include "lidar.h"
#include "lidar_point_model.h"
#include "occupancy_grid.h"
#include "autonomy.h"

#define WW 1280
#define WH 720

static void report_error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    fprintf(stderr, "[!] Error: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");

    va_end(args);
}

#define WINDOW_TITLE_BUFFER_SIZE 1000

#define GRID_SIZE 50
#define CELL_SIZE 0.25
#define BORDER_WIDTH 0.04
#define ROVER_SIZE 0.9

#define MIN_PPM 20.0f
#define MAX_PPM 120.0f

#define CONTROL_ROTATION_SPEED 1.1f
#define CONTROL_MOVEMENT_SPEED 0.01f

Mat3f projection;
Mat3f camera;

Map map;

// For control.
float cvector_rotate = 0.0f;
float cvector_move = 0.0f;

OccupancyGrid occupancy_grid;
OccupancyGrid frame_occupancy_grid;

float pixels_per_meter = MAX_PPM; // This is also the camera scale.
float x_offset = 0; // In pixels.
float y_offset = 0; // Same.
float last_cursor_x = 0;
float last_cursor_y = 0;

float rover_x = 0, rover_y = 0;
float rover_angle = 0;

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_W) {
        if (action == GLFW_PRESS) cvector_move += 1.0f;
        else if (action == GLFW_RELEASE) cvector_move -= 1.0f;
    } else if (key == GLFW_KEY_S) {
        if (action == GLFW_PRESS) cvector_move -= 1.0f;
        else if (action == GLFW_RELEASE) cvector_move += 1.0f;
    } else if (key == GLFW_KEY_A) {
        if (action == GLFW_PRESS) cvector_rotate += 1.0f;
        else if (action == GLFW_RELEASE) cvector_rotate -= 1.0f;
    } else if (key == GLFW_KEY_D) {
        if (action == GLFW_PRESS) cvector_rotate -= 1.0f;
        else if (action == GLFW_RELEASE) cvector_rotate += 1.0f;
    } else if (key == GLFW_KEY_ESCAPE) {
        exit(0);
    }
}

static void scroll_callback(GLFWwindow* window, double x, double y) {
    float scale_factor = 0;
    
    if (y > 0) {
        scale_factor = 1.1f;
    } else if (y < 0) {
        scale_factor = 1.0f/1.1f;
    } else {
        return;
    }

    float world_x = (last_cursor_x - x_offset) / pixels_per_meter;
    float world_y = (last_cursor_y - y_offset) / pixels_per_meter;

    pixels_per_meter *= scale_factor;
    if (pixels_per_meter < MIN_PPM) {
        pixels_per_meter = MIN_PPM;
    } else if (pixels_per_meter > MAX_PPM) {
        pixels_per_meter = MAX_PPM;
    }

    float new_x = world_x * pixels_per_meter + x_offset;
    float new_y = world_y * pixels_per_meter + y_offset;

    x_offset -= new_x - last_cursor_x;
    y_offset -= new_y - last_cursor_y;
}

static void cursor_position_callback(GLFWwindow* window, double x, double y) {
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        float dx = x - last_cursor_x;
        float dy = y - last_cursor_y;

        x_offset += dx;
        y_offset += dy;
    }

    last_cursor_x = x;
    last_cursor_y = y;
}

static void world_to_cell(float wx, float wy, int* out_cx, int* out_cy) {
    *out_cx = roundf(wx / (float)CELL_SIZE);
    *out_cy = roundf(wy / (float)CELL_SIZE);
}

static void cell_to_world(int cx, int cy, float* out_wx, float* out_wy) {
    *out_wx = (float)cx * (float)CELL_SIZE;
    *out_wy = (float)cy * (float)CELL_SIZE;
}

static void render_obstacle(FillProgram* fill_program, Obstacle* obstacle) {
    Mat3f model = mat3f_transformation_copy(1, 0, 0, 0);
    fill_program_set_model(fill_program, model);

    fill_program_set_color(fill_program, 1, 1, 0);

    glUseProgram(fill_program->prog.id);
    glBindVertexArray(obstacle->model.model.vao);
    glDrawArrays(GL_TRIANGLE_FAN, 0, obstacle->num_vertices + 2);
}

static void render_rover(RoverProgram* rover_program, RoverModel* rover_model) {
    Mat3f model = mat3f_transformation_copy(1, rover_angle, rover_x, rover_y);
    rover_program_set_model(rover_program, model);

    glUseProgram(rover_program->prog.id);
    glBindVertexArray(rover_model->model.vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

static void render_lidar_point(FillProgram* fill_program, LidarPointModel* point_model, float x, float y) {
    Mat3f model = mat3f_transformation_copy(2, 0, x, y);
    fill_program_set_model(fill_program, model);

    fill_program_set_color(fill_program, 0, 1, 0);

    glUseProgram(fill_program->prog.id);
    glBindVertexArray(point_model->model.vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

static void fill_origin(FillProgram* fill_program, CellModel* fill_model) {
    Mat3f model;

    float gcx, gcy;
    cell_to_world(0, 0, &gcx, &gcy);

    mat3f_transformation_inplace(&model, CELL_SIZE, 0, gcx, gcy);
    fill_program_set_model(fill_program, model);

    fill_program_set_color(fill_program, 0, 1, 0);

    glUseProgram(fill_program->prog.id);
    glBindVertexArray(fill_model->model.vao);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 6);
}

static void render_grid(GridProgram* grid_program, CellModel* cell_model) {
    Mat3f model;

    // Figure out which cells are in view.
    // We start out with an estimate of the center cell.
    float wcx = (WW/2 - x_offset) / pixels_per_meter;
    float wcy = (y_offset - WH/2) / pixels_per_meter;

    int cq, cr;
    world_to_cell(wcx, wcy, &cq, &cr);

    // How many cells fit on the screen?
    int screen_cells_x = (int)((WW/pixels_per_meter) / GRID_SIZE);
    int screen_cells_y = (int)((WH/pixels_per_meter) / GRID_SIZE);

    // Add some for a buffer.
    screen_cells_x += 2;
    screen_cells_y += 2;

    int q_offset = screen_cells_x / 2;
    int r_offset = screen_cells_y / 2;

    int r_start = cr - r_offset < -GRID_SIZE/2 ? -GRID_SIZE/2 : cr - r_offset;
    int r_end = cr + r_offset > GRID_SIZE/2 ? GRID_SIZE/2 - 1 : cr + r_offset;
    int q_start = cq - q_offset < -GRID_SIZE/2 ? -GRID_SIZE/2 : cq - q_offset;
    int q_end = cq + q_offset > GRID_SIZE/2 ? GRID_SIZE/2 - 1 : cq + q_offset;

    // TODO: Only draw some!

    //for (int r = r_start; r <= r_end; r++) {
        //for (int q = q_start; q <= q_end; q++) {
    for (int r = -GRID_SIZE/2; r <= GRID_SIZE/2 - 1; r++) {
        for (int q = -GRID_SIZE/2; q <= GRID_SIZE/2 - 1; q++) {
            int occupancy = occupancy_grid_get(&occupancy_grid, q, r);
            grid_program_set_background_alpha(grid_program, (float)occupancy/(float)occupancy_grid.max);

            float gcx, gcy;
            cell_to_world(q, r, &gcx, &gcy);

            grid_program_set_cell_center(grid_program, gcx, gcy);

            mat3f_transformation_inplace(&model, CELL_SIZE, 0, gcx, gcy);
            grid_program_set_model(grid_program, model);

            glUseProgram(grid_program->prog.id);
            glBindVertexArray(cell_model->model.vao);
            glDrawArrays(GL_TRIANGLE_FAN, 0, 6);
        }
    }
}

int main(int argc, char** argv) {
    printf("%f\n", roundf(-0.6));
    if (argc > 1) {
        printf("> Loading map from '%s'\n", argv[1]);
        MapError err = map_load_from_file(&map, argv[1]);
        if (err != MAP_ERROR_OK) {
            fprintf(stderr, "[!] Failed to read map! Error: %s.\n", map_error_string(err));
            exit(1);
        }
    } else {
        map.obstacles = NULL;
        map.num_obstacles = 0;
    }

    if (!glfwInit()) {
        report_error("failed to init GLFW");
        return 1;
    }

    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 16);

    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    static char window_title_buffer[WINDOW_TITLE_BUFFER_SIZE];

    snprintf(window_title_buffer, WINDOW_TITLE_BUFFER_SIZE, "BURT Autonomy Simulator (%s) [%s]", autonomy_get_name(), 
        argc > 1 ? argv[1] : "NO FILE LOADED");

    GLFWwindow* window = glfwCreateWindow(WW, WH, window_title_buffer, NULL, NULL);
    if (!window) {
        report_error("failed to create window");
        return 1;
    }

    glfwMakeContextCurrent(window);

    glewInit();

    glfwSetScrollCallback(window, scroll_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetKeyCallback(window, key_callback);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_MULTISAMPLE);

    GridProgram grid_program = grid_program_create();
    CellModel cell_model = cell_model_create(&grid_program.prog);

    grid_program_set_cell_size(&grid_program, CELL_SIZE);
    grid_program_set_border_width(&grid_program, BORDER_WIDTH);

    FillProgram fill_program = fill_program_create();
    CellModel fill_model = cell_model_create(&fill_program.prog);

    RoverProgram rover_program = rover_program_create();
    RoverModel rover_model = rover_model_create(&rover_program.prog, ROVER_SIZE);

    LidarPointModel lidar_point_model = lidar_point_model_create(&fill_program.prog, 0.03);

    mat3f_projection_inplace(&projection, 0, WW, 0, WH);
    grid_program_set_projection(&grid_program, projection);
    fill_program_set_projection(&fill_program, projection);
    rover_program_set_projection(&rover_program, projection);

    // Load obstacles.
    Obstacle* obstacles = (Obstacle*) malloc(map.num_obstacles * sizeof(Obstacle));
    for (size_t i = 0; i < map.num_obstacles; i++) {
        float* obstacle_vertices = (float*) malloc(map.obstacles[i].num_vertices * 2 * sizeof(float));
        for (size_t j = 0 ; j < map.obstacles[i].num_vertices * 2; j++) {
            obstacle_vertices[j] = map.obstacles[i].vertices[j];
        }

        obstacles[i] = obstacle_create(&fill_program.prog, obstacle_vertices, map.obstacles[i].num_vertices);
    }

    // Start with the origin at the center.
    x_offset = WW/2;
    y_offset = WH/2;

    // Start with the rover at the center.
    rover_x = 0;
    rover_y = 0;
    rover_angle = 90.0f;

    occupancy_grid = occupancy_grid_create(GRID_SIZE);
    frame_occupancy_grid = occupancy_grid_create(GRID_SIZE);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);

            float wx = ((float)x - x_offset) / pixels_per_meter;
            float wy = (y_offset - (float)y) / pixels_per_meter;

            printf("%.2f, %.2f\n", wx, wy);
            printf("Rover: %.2f, %.2f\n", rover_x, rover_y);
        }

        rover_x += cvector_move * CONTROL_MOVEMENT_SPEED * cosf(rover_angle * M_PI / 180.0f);
        rover_y += cvector_move * CONTROL_MOVEMENT_SPEED * sinf(rover_angle * M_PI / 180.0f);
        rover_angle += cvector_rotate * CONTROL_ROTATION_SPEED;

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        mat3f_camera_inplace(&camera, pixels_per_meter, -pixels_per_meter, 0.0f, x_offset, y_offset);
        grid_program_set_view(&grid_program, camera);
        fill_program_set_view(&fill_program, camera);
        rover_program_set_view(&rover_program, camera);

        fill_origin(&fill_program, &fill_model);
        render_grid(&grid_program, &cell_model);
        render_rover(&rover_program, &rover_model);

        for (size_t i = 0; i < map.num_obstacles; i++) {
            render_obstacle(&fill_program, obstacles + i);
        }

        occupancy_grid_clear(&frame_occupancy_grid);

        float lidar_points[LIDAR_NUM_POINTS];
        lidar_scan(rover_x, rover_y, rover_angle, obstacles, map.num_obstacles, lidar_points);
        for (int i = 0; i < LIDAR_NUM_POINTS; i++) {
            if (lidar_points[i] < LIDAR_RANGE) {
                float theta = (float)(i + LIDAR_SCAN_START) + rover_angle;
                float x = rover_x + lidar_points[i] * cosf(theta * M_PI / 180.0f);
                float y = rover_y + lidar_points[i] * sinf(theta * M_PI / 180.0f);

                int q, r; 
                world_to_cell(x, y, &q, &r);
                
                occupancy_grid_inc(&frame_occupancy_grid, q, r);

                render_lidar_point(&fill_program, &lidar_point_model, x, y);
            }
        }

        occupancy_grid_copy_if_max(&frame_occupancy_grid, &occupancy_grid);

        // TODO: Update autonomy with occupancy_grid.

        glfwSwapBuffers(window);
    }

    glfwTerminate();

    return 0;
}
