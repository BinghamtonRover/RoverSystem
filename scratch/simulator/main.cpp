#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "../../src/autonomy/autonomy.hpp"

#include "mutils.hpp"
#include "program.hpp"
#include "cell_model.hpp"
#include "grid_program.hpp"
#include "fill_program.hpp"
#include "rover_model.hpp"
#include "rover_program.hpp"
#include "obstacle.hpp"
#include "map.hpp"
#include "lidar.hpp"
#include "lidar_point_model.hpp"

const int WW = 1280;
const int WH = 720;

static void report_error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    fprintf(stderr, "[!] Error: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");

    va_end(args);
}

const int WINDOW_TITLE_BUFFER_SIZE = 1000;

// In milliseconds.
const int AUTONOMY_TICK_INTERVAL = 500;

const int GRID_SIZE = 400;
const float CELL_SIZE = 0.25;
const float BORDER_WIDTH = 0.04;
const float ROVER_SIZE = 0.9;
const float ROVER_WHEEL_AXIS_LENGTH = 0.9;

const float MIN_PPM = 20.0f;
const float MAX_PPM = 120.0f;

const float CONTROL_ROTATION_SPEED = 1.1f;

// In meters/sec.
const float CONTROL_MOVEMENT_SPEED = 0.25f;
const float ROVER_MAX_SPEED = 0.25f;

Mat3f projection;
Mat3f camera;

Map map;
autonomy::Context context;

// For control.
// float cvector_rotate = 0.0f;
// float cvector_move = 0.0f;

autonomy::OccupancyGrid frame_occupancy_grid;

float pixels_per_meter = MAX_PPM; // This is also the camera scale.
float x_offset = 0; // In pixels.
float y_offset = 0; // Same.
float last_cursor_x = 0;
float last_cursor_y = 0;

float rover_x = 0, rover_y = 0;
float rover_angle = 0;
int8_t left_set = 0, right_set = 0;
float prev_angle_to_path = 0;

// For timing.
long start_time;

static long timespec_to_millis(struct timespec* t) {
    return t->tv_sec * 1000 + t->tv_nsec / 1000000;
}

static void init_clock() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    start_time = timespec_to_millis(&now);
}

static long get_tick() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return timespec_to_millis(&now) - start_time;
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    /*
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
    } */ 
    if (key == GLFW_KEY_ESCAPE) {
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

static void wtc(float wx, float wy, int* out_cx, int* out_cy) {
    context.world_to_cell(wx, wy, out_cx, out_cy);
}

static void ctw(int cx, int cy, float* out_wx, float* out_wy) {
    context.cell_to_world(cx, cy, out_wx, out_wy);
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
    ctw(0, 0, &gcx, &gcy);

    mat3f_transformation_inplace(&model, context.cell_size, 0, gcx, gcy);
    fill_program_set_model(fill_program, model);

    fill_program_set_color(fill_program, 0, 1, 1);

    glUseProgram(fill_program->prog.id);
    glBindVertexArray(fill_model->model.vao);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 6);
}

static void fill_target(FillProgram* fill_program, CellModel* fill_model) {
    Mat3f model;

    float gcx, gcy;
    ctw(context.target_x, context.target_y, &gcx, &gcy);

    mat3f_transformation_inplace(&model, context.cell_size, 0, gcx, gcy);
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
    wtc(wcx, wcy, &cq, &cr);

    // How many cells fit on the screen?
    int screen_cells_x = (int)(((float)WW/pixels_per_meter) / context.cell_size);
    int screen_cells_y = (int)(((float)WH/pixels_per_meter) / context.cell_size);

    // Add some for a buffer.
    screen_cells_x += 2;
    screen_cells_y += 2;

    int q_offset = screen_cells_x / 2;
    int r_offset = screen_cells_y / 2;

    int r_start = cr - r_offset < -context.grid_size/2 ? -context.grid_size/2 : cr - r_offset;
    int r_end = cr + r_offset > context.grid_size/2 ? context.grid_size/2 - 1 : cr + r_offset;
    int q_start = cq - q_offset < -context.grid_size/2 ? -context.grid_size/2 : cq - q_offset;
    int q_end = cq + q_offset > context.grid_size/2 ? context.grid_size/2 - 1 : cq + q_offset;

    for (int r = r_start; r <= r_end; r++) {
        for (int q = q_start; q <= q_end; q++) {
            int occupancy = context.occupancy_grid.get(q, r);
            grid_program_set_background_alpha(grid_program, (float)occupancy/(float)context.occupancy_grid.max);

            float gcx, gcy;
            ctw(q, r, &gcx, &gcy);

            grid_program_set_cell_center(grid_program, gcx, gcy);

            mat3f_transformation_inplace(&model, context.cell_size, 0, gcx, gcy);
            grid_program_set_model(grid_program, model);

            glUseProgram(grid_program->prog.id);
            glBindVertexArray(cell_model->model.vao);
            glDrawArrays(GL_TRIANGLE_FAN, 0, 6);
        }
    }
}

void diff_steer(int8_t left, int8_t right, float delta){
    if(right > 127){
        right = 127;
    }
    if(left > 127){
        left = 127;
    }
    float left_velocity = (left / 127.0f) * ROVER_MAX_SPEED;
    float right_velocity = (right / 127.0f) * ROVER_MAX_SPEED;
    float rotational_velocity = (right_velocity - left_velocity) / ROVER_WHEEL_AXIS_LENGTH;
    float rotation_angle = (rotational_velocity * delta) * 180.0f / M_PI;
    float change_x = 0;
    float change_y = 0;
    // If we are turning (use radius/circle movement calculation)
    if(left != right){
        float radius = (ROVER_WHEEL_AXIS_LENGTH / 2.0f) * (left_velocity + right_velocity) / (right_velocity - left_velocity);
        change_x = cosf(rotation_angle * M_PI / 180.0f) * (radius * sinf(rover_angle * M_PI / 180.0f)) - sinf(rotation_angle * M_PI / 180.0f) * (-1 * radius * cosf(rover_angle * M_PI / 180.0f)) - (radius * sinf(rover_angle * M_PI / 180.0f));
        change_y = sinf(rotation_angle * M_PI / 180.0f) * (radius * sinf(rover_angle * M_PI / 180.0f)) + cosf(rotation_angle * M_PI / 180.0f) * (-1 * radius * cosf(rover_angle * M_PI / 180.0f)) + (radius * cosf(rover_angle * M_PI / 180.0f));
    }
    // If we are moving in a straight line (circle movement calculation makes no sense; radius would be undefined)
    else {
        // Note: In this case, left_velocity == right_velocity == velocity
        change_x = delta * left_velocity * cosf(rover_angle * M_PI / 180.0f);
        change_y = delta * left_velocity * sinf(rover_angle * M_PI / 180.0f);
    }

    rover_angle += rotation_angle;
    rover_x += change_x;
    rover_y += change_y;
}

// Returns the distance between two points
float getDistance(float x1, float y1, float x2, float y2){
    float delta_x = x2 - x1;
    float delta_y = y2 - y1;
    return(sqrt((delta_x * delta_x) + (delta_y * delta_y)));
}

// Controls the steepness of the curve.
const float INTENSITY_STEEPNESS = 50.0f;
// Controls how close to zero the curve levels out.
const float INTENSITY_LEVELOUT = 5.0f;

// Function to input angle_to_path and output the intensity decision
float getIntensity(float theta) {
    theta = fabsf(theta);

    // Use a modified ln(x) where f(0) = 0 and f(1) = 1 with variable steepness.
    float dominant_curve = (1.0f / logf(INTENSITY_STEEPNESS + 1)) * logf(INTENSITY_STEEPNESS*theta + 1);

    // Use plain old x^2.
    float levelout_curve = theta*theta;

    // This is the function that selects how much levelout to apply.
    // It is a modified e^(-x) with controlled steepness.
    float control_curve = (-1.0f / (expf(-INTENSITY_LEVELOUT) - 1.0f)) * expf(-INTENSITY_LEVELOUT*theta) + 1.0f + (1.0f / (expf(-INTENSITY_LEVELOUT) - 1.0f));


    return (1.0f - control_curve) * dominant_curve + control_curve * levelout_curve;
}

int main(int argc, char** argv) {
    init_clock();

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

        map.target_x = 50;
        map.target_y = 50;
    }

    // Start with the rover at the center.
    rover_x = 0;
    rover_y = 0;
    rover_angle = 90.0f;

    context = autonomy::create_context(
        rover_x,
        rover_y,
        rover_angle,
        map.target_x,
        map.target_y,
        CELL_SIZE,
        GRID_SIZE);

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

    snprintf(window_title_buffer, WINDOW_TITLE_BUFFER_SIZE, "BURT Autonomy Simulator (%s) [%s]", autonomy::get_name(), 
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

    grid_program_set_cell_size(&grid_program, context.cell_size);
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

    // Start with the rover moving forward.
    // cvector_move = 1.0f;

    frame_occupancy_grid = autonomy::OccupancyGrid::create(context.grid_size);

    // Timing for autonomy.
    long autonomy_last_tick = get_tick();

    // Timing for movement.
    long last_tick = get_tick();

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

        long now = get_tick();
        long time_diff = now - last_tick;
        last_tick = now;

        if (time_diff == 0) time_diff = 1;

        float delta = (float)time_diff / 1000.0f;
        diff_steer(left_set, right_set, delta);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        mat3f_camera_inplace(&camera, pixels_per_meter, -pixels_per_meter, 0.0f, x_offset, y_offset);
        grid_program_set_view(&grid_program, camera);
        fill_program_set_view(&fill_program, camera);
        rover_program_set_view(&rover_program, camera);

        fill_origin(&fill_program, &fill_model);
        fill_target(&fill_program, &fill_model);
        render_grid(&grid_program, &cell_model);
        render_rover(&rover_program, &rover_model);

        for (size_t i = 0; i < map.num_obstacles; i++) {
            render_obstacle(&fill_program, obstacles + i);
        }

        frame_occupancy_grid.clear();

        float lidar_points[LIDAR_NUM_POINTS];
        lidar_scan(rover_x, rover_y, rover_angle, obstacles, map.num_obstacles, lidar_points);
        for (int i = 0; i < LIDAR_NUM_POINTS; i++) {
            if (lidar_points[i] < LIDAR_RANGE) {
                float theta = (float)(i + LIDAR_SCAN_START) + rover_angle;
                float x = rover_x + lidar_points[i] * cosf(theta * M_PI / 180.0f);
                float y = rover_y + lidar_points[i] * sinf(theta * M_PI / 180.0f);

                int q, r; 
                wtc(x, y, &q, &r);
                
                frame_occupancy_grid.inc(q, r);

                render_lidar_point(&fill_program, &lidar_point_model, x, y);
            }
        }

        frame_occupancy_grid.copy_if_max(&context.occupancy_grid);

        // Update autonomy with occupancy_grid.
        long tick = get_tick();
        if (tick - autonomy_last_tick >= AUTONOMY_TICK_INTERVAL) {
            // Update the context for this frame.
            context.rover_x = rover_x;
            context.rover_y = rover_y;
            context.rover_angle = rover_angle;
            
            float target_offset_x, target_offset_y;
            auto autonomy_status = autonomy::step(&context, &target_offset_x, &target_offset_y);

            float target_x = rover_x + target_offset_x;
            float target_y = rover_y + target_offset_y;

            float dist_from_target = getDistance(rover_x, rover_y, target_x, target_y);
            printf("Distance: %f\n", dist_from_target);

            float target_angle = atan2f(target_y - rover_y, target_x - rover_x) * 180.0f / M_PI;

            printf("Target Angle: %f\n", target_angle);

            float adjusted_rover_angle = rover_angle - target_angle;
            if (adjusted_rover_angle < 0) {
                adjusted_rover_angle += 360;
            }

            printf("Angle: %f\n", adjusted_rover_angle);
            
            if(adjusted_rover_angle <= 180.0f){
                printf("Turn Right\n");
                left_set = 127;
                right_set = 127 - 256.0f * getIntensity(adjusted_rover_angle / 360.0f);
            } else {
                printf("Turn Left\n");
                left_set = 127 - 256.0f * getIntensity(adjusted_rover_angle / 360.0f);
                right_set = 127;
            }

            autonomy_last_tick = tick;
        }

        glfwSwapBuffers(window);
    }

    glfwTerminate();

    return 0;
}
