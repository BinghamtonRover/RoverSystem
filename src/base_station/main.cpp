#include "../network/network.hpp"
#include "../simple_config/simpleconfig.h"
#include "../util/util.hpp"

#include "camera_feed.hpp"
#include "controller.hpp"
#include "debug_console.hpp"
#include "gui.hpp"
#include "log_view.hpp"
#include "logger.hpp"
#include "waypoint.hpp"
#include "waypoint_map.hpp"

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// For control smoothing.
const float ALPHA = 30;

// Speed for the DPAD up/down.
const int16_t JOINT_DRIVE_SPEED = 50;

// Send movement updates 3x per second.
const int MOVEMENT_SEND_INTERVAL = 1000 / 20;

// Update network statistics once per second.
const int NETWORK_STATS_INTERVAL = 1000;

const int LOG_VIEW_WIDTH = 572;
const int LOG_VIEW_HEIGHT = 458;

const int PRIMARY_FEED_WIDTH = 1298;
const int PRIMARY_FEED_HEIGHT = 730;

const int SECONDARY_FEED_WIDTH = 533;
const int SECONDARY_FEED_HEIGHT = 300;

// Network feeds.
network::Feed r_feed, bs_feed;

// TODO: Refactor texture ids and fonts (and etc.) into some GuiResources thingy.
gui::Font global_font;

unsigned int map_texture_id;
unsigned int stopwatch_texture_id;

float last_rover_tick = 0;

enum class StopwatchState { STOPPED, PAUSED, RUNNING };

struct {
    StopwatchState state;
    unsigned int start_time;
    unsigned int pause_time;
} stopwatch;

struct {
    float r_tp = 0;
    float bs_tp = 0;
    float t_tp = 0;
} last_network_stats;

// Clock!
util::Clock global_clock;

std::vector<std::string> split_by_spaces(std::string s) {
    std::vector<std::string> strings;

    size_t last = 0;
    size_t next = 0;

    while ((next = s.find(" ", last)) != std::string::npos) {
        strings.push_back(s.substr(last, next - last));
        last = next + 1;
    }

    strings.push_back(s.substr(last));

    return strings;
}

network::MovementMessage last_movement_message = { 0, 0 };

void command_callback(std::string command) {
    auto parts = split_by_spaces(command);

    if (parts.size() == 0) {
        return;
    }

    if (parts[0] == "move") {
        if (parts.size() != 3) {
            return;
        }

        char left_direction_char = parts[1][0];
        int16_t left_speed = (int16_t) atoi(parts[1].substr(1).c_str());

        char right_direction_char = parts[2][0];
        int16_t right_speed = (int16_t) atoi(parts[2].substr(1).c_str());

        last_movement_message.left = left_speed;
        last_movement_message.right = right_speed;

        if (left_direction_char == 'b') {
            last_movement_message.left *= -1;
        }

        if (right_direction_char == 'b') {
            last_movement_message.right *= -1;
        }

        logger::log(
            logger::DEBUG,
            "> Update movement to %d, %d",
            last_movement_message.left,
            last_movement_message.right);
    }
}

void stderr_handler(logger::Level level, std::string message) {
    fprintf(stderr, "%s\n", message.c_str());
}

void log_view_handler(logger::Level level, std::string message) {
    float r, g, b, a = 1.0f;

    switch (level) {
        case logger::DEBUG:
            r = 0.70f;
            g = 0.70f;
            b = 0.70f;
            break;
        case logger::INFO:
            r = 1.0f;
            g = 1.0f;
            b = 1.0f;
            break;
        case logger::WARNING:
            r = 1.00f;
            g = 0.52f;
            b = 0.01f;
            break;
        case logger::ERROR:
            r = 1.0f;
            g = 0.0f;
            b = 0.0f;
            break;
    }

    time_t current_time;
    time(&current_time);
    struct tm* time_info = localtime(&current_time);

    char time_string_buffer[200];
    strftime(time_string_buffer, sizeof(time_string_buffer), "[%H:%M:%S] ", time_info);

    std::string full_string = std::string(time_string_buffer) + message;

    gui::log_view::print(&global_font, LOG_VIEW_WIDTH, full_string, r, g, b, a);
}

struct Config {
    int rover_port;
    int base_station_port;

    char rover_multicast_group[16]; // Max length of string ipv4 addr is 15, plus one for nt.
    char base_station_multicast_group[16];

    static const int MAX_PREFERRED_MONITOR_LEN = 32;
    char preferred_monitor[MAX_PREFERRED_MONITOR_LEN + 1];
};

Config load_config(const char* filename) {
    Config config;

    sc::SimpleConfig* sc_config;

    auto err = sc::parse(filename, &sc_config);
    if (err != sc::Error::OK) {
        logger::log(logger::ERROR, "Failed to parse config file: %s", sc::get_error_string(sc_config, err));
        exit(1);
    }

    char* rover_port = sc::get(sc_config, "rover_port");
    if (!rover_port) {
        logger::log(logger::ERROR, "Config missing 'rover_port'!");
        exit(1);
    }
    config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        logger::log(logger::ERROR, "Config missing 'base_station_port'!");
        exit(1);
    }
    config.base_station_port = atoi(base_station_port);

    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'rover_multicast_group'!");
        exit(1);
    }
    strncpy(config.rover_multicast_group, rover_multicast_group, 16);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'base_station_multicast_group'!");
        exit(1);
    }
    strncpy(config.base_station_multicast_group, base_station_multicast_group, 16);

    char* preferred_monitor = sc::get(sc_config, "preferred_monitor");
    if (!preferred_monitor) {
        // Default: empty string means primary monitor.
        config.preferred_monitor[0] = 0;
    } else {
        strncpy(config.preferred_monitor, preferred_monitor, Config::MAX_PREFERRED_MONITOR_LEN + 1);
    }

    sc::free(sc_config);

    return config;
}

void set_stopwatch_icon_color() {
    switch (stopwatch.state) {
        case StopwatchState::RUNNING:
            glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
            break;
        case StopwatchState::PAUSED:
            glColor4f(1.0f, 0.67f, 0.0f, 1.0f);
            break;
        default:
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            break;
    }
}

const char* get_stopwatch_text() {
    static char buffer[50];

    if (stopwatch.state == StopwatchState::STOPPED) {
        sprintf(buffer, "00:00:00");

        return buffer;
    }

    unsigned int time_to_use;

    if (stopwatch.state == StopwatchState::PAUSED) {
        time_to_use = stopwatch.pause_time;
    } else {
        time_to_use = global_clock.get_millis();
    }

    unsigned int millis = time_to_use - stopwatch.start_time;

    unsigned int seconds = millis / 1000;
    unsigned int minutes = seconds / 60;
    seconds = seconds % 60;
    unsigned int hours = minutes / 60;
    minutes = minutes % 60;

    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);

    return buffer;
}

void do_info_panel(gui::Layout* layout, gui::Font* font) {
    static char info_buffer[200];

    int x = layout->current_x;
    int y = layout->current_y;

    int w = 445;
    int h = 300;

    gui::do_solid_rect(layout, w, h, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);

    sprintf(
        info_buffer,
        "Rover net status: %s",
        r_feed.status == network::FeedStatus::ALIVE ? "alive" : "dead");

    if (r_feed.status == network::FeedStatus::ALIVE) {
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    } else {
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    }
    gui::draw_text(font, info_buffer, x + 5, y + 5, 15);

    sprintf(
        info_buffer,
        "Net thpt (r/bs/t): %.2f/%.2f/%.2f MiB/s",
        last_network_stats.r_tp,
        last_network_stats.bs_tp,
        last_network_stats.t_tp);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    gui::draw_text(font, info_buffer, x + 5, y + 20 + 5, 15);

    sprintf(info_buffer, "Rover lat/lon: %.6f,%.6f", waypoint::rover_latitude, waypoint::rover_longitude);
    gui::draw_text(font, info_buffer, x + 5, y + 40 + 5, 15);

    sprintf(info_buffer, "Rover tps: %.0f", last_rover_tick);
    gui::draw_text(font, info_buffer, x + 5, y + 60 + 5, 15);

    time_t current_time;
    time(&current_time);
    struct tm* time_info = localtime(&current_time);

    strftime(info_buffer, sizeof(info_buffer), "%I:%M:%S", time_info);

    gui::draw_text(font, info_buffer, x + 5, y + h - 20 - 5, 20);

    auto stopwatch_buffer = get_stopwatch_text();

    int stopwatch_text_width = gui::text_width(font, stopwatch_buffer, 20);

    gui::draw_text(font, stopwatch_buffer, x + w - 5 - stopwatch_text_width, y + h - 20 - 5, 20);

    set_stopwatch_icon_color();
    gui::fill_textured_rect_mix_color(
        x + w - 5 - stopwatch_text_width - 3 - 20,
        y + h - 20 - 5,
        20,
        20,
        stopwatch_texture_id);
}

void do_stopwatch_menu(gui::Font* font) {
    const int w = 150;
    const int h = 110;

    const int x = WINDOW_WIDTH - 20 - w;
    const int y = WINDOW_HEIGHT - 20 - h;

    glColor4f(0.2f, 0.2f, 0.2f, 1.0f);
    gui::fill_rectangle(x, y, w, h);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    gui::draw_text(font, "Stopwatch", x + 5, y + 5, 20);

    const char* space_help_text;
    switch (stopwatch.state) {
        case StopwatchState::STOPPED:
            space_help_text = "<space> : start";
            break;
        case StopwatchState::PAUSED:
            space_help_text = "<space> : resume";
            break;
        case StopwatchState::RUNNING:
            space_help_text = "<space> : pause";
            break;
    }

    gui::draw_text(font, space_help_text, x + 5, y + 5 + 20 + 15, 15);
    gui::draw_text(font, "r       : reset", x + 5, y + 5 + 20 + 15 + 5 + 15, 15);

    // Draw the actual stopwatch.

    auto stopwatch_buffer = get_stopwatch_text();
    int stopwatch_text_width = gui::text_width(font, stopwatch_buffer, 20);
    gui::draw_text(
        font,
        stopwatch_buffer,
        WINDOW_WIDTH - 20 - stopwatch_text_width - 5,
        WINDOW_HEIGHT - 20 - 5 - 20,
        20);

    set_stopwatch_icon_color();
    gui::fill_textured_rect_mix_color(
        WINDOW_WIDTH - 20 - 5 - stopwatch_text_width - 3 - 20,
        WINDOW_HEIGHT - 20 - 5 - 20,
        20,
        20,
        stopwatch_texture_id);
}

void do_help_menu(gui::Font* font, std::vector<const char*> commands, std::vector<const char*> debug_commands) {
    // Texts have a fixed height and spacing
    int height = 20;
    int spacing = height + 10;

    const char* title = "Help Menu";
    int title_height = 25;
    int title_width = gui::text_width(font, title, title_height);
    int debug_title_height = 25;
    const char* debug_title = "Debug Commands";
    int debug_title_width = gui::text_width(font, debug_title, debug_title_height);
    const char* exit_prompt = "Press 'escape' to exit menu";

    int max_width = debug_title_width;
    for (unsigned int i = 0; i < commands.size(); i++) {
        int current_width = gui::text_width(font, commands[i], height);
        if (current_width > max_width) max_width = current_width;
    }
    for (unsigned int i = 0; i < debug_commands.size(); i++) {
        int current_width = gui::text_width(font, debug_commands[i], height);
        if (current_width > max_width) max_width = current_width;
    }

    int right_padding = 150;
    int bottom_padding = (spacing * 1.5) + 15; // The +15 is to account for the exit prompt
    int top_padding = 20;
    int menu_width = max_width + right_padding;
    int menu_height = (spacing * commands.size()) + (spacing * debug_commands.size()) + title_height +
        debug_title_height + top_padding + bottom_padding;

    gui::Layout help_layout{};

    help_layout.advance_x((WINDOW_WIDTH / 2) - (menu_width / 2));
    help_layout.advance_y((WINDOW_HEIGHT / 2) - (menu_height / 2));
    help_layout.push();

    int x = help_layout.current_x;
    int y = help_layout.current_y;

    gui::do_solid_rect(&help_layout, menu_width, menu_height, 0.2, 0.2f, 0.2);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int left_margin = 5;

    gui::draw_text(font, title, x + (menu_width / 2) - (title_width / 2), y, title_height);

    for (unsigned int i = 0; i < commands.size(); i++) {
        gui::draw_text(font, commands[i], x + left_margin, y + (spacing * i) + top_padding + title_height, height);
    }

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    int debug_commands_x = x + (menu_width / 2) - (debug_title_width / 2);
    int debug_commands_y = y + (spacing * commands.size()) + top_padding + title_height;
    gui::draw_text(font, debug_title, debug_commands_x, debug_commands_y, debug_title_height);
    debug_commands_y += 10; // To give extra room for the commands after the debug title

    for (unsigned int i = 0; i < debug_commands.size(); i++) {
        gui::draw_text(font, debug_commands[i], x + left_margin, debug_commands_y + (spacing * (i + 1)), height);
    }

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int exit_prompt_width = gui::text_width(font, exit_prompt, 15);
    gui::draw_text(font, exit_prompt, x + (menu_width / 2) - (exit_prompt_width / 2), y + menu_height - 20, 15);

    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glLineWidth(4.0f);
    glBegin(GL_LINES);

    // Footer
    glVertex2f(x, y + menu_height - 20);
    glVertex2f(x + menu_width, y + menu_height - 20);

    // Title Line
    glVertex2f(x, y + title_height + 10);
    glVertex2f(x + menu_width, y + title_height + 10);

    // Before debug title
    glVertex2f(x, debug_commands_y - 8);
    glVertex2f(x + menu_width, debug_commands_y - 8);

    // After debug title
    glVertex2f(x, debug_commands_y + debug_title_height + 2);
    glVertex2f(x + menu_width, debug_commands_y + debug_title_height + 2);

    glEnd();

    help_layout.pop();
}

std::vector<uint16_t> lidar_points;

void do_lidar(gui::Layout* layout) {
    /*
    int wx = layout->current_x;
    int wy = layout->current_y;
    */

    gui::do_solid_rect(layout, 300, 300, 0, 0, 0);

    /*

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glLineWidth(2.0f);

    for (int q = -3; q <= 3; q++) {
        for (int r = -3; r <= 3; r++) {
            float x = 1.2f * (3.0f / 2.0f) * q;
            float y = 1.2f * ((sqrtf(3.0f)/2.0f) * q + sqrtf(3) * r);

            float ppm = 150.0f / 10.0f;

            float px = ppm * x;
            float py = ppm * y;

            glBegin(GL_LINE_LOOP);

            for (int i = 0; i < 6; i++) {
                float angle = M_PI * (float)i / 3.0f;

                float vx = wx + 150.0f + px + ppm * 1.2 * cosf(angle);
                float vy = wy + 150.0f + py + ppm * 1.2 * sinf(angle);

                glVertex2f(vx, vy);
            }

            glEnd();
        }
    }
    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

    glBegin(GL_QUADS);

    for (size_t i = 0; i < lidar_points.size(); i++) {
        float angle = 225.0f - (float) i;
        float theta = angle * M_PI / 180.0f;

        theta -= M_PI;

        uint16_t dist = lidar_points[i];

        float r = dist * 150.0f / 10000.0f;

        float hs = 1.0f;

        float x = wx + 150.0f + r * cosf(theta);
        float y = wy + 150.0f + r * sinf(theta);

        if (dist < 100) continue;

        glVertex2f(x - hs, y - hs);
        glVertex2f(x + hs, y - hs);
        glVertex2f(x + hs, y + hs);
        glVertex2f(x - hs, y + hs);
    }

    float hs = 1.2 * 15.0f / 2.0f;

    float x = wx + 150.0f;
    float y = wy + 150.0f;

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

    glVertex2f(x - hs, y - hs);
    glVertex2f(x + hs, y - hs);
    glVertex2f(x + hs, y + hs);
    glVertex2f(x - hs, y + hs);

    glEnd();
    */ 
}

// Camera stuff.
// These get initialized off-the-bat.
int primary_feed = 0;
int secondary_feed = 1;

// We only care about this value when we are in camera move mode.
int feed_to_move = -1;

const int MAX_FEEDS = 9;
camera_feed::Feed camera_feeds[MAX_FEEDS];

void do_camera_move_target(gui::Font* font) {
    const float COVER_ALPHA = 0.5f;
    glColor4f(1.0f, 1.0f, 1.0f, COVER_ALPHA);
    gui::fill_rectangle(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    const int PRIMARY_TEXT_SIZE = 50;
    const int SECONDARY_TEXT_SIZE = 30;

    const char* primary_text = "Press '0' to move feed here";
    const char* secondary_text = "Press '1' to move feed here";

    // TODO: These are hardcoded from the do_gui layout stuff.
    // Maybe calculate the positions of everything at runtime start?
    const int X = 20 + 572 + 10;

    const int Y_PRIMARY = 20;
    const int Y_SECONDARY = Y_PRIMARY + PRIMARY_FEED_HEIGHT + 10;

    int primary_text_width = gui::text_width(font, primary_text, PRIMARY_TEXT_SIZE);
    int secondary_text_width = gui::text_width(font, secondary_text, SECONDARY_TEXT_SIZE);

    int ptx = X + (PRIMARY_FEED_WIDTH / 2) - (primary_text_width / 2);
    int pty = Y_PRIMARY + (PRIMARY_FEED_HEIGHT / 2);

    int stx = X + (SECONDARY_FEED_WIDTH / 2) - (secondary_text_width / 2);
    int sty = Y_SECONDARY + (SECONDARY_FEED_HEIGHT / 2);

    const int BG_PADDING = 10;

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    gui::fill_rectangle(
        ptx - BG_PADDING,
        pty - BG_PADDING,
        primary_text_width + 2 * BG_PADDING,
        PRIMARY_TEXT_SIZE + 2 * BG_PADDING);
    gui::fill_rectangle(
        stx - BG_PADDING,
        sty - BG_PADDING,
        secondary_text_width + 2 * BG_PADDING,
        SECONDARY_TEXT_SIZE + 2 * BG_PADDING);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    gui::draw_text(font, primary_text, ptx, pty, PRIMARY_TEXT_SIZE);
    gui::draw_text(font, secondary_text, stx, sty, SECONDARY_TEXT_SIZE);
}

void do_camera_matrix(gui::Font* font) {
    switch (gui::state.input_state) {
        case gui::InputState::CAMERA_MATRIX:
        case gui::InputState::CAMERA_MOVE:
            break;
        case gui::InputState::CAMERA_MOVE_TARGET:
            do_camera_move_target(font);
            return;
        default:
            return;
    }

    const int SIDE_MARGIN = 50;
    const int TOP_MARGIN = 50;

    const int BACKGROUND_SHADE = 40;

    const int MATRIX_WIDTH = WINDOW_WIDTH - 2 * SIDE_MARGIN;
    const int MATRIX_HEIGHT = WINDOW_HEIGHT - 2 * SIDE_MARGIN;

    const int MATRIX_PADDING = 25;

    glColor4f(BACKGROUND_SHADE / 255.0f, BACKGROUND_SHADE / 255.0f, BACKGROUND_SHADE / 255.0f, 1.0f);
    gui::fill_rectangle(SIDE_MARGIN, TOP_MARGIN, MATRIX_WIDTH, MATRIX_HEIGHT);

    const int VIEW_WIDTH = 500;
    const int VIEW_HEIGHT = VIEW_WIDTH * 9 / 16;
    const int NUM_VIEWS_PER_ROW = (MATRIX_WIDTH + MATRIX_PADDING) / (VIEW_WIDTH + MATRIX_PADDING);
    const int NUM_ROWS = (MAX_FEEDS + NUM_VIEWS_PER_ROW - 1) / NUM_VIEWS_PER_ROW;
    const int WIDTH_OF_ROW = NUM_VIEWS_PER_ROW * (VIEW_WIDTH + MATRIX_PADDING) + MATRIX_PADDING;
    const int EXTRA_SIDE_PADDING = (MATRIX_WIDTH - WIDTH_OF_ROW) / 2;

    const int TEXT_PADDING = 5;
    const int TEXT_SIZE = 20;

    // Stuff will overflow if we go over one digit!
    // Plus 2 for ": ".
    const int TEXT_MAX_LEN = camera_feed::FEED_NAME_MAX_LEN + 1 + 2;
    // Can be equal since ids are 0-9.
    assert(MAX_FEEDS <= 10);

    int feed_idx = 0;
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_VIEWS_PER_ROW; c++) {
            if (feed_idx >= MAX_FEEDS) break;

            int view_x = SIDE_MARGIN + MATRIX_PADDING + EXTRA_SIDE_PADDING + c * (VIEW_WIDTH + MATRIX_PADDING);
            int view_y = TOP_MARGIN + MATRIX_PADDING + r * (VIEW_HEIGHT + MATRIX_PADDING);

            gui::fill_textured_rect(view_x, view_y, VIEW_WIDTH, VIEW_HEIGHT, camera_feeds[feed_idx].gl_texture_id);

            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            static char feed_display_buffer[TEXT_MAX_LEN + 1];
            snprintf(feed_display_buffer, sizeof(feed_display_buffer), "%d: %s", feed_idx, camera_feeds[feed_idx].name);

            draw_text(
                font,
                feed_display_buffer,
                view_x + TEXT_PADDING,
                view_y + VIEW_HEIGHT - TEXT_SIZE - TEXT_PADDING,
                TEXT_SIZE);

            feed_idx++;
        }
    }

    const int HELP_TEXT_SIZE = 15;

    const char* help_text;
    if (gui::state.input_state == gui::InputState::CAMERA_MATRIX) {
        help_text = "m: move camera | escape: exit matrix";
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE) {
        help_text = "escape: cancel move";
    } else {
        help_text = "?";
    }

    int help_text_width = text_width(font, help_text, HELP_TEXT_SIZE);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    draw_text(
        font,
        help_text,
        WINDOW_WIDTH - SIDE_MARGIN - TEXT_PADDING - help_text_width,
        WINDOW_HEIGHT - TOP_MARGIN - TEXT_PADDING - HELP_TEXT_SIZE,
        HELP_TEXT_SIZE);

    if (gui::state.input_state == gui::InputState::CAMERA_MOVE) {
        glColor4f(255.0f / 255.0f, 199.0f / 255.0f, 0.0f, 1.0f);
        draw_text(
            font,
            "Press key '0'-'9' to select feed to move",
            SIDE_MARGIN + TEXT_PADDING,
            WINDOW_HEIGHT - TOP_MARGIN - TEXT_PADDING - HELP_TEXT_SIZE,
            HELP_TEXT_SIZE);
    }
}

void do_gui(gui::Font* font) {
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gui::Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Renders a map that shows where the rover is relative to other waypoints, the current waypoints, and its orientation
    gui::waypoint_map::do_waypoint_map(&layout,572,572);
    
    layout.reset_x();
    layout.advance_y(10);

    // Draw the log.
    gui::log_view::do_log(&layout, LOG_VIEW_WIDTH, LOG_VIEW_HEIGHT, font);

    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed.
    gui::do_textured_rect(&layout, PRIMARY_FEED_WIDTH, PRIMARY_FEED_HEIGHT, camera_feeds[primary_feed].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    // Draw the other camera feed.
    layout.reset_y();
    gui::do_textured_rect(
        &layout,
        SECONDARY_FEED_WIDTH,
        SECONDARY_FEED_HEIGHT,
        camera_feeds[secondary_feed].gl_texture_id);

    layout.reset_y();
    layout.advance_x(10);

    // Draw the lidar.
    do_lidar(&layout);

    layout.reset_y();
    layout.advance_x(10);

    // Draw the info panel.
    do_info_panel(&layout, font);

    int help_text_width = gui::text_width(font, "Press 'h' for help", 15);
    glColor4f(0.0f, 0.5f, 0.0f, 1.0f);
    gui::draw_text(font, "Press 'h' for help", WINDOW_WIDTH - help_text_width - 2, WINDOW_HEIGHT - 18, 15);

    // Draw the debug overlay.
    layout = {};
    gui::debug_console::do_debug(&layout, font);

    // Draw the camera matrix.
    // Note: this currently needs to be called last here in order for the camera movement
    // effects to work properly!
    do_camera_matrix(font);
}

void glfw_character_callback(GLFWwindow* window, unsigned int codepoint) {
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (codepoint < 128) {
            gui::debug_console::handle_input((char) codepoint);
        }
    }
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            gui::debug_console::handle_keypress(key, mods);
        }
    } else if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
        // We open the menu on release to prevent the D key from being detected
        // in the character callback upon release.
        if (action == GLFW_RELEASE && key == GLFW_KEY_D) {
            gui::state.show_debug_console = true;
            gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_C) {
            if (mods & GLFW_MOD_SHIFT) {
                gui::state.input_state = gui::InputState::CAMERA_MATRIX;
            } else {
                int temp = primary_feed;
                primary_feed = secondary_feed;
                secondary_feed = temp;
            }
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MATRIX) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_M) {
            gui::state.input_state = gui::InputState::CAMERA_MOVE;
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::CAMERA_MATRIX;
        } else if (action == GLFW_PRESS) {
            int selected_feed_idx = -1;
            switch (key) {
                case GLFW_KEY_0:
                    selected_feed_idx = 0;
                    break;
                case GLFW_KEY_1:
                    selected_feed_idx = 1;
                    break;
                case GLFW_KEY_2:
                    selected_feed_idx = 2;
                    break;
                case GLFW_KEY_3:
                    selected_feed_idx = 3;
                    break;
                case GLFW_KEY_4:
                    selected_feed_idx = 4;
                    break;
                case GLFW_KEY_5:
                    selected_feed_idx = 5;
                    break;
                case GLFW_KEY_6:
                    selected_feed_idx = 6;
                    break;
                case GLFW_KEY_7:
                    selected_feed_idx = 7;
                    break;
                case GLFW_KEY_8:
                    selected_feed_idx = 8;
                    break;
                case GLFW_KEY_9:
                    selected_feed_idx = 9;
                    break;
            }

            if (selected_feed_idx >= 0 && selected_feed_idx < MAX_FEEDS) {
                feed_to_move = selected_feed_idx;
                gui::state.input_state = gui::InputState::CAMERA_MOVE_TARGET;
            }
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE_TARGET) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::CAMERA_MATRIX;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_0) {
            primary_feed = feed_to_move;
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_1) {
            secondary_feed = feed_to_move;
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        }
    }
}

// Takes values between 0 and 255 and returns them between 0 and 255.
float smooth_rover_input(float value) {
    // We want to exponentially smooth this.
    // We do that by picking alpha in (1, whatever).
    // The higher the alpha, the more exponential the thing.
    // We then make sure that f(0) = 0 and f(1) = 255.

    return (255.0f / (ALPHA - 1)) * (powf(1 / ALPHA, -value / 255.0f) - 1);
}

int main() {
    util::Clock::init(&global_clock);

    logger::register_handler(stderr_handler);

    gui::debug_console::set_callback(command_callback);

    // Load config.
    Config config = load_config("res/bs.sconfig");

    // Clear the stopwatch.
    stopwatch.state = StopwatchState::STOPPED;
    stopwatch.start_time = 0;
    stopwatch.pause_time = 0;

    // Init GLFW.
    if (!glfwInit()) {
        logger::log(logger::ERROR, "Failed to init GLFW!");
        return 1;
    }

    GLFWmonitor* monitor_to_use = glfwGetPrimaryMonitor();

    if (config.preferred_monitor[0] != 0) {
        // A monitor was specified. Does it exist?
        int num_monitors;
        GLFWmonitor** monitors = glfwGetMonitors(&num_monitors);

        bool found = false;

        for (int i = 0; i < num_monitors; i++) {
            if (strncmp(config.preferred_monitor, glfwGetMonitorName(monitors[i]), Config::MAX_PREFERRED_MONITOR_LEN) == 0) {
                monitor_to_use = monitors[i];
                found = true;
                break;
            }
        }

        if (!found) logger::log(logger::WARNING, "Preferred monitor %s not found!", config.preferred_monitor);
    }

    // Get the correct resolution for the monitor we want to use.
    const GLFWvidmode* mode = glfwGetVideoMode(monitor_to_use);

    // Create a fullscreen window. Title isn't displayed, so doesn't really
    // matter.
    GLFWwindow* window =
        glfwCreateWindow(mode->width, mode->height, "Base Station", monitor_to_use, NULL);

    // Update the window so everyone can access it.
    gui::state.window = window;

    // Set sticky keys mode. It makes our input work as intended.
    // glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

    // Disable mouse.
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

    glfwSetCharCallback(window, glfw_character_callback);
    glfwSetKeyCallback(window, glfw_key_callback);

    // Create an OpenGL context.
    glfwMakeContextCurrent(window);

    // OpenGL Setup.
    glViewport(0, 0, mode->width, mode->height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, 0, 0, 0.5);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Initial setup for GUI here so that network errors are printed to log view.
    map_texture_id = gui::load_texture("res/binghamton.jpg");
    stopwatch_texture_id = gui::load_texture_alpha("res/stopwatch_white.png");

    // Load this down here so that sizing is correct.
    gui::Font font;
    bool loaded_font = gui::load_font(&font, "res/FiraMono-Regular.ttf", 100);
    if (!loaded_font) {
        logger::log(logger::ERROR, "Failed to load font!");
        return 1;
    }
    // TODO: Fix this hack (for the log view handler) and clean up globals in general.
    global_font = font;

    // TODO: Make everything just use the global font.
    gui::state.font = font;

    logger::register_handler(log_view_handler);

    // Initialize camera stuff.
    camera_feed::init();

    // Create the camera streams.
    for (int i = 0; i < MAX_FEEDS; i++) {
        static char init_feed_name_buffer[camera_feed::FEED_NAME_MAX_LEN + 1];
        snprintf(init_feed_name_buffer, sizeof(init_feed_name_buffer), "UNKNOWN-%d", i);

        camera_feed::init_feed(camera_feeds + i, init_feed_name_buffer, 1280, 720);
    }

    // Initialize network functionality.
    {
        auto err = network::init(
            &bs_feed,
            network::FeedType::OUT,
            config.base_station_multicast_group,
            config.base_station_port,
            &global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "Failed to init base station feed: %s", network::get_error_string(err));
            return 1;
        }

        logger::log(
            logger::INFO,
            "Network: publishing base station feed on %s:%d",
            config.base_station_multicast_group,
            config.base_station_port);
    }

    {
        auto err = network::init(
            &r_feed,
            network::FeedType::IN,
            config.rover_multicast_group,
            config.rover_port,
            &global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "Failed to init rover feed: %s", network::get_error_string(err));
            return 1;
        }

        logger::log(
            logger::INFO,
            "Network: subscribed to rover feed on %s:%d",
            config.rover_multicast_group,
            config.rover_port);
    }

    // Keep track of when we last sent movement info.
    util::Timer movement_send_timer;
    util::Timer::init(&movement_send_timer, MOVEMENT_SEND_INTERVAL, &global_clock);

    // Grab network stats every second.
    util::Timer network_stats_timer;
    util::Timer::init(&network_stats_timer, NETWORK_STATS_INTERVAL, &global_clock);

    // Init the controller.
    // TODO: QUERY /sys/class/input/js1/device/id/{vendor,product} TO FIND THE RIGHT CONTROLLER.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js1") == controller::Error::OK) {
        controller_loaded = true;
        logger::log(logger::INFO, "Controller connected.");
    } else {
        logger::log(logger::WARNING, "No controller connected!");
    }

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    bool help_menu_up = false;
    bool stopwatch_menu_up = false;

    // Add the help menu commands here
    std::vector<const char*> commands;
    std::vector<const char*> debug_commands;
    commands.push_back("d: Show debug console");
    commands.push_back("ctrl + q: Exit");
    commands.push_back("<shift>c: Open camera matrix");
    commands.push_back("c: Swap camera feeds");
    commands.push_back("s: Open stopwatch menu");
    commands.push_back("z + UP ARROW: Zoom in map");
    commands.push_back("z + DOWN ARROW: Zoom out map");
    commands.push_back("z + r: Reset map");
    debug_commands.push_back("'test': displays red text");
    debug_commands.push_back("'aw <number> <number>: adds a waypoint (in latitude and longitude)");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        bool z_on = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;

        if (z_on && (glfwGetKey(window,GLFW_KEY_UP) == GLFW_PRESS)) {
            gui::waypoint_map::zoom_in();    
        } else if (z_on && (glfwGetKey(window,GLFW_KEY_DOWN) == GLFW_PRESS)) {
            gui::waypoint_map::zoom_out();    
        } else if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                gui::log_view::moveTop();
            } else {
                gui::log_view::moveUpOne();
            }
        } else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                gui::log_view::moveBottom();
            } else {
                gui::log_view::moveDownOne();
            }
        } else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                break;
            }
        } else if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
            if (gui::state.input_state == gui::InputState::KEY_COMMAND) help_menu_up = true;
        } else if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            if (help_menu_up) help_menu_up = false;
            if (stopwatch_menu_up) {
                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            }
        } else if (
            glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            stopwatch_menu_up = true;
            gui::state.input_state = gui::InputState::STOPWATCH_MENU;
        }

        if (gui::state.input_state == gui::InputState::STOPWATCH_MENU) {
            if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
                switch (stopwatch.state) {
                    case StopwatchState::RUNNING:
                        stopwatch.state = StopwatchState::PAUSED;
                        stopwatch.pause_time = global_clock.get_millis();
                        break;
                    case StopwatchState::PAUSED:
                        stopwatch.state = StopwatchState::RUNNING;
                        stopwatch.start_time =
                            global_clock.get_millis() - (stopwatch.pause_time - stopwatch.start_time);
                        break;
                    case StopwatchState::STOPPED:
                        stopwatch.state = StopwatchState::RUNNING;
                        stopwatch.start_time = global_clock.get_millis();
                        break;
                }

                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            } else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
                switch (stopwatch.state) {
                    case StopwatchState::RUNNING:
                        stopwatch.start_time = global_clock.get_millis();
                        break;
                    case StopwatchState::PAUSED:
                        stopwatch.state = StopwatchState::STOPPED;
                        break;
                    case StopwatchState::STOPPED:
                        break;
                }

                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            }
        }

        // Handle incoming network messages from the rover feed.
        network::IncomingMessage message;
        while (true) {
            auto neterr = network::receive(&r_feed, &message);
            if (neterr != network::Error::OK) {
                if (neterr == network::Error::NOMORE) {
                    break;
                } else {
                    logger::log(
                        logger::WARNING,
                        "Failed to read network packets: %s",
                        network::get_error_string(neterr));
                    break;
                }
            }

            switch (message.type) {
                case network::MessageType::CAMERA: {
                    // Static buffer so we don't have to allocate and reallocate every
                    // frame.
                    static uint8_t camera_message_buffer[camera_feed::CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE];

                    network::CameraMessage camera_message;
                    camera_message.data = camera_message_buffer;
                    network::deserialize(&message.buffer, &camera_message);

                    auto camerr = camera_feed::handle_section(
                        camera_feeds + camera_message.stream_index,
                        camera_message.data,
                        camera_message.size,
                        camera_message.section_index,
                        camera_message.section_count,
                        camera_message.frame_index);

                    if (camerr != camera_feed::Error::OK) {
                        logger::log(logger::WARNING, "Failed to handle video frame section!");
                    }

                    break;
                }
                case network::MessageType::LIDAR: {
                    lidar_points.clear();

                    network::LidarMessage lidar_message;
                    network::deserialize(&message.buffer, &lidar_message);

                    for (int i = 0; i < network::NUM_LIDAR_POINTS; i++) {
                        lidar_points.push_back(lidar_message.points[i]);
                    }

                    break;
                }
                case network::MessageType::TICK: {
                    network::TickMessage tick_message;
                    network::deserialize(&message.buffer, &tick_message);

                    last_rover_tick = tick_message.ticks_per_second;
                    break;
                }
                case network::MessageType::LOCATION: {
                    network::LocationMessage location_message;
                    network::deserialize(&message.buffer,&location_message);
                    waypoint::set_rover_coordinates(location_message.latitude,location_message.longitude);
                    //waypoint::rover_latitude = location_message.latitude;
                    //rover_longitude = location_message.longitude;

                    break;
                }
                default:
                    break;
            }
        }

        // Update network stuff.
        network::update_status(&r_feed);
        network::update_status(&bs_feed);

        if (network_stats_timer.ready()) {
            last_network_stats.r_tp = (float) r_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            last_network_stats.bs_tp = (float) bs_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            last_network_stats.t_tp = last_network_stats.r_tp + last_network_stats.bs_tp;

            r_feed.bytes_transferred = 0;
            bs_feed.bytes_transferred = 0;
        }

        if (controller_loaded) {
            // Process controller input.
            controller::Event event;
            controller::Error err;

            // Do nothing since we just want to update current values.
            while ((err = controller::poll(&event)) == controller::Error::OK) {
                if (event.type == controller::EventType::AXIS) {
                    bool forward = event.value <= 0;
                    int16_t abs_val = event.value < 0 ? -event.value : event.value;

                    if (event.axis == controller::Axis::JS_LEFT_Y) {
                        if (controller::get_value(controller::Axis::DPAD_X) != 0 ||
                            controller::get_value(controller::Axis::DPAD_Y) != 0) {
                            continue;
                        }

                        int16_t smoothed = (int16_t) smooth_rover_input((float) (abs_val >> 7));
                        logger::log(logger::DEBUG, "Left orig: %d, smooth: %d", abs_val, smoothed);

                        last_movement_message.left = smoothed;
                        if (!forward) last_movement_message.left *= -1;
                    } else if (event.axis == controller::Axis::JS_RIGHT_Y) {
                        if (controller::get_value(controller::Axis::DPAD_X) != 0 ||
                            controller::get_value(controller::Axis::DPAD_Y) != 0) {

                            continue;
                        }

                        int16_t smoothed = (int16_t) smooth_rover_input((float) (abs_val >> 7));
                        logger::log(logger::DEBUG, "Right orig: %d, smooth: %d", abs_val, smoothed);

                        last_movement_message.right = smoothed;
                        if (!forward) last_movement_message.right *= -1;
                    } else if (event.axis == controller::Axis::DPAD_Y) {
                        int16_t val = -event.value;

                        if (val > 0) {
                            last_movement_message.left = JOINT_DRIVE_SPEED;
                            last_movement_message.right = JOINT_DRIVE_SPEED;
                        } else if (val < 0) {
                            last_movement_message.left = -JOINT_DRIVE_SPEED;
                            last_movement_message.right = -JOINT_DRIVE_SPEED;
                        } else {
                            last_movement_message.left = 0;
                            last_movement_message.right = 0;
                        }
                    } else if (event.axis == controller::Axis::DPAD_X) {
                        if (event.value > 0) {
                            last_movement_message.left = JOINT_DRIVE_SPEED;
                            last_movement_message.right = -JOINT_DRIVE_SPEED;
                        } else if (event.value < 0) {
                            last_movement_message.left = -JOINT_DRIVE_SPEED;
                            last_movement_message.right = JOINT_DRIVE_SPEED;
                        } else {
                            last_movement_message.left = 0;
                            last_movement_message.right = 0;
                        }
                    }

                } else if (event.type == controller::EventType::BUTTON) {
                    if (event.button == controller::Button::BACK && event.value != 0) {
                        int temp = primary_feed;
                        primary_feed = secondary_feed;
                        secondary_feed = temp;
                    }
                }
            }

            if (err != controller::Error::DONE) {
                logger::log(logger::ERROR, "Failed to read from the controller! Disabling");
                controller_loaded = false;
            }
        }

        if (movement_send_timer.ready()) {
            // printf("sending movement with %d, %d\n", last_movement_message.left, last_movement_message.right);

            network::publish(&bs_feed, &last_movement_message);
        }

        // Update and draw GUI.
        do_gui(&font);

        if (help_menu_up) do_help_menu(&font, commands, debug_commands);

        if (stopwatch_menu_up) {
            do_stopwatch_menu(&font);
        }

        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        // Display our buffer.
        glfwSwapBuffers(window);
    }

    // Cleanup.
    glfwTerminate();

    return 0;
}
