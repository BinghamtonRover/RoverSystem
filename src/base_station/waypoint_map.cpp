#include "waypoint_map.hpp"
#include "waypoint.hpp"

#include "../logger/logger.hpp"

#include <GL/gl.h>

#include <math.h>

namespace gui {
namespace waypoint_map {

float ppm = PPM_MIN;
bool gridMap = true;

void zoom_in() {
    ppm *= PPM_SCALE_FACTOR;
    if (ppm > PPM_MAX) ppm = PPM_MAX;
}

void zoom_out() {
    ppm /= PPM_SCALE_FACTOR;
    if (ppm < PPM_MIN) ppm = PPM_MIN;
}

static void get_meter_offset(float lat1, float long1, float lat2, float long2, float* out_x, float* out_y) {
    const float EARTH_RADIUS = 6378.137e3;

    float dx = (long2 - long1) * cosf(lat1 * M_PI / 180.0f) * EARTH_RADIUS * M_PI / 180.0f;
    float dy = (lat2 - lat1) * EARTH_RADIUS * M_PI / 180.0f;

    *out_x = dx;
    *out_y = dy;
}

void do_waypoint_map(gui::Layout * layout, int w, int h){
    int x = layout->current_x;
    int y = layout->current_y;
    
    glEnable(GL_SCISSOR_TEST);
    glScissor(x, WINDOW_HEIGHT - y - h, w, h);

    gui::do_solid_rect(layout,w,h,0,0,0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    /*
    //shift to (0,0)
    glTranslatef(x +(w/2),y +(h/2),0);
    //zoom
    glScalef(ppm,ppm,1.0f);
    //shift back
    glTranslatef(-x - (w/2),-y - (h/2),0);
    */

    float xpt = x + (w/2.0f);
    float ypt = y + (h/2.0f);

    glTranslatef(xpt, ypt, 0);
    glScalef(ppm, ppm, 1.0);

    glColor4f(0.0,1.0,0.0,0.3);


    float mw = w / ppm;
    float mh = h / ppm;

    int num_x = mw / GRID_SPACING;
    int num_y = mh / GRID_SPACING;
    
    if (gridMap){
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (int xg = -num_x/2; xg <= num_x/2; xg++) {
            glVertex2f(xg * GRID_SPACING, -mh/2.0f);
            glVertex2f(xg * GRID_SPACING, mh/2.0f);
        }
        for (int yg = -num_y/2; yg <= num_y/2; yg++) {
            glVertex2f(-mw/2.0f, yg * GRID_SPACING);
            glVertex2f(mw/2.0f, yg * GRID_SPACING);
        }
        glEnd();
    }
    else {
        for(int radius = GRID_SPACING; radius < (mw/2) * GRID_SPACING; radius += GRID_SPACING){
            //The coordinates are (0,0) since we shifted the origin to the middle instead of the top-left
            gui::do_circle(0,0,radius); 
        }
    }

    /*

    for (float xgo = x_min; xgo <= x_max; xgo += GRID_SPACING) {
        glVertex2f(xgo, y_min);
        glVertex2f(xgo, y_max);
    }

    for (float ygo = y_min; ygo <= y_max; ygo += GRID_SPACING) {
        glVertex2f(x_min, ygo);
        glVertex2f(x_max, ygo);
    }
    */

    //If the rover coordinates are outside of the valid ranges of latitude and longitude, don't draw anything
    //This should only occur when the initial values for the rover's coordinates are set outside the range or when the basestation
    //is sent incorrect coordinates
    if ( (-90 <= waypoint::rover_latitude && waypoint::rover_latitude <= 90) && (-180 <= waypoint::rover_longitude && waypoint::rover_longitude <= 180) ){
        glColor4f(0.0,0.0,1.0,0.8);
        // For now, these are in meters.
        float waypoint_width = 10 / ppm;
        float waypoint_height = 10 / ppm;
        for (auto curWaypoint : waypoint::get_waypoints()) {
            float waypointY = curWaypoint.latitude;
            float waypointX = curWaypoint.longitude;

            float distance_x, distance_y;
            get_meter_offset(waypoint::rover_latitude, waypoint::rover_longitude, waypointY, waypointX, &distance_x, &distance_y);

            // Flip this because +y should actually be up, not down as in screen coords.
            distance_y *= -1.0f;

            glBegin(GL_QUADS);
            glVertex2f(distance_x - (waypoint_width/2), distance_y + (waypoint_height/2));
            glVertex2f(distance_x + (waypoint_width/2), distance_y + (waypoint_height/2));
            glVertex2f(distance_x + (waypoint_width/2), distance_y - (waypoint_height/2));
            glVertex2f(distance_x - (waypoint_width/2), distance_y - (waypoint_height/2));
            glEnd();
        }
    }
    glDisable(GL_SCISSOR_TEST);
    glPopMatrix(); 

    //Handle drawing the rover in the middle of the map
    glColor4f(1.0,0.0,0.0,1.0);
    float triangleWidth = w * 0.03;
    float triangleHeight = triangleWidth; //for a equilateral triangle
    float xMiddle = w / 2.0f;
    float yMiddle = h / 2.0f;
    glBegin(GL_TRIANGLE_STRIP);
    glVertex2f((x + xMiddle) - (triangleWidth/2),y + yMiddle + triangleHeight/2);
    glVertex2f((x + xMiddle) + (triangleWidth/2), y + yMiddle + triangleHeight/2);
    glVertex2f(x + xMiddle,y + yMiddle - triangleHeight/2);
    glEnd();   

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    static char ppm_text_buffer[100];
    sprintf(ppm_text_buffer, "ppm: %.2f | grid: %.2f", ppm, GRID_SPACING);
    gui::draw_text(&gui::state.global_font, ppm_text_buffer, x + 5, y + h - 15 - 5, 15);
}

}} // namespace gui::waypoint_map
