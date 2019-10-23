#include "waypoint_map.hpp"
#include "waypoint.hpp"

#include <GL/gl.h>

#include <math.h>

namespace gui {
namespace waypoint_map {

float ppm = 1;

void zoom_in() {
    if (ppm < PPM_MAX) ppm *= PPM_SCALE_FACTOR;
}

void zoom_out() {
    if (ppm > PPM_MIN) ppm /= PPM_SCALE_FACTOR;
}

static float getDistance(float lat1, float long1, float lat2, float long2){
    float R = 6378.137;
    float dlat1 = lat1*(M_PI/180);
    float dlong1 = long1*(M_PI/180);
    float dlat2 = lat2*(M_PI/180);
    float dlong2 = long2*(M_PI/180);
    float dLong = dlong1-dlong2;
    float dLat = dlat1-dlat2;
    float a = pow(sin(dLat/2.0),2.0)+cos(dlat1)*cos(dlat2)*pow(sin(dLong/2),2);
    float c = 2*atan2(sqrt(a),sqrt(1.0-a));
    return R * c * 1000;

}

void do_waypoint_map(gui::Layout * layout, int w, int h){
    int x = layout->current_x;
    int y = layout->current_y;
    
    glPushMatrix();
    glEnable(GL_SCISSOR_TEST);
    glScissor(x, WINDOW_HEIGHT - y - h, w, h);
    gui::do_solid_rect(layout,w,h,0,0,0);
    glMatrixMode(GL_MODELVIEW);
    //shift to (0,0)
    glTranslatef(x +(w/2),y +(h/2),0);
    //zoom
    glScalef(ppm,ppm,1.0f);
    //shift back
    glTranslatef(-x - (w/2),-y - (h/2),0);
    glColor4f(0.0,1.0,0.0,0.3);

    glBegin(GL_LINES);
    glLineWidth(1.0f);
    //Recommended not to have the spacing > size of window (things get bigger than map space) but encouraged to mess with spacing
    int spaceBetweenGridLines = std::min((w/4),(h/4)); 
    float xMiddle = w/2;
    float yMiddle = h/2;
    int metersPerGridLine = 10;
    int dimensions = 100; //Draws (dimensions * metersPerGridLine)/2 meters out from the rover
    for(int xOffset = -(dimensions * spaceBetweenGridLines)/2 - spaceBetweenGridLines; xOffset < (dimensions * spaceBetweenGridLines)/2; xOffset+= spaceBetweenGridLines){
        glVertex2f(xOffset + x,y - (dimensions * spaceBetweenGridLines)/2);
        glVertex2f(xOffset + x,y + (dimensions * spaceBetweenGridLines)/2);
    }
    for(int yOffset = -(dimensions * spaceBetweenGridLines)/2 - spaceBetweenGridLines; yOffset < (dimensions * spaceBetweenGridLines)/2; yOffset+= spaceBetweenGridLines){
        glVertex2f(x - (dimensions * spaceBetweenGridLines)/2,yOffset + y);
        glVertex2f(x + (dimensions * spaceBetweenGridLines)/2,y + yOffset);
    }
    glEnd();
    //If the rover coordinates are outside of the valid ranges of latitude and longitude, don't draw anything
    //This should only occur when the initial values for the rover's coordinates are set outside the range or when the basestation
    //is sent incorrect coordinates
    if ( (-90 <= waypoint::rover_latitude && waypoint::rover_latitude <= 90) && (-180 <= waypoint::rover_longitude && waypoint::rover_longitude <= 180) ){
        glColor4f(0.0,0.0,1.0,0.8);
        float waypoint_width = w * 0.10;
        float waypoint_height = h * 0.10;
        auto temp = waypoint::get_waypoints();
        while(temp.size() > 0){
            auto curWaypoint = temp.front();
            float waypointY = curWaypoint.latitude;
            float waypointX = curWaypoint.longitude;
            temp.erase(temp.begin());
            float distance_x = getDistance(waypoint::rover_latitude,waypoint::rover_longitude,waypoint::rover_latitude,waypointX);
            float distance_y = getDistance(waypoint::rover_latitude,waypoint::rover_longitude,waypointY,waypoint::rover_longitude);
            if (waypoint::rover_latitude > waypointY)
                distance_y = distance_y * -1;
            if (waypoint::rover_longitude > waypointX)
                distance_x = distance_x * -1;
            distance_x = distance_x/metersPerGridLine;
            distance_y = distance_y/metersPerGridLine; 
            
            if (abs(distance_x) <= (w/2)*(1/ppm) && abs(distance_y) <= (h/2)*(1/ppm)){
                glBegin(GL_QUADS);
                glVertex2f((x + xMiddle) + distance_x - (waypoint_width/2),(y + yMiddle) - distance_y + (waypoint_height/2));
                glVertex2f((x + xMiddle) +  distance_x + (waypoint_width/2),(y + yMiddle) - distance_y + (waypoint_height/2));
                glVertex2f((x + xMiddle) + distance_x + (waypoint_width/2),(y + yMiddle) - distance_y - (waypoint_height/2));
                glVertex2f((x + xMiddle) + distance_x - (waypoint_width/2),(y + yMiddle) - distance_y - (waypoint_height/2));
                glEnd();
            }
        }
    }
    glDisable(GL_SCISSOR_TEST);
    glPopMatrix(); 
    //Handle drawing the rover in the middle of the map
    glColor4f(1.0,0.0,0.0,1.0);
    float triangleWidth = w * 0.03;
    float triangleHeight = triangleWidth; //for a equilateral triangle
    glBegin(GL_TRIANGLE_STRIP);
    glVertex2f((x + xMiddle) - (triangleWidth/2),y + yMiddle + triangleHeight/2);
    glVertex2f((x + xMiddle) + (triangleWidth/2), y + yMiddle + triangleHeight/2);
    glVertex2f(x + xMiddle,y + yMiddle - triangleHeight/2);
    glEnd();   
}

}} // namespace gui::waypoint_map
