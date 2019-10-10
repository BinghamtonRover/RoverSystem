/*
    Michael Schutzman GitHub Issue #122
    Implement VFH Algorithm into simulator
    VFH Algorithm:
    http://www-personal.umich.edu/~johannb/Papers/paper16.pdf
*/

/* useful online implimentations of VFH algorithm
    https://clearpathrobotics.com/blog/2014/05/vector-field-histogram/
    https://github.com/agarie/vector-field-histogram/blob/master/src/histogram_grid.c
    https://robotics.stackexchange.com/questions/9925/vfh-vector-field-histogram-obtaining-the-primary-polar-histogram
    http://www-personal.umich.edu/~johannb/vff&vfh.htm
    
*/
#include "autonomy.h"
#include <math.h>

AutonomyStatus autonomy_step(World* world, float rover_x, float rover_y, float rover_angle, float* out_offset_x, float* out_offset_y) {
    *out_offset_x = 0.5;
    *out_offset_y = 0.5;

    return AS_OK;
}

const char* autonomy_get_name() {
    return "VFH";
}

float get_direction_from_active_cell_to_vcp(x_0, x_i, y_0, y_j) {
    return atan ( (y_j-y_0)/(x_i, x_0) );
}

float get_magnitude_obstacle_vector(c_i_j, d_i_j) {
    a = 1;
    b = 2; //how do i get these?
    return pow ( (c_i_j), 2 ) * ( a - (b * d_i_j) );
}

int get_sector_k(b_i_j, alpha) {
    return int( b_i_j/alpha );
}

float get_polar_obstacle_density(k) {
    certainty = 1; // how do i get this? 
    //what is summation from, to?
}

float get_smoothed_polar_obstacle_density(k) {
    sum = 0;
    l = 10 // constant
    // where's start and finish?
    for ( i = x, x < y, x++ )
    {
        sum += (get_polar_obstacle_density(k) / (2l + 1) )
    }
    return sum;
}

float get_speed_reduction(v_max, h_c, h_m) {
    return v_max * (1 - h_c / h_m);
}

float get_h_c_double_prime(h_c, h_m) {
    if ( h_c < h_m )
    {
        return h_c;
    }
    else
    {
        return h_m;
    }
}

float get_speed_reduction_based_on_steering_rate(v_prime, ohm, ohm_max, v_min) {
    return v_prime * ( 1 - ohm / ohm_max ) + v_min;
}