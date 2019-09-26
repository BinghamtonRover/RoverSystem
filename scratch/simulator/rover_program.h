#ifndef ROVER_PROGRAM_H
#define ROVER_PROGRAM_H

#include "program.h"
#include "mutils.h"

typedef struct {
    Program prog;

    int u_model_loc;
    int u_view_loc;
    int u_projection_loc;
} RoverProgram;

RoverProgram rover_program_create();

void rover_program_set_model(RoverProgram* rp, Mat3f model);

void rover_program_set_view(RoverProgram* rp, Mat3f view);

void rover_program_set_projection(RoverProgram* rp, Mat3f projection);

#endif
