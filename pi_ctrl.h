#ifndef PI_CTRL_H
#define PI_CTRL_H

#include "simstruc.h"

typedef struct {
    real_T out_max;
    real_T out_min;
    real_T integral;
} PI_Ctrl;

static void PI_Init(PI_Ctrl *pi, real_T out_min, real_T out_max)
{
    pi->out_max  = out_max;
    pi->out_min  = out_min;
    pi->integral = 0.0;
}

static void PI_Reset(PI_Ctrl *pi)
{
    pi->integral = 0.0;
}

static real_T PI_Update(PI_Ctrl *pi, real_T Kp, real_T Ki, real_T error)
{
    real_T output;

    pi->integral += error;

    output = Kp * error + Ki * pi->integral;

    if (output > pi->out_max) {
        output = pi->out_max;
        pi->integral -= error;
    } else if (output < pi->out_min) {
        output = pi->out_min;
        pi->integral -= error;
    }

    return output;
}

#endif /* PI_CTRL_H */
