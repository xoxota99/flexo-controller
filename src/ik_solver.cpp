#include "ik_solver.h"
ik_config_t dh_params[MOTOR_COUNT]{
    {.a = -89.978, .alpha = PI / 2, .d = -139.75, .theta = -PI},
    {.a = 0.0, .alpha = 0.0, .d = 0.0, .theta = -PI / 2},
    {.a = 66.125, .alpha = PI / 2, .d = 0.0, .theta = 0.0},
    {.a = 0.0, .alpha = -PI / 2, .d = 342.472, .theta = 0.0},
    {.a = 0.0, .alpha = -PI / 2, .d = 0.0, .theta = 0.0},
    {.a = 0.0, .alpha = PI, .d = -47.425, .theta = PI}};

//Translate / rotate the tool origin to the given location in the o0 coordinate frame
void moveTo(double x_pos, double y_pos, double z_pos, double roll_theta, double pitch_theta, double yaw_theta)
{
    // TODO
}

//Translate the tool origin (with no rotation) to the given location in the o0 coordinate frame
void moveTo(double x_pos, double y_pos, double z_pos)
{
    // TODO
}
