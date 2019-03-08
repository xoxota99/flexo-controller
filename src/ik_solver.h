/*
  ik_solver.h - Denavit-Hartenberg parameters for inverse kinematics.
  Part of flexo-controller

  Copyright (c) 2019 Phil Desrosiers

  Flexo is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Flexo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Flexo.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __IK_SOLVER_H__
#define __IK_SOLVER_H__

// #include "config.h"
#include "cpu_map.h"

typedef struct ik_config_t
{
  double a;
  double alpha;
  double d;
  double theta;
} ik_config_t;

ik_config_t dh_params[MOTOR_COUNT]{
    {.a = -89.978, .alpha = PI / 2, .d = -139.75, .theta = -PI},
    {.a = 0.0, .alpha = 0.0, .d = 0.0, .theta = -PI / 2},
    {.a = 66.125, .alpha = PI / 2, .d = 0.0, .theta = 0.0},
    {.a = 0.0, .alpha = -PI / 2, .d = 342.472, .theta = 0.0},
    {.a = 0.0, .alpha = -PI / 2, .d = 0.0, .theta = 0.0},
    {.a = 0.0, .alpha = PI, .d = -47.425, .theta = PI}};

//Translate / rotate the tool origin to the given location in the o0 coordinate frame
void moveTo(double x_pos, double y_pos, double z_pos, double roll_theta, double pitch_theta, double yaw_theta);

//Translate the tool origin (with no rotation) to the given location in the o0 coordinate frame
void moveTo(double x_pos, double y_pos, double z_pos);

#endif // __IK_SOLVER_H__