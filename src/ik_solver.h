/*
  ik_solver.h - Inverse / Forward kinematics solver
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

#include "flexo.h"

extern const frame_t work_frame;
extern const frame_t tool_frame;

typedef struct
{
  // double theta;
  double alpha;
  double d;
  double a;
} ik_config_t;

extern const ik_config_t dh_params[];

/** 
 * Translate / rotate the tool origin to the given location in the world coordinate frame. 
 * Yaw, Pitch and Roll are given in degrees.
 * 
 * Solution is returned as an array of doubles, representing the rotation (in degrees) 
 * of each joint in the IK chain.
 **/
void inverse(double *solution, const double x_pos, const double y_pos, const double z_pos, const double yaw_theta, const double pitch_theta, const double roll_theta);

/**
 * Given a set of joint angles, return the end effector's position (x/y/z/yaw/pitch/roll) in the world frame.
 **/
void forward(frame_t *solution, const double *jointAngles);

#endif // __IK_SOLVER_H__