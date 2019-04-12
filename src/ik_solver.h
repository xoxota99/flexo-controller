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

typedef struct frame_t
{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
} frame_t;

extern const frame_t work_frame;
extern const frame_t tool_frame;

typedef struct ik_config_t
{
  double theta;
  double alpha;
  double d;
  double a;
} ik_config_t;

extern const ik_config_t dh_params[];

/** 
 * Translate / rotate the tool origin to the given location in the world coordinate frame. 
 * Solution is returned as an array of doubles, representing the rotation (in radians) 
 * of each joint in the IK chain.
 **/
void inverse(double *solution, double x_pos, double y_pos, double z_pos, double roll_theta, double pitch_theta, double yaw_theta);

/**
 * Given a set of joint angles, return the end effector's position (x/y/z/yaw/pitch/roll) in the world frame.
 **/
void forward(frame_t *solution, double *jointAngles);

//Translate the tool origin (with no rotation) to the given location in the o0 coordinate frame
void translate(double *solution, double x_pos, double y_pos, double z_pos);

//Rotate the tool around the tool origin (with no translation).
void rotate(double *solution, double roll_theta, double pitch_theta, double yaw_theta);

#endif // __IK_SOLVER_H__