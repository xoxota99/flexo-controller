/*
  joint.h - Methods for commanding joints.
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

#ifndef __JOINT_H__
#define __JOINT_H__

enum movementMode_t
{
  ABSOLUTE,
  RELATIVE
};

#include "flexo.h"

#define HOMING_SPEED_REL 0.1f //relative homing speed for joints (as a ratio of maxSpeed)

#define FOREACH_ERROR(MODE)                                                                                      \
  MODE(ERR_SUCCESS /* The command was successful. */)                                                            \
  MODE(ERR_RESULT_OUTSIDE_WORKSPACE /* The result of movement places the end effector outside the workspace.*/)  \
  MODE(ERR_RESULT_SINGULARITY /* Movement calculations result in an impossible pose (NaN, or other result) */)   \
  MODE(ERR_BAD_RUN_STATE /* Robot is in a state that does not support movement (such as STARTUP or SHUTDOWN) */) \
  MODE(ERR_MOVEMENT_IN_PROGRESS /* The robot is already moving, so this command has been ignored. */)            \
  MODE(ERR_IO_TIMEOUT /* Limit switches flapped / bounced for longer than the maximum timeout */)

enum error_t
{
  FOREACH_ERROR(GENERATE_ENUM)
};

/**
 * Configuration of a robot arm joint.
 **/
typedef struct
{
  /**
   * Minimum position (in steps), including gear ratio.
   **/
  int32_t minPosition;
  /**
   * Maximum position (in steps), including gear ratio.
   **/
  int32_t maxPosition;
  /**
   * Home position (in steps). DEPRECATED. This is the min position (until I think of a good reason why it shouldn't be. Maybe it screws with the IK solver?)
   **/
  // int32_t homePosition;
  /**
   * Allowed acceleration during move commands (in steps).
   **/
  unsigned int acceleration;
  /**
   * Should this motor's direction be reversed?
   **/
  bool inverseRotation;
  /**
   * Maximum speed allowed during move commands (in steps)
   **/
  int maxSpeed;
  /**
   * if a motor is already moving at this speed (in steps / second), there is no need to accelerate up to speed.
   **/
  unsigned int pullInFreq;
  /**
   * The ACTIVE polarity for this motor's STEP pin (usually HIGH)
   **/
  int stepPinPolarity;
  /**
   * Gear ratio for this motor. This is the ratio of the output shaft to the input shaft. So a gear ratio of (e.g.) 0.25 
   * means that for every revolution of this motor, we will expect 0.25 revolutions (90 degrees movement) of the joint.
   **/
  float gearRatio;
  /**
   * The pin to use for setting the direction of this motor.
   **/
  byte dirPin;
  /**
   * The pin to use for pulsing this motor.
   **/
  byte stepPin;
  /**
   * The GPIO Pin for this joint's limit switch (if any). This pin will be pulled 
   * LOW when the limit switch is engaged. Set to -1 to disable.
   **/
  byte limitPin;
  /**
   * Limit pin active (HIGH / LOW).
   **/
  bool limitPinActive;
  /**
   * Number of steps per revolution of the motor, ignoring the gear ratio. Used for configuration of microstepping.
   **/
  unsigned int stepsPerRev;
} jointConfig_t;

/**
 * The current X/Y/Z/Yaw/Pitch/Roll of the tool end effector, in the world frame (in millimeters and degrees)
 **/
extern frame_t ee_frame;

/**
 * The "home" pose, in the world frame. Useful when we want to tell the robot to "go home".
 **/
extern const frame_t home_frame;

/** 
 * The current joint position (in degrees) of each joint.
 **/
extern double joint_position[6];

extern const char *errorNames[];
extern error_t joint_movement_error;

extern movementMode_t movement_mode;

// extern motorConfig_t motorConfig[];
extern jointConfig_t jointConfig[];

extern Stepper *motors[MOTOR_COUNT];
extern StepControl controller;

void setup_joints();
void loop_joints();

/**
 * Move all joints to their minimums (one at a time, starting with J6), stopping when limit switches are hit.
 **/
void initialize_joint_stop();

/**
 * Given an end-effector pose, initiate movement of all the joints of the robot, to move the end-effector to the desired pose.
 * 
 * Return true if the movement is successful, otherwise false. If the movement was not successful, the reason code will be stored in the joint_movement_error field.
 **/
bool move_linear(frame_t position, float speed = 1.0f);
bool move_linear(double x_pos, double y_pos, double z_pos, double roll_theta, double pitch_theta, double yaw_theta, float speed = 1.0f);

#ifdef LIMIT_SWITCHES_SUPPORTED
/**
 * Move specified joints to their minimum limits. Each joint is moved separately, starting with J6, down to J1. 
 * Joints will stop moving when they hit their limit switches, then move forward slightly until the limit 
 * switch is no longer depressed.
 * 
 * Calling this function with no parameters will home all joints at the speed defined in HOMING_SPEED_REL.
 **/
bool safe_zero(bool j1 = true, bool j2 = true, bool j3 = true, bool j4 = true, bool j5 = true, bool j6 = true, float speed = HOMING_SPEED_REL);
#endif

/**
* Jog a given joint by a given angle (in degrees), in the given direction. Movement may be relative or absolute.
*
* Return: True if the movement was successful. False if the movement was not successful.
**/
bool move_joints(int idx, double theta, float speed = 1.0f);
bool move_joints(bool *shouldMove, double *theta, float speed = 1.0f);

bool move_circular(frame_t position, float speed = 1.0f);

#endif //__JOINT_H__