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

#include "flexo.h"

#define HOMING_SPEED_REL 0.1f //relative homing speed for joints (as a ratio of maxSpeed)

#define FOREACH_ERROR(MODE)                                                                                      \
  MODE(ERR_SUCCESS /* The command was successful. */)                                                            \
  MODE(ERR_RESULT_OUTSIDE_WORKSPACE /* The result of movement places the end effector outside the workspace.*/)  \
  MODE(ERR_RESULT_SINGULARITY /* Movement calculations result in an impossible pose (NaN, or other result) */)   \
  MODE(ERR_BAD_RUN_STATE /* Robot is in a state that does not support movement (such as STARTUP or SHUTDOWN) */) \
  MODE(ERR_MOVEMENT_IN_PROGRESS /* The robot is already moving, so this command has been ignored. */)            \
  MODE(ERR_IO_TIMEOUT /* Limit switches flapped / bounced for longer than the maximum timeout */)                \
  MODE(ERR_UNSUPPORTED_FUNCTION /* An unimplemented function was called. */)

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
 * The current joint position (in degrees) of each joint.
 **/
extern double joint_position[6];

extern const char *errorNames[];
extern error_t joint_movement_error;

// extern motorConfig_t motorConfig[];
extern jointConfig_t jointConfig[];

extern Stepper *motors[MOTOR_COUNT];
extern StepControl controller;

extern double px, py, pz, pu, pv, pw, feed_rate;

void setup_joints();
void loop_joints();

/**
 * Move all joints to their minimums (one at a time, starting with J6), stopping when limit switches are hit.
 **/
void initialize_joint_stop();

/**
 * Linear, simultaneous movement of all motors, relative to their current position.
 * 
 * theta - an array of exactly MOTOR_COUNT angles (in degrees) to add to the current position of each joint.
 * relspeed - (optional) relative speed to move all joints (between 0 and 1).
 * 
 * This is a convenience method, when we know we want to move ALL joints.
 **/
bool move_relative(double *theta, float relspeed = 1.0f);

/**
 * Linear, simultaneous movement of motors, relative to their current position.
 * 
 * shouldMove - a bitfield indicating which joints (if any) should be moved. Nth bit will be set to 1 if motor n should move.
 * theta - an array of exactly MOTOR_COUNT angles (in degrees) to add to the current position of joints specified in shouldMove.
 * 
 * This is an *inconvenience* method, for when we want to move SOME joints, but not ALL. We need the bit field 
 * because any conceivable magic number (that would indicate that a joint should not move at all) is actually 
 * a valid value for a joint angle.
 **/
bool move_relative(uint8_t shouldMove, double *theta, float relspeed = 1.0f);

/**
 * Linear, simultaneous movement of all motors, to an absolute angle (in degrees).
 * 
 * theta - an array of exactly MOTOR_COUNT angles (in degrees) to set as the new position of each joint.
 * relspeed - (optional) relative speed to move all joints (between 0 and 1).
 * 
 * This is a convenience method, when we know we want to move ALL joints.
 **/
bool move_absolute(double *theta, float relspeed = 1.0f);

/**
 * Linear, simultaneous movement of motors, to an absolute angle (in degrees).
 * 
 * shouldMove - a bitfield indicating which joints (if any) should be moved. Only the first six bits are used.
 * theta - an array of exactly MOTOR_COUNT angles (in degrees) to set as the new position for each joint specified in shouldMove.
 * 
 * This is an *inconvenience* method, for when we want to move SOME joints, but not ALL. We need the bit field 
 * because any conceivable magic number (that would indicate that a joint should not move at all) is actually 
 * a valid value for a joint angle.
 **/
bool move_absolute(uint8_t shouldMove, double *theta, float relspeed = 1.0f);

#ifdef LIMIT_SWITCHES_SUPPORTED
/**
 * Move specified joints to their minimum limits. Each joint is moved separately, starting with J6, down to J1. 
 * Joints will stop moving when they hit their limit switches, then move forward slightly until the limit 
 * switch is no longer depressed.
 * 
 * Calling this function with no parameters will home all joints at the speed defined in HOMING_SPEED_REL.
 **/
bool safe_zero(bool j1 = true, bool j2 = true, bool j3 = true, bool j4 = true, bool j5 = true, bool j6 = true, float speed = HOMING_SPEED_REL);
#else
bool unsafe_zero(bool j1 = true, bool j2 = true, bool j3 = true, bool j4 = true, bool j5 = true, bool j6 = true, float speed = HOMING_SPEED_REL);
#endif

/**
 * Move all joints through a path described as a series of waypoints. 
 * The controller will manage acceleration / deceleration 
 * over the entire path, rather than on a per-waypoint basis.
 * 
 * argc - the number of waypoints in the path.
 * theta - An array of arrays. Each entry represents a waypoint in the curve.
 *          Each waypoint entry is represented by an array of MOTOR_COUNT angles (in degrees), 
 *          which is the absolute angle of one motor.
 * relspeed - (Optional) The relative speed (between 0.0 and 1.0) to move.
 **/
bool move_path(int argc, double **theta, float relspeed = 1.0f);

void moveCompleteCallback();

#endif //__JOINT_H__