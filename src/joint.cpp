/*
  joint.cpp - Methods for commanding joints.
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

#include "joint.h"

//TODO: Determine correct "home" values.
const frame_t home_frame = {
  x : 0,
  y : 0,
  z : 0,
  yaw : 0,
  pitch : 0,
  roll : 0
};

const char *errorNames[] = {FOREACH_ERROR(GENERATE_STRING)};

frame_t current_frame;
double joint_position[6];
error_t joint_movement_error;
movementMode_t movement_mode = ABSOLUTE;

/**
 * Stepper Motors that make up the robot.
 **/
Stepper *motors[MOTOR_COUNT] = {
    new Stepper(PIN_STEP_1, PIN_DIR_1),
    new Stepper(PIN_STEP_2, PIN_DIR_2),
    new Stepper(PIN_STEP_3, PIN_DIR_3),
    new Stepper(PIN_STEP_4, PIN_DIR_4),
    new Stepper(PIN_STEP_5, PIN_DIR_5),
    new Stepper(PIN_STEP_6, PIN_DIR_6)};

/**
 * The Stepper motor controller.
 **/
StepControl<> controller;

/**
 * Configuration of joints in the robot.
 **/
jointConfig_t jointConfig[] = {
    {minPosition : -3600,
     maxPosition : 3600,
     homePosition : 0,
     acceleration : 7500,
     inverseRotation : false,
     maxSpeed : 5000, //Trying for 1 rotation per second (accounting for gearRatio)
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.2250f, //19:80
     dirPin : PIN_DIR_1,
     stepPin : PIN_STEP_1,
     csPin : PIN_CS_1,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {minPosition : 0,
     maxPosition : 0,
     homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.02127659574f, //1:47
     dirPin : PIN_DIR_2,
     stepPin : PIN_STEP_2,
     csPin : PIN_CS_2,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {minPosition : -50,
     maxPosition : 50,
     homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.02127659574f, //1:47
     dirPin : PIN_DIR_3,
     stepPin : PIN_STEP_3,
     csPin : PIN_CS_3,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {minPosition : -50,
     maxPosition : 50,
     homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.266666666667f, //16:60
     dirPin : PIN_DIR_4,
     stepPin : PIN_STEP_4,
     csPin : PIN_CS_4,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {minPosition : -50,
     maxPosition : 50,
     homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.6f, //30:50
     dirPin : PIN_DIR_5,
     stepPin : PIN_STEP_5,
     csPin : PIN_CS_5,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {minPosition : -50,
     maxPosition : 50,
     homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 1.0f,
     dirPin : PIN_DIR_6,
     stepPin : PIN_STEP_6,
     csPin : PIN_CS_6,
     stepsPerRev : 1600,
     unsafeStartup : false}};

bool move_linear(frame_t position, float speed)
{
  speed = min(max(0.01, speed), 1.0);
  return move_linear(position.x, position.y, position.z, position.roll, position.pitch, position.yaw, speed);
}

bool move_linear(double x_pos, double y_pos, double z_pos, double roll_theta, double pitch_theta, double yaw_theta, float speed)
{
  speed = min(max(0.01, speed), 1.0);
  if (runState == RUNNING)
  {
    if (!controller.isRunning())
    {
      joint_movement_error = ERR_SUCCESS; //clear the error field.
      double solution[MOTOR_COUNT];
      double x, y, z, u, v, w;
      switch (movement_mode)
      {
      case RELATIVE:
        x = current_frame.x + x_pos;
        y = current_frame.y + y_pos;
        z = current_frame.z + z_pos;
        u = current_frame.yaw + yaw_theta;
        v = current_frame.pitch + pitch_theta;
        w = current_frame.roll + roll_theta;
        break;
      case ABSOLUTE:
      default:
        x = x_pos;
        y = y_pos;
        z = z_pos;
        u = yaw_theta;
        v = pitch_theta;
        w = roll_theta;
      }
      inverse(solution, z, y, z, u, v, w);
      int32_t target[MOTOR_COUNT];

      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        //TODO: Need to map motor position (in steps) to angles (in DH params)
        target[i] = jointConfig[i].homePosition + ((solution[i] / 360 * jointConfig[i].stepsPerRev) / jointConfig[i].gearRatio);
        if (jointConfig[i].minPosition > target[i] ||
            jointConfig[i].maxPosition < target[i])
        {
          joint_movement_error = ERR_RESULT_OUTSIDE_WORKSPACE;
          return false;
        }
      }

      //still good.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setTargetAbs(target[i]);
        joint_position[i] = target[i]; //TODO: Non-atomic here. It's possible to fail mid-move, and have incorrect data in joint_position. Move this to joint_loop.
      }
      controller.moveAsync(motors, speed);

      //TODO: Non-atomic here. It's possible to fail mid-move, and have incorrect data in current_frame. Move this to joint_loop.
      current_frame.x = x;
      current_frame.y = y;
      current_frame.z = z;
      current_frame.roll = u;
      current_frame.pitch = v;
      current_frame.yaw = w;

      return true;
    }
    else
    {
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
    }
  }
  else
  {
    joint_movement_error = ERR_BAD_RUN_STATE;
  }
  return false;
}

/**
* Jog one or more joints.
*
* Return: True if the movement was successful. False if the movement was not successful.
**/
bool move_joints(int idx, double theta)
{
  if (runState == RUNNING)
  {
    if (!controller.isRunning())
    {
      int steps_per_rad = (int)((double)jointConfig[idx].stepsPerRev / jointConfig[idx].gearRatio / TWO_PI);
      int steps = steps_per_rad * theta;

      Logger::trace("Steps_per_rad=%d", steps_per_rad);

      switch (movement_mode)
      {
      case ABSOLUTE:
        motors[idx]->setTargetAbs(steps);
        break;
      case RELATIVE:
      default:
        motors[idx]->setTargetRel(steps);
      }

      controller.moveAsync(*(motors[idx]));
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
    }
  }
  else
  { //systemState!=RUNNING
  }
  return false;
}

bool move_joints(double *theta)
{

  if (runState == RUNNING)
  {
    if (!controller.isRunning())
    {
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        int steps_per_rad = (int)((double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio / TWO_PI);
        int steps = steps_per_rad * theta[i];

        Logger::trace("Steps_per_rad[%d]=%d", i, steps_per_rad);

        switch (movement_mode)
        {
        case ABSOLUTE:
          motors[i]->setTargetAbs(steps);
          break;
        case RELATIVE:
        default:
          motors[i]->setTargetRel(steps);
        }

        controller.moveAsync(*(motors[i]));
        return true;
      }
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
    }
  }
  else
  { //systemState!=RUNNING
  }
  return false;
}

bool move_home(float speed)
{
  speed = min(max(0.01, speed), 1.0);
  if (runState == RUNNING)
  {
    if (!controller.isRunning())
    {
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setTargetAbs(jointConfig[i].homePosition);
      }
      controller.moveAsync(motors, speed);
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
    }
  }
  else
  {
    Logger::error("Movement aborted: Robot is not ready.");
  }
  return false;
}

void setup_joints()
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i]->setAcceleration(jointConfig[i].acceleration);
    motors[i]->setInverseRotation(jointConfig[i].inverseRotation);
    motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
    motors[i]->setPullInSpeed(jointConfig[i].pullInFreq);
    motors[i]->setStepPinPolarity(jointConfig[i].stepPinPolarity);
    motors[i]->setPosition(0);
  }
}

void loop_joints()
{
  //TODO: update joint_position
}
