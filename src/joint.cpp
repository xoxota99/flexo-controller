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

Stepper *motors[MOTOR_COUNT] = {
    new Stepper(PIN_STEP_1, PIN_DIR_1),
    new Stepper(PIN_STEP_2, PIN_DIR_2),
    new Stepper(PIN_STEP_3, PIN_DIR_3),
    new Stepper(PIN_STEP_4, PIN_DIR_4),
    new Stepper(PIN_STEP_5, PIN_DIR_5),
    new Stepper(PIN_STEP_6, PIN_DIR_6)};

StepControl<> controller;

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

/**
* Jog a given joint by a given angle (in radians), in the given direction. Movement may be relative or absolute.
*
* Return: True if the movement was successful. False if the movement was not successful.
**/
bool jog(int idx, double theta, movementMode_t moveMode)
{
  if (systemState == RUNNING)
  {
    if (moveState == STOPPED)
    {
      int steps_per_rad = (int)((double)jointConfig[idx].stepsPerRev / jointConfig[idx].gearRatio / TWO_PI);
      int steps = steps_per_rad * theta;

      Logger::trace("Steps_per_rad=%d", steps_per_rad);

      switch (moveMode)
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

/**
 * Return the current position (in radians) of the given motor, relative to the zero point for that motor.
 **/
double getTheta(int motorId) { return 0.0; }

void setup_motors()
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

void loop_motors()
{
  switch (moveState)
  {
  case MOVING:
    if (!controller.isRunning())
    {
      moveState = STOPPED;
    }
    break;
  case STOPPED:
    if (controller.isRunning())
    {
      moveState = MOVING;
      Logger::warn("Motors are moving, but moveState was STOPPED. Something set the motors moving without updating state.");
    }
    break;
  }
}
