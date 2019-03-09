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

typedef struct jointConfig_t
{
  float gearRatio; // joints can have a gear ratio, separate from the motor's gearbox.
  int32_t minPosition;
  int32_t maxPosition;
  int32_t zeroPosition;
} jointConfig_t;

extern jointConfig_t jointConfig[];

typedef struct motorConfig_t
{
  unsigned int acceleration;
  bool inverseRotation;
  int maxSpeed;
  unsigned int pullInFreq;
  int stepPinPolarity;
  float gearRatio; // Motors can have their own gear ratio / gearbox
  byte dirPin;
  byte stepPin;
  byte csPin;
  unsigned int stepsPerRev;
  bool unsafeStartup;
} motorConfig_t;

extern motorConfig_t motorConfig[];

enum Direction
{
  CW = HIGH,
  CCW = LOW
};

enum MovementMode
{
  ABSOLUTE,
  RELATIVE
};

/**
* Jog a given joint by a given angle (in radians), in the given direction. Movement may be relative or absolute.
*
* Return: True if the momvent was successful. False if the movement was not successful.
**/
bool jog(motorConfig_t motor, double theta, Direction dir, MovementMode movementMode);

/**
 * Return the current position (in radians) of the given motor, relative to the zero point for that motor.
 **/
double getTheta(int motorId);

#endif //__JOINT_H__