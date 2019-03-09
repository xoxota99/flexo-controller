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

jointConfig_t jointConfig[] = {
    {gearRatio : 0.25f, //20:80
     minPosition : -50,
     maxPosition : 50,
     zeroPosition : 0},
    {gearRatio : 0.02127659574f, //1:47
     minPosition : -50,
     maxPosition : 50,
     zeroPosition : 0},
    {gearRatio : 0.02127659574f, //1:47
     minPosition : -50,
     maxPosition : 50,
     zeroPosition : 0},
    {gearRatio : 0.266666666667f, //16:60
     minPosition : -50,
     maxPosition : 50,
     zeroPosition : 0},
    {gearRatio : 0.6f, //30:50
     minPosition : -50,
     maxPosition : 50,
     zeroPosition : 0},
    {gearRatio : 1.0, // 1:1
     minPosition : -50,
     maxPosition : 50,
     zeroPosition : 0},
};

motorConfig_t motorConfig[] = {
    {acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 1.0f,
     dirPin : PIN_DIR_1,
     stepPin : PIN_STEP_1,
     csPin : PIN_CS_1,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {acceleration : 50000,
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
    {acceleration : 50000,
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
    {acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 1.0f,
     dirPin : PIN_DIR_4,
     stepPin : PIN_STEP_4,
     csPin : PIN_CS_4,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 1.0f,
     dirPin : PIN_DIR_5,
     stepPin : PIN_STEP_5,
     csPin : PIN_CS_5,
     stepsPerRev : 1600,
     unsafeStartup : false},
    {acceleration : 50000,
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
* Return: True if the momvent was successful. False if the movement was not successful.
**/
bool jog(motorConfig_t motor, double theta, Direction dir, MovementMode movementMode)
{
  return false;
}

/**
 * Return the current position (in radians) of the given motor, relative to the zero point for that motor.
 **/
double getTheta(int motorId) { return 0.0; }
