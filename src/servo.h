/*
  servo.h - An abstraction of the AS5048A encoders, and TeensyStepper controller, smooshed together and treated as a servo.
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
#ifndef __SERVO_H__
#define __SERVO_H__

#include "flexo.h"

//TODO: Convert to Servo class, which extends Stepper class.
extern Stepper *motors[MOTOR_COUNT];
extern StepControl<> controller;

void setup_motors();
void loop_motors();

#endif // __SERVO_H__