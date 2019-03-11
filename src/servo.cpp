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

#include "servo.h"

Stepper *motors[MOTOR_COUNT] = {
    new Stepper(PIN_STEP_1, PIN_DIR_1),
    new Stepper(PIN_STEP_2, PIN_DIR_2),
    new Stepper(PIN_STEP_3, PIN_DIR_3),
    new Stepper(PIN_STEP_4, PIN_DIR_4),
    new Stepper(PIN_STEP_5, PIN_DIR_5),
    new Stepper(PIN_STEP_6, PIN_DIR_6),
};

StepControl<> controller;

void setup_motors()
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i]->setAcceleration(motorConfig[i].acceleration);
    motors[i]->setInverseRotation(motorConfig[i].inverseRotation);
    motors[i]->setMaxSpeed(motorConfig[i].maxSpeed);
    motors[i]->setPullInSpeed(motorConfig[i].pullInFreq);
    motors[i]->setStepPinPolarity(motorConfig[i].stepPinPolarity);
  }
}

void loop_motors()
{
  if (systemMode == RUNNING)
  {
    switch (moveMode)
    {
    case MOVING:
      if (!controller.isRunning())
      {
        moveMode = STOPPED;
      }
      break;
    case STOPPED:
      if (controller.isRunning())
      {
        moveMode = MOVING;
        Logger::error("Motors are moving, but moveState is STOPPED. Something set the motors moving without updating state.");
      }
      break;
    }
  }
}
