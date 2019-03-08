/*
  joint.h - methods and configuration related to joint manipulation.
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

#include "flexo.h"

#define ENC_SCALE 0x3fff
#define STEPSILON 10

// this is the magic trick for printf to support float
asm(".global _printf_float");
// this is the magic trick for scanf to support float
asm(".global _scanf_float");

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
    motors[i]->setAcceleration(kMotorConfig[i].acceleration);
    motors[i]->setInverseRotation(kMotorConfig[i].inverseRotation);
    motors[i]->setMaxSpeed(kMotorConfig[i].maxSpeed);
    motors[i]->setPullInSpeed(kMotorConfig[i].pullInFreq);
    motors[i]->setStepPinPolarity(kMotorConfig[i].stepPinPolarity);
  }
}

void loop_motors()
{
}

void setup()
{

  while (!Serial && millis() < 1000)
    ;

  setup_motors();
}

void loop()
{
  // delay(10);

  loop_motors();
  // Serial.printf("%d\t%d\n", motors[0]->getPosition(), angleSensor[0]->getRawRotation());
}
