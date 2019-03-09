/*
  main.cpp - A controller for a 6DOF robot arm.
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

/**
 * The controller manages the execution state of the robot 
 * (Calibration mode, interactive mode or binary mode), initializes 
 * a command processor, and executes commands using the configured joints.
 **/

#include "flexo.h"

// this is the magic trick for printf to support float
asm(".global _printf_float");
// this is the magic trick for scanf to support float
asm(".global _scanf_float");

const char *shellModeNames[] = {
    FOREACH_SHELLMODE(GENERATE_STRING)};

shellMode_t shellMode = INTERACTIVE;

void setup()
{

  while (!Serial && millis() < 1000)
    ;
    
  setup_uptime();
  setup_motors();

  setup_endpoint();
  setup_shell();
}

void loop()
{
  loop_uptime();
  loop_motors();

  loop_endpoint();
  loop_shell();
}
