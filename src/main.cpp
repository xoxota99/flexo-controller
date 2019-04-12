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

const char *moveStateNames[] = {
    FOREACH_MOVESTATE(GENERATE_STRING)};

move_state_t moveState = STOPPED;

const char *systemStateNames[] = {
    FOREACH_SYSTEMSTATE(GENERATE_STRING)};

system_state_t systemState = STARTUP;

void setup()
{
  // Logger::level = Logger::DEBUG;

  while (!Serial && millis() < 1000)
    ;
  if (Serial)
  {
    delay(1000); //give time for client to connect.
  }

  setup_motors();

  // setup_endpoint();
  setup_gcode();
  setup_led();

  systemState = RUNNING;
}

void loop()
{
  loop_motors();

  // loop_endpoint();
  loop_gcode();
  loop_led();
}

//=====
// Blink an LED every once in a while, to let me know the Arduino is still alive.
void setup_led()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);
}

void loop_led()
{
  static uint32_t last_ms = 0;
  uint32_t m = millis();
  if (m - last_ms >= 1000)
  {
    digitalWriteFast(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    last_ms = m;
  }
}
