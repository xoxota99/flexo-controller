/*
  config.h - compile-time configuration.
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

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define SERIAL_BAUD 115200 //baud rate for shell commands. Ignored for Teensy.
#define CPU_MAP_TEENSY_3_2 // What CPU pin mapping should we use?
#define MOTOR_COUNT 6      // How many motors do we have?

#define FIRMWARE_VERSION "0.1"
#define FIRMWARE_URL "https://github.com/xoxota99/flexo-controller"
#define MACHINE_TYPE "Flexo"

#define FOREACH_RUNSTATE(MODE) \
  MODE(STARTUP)                \
  MODE(CALIBRATING)            \
  MODE(READY)                  \
  MODE(HALTED)                 \
  MODE(SHUTDOWN)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

#include "flexo.h" // for Arduino headers

typedef struct
{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
} frame_t;

enum run_state_t
{
  FOREACH_RUNSTATE(GENERATE_ENUM)
};

extern const char *runStateNames[];
extern run_state_t runState;

#endif // __CONFIG_H__