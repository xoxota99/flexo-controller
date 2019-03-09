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

#include "flexo.h" // for Arduino headers

#define CONFIG_SHELL_MAX_COMMANDS 10 //Maximum number of supported Shell commands.
#define SERIAL_BAUD 115200           //baud rate for shell commands.
#define CPU_MAP_TEENSY_3_2           // What CPU pin mapping should we use?
#define MOTOR_COUNT 6                // How many motors do we have?

#define FOREACH_SHELLMODE(MODE) \
  MODE(BINARY)                  \
  MODE(INTERACTIVE)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum shellMode_t
{
  FOREACH_SHELLMODE(GENERATE_ENUM)
};

extern const char *shellModeNames[];
extern shellMode_t shellMode;
#endif // __CONFIG_H__