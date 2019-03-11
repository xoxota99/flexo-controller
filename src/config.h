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

//These types should be defined in joint.h, but when I move them there, the compiler freaks out (because they're also used in protocol.h)
typedef struct jointConfig_t
{
  float gearRatio; // joints can have a gear ratio, separate from the motor's gearbox.
  int32_t minPosition;
  int32_t maxPosition;
  int32_t zeroPosition;
} jointConfig_t;

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

enum movementMode_t
{
  ABSOLUTE,
  RELATIVE
};

#endif // __CONFIG_H__