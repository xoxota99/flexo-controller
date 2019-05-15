/*
  cpu_map.h - CPU and pin mapping configuration file.
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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Flexo officially supports
   only the PJRC Teensy 3.2. */

#ifndef __CPU_MAP_H__
#define __CPU_MAP_H__

#ifdef CPU_MAP_TEENSY_3_2

#define PIN_DIR_1 2
#define PIN_STEP_1 3
#define PIN_LIMIT_1 17

#define PIN_DIR_2 7
#define PIN_STEP_2 4
#define PIN_LIMIT_2 18

#define PIN_DIR_3 8
#define PIN_STEP_3 5
#define PIN_LIMIT_3 19

#define PIN_DIR_4 14
#define PIN_STEP_4 6
#define PIN_LIMIT_4 20

#define PIN_DIR_5 15
#define PIN_STEP_5 9
#define PIN_LIMIT_5 21

#define PIN_DIR_6 16
#define PIN_STEP_6 10
#define PIN_LIMIT_6 22

// #define PIN_MOSI 11
// #define PIN_MISO 12
// #define PIN_SCK 13
// #define PIN_ENABLE 23

#endif //CPU_MAP_TEENSY_3_2

#include "flexo.h"

#endif // __CPU_MAP_H__