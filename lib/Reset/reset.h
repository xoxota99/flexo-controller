/*
  reset.h - reset macros.
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
#ifndef __RESET_H__
#define __RESET_H__

#include <Arduino.h>

#ifdef CORE_TEENSY
//Software reset macros / MMap FOR TEENSY ONLY
#define CPU_RESTART_VAL 0x5FA0004                         // write this magic number...
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C           // to this memory location...
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL) // presto!

#else

#define CPU_RESTART asm volatile("  jmp 0") // close enough for arduino

#endif

#endif