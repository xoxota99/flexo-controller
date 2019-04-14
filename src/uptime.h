/*
  uptime.h - utility functions for tracking uptime.
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
#ifndef __UPTIME_H__
#define __UPTIME_H__

#define MILLIS_PER_SECOND 1000
#define MILLIS_PER_MINUTE 60000
#define MILLIS_PER_HOUR 3600000
#define MILLIS_PER_DAY 86400000L
#define MILLIS_PER_YEAR 22896000000L //TODO: Leap years?

#include "flexo.h"

typedef struct
{
  int millis;  // milliseconds from 0 to 1000
  int seconds; // seconds of minutes from 0 to 61
  int minutes; // minutes of hour from 0 to 59
  int hours;   // hours of day from 0 to 24
  int days;    // day of year from 0 to 365
  int years;   // years elapsed
} elapsed_t;

elapsed_t elapse(uint32_t millis);
String uptime();

#endif // __UPTIME_H__