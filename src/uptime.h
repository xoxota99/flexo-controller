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

#include "flexo.h"

#define MILLIS_PER_SECOND 1000
#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define HOURS_PER_DAY 24
#define DAYS_PER_YEAR 365 //TODO: Leap years?

typedef struct elapsed_t
{
  uint16_t millis; // milliseconds from 0 to 1000
  uint8_t seconds; // seconds of minutes from 0 to 61
  uint8_t minutes; // minutes of hour from 0 to 59
  uint8_t hours;   // hours of day from 0 to 24
  uint16_t days;   // day of year from 0 to 365
  uint64_t years;  // years elapsed
} elapsed_t;

extern elapsed_t elapsed_time;

String uptime();
void setup_uptime();
void loop_uptime();

#endif // __UPTIME_H__