/*
  shell.cpp - Interactive shell command processor functions.
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

#include "uptime.h"

elapsed_t elapse(uint32_t millis);
String uptime()
{
    elapsed_t elapsed_time = elapse(millis());
    String retval;
    char buf[50];

    if (elapsed_time.years > 0)
    {
        sprintf(buf, "%d year%s, ", elapsed_time.years, elapsed_time.years != 1 ? "s" : "");
        retval.append(buf);
    }
    if (elapsed_time.days > 0)
    {
        sprintf(buf, "%d day%s, ", elapsed_time.days, elapsed_time.days != 1 ? "s" : "");
        retval.append(buf);
    }
    if (elapsed_time.hours > 0)
    {
        sprintf(buf, "%d hour%s, ", elapsed_time.hours, elapsed_time.hours != 1 ? "s" : "");
        retval.append(buf);
    }
    sprintf(buf, "%d minute%s, %d.%d seconds, ", elapsed_time.minutes, (elapsed_time.minutes != 1 ? "s" : ""), elapsed_time.seconds, elapsed_time.millis);
    retval.append(buf);

    return retval;
}

elapsed_t elapse(uint32_t millis)
{
    long m = millis;
    elapsed_t retval;

    retval.years = m / MILLIS_PER_YEAR;
    m -= MILLIS_PER_YEAR;
    m = max(0, m);

    retval.days = m / MILLIS_PER_DAY;
    m -= MILLIS_PER_DAY;
    m = max(0, m);

    retval.hours = m / MILLIS_PER_HOUR;
    m -= MILLIS_PER_HOUR;
    m = max(0, m);

    retval.minutes = m / MILLIS_PER_MINUTE;
    m -= MILLIS_PER_MINUTE;
    m = max(0, m);

    retval.seconds = m / MILLIS_PER_SECOND;
    m -= MILLIS_PER_SECOND;
    m = max(0, m);

    retval.millis = m;

    return retval;
}
