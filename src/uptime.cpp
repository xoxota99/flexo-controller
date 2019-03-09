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

elapsed_t elapsed_time;

String uptime()
{
    String retval;
    char buf[50];

    if (elapsed_time.years > 0)
    {
        sprintf(buf, "%llu year%s, ", elapsed_time.years, elapsed_time.years != 1 ? "s" : "");
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

void setup_uptime()
{
    elapsed_time = {
        millis : 0,
        seconds : 0,
        minutes : 0,
        hours : 0,
        days : 0,
        years : 0
    };
}

void loop_uptime()
{
    static uint32_t m = 0;
    uint32_t t = millis();
    if (t - m >= MILLIS_PER_SECOND)
    {
        m = t - MILLIS_PER_SECOND;
        elapsed_time.seconds++;
        if (elapsed_time.seconds >= SECONDS_PER_MINUTE)
        {
            elapsed_time.minutes++;
            elapsed_time.seconds = 0;
            if (elapsed_time.minutes >= MINUTES_PER_HOUR)
            {
                elapsed_time.hours++;
                elapsed_time.minutes = 0;
                if (elapsed_time.hours >= HOURS_PER_DAY)
                {
                    elapsed_time.days++;
                    elapsed_time.hours = 0;
                    if (elapsed_time.days >= DAYS_PER_YEAR)
                    {
                        elapsed_time.years++;
                        elapsed_time.days = 0;
                    }
                }
            }
        }
    }
}