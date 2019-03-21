/*
  flexo.h - main Flexo include file
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

#ifndef __FLEXO_H__
#define __FLEXO_H__

// Flexo versioning system
#define FLEXO_VERSION "0.1a"

// Define standard libraries used by Flexo.
#include <Arduino.h>
#include <AS5048A.h>
#include <StepControl.h>

// Define the Flexo system include files. NOTE: Do not alter organization.
#include "config.h"
#include "uptime.h"
#include "reset.h"
#include "Logger.h"
#include "Shell.h"
#include "cpu_map.h"
#include "joint.h"
#include "ik_solver.h"
#include "commands.h"
#include "protocol.h"
#include "endpoint.h"
// ---------------------------------------------------------------------------------------

void setup_led();
void loop_led();

#endif // __FLEXO_H__