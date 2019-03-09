/*
  protocol.h - Stuff related to the client/controller binary protocol.
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

#ifndef __ENDPOINT_H__
#define __ENDPOINT_H__

#include "flexo.h"
/**
 * This is the binary equivalent of shell.h. Provides an interface (over UART) that processes and executes commands from the client application.
 **/

void setup_endpoint();
void loop_endpoint();

#endif //__ENDPOINT_H__