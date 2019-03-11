/*
  protocol.cpp - Stuff related to the client/controller binary protocol.
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
#include "endpoint.h"

void setup_endpoint() {}
void loop_endpoint()
{
    if (shellMode == BINARY)
    {
        if (Serial.available())
        {
            //there are bytes.
            header_t msgHead;
            //STEP 1: Read the message header, to figure out how to load the message.
            //STEP 2: Load the message. Calculate the CRC.
            //STEP 3: Read the message footer, compare the CRC.
            //STEP 4: ACK.
            //STEP 5: Process the command (Asynchronously?)
            //STEP 6: construct and send a response.
        }
    }
}