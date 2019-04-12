/*
  gcode.h - GCODE parser functionality / implementation.
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

#include "gcode.h"

void setup_gcode(int bps)
{

  if (!Serial)
  {
    Serial.begin(bps);
    delay(50);
  }

  if (shell_init(gcode_reader, gcode_writer, ""))
  {

    Serial.println("start");

    const int c1 = sizeof(commands);
    if (c1 > 0)
    {
      const int c2 = sizeof(commands[0]);
      const int ccount = c1 / c2;

      for (int i = 0; i < ccount; i++)
      {
        if (shell_register(commands[i].callback, commands[i].commandStr))
        {
          // cCount++;
        }
        else
        {
          Serial.println("!!");
        }
      }
    }
  }
  else
  {
    Serial.println("!!");
  }
  delay(100);
}

void loop_gcode()
{
  shell_task();
}

int gcode_reader(char *data)
{
  // Wrapper for Serial.read() method
  if (Serial.available())
  {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

void gcode_writer(char data)
{
  Serial.write(data);
}

//===== HANDLERS

/**
 * The G0 command tells the print head to move at maximum travel speed from 
 * the current position to the coordinates specified by the command. The head 
 * will move in a coordinated fashion such that both axes complete the travel 
 * simultaneously. 
 * 
 * Example: G0 X7 Y18 Z150 U12 V-50 W0
 **/
int handleRapidMove(int argc, char **argv) // G00
{
  return SHELL_RET_SUCCESS;
}

/**
 * The G1 command tells the print head to move at specified speed from the 
 * current position to the coordinated specified by the G-code command. The 
 * speed is specified by the Feed rate parameter F.
 * 
 * Example: G1 X7 Y18 Z150 U12 V-50 W0 F500 
 **/
int handleControlledMove(int argc, char **argv) // G01
{
  return SHELL_RET_SUCCESS;
}

// int handleCircleMoveCW(int argc, char **argv) // G02
// {
//   return SHELL_RET_SUCCESS;
// }

// int handleCircleMoveCCW(int argc, char **argv) // G03
// {
//   return SHELL_RET_SUCCESS;
// }

/**
 * 
 **/
int handleDwell(int argc, char **argv) // G04
{
  return SHELL_RET_SUCCESS;
}

int handleInch(int argc, char **argv) // G20
{
  return SHELL_RET_SUCCESS;
}

int handleMillimeters(int argc, char **argv) // G21
{
  return SHELL_RET_SUCCESS;
}

int handleHome(int argc, char **argv) // G28
{
  return SHELL_RET_SUCCESS;
}

int handleAbsolute(int argc, char **argv) // G90
{
  return SHELL_RET_SUCCESS;
}

int handleRelative(int argc, char **argv) // G91
{
  return SHELL_RET_SUCCESS;
}

// MCode
int handleStop(int argc, char **argv) // M00 / M01 / M02 / M30
{
  return SHELL_RET_SUCCESS;
}

int handleGetPosition(int argc, char **argv) // M114
{
  return SHELL_RET_SUCCESS;
}

int handleGetVersion(int argc, char **argv) // M115
{
  return SHELL_RET_SUCCESS;
}

int handleWait(int argc, char **argv) // M116
{
  return SHELL_RET_SUCCESS;
}

int handleMsg(int argc, char **argv) // M117
{
  return SHELL_RET_SUCCESS;
}

int handlePush(int argc, char **argv) // M120
{
  return SHELL_RET_SUCCESS;
}

int handlePop(int argc, char **argv) // M121
{
  return SHELL_RET_SUCCESS;
}

int handleDiags(int argc, char **argv) // M122
{
  return SHELL_RET_SUCCESS;
}

int handlesetP(int argc, char **argv) // M130
{
  return SHELL_RET_SUCCESS;
}

int handlesetI(int argc, char **argv) // M131
{
  return SHELL_RET_SUCCESS;
}

int handlesetD(int argc, char **argv) // M132
{
  return SHELL_RET_SUCCESS;
}
