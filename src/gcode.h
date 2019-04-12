/*
  gcode.h - GCODE parser message definitions.
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

#ifndef __GCODE_H__

#define __GCODE_H__

#include "flexo.h"

typedef struct command_t
{
  shell_program_t callback;
  const char *commandStr;
  // const char *shell_help_string;
} command_t;

void setup_gcode(int bps = SERIAL_BAUD);
void loop_gcode();

int gcode_reader(char *data);
void gcode_writer(char data);

int handleRapidMove(int argc, char **argv);      // G00
int handleControlledMove(int argc, char **argv); // G01
// int handleCircleMoveCW(int argc, char **argv);   // G02
// int handleCircleMoveCCW(int argc, char **argv);  // G03
int handleDwell(int argc, char **argv);       // G04
int handleInch(int argc, char **argv);        // G20
int handleMillimeters(int argc, char **argv); // G21
int handleHome(int argc, char **argv);        // G28
int handleAbsolute(int argc, char **argv);    // G90
int handleRelative(int argc, char **argv);    // G91

// MCode
int handleStop(int argc, char **argv);        // M00 / M01 / M02 / M30
int handleGetPosition(int argc, char **argv); // M114
int handleGetVersion(int argc, char **argv);  // M115
int handleWait(int argc, char **argv);        // M116
int handleMsg(int argc, char **argv);         // M117
int handlePush(int argc, char **argv);        // M120
int handlePop(int argc, char **argv);         // M121
int handleDiags(int argc, char **argv);       // M122
int handlesetP(int argc, char **argv);        // M130
int handlesetI(int argc, char **argv);        // M131
int handlesetD(int argc, char **argv);        // M132

//===

const command_t commands[] = {
    {handleRapidMove, "G00"},
    {handleControlledMove, "G01"},
    // {handleCircleMoveCW, "G02"},
    // {handleCircleMoveCCW, "G03"},
    {handleDwell, "G04"},
    {handleRapidMove, "G0"},
    {handleControlledMove, "G1"},
    // {handleCircleMoveCW, "G2"},
    // {handleCircleMoveCCW, "G3"},
    {handleDwell, "G4"},
    {handleInch, "G20"},
    {handleMillimeters, "G21"},
    {handleHome, "G28"},
    {handleAbsolute, "G90"},
    {handleRelative, "G91"},
    {handleStop, "M00"},
    {handleStop, "M01"},
    {handleStop, "M02"},
    {handleStop, "M0"},
    {handleStop, "M1"},
    {handleStop, "M2"},
    {handleStop, "M30"},
    {handleGetPosition, "M114"},
    {handleGetVersion, "M115"},
    {handleWait, "M116"},
    {handleMsg, "M117"},
    {handlePush, "M120"},
    {handlePop, "M121"},
    {handleDiags, "M122"},
    {handlesetP, "M130"},
    {handlesetI, "M131"},
    {handlesetD, "M132"}};

#endif //__GCODE_H__