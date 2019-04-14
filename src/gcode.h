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

#define CONFIG_MAX_STACK_DEPTH 5
#define CONFIG_SHELL_MAX_INPUT 80        //how many characters max per line of serial input?
#define CONFIG_SHELL_MAX_COMMAND_ARGS 10 //how many arguments maximum per GCode command?
#define INSTRUCTION_QUEUE_DEPTH 32       //how many instructions can we have waiting for processing.

// #define SHELL_ASCII_NUL 0x00
#define SHELL_ASCII_BEL 0x07
#define SHELL_ASCII_BS 0x08
// #define SHELL_ASCII_HT 0x09
#define SHELL_ASCII_LF 0x0A
#define SHELL_ASCII_CR 0x0D
#define SHELL_ASCII_ESC 0x1B
// #define SHELL_ASCII_DEL 0x7F
// #define SHELL_ASCII_US 0x1F
#define SHELL_ASCII_SP 0x20
// #define SHELL_VT100_ARROWUP 'A'
// #define SHELL_VT100_ARROWDOWN 'B'
// #define SHELL_VT100_ARROWRIGHT 'C'
// #define SHELL_VT100_ARROWLEFT 'D'
#define SHELL_RET_SUCCESS 1
#define SHELL_RET_FAILURE 0
#define INCHES_TO_MM 25.4

#include "flexo.h"

enum unit_t
{
  MM,
  INCH
};

/**
* Handler signature
**/
typedef int (*gcode_handler_t)(int, char **);

typedef struct
{
  gcode_handler_t callback;
  const char *commandStr;
} command_t;

typedef struct
{
  movementMode_t movement_mode;
  frame_t current_frame;
  unit_t unit;
} stack_entry_t;

/**
 * Set up serial communications and instruction Queue.
 **/
void setup_gcode(int bps = SERIAL_BAUD);

/**
 * Part 1: Read communications off the UART until no bytes are available. When an EOL is encountered, enqueue the buffer as an instruction.
 * Part 2: Dequeue one instruction (if available), and process it.
 * 
 * Instructions are guaranteed to execute serially. That means that, if the robot is already in motion, no additional instructions will be processed until it stops moving.
 * 
 * Caveat: traffic on the UART blocks processing of instructions, and vice-versa. 
 **/
void loop_gcode();

int parse(char *buf, char **argv); //convert a command string to a count of arguments, and a tokenized list of parameters.
int execute(char *instruction);    //execute a command (delegating to a command-specific handler)

int handleRapidMove(int argc, char **argv);      // G00
int handleControlledMove(int argc, char **argv); // G01
int handleCircleMoveCW(int argc, char **argv);   // G02
int handleCircleMoveCCW(int argc, char **argv);  // G03
int handleDwell(int argc, char **argv);          // G04
int handleInch(int argc, char **argv);           // G20
int handleMillimeters(int argc, char **argv);    // G21
int handleHome(int argc, char **argv);           // G28
int handleAbsolute(int argc, char **argv);       // G90
int handleRelative(int argc, char **argv);       // G91

// MCode
/**
 * The machine will finish any commands left in its queue, and then shut down.
 * This command can be used without any additional parameters.
 *    Pnnn Time to wait, in milliseconds1
 *    Snnn Time to wait, in seconds2
 **/
int handleStop(int argc, char **argv); // M00

/**
 * Immediately stop all movement and shut down unconditionally.
 **/
int handleEmergencyStop(int argc, char **argv); // M112
int handleGetPosition(int argc, char **argv);   // M114
int handleGetVersion(int argc, char **argv);    // M115
// int handleWait(int argc, char **argv);        // M116
int handleMsg(int argc, char **argv);  // M117
int handlePush(int argc, char **argv); // M120
int handlePop(int argc, char **argv);  // M121
/**
 * Set the home position for the joint to the current position of the joint.
 * Parameters:
 *    Jnnn (Optional) - The joint to set.
 * 
 * Example:
 *    G306 - Set all joint homes to current positions.
 *    G306 J1 - Set home position for Joint #1.
 **/
int handleSetHome(int argc, char **argv); // M306

/**
 * Set the minimum position for the joint to the current position of the joint.
 * Parameters:
 *    Jnnn (Optional) - The joint to set.
 * 
 * Example:
 *    G306 - Set all joint minimums to current positions.
 *    G306 J1 - Set minimum position for Joint #1.
 **/
int handleSetMinimum(int argc, char **argv); // G161

/**
 * Set the maximum position for the joint to the current position of the joint.
 * Parameters:
 *    Jnnn (Optional) - The joint to set.
 * 
 * Example:
 *    G306 - Set all joint maximums to current positions.
 *    G306 J1 - Set maximum position for Joint #1.
 **/

int handleSetMaximum(int argc, char **argv); // G162

const command_t commands[] = {
    {handleRapidMove, "G00"},
    {handleControlledMove, "G01"},
    {handleCircleMoveCW, "G02"},
    {handleCircleMoveCCW, "G03"},
    {handleDwell, "G04"},
    {handleRapidMove, "G0"},
    {handleControlledMove, "G1"},
    {handleCircleMoveCW, "G2"},
    {handleCircleMoveCCW, "G3"},
    {handleDwell, "G4"},
    {handleInch, "G20"},
    {handleMillimeters, "G21"},
    {handleHome, "G28"},
    {handleAbsolute, "G90"},
    {handleRelative, "G91"},
    {handleSetMinimum, "G161"},
    {handleSetMaximum, "G162"},
    {handleStop, "M00"},
    {handleStop, "M0"},
    {handleEmergencyStop, "M112"},
    {handleGetPosition, "M114"},
    {handleGetVersion, "M115"},
    // {handleWait, "M116"},
    {handleMsg, "M117"},
    {handlePush, "M120"},
    {handlePop, "M121"},
    {handleSetHome, "M306"}};

#endif //__GCODE2_H__