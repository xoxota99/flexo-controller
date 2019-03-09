/*
  shell.h - Interactive shell command processor functions.
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
#ifndef __SHELL_H__
#define __SHELL_H__

#include "flexo.h"
#include <Shell.h> //https://github.com/geekfactory/Shell

// Usage messages
#define USAGE_MOVE "Move the given joint, by a relative amount (in steps).\n \
\t - The joint will accelerate and move at a speed determined by it's motor configuration.\n \
\t - The joint will not exceed its preset minimum / maximum limits unless the -o parameter is specified.\n\n \
usage: move <joint #> <steps> [-o]\n \
\t - joint # : The (zero-based) index of the joint to move (0 - 5).\n \
\t - steps : The number of steps to move. A positive number indicates clockwise movement. A negative number indicates conuterclockwise movement.\n \
\t - -o : (Optional) Override the preset limits (minimum / maximum) for this joint.\n \
\n \
Example : \"move 3 -20 -o\" - Move joint #3 counterclockwise by 20 steps, ignoring the joint's min/max limits."

#define USAGE_LOG "Set the log level for the software. \n \
usage: log [TRACE | DEBUG | INFO | WARN | ERROR | FATAL]"

#define USAGE_SET "Set the extent (minimum, maximum or zero point) of a given joint. \nThis command is guaranteed to NOT cause the robot to move.\n \
usage: set <extent> [<joint #>]\n \
\t - joint # : (Optional) The (zero-based) index of the joint to configure (0 - 5). If ommitted, all joints will be configured\n \
\t - extent : The extent to configure. One of:\n \
\t\t - min : The minimum angle of the joint.\n \
\t\t - max : The maximum angle of the joint.\n \
\t\t - zero : The zero position of the joint.\n\n \
Example : \"set 3 min\" - Set the minimum extent of joint #3 to its current position."

typedef struct Command
{
  shell_program_t shell_program;
  const char *shell_command_string;
  const char *shell_help_string;
} Command;

void setup_shell(int bps = SERIAL_BAUD);
void loop_shell();

int shell_reader(char *data);
void shell_writer(char data);

int handleHelp(int argc, char **argv);   // Help message
int handleMove(int argc, char **argv);   // Move a joint
int handleLog(int argc, char **argv);    // Change log level
int handleStatus(int argc, char **argv); // Dump current joint information.
int handleSet(int argc, char **argv);    // Set (minimum / maximum / zero point)
int handleReset(int argc, char **argv);  // Reset arduino

//===

const Command commands[] = {
    {handleHelp, "?", "This message."},
    {handleHelp, "help", "This message."},
    {handleMove, "move", "Move a joint by/to a specific angle in degrees (relative or absolute movement)"},
    {handleLog, "log", "Change logging level."},
    {handleStatus, "status", "Dump system information"},
    {handleReset, "reset", "Reset controller"},
    {handleSet, "set", "Set a joint minimum / maximum / zero point"}};

#endif // __SHELL_H__