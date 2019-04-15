/*
  gcode.cpp - GCODE parser functionality / implementation.
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

Queue<char *> queue = Queue<char *>(INSTRUCTION_QUEUE_DEPTH);
Stack<stack_entry_t> stack = Stack<stack_entry_t>(CONFIG_MAX_STACK_DEPTH);
char buff[CONFIG_SHELL_MAX_INPUT];
unit_t unit;

bool bStopRequested = false;

void setup_gcode(int bps)
{
    Serial.begin(bps);
}

void loop_gcode()
{
    static unsigned short count = 0;
    char rxchar = 0;
    char finished = 0;

    // Read communications off the UART until no bytes are available.
    while (Serial.available())
    {
        rxchar = Serial.read();
        rxchar = toupper(rxchar);

        //TODO: Handle interactive stuff (escape sequences, Backspace, arrow keys, etc) here.
        switch (rxchar)
        {
        case SHELL_ASCII_CR: // Enter key pressed
            buff[count] = '\0';
            Serial.println("");
            finished = 1;
            break;
        case SHELL_ASCII_BS: // Backspace pressed
            if (count > 0)
            {
                count--;
                Serial.write(SHELL_ASCII_BS);
                Serial.write(SHELL_ASCII_SP);
                Serial.write(SHELL_ASCII_BS);
            }
            else
                Serial.write(SHELL_ASCII_BEL);
            break;

        default:
            if (count < CONFIG_SHELL_MAX_INPUT && rxchar >= 0x20 && rxchar < 0x7F)
            {
                buff[count] = rxchar;
                Serial.write(rxchar); //echo back.
                count++;
            }
        }
    }

    if (!bStopRequested)
    {
        if (finished)
        {
            // When an EOL is encountered, enqueue the buffer as an instruction.

            finished = 0;
            count = 0;
            //we need to make a copy, since we are enqueueing a reference, and we want to reuse buff.
            char *copy = (char *)malloc(sizeof(char) * CONFIG_SHELL_MAX_INPUT);
            strncpy(copy, buff, CONFIG_SHELL_MAX_INPUT);
            queue.push(copy);
            Serial.println("ok");
        }
    }
    else
    {
        Serial.println("ok //Command ignored. System stopping (M0)");
    }

    // Part 2: Dequeue one instruction (if available), and process it.
    if (runState == RUNNING)
    {
        if (!controller.isRunning())
        {
            if (queue.count() > 0)
            {
                //pop off the queue
                char *instruction = queue.pop();
                execute(instruction);
                //Don't forget to free!
                free(instruction);
            }
            else
            {
                if (bStopRequested)
                {
                    runState = SHUTDOWN;
                    Serial.println("//M0: Shutting down...");
                }
            }
        }
        else
        {
            //already moving
        }
    }
    else
    {
        //robot not ready.
    }
}
// Note: This function returns a pointer to a substring of the original string.
// If the given string was allocated dynamically, the caller must not overwrite
// that pointer with the returned value, since the original pointer must be
// deallocated using the same allocator with which it was allocated.  The return
// value must NOT be deallocated using free() etc.
char *trimwhitespace(char *str)
{
    char *end;

    // Trim leading space
    while (isspace((unsigned char)*str))
        str++;

    if (*str == 0) // All spaces?
        return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end))
        end--;

    // Write new null terminator character
    end[1] = '\0';

    return str;
}

int execute(char *instruction)
{
    //parse the instruction, and dispatch to a handler.
    char *buf = trimwhitespace(instruction);

    if (buf[0] != '\0')
    {
        switch (buf[0])
        {
        case '(':
        case '\'':
        case '/':
            //ignore the entire line?
            return SHELL_RET_SUCCESS;
            break;
        case 'N':
            //line number. Remove this token, and reparse.
            while (isspace((unsigned char)*buf))
                buf++;
            if (*buf == '\0')
            {
                //empty line. We're done.
            }
            else
            {
                execute(buf); //recurse, but without the line number.
            }
            break;
        case 'G':
            //G Code
        case 'M':
            //M Code
        default:
            char *argv_list[CONFIG_SHELL_MAX_COMMAND_ARGS];
            int argc = parse(buf, argv_list);

            if (argc > 0)
            {
                for (int i = 0; i < (sizeof(commands) / sizeof(command_t)); i++)
                {
                    if (!strcmp(commands[i].commandStr, argv_list[0]))
                    {
                        //there is a matching handler.
                        int retval = commands[i].callback(argc, argv_list);
                        //TODO: Do some cleanup here? Something?
                        return retval;
                        break;
                    }
                }
            }
        }
    }
}

int parse(char *buf, char **argv)
{
    int i = 0;
    int argc = 0;
    int length = strlen(buf) + 1; //String length to parse = strlen + 1
    bool toggle = 0;
    for (i = 0; i < length && argc < CONFIG_SHELL_MAX_COMMAND_ARGS; i++)
    {
        switch (buf[i])
        {
        case '\0':
            i = length;
            argc++;
            break;
            // Check for double quotes for strings as parameters
        case '\"':
            if (toggle == 0)
            {
                toggle = 1;
                buf[i] = '\0';
                argv[argc] = &buf[i + 1];
            }
            else
            {
                toggle = 0;
                buf[i] = '\0';
            }
            break;

        case ' ':
            if (toggle == 0)
            {
                buf[i] = '\0';
                argc++;
                argv[argc] = &buf[i + 1];
            }
            break;
        }
    }
    return argc;
}

//===== HANDLERS

/**
 * The G0 command tells the print head to move at maximum travel speed from 
 * the current position to the location / angle specified by the command. 
 * 
 * - The head will move in a coordinated fashion such that all axes complete 
 * the travel simultaneously. 
 * - Movement occurs at the fastest possible speed and acceleration as set 
 * by the configuration for each joint.
 * - Coordinates are given in millimeters, angles are given in degrees, and 
 * these are absolute coordinates in the world frame.
 * 
 * In the context of Flexo, the various axes map to:
 *    X: End effector X coordinate in the world frame.
 *    Y: End effector X coordinate in the world frame.
 *    Z: End effector Z coordinate in the world frame.
 *    U: End effector Yaw angle (in degrees)
 *    V: End effector Pitch angle (in degrees)
 *    W: End effector Roll angle (in degrees)
 * 
 * All Axis parameters are optional. (Though if you don't provide any at all, the robot will not move.)
 * 
 * Example: G0 X7 Y18 Z150 U12 V-50 W0
 **/

int handleMove(int argc, char **argv)
{

    // TODO: "movement occurs at the fastest possible speed as set in the configuration
    // for each joint". But this configuration is on a per-joint basis. The max speed of
    // a joint when moving independently can be very different from the max speed of a
    // joint when moving simultaneously with other joints. We need a mathematical approach
    // to determining realtime acceleration and top speed, on a per-command basis.

    frame_t target = current_frame;

    float speed = 1.0;
    bool bCh = false;

    //parse parameters
    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] == 'X')
        {
            target.x = atof(&argv[i][1]);
            bCh = true;
        }
        else if (argv[i][0] == 'Y')
        {
            target.y = atof(&argv[i][1]);
            bCh = true;
        }
        else if (argv[i][0] == 'Z')
        {
            target.z = atof(&argv[i][1]);
            bCh = true;
        }
        else if (argv[i][0] == 'U')
        {
            target.yaw = atof(&argv[i][1]);
            bCh = true;
        }
        else if (argv[i][0] == 'V')
        {
            target.pitch = atof(&argv[i][1]);
            bCh = true;
        }
        else if (argv[i][0] == 'W')
        {
            target.roll = atof(&argv[i][1]);
            bCh = true;
        }
        else if (argv[i][0] == 'F')
        {
            speed = (float)atof(&argv[i][1]);
            speed = min(1.0f, max(0.0f, speed));
            bCh = true;
        }
        else
        {
            //ignore it.
            //TODO: Support multi-command mode (multiple GCode commands on a single line)
        }
    }

    if (bCh)
    {
        if (unit == INCH)
        {
            target.x *= INCHES_TO_MM;
            target.y *= INCHES_TO_MM;
            target.z *= INCHES_TO_MM;
        }

        if (move_linear(target, speed))
        {
            // Serial.println("ok"); //TODO: Better response string.
            return SHELL_RET_SUCCESS;
        }
        else
        {
            Serial.print("// Error moving: ");
            Serial.println(errorNames[joint_movement_error]);
        }
    }
    else
    {
        // Serial.println("ok // NOP: No movement");
        return SHELL_RET_SUCCESS;
    }
    return SHELL_RET_FAILURE;
}

int handleRapidMove(int argc, char **argv) // G00
{
    return handleMove(argc, argv);
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
    //for now, just do the same thing as G01.

    //TODO: Implement G01 correctly.
    return handleMove(argc, argv);
}

/**
 * Move in an Clockwise arc defined by the provided parameters:
 * 
 * Xnnn The position to move to on the X axis
 * Ynnn The position to move to on the Y axis
 * Znnn The position to move to on the Z axis
 * Unnn Target yaw
 * Vnnn Target pitch
 * Wnnn Target roll
 * Innn The point in X space from the current X position to maintain a constant distance from
 * Jnnn The point in Y space from the current Y position to maintain a constant distance from
 * Knnn The point in Z space from the current Z position to maintain a constant distance from
 * Fnnn (optional) The relative speed (from 0.0 to 0.1) of the move between the starting point and ending point
 **/
int handleCircleMoveCW(int argc, char **argv) // G02
{
    //TODO: Prerequisite: Support waypoint queueing.
    Serial.println("ok // G02 Not yet implemented.");
    return SHELL_RET_SUCCESS;
}

/**
 * Move in an Counter-Clockwise arc defined by the provided parameters:
 * 
 * Parameters:
 *    Xnnn The position to move to on the X axis
 *    Ynnn The position to move to on the Y axis
 *    Znnn The position to move to on the Z axis
 *    Unnn Target yaw
 *    Vnnn Target pitch
 *    Wnnn Target roll
 *    Innn The point in X space from the current X position to maintain a constant distance from
 *    Jnnn The point in Y space from the current Y position to maintain a constant distance from
 *    Knnn The point in Z space from the current Z position to maintain a constant distance from
 *    Fnnn (optional) The relative speed (from 0.0 to 0.1) of the move between the starting point and ending point
 **/
int handleCircleMoveCCW(int argc, char **argv) // G03
{
    Serial.println("ok // G03 Not yet implemented.");
    return SHELL_RET_SUCCESS;
}

/**
 * Pause the machine for a period of time.
 * Parameters:
 *     Pnnn Time to wait, in milliseconds (P0, wait until all previous moves are finished)
 *     Snnn Time to wait, in seconds
 **/
int handleDwell(int argc, char **argv) // G04
{
    int delay_ms = -1;
    //parse parameters
    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] == 'P')
        {
            delay_ms = atoi(&argv[i][1]);
        }
        else if (argv[i][0] == 'S')
        {
            delay_ms = atoi(&argv[i][1]);
            delay_ms *= 1000;
        }
    }

    if (delay_ms == -1)
    {
        Serial.println("//G04: Missing required parameter: Pnnn or Snnn");
        return SHELL_RET_FAILURE;
    }

    if (delay_ms == 0)
    {
        while (controller.isRunning()) //because the controller is interrupt driven, it's better to check this than moveState
        {
            delay(1);
        } //Note: We don't set the moveState here. That'll be handled in loop_joints
    }
    else
    {
        delay(delay_ms); //just sleep.
    }
    Serial.println("ok //G04: Dwell complete.");

    return SHELL_RET_SUCCESS;
}

int handleInch(int argc, char **argv) // G20
{
    unit = MM;
    Serial.println("ok //G20: Unit set to INCHES");
    return SHELL_RET_SUCCESS;
}

int handleMillimeters(int argc, char **argv) // G21
{
    unit = INCH;
    Serial.println("ok //G21: Unit set to MILLIMETERS");
    return SHELL_RET_SUCCESS;
}

/**
 * Parameters:
 *    This command can be used without any additional parameters.
 *    X Flag to go back to the X axis origin
 *    Y Flag to go back to the Y axis origin
 *    Z Flag to go back to the Z axis origin
 *    U Flag to go back to the yaw origin
 *    V Flag to go back to the pitch origin
 *    W Flag to go back to the roll origin
 **/
int handleHome(int argc, char **argv) // G28
{
    move_home();
    Serial.println("ok //G28: Homing.");
    return SHELL_RET_SUCCESS;
}

int handleAbsolute(int argc, char **argv) // G90
{
    movement_mode = ABSOLUTE;
    Serial.println("ok //G90: Movement absolute.");
    return SHELL_RET_SUCCESS;
}

int handleRelative(int argc, char **argv) // G91
{
    movement_mode = RELATIVE;
    Serial.println("ok //G91: Movement relative.");
    return SHELL_RET_SUCCESS;
}

int handleSetMinimum(int argc, char **argv) // G161
{
    if (argc > 1)
    {
        //parse parameters
        for (int i = 0; i < argc; i++)
        {
            if (argv[i][0] == 'J') //we only handle the first J parameter we ome across.
            {
                int joint_num = atoi(&argv[1][1]);
                jointConfig[joint_num].minPosition = motors[joint_num]->getPosition();
                Serial.printf("ok //G162: New minimum set for joint #%d.\n", joint_num);
                return SHELL_RET_SUCCESS;
            }
        }
    }

    //no J parameters.
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        jointConfig[i].minPosition = motors[i]->getPosition();
    }
    Serial.println("ok //G161: New minimums set.");
    return SHELL_RET_SUCCESS;
}

int handleSetMaximum(int argc, char **argv) // G162
{
    if (argc > 1)
    {
        //parse parameters
        for (int i = 0; i < argc; i++)
        {
            if (argv[i][0] == 'J') //we only handle the first J parameter we ome across.
            {
                int joint_num = atoi(&argv[1][1]);
                jointConfig[joint_num].maxPosition = motors[joint_num]->getPosition();
                Serial.printf("ok //G162: New maximum set for joint #%d.\n", joint_num);
                return SHELL_RET_SUCCESS;
            }
        }
    }

    //no J parameters.
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        jointConfig[i].maxPosition = motors[i]->getPosition();
    }
    Serial.println("ok //G162: New maximums set.");
    return SHELL_RET_SUCCESS;
}

int handleSetPosition(int argc, char **argv) //G92
{
    if (argc == 0)
    {
        //set current position as the zero point.
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            motors[i]->setPosition(0);
        }
    }
    else
    {
        Serial.println("ok //G92: Not yet supported.");
    }
}

int handleStop(int argc, char **argv) // M00
{
    bStopRequested = true;
    Serial.println("ok //M00: Stop requested.");
    return SHELL_RET_SUCCESS;
}

int handleEmergencyStop(int argc, char **argv) // M112
{
    bStopRequested = true;
    controller.emergencyStop();
    runState = SHUTDOWN;

    Serial.println("ok //M112: Emergency Stop requested.");
    return SHELL_RET_SUCCESS;
}

int handleGetPosition(int argc, char **argv) // M114
{
    Serial.printf("ok X:%8.4f Y:%8.4f Z:%8.4f U:%8.4f V:%8.4f W:%8.4f\n", current_frame.x, current_frame.y, current_frame.z, current_frame.yaw, current_frame.pitch, current_frame.roll);
    return SHELL_RET_SUCCESS;
}

int handleGetVersion(int argc, char **argv) // M115
{
    Serial.printf("ok FIRMWARE_VERSION:%s FIRMWARE_URL:%s MACHINE_TYPE:%s\n", FIRMWARE_VERSION, FIRMWARE_URL, MACHINE_TYPE);
    return SHELL_RET_SUCCESS;
}

// int handleWait(int argc, char **argv) // M116
// {
//     return SHELL_RET_SUCCESS;
// }

int handleMsg(int argc, char **argv) // M117
{
    Serial.println("ok //M117: Not yet implemented");
    return SHELL_RET_SUCCESS;
}

int handlePush(int argc, char **argv) // M120
{
    //push some state on to the stack.
    //movementMode
    //current_frame
    //units
    stack_entry_t e = {
        movement_mode,
        current_frame,
        unit};
    stack.push(e);
    Serial.println("ok //M120 Stack push complete.");
    return SHELL_RET_SUCCESS;
}

int handlePop(int argc, char **argv) // M121
{
    //pop state off the stack, and move the robot.
    //movementMode
    //current_frame
    //units
    if (stack.count() > 0)
    {
        while (controller.isRunning())
        {
            delay(1); //wait for motors to stop moving.
        }
        stack_entry_t e = stack.pop();
        movement_mode = e.movement_mode;
        unit = e.unit;

        move_linear(e.current_frame);
        while (controller.isRunning())
        {
            delay(1); //wait for motors to stop moving.
        }
        Serial.println("ok //M121 State pop complete.");
        return SHELL_RET_SUCCESS;
    }
    return SHELL_RET_FAILURE;
}

int handleSetHome(int argc, char **argv) // M306
{
    if (argc > 1)
    {
        //parse parameters
        for (int i = 0; i < argc; i++)
        {
            if (argv[i][0] == 'J') //we only handle the first J parameter we ome across.
            {
                int joint_num = atoi(&argv[1][1]);
                jointConfig[joint_num].homePosition = motors[joint_num]->getPosition();
                Serial.printf("ok //M306: New home set for joint #%d.\n", joint_num);
                return SHELL_RET_SUCCESS;
            }
        }
    }

    //no J parameters.
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        jointConfig[i].homePosition = motors[i]->getPosition();
    }
    Serial.println("ok //M306: New home set.");
    return SHELL_RET_SUCCESS;
}
