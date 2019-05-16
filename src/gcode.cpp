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

//TODO: This entire GCode implementation is just awful. Replace it with GRBL if possible.

//TODO: GCode comes in "words". Tehre are "command words" and "parameter words". Parse
// using this model, and become less reliant on whitespace.

/** 
 * TODO: Implement "modal groups" (See https://www.autodesk.com/industry/manufacturing/resources/manufacturing-engineer/g-code ):
 * Group 1 (motion): G00, G01, G02, G03, G80, G81, G82, G84, G85, G86, G87, G88, G89
 * Group 2 (plane selection – XY, YZ, ZX): G17, G18, G19
 * Group 3 (absolute/incremental mode): G90, G91
 * Group 5 (feed rate mode): G93, G94
 * Group 6 (units – inches/millimeters): G20, G21
 * Group 7 (cutter radius compensation – CRC): G40, G41, G42
 * Group 8 (tool length offset – TLO): G43, G49
 * Group 10 (return mode in canned cycles): G98, G99
 * Group 12 (work coordinate system selection – WCSS): G54, G55, G56, G57, G58, G59)
 **/

#include "gcode.h"

// Queue<char *> queue = Queue<char *>(INSTRUCTION_QUEUE_DEPTH);
Stack<stack_entry_t> stack = Stack<stack_entry_t>(CONFIG_MAX_STACK_DEPTH);
char buff[CONFIG_SHELL_MAX_INPUT];

int gcode_line_num = 0;

bool bStopRequested = false;

char *argv_list[CONFIG_SHELL_MAX_COMMAND_ARGS]; //reuse array of string pointers.

movementMode_t movement_mode = ABSOLUTE;

void setup_gcode(int bps)
{
    Serial.begin(bps);
}

// Read -> Parse -> Execute(GCODE | NON-GCODE)
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

        // Handle interactive stuff (escape sequences, Backspace, arrow keys, etc) here.
        switch (rxchar)
        {
        case 0:
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
            if (count < CONFIG_SHELL_MAX_INPUT && rxchar >= 0x20 && rxchar < 0x7F) // alphanumeric
            {
                buff[count] = rxchar;
                Serial.write(rxchar); //echo back.
                count++;
            }
        }
    }

    if (!bStopRequested) // A previous command has requested a stop of the robot, so ignore any subsequent commands.
    {
        if (finished) // we have finished reading the command "Sentence".
        {
            finished = 0;
            count = 0;
            if (runState == READY)
            {
                if (!controller.isRunning())
                {
                    //parse
                    execute_gcode(buff);
                }
                else //!controller.isRunning()
                {
                    //already moving
                    Serial.println("ok //Command ignored. Robot is already moving");
                }
            }
            else //runState != READY
            {
                //robot not ready.
                Serial.printf("ok //Command ignored. Robot is in state %s", runStateNames[runState]);
            }
        }
    }
    else
    {
        if (finished)
        {
            Serial.println("ok //Command ignored. System stopping (M0)");
        }
    }
}

/**
 *  returns a pointer to a substring of the original string.
 * If the given string was allocated dynamically, the caller must not overwrite
 * that pointer with the returned value, since the original pointer must be
 * deallocated using the same allocator with which it was allocated.  The return
 * value must NOT be deallocated using free() etc.
 **/
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

/**
 * parse, then execute the raw buffer.
 **/
int execute_gcode(char *instruction)
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
                return execute_gcode(buf); //recurse, but without the line number.
            }
            break;
        case 'G':
            //G Code
        case 'M':
            //M Code
        default:
            char *argv_list[CONFIG_SHELL_MAX_COMMAND_ARGS];
            int argc = parse_gcode(buf, argv_list);

            if (argc > 0)
            {
                for (unsigned i = 0; i < (sizeof(commands) / sizeof(command_t)); i++)
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
    return SHELL_RET_FAILURE;
}

/**
 * Take in a string, and return an array of argument strings, and an integer count of arguments.
 **/
int parse_gcode(char *buf, char *argv[])
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
 * - Coordinates represent the target angle of each joint in degrees.
 * 
 * In the context of Flexo, the various axes map to:
 *    X: Joint 1 angle (in degrees)
 *    Y: Joint 2 angle (in degrees)
 *    Z: Joint 3 angle (in degrees)
 *    U: Joint 4 angle (in degrees)
 *    W: Joint 5 angle (in degrees)
 *    V: Joint 6 angle (in degrees)
 *    F: relative speed (between 0 and 1).
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

    double theta[MOTOR_COUNT];
    uint8_t shouldMove = 0;
    float speed = 1.0;

    //parse parameters
    for (int i = 1; i < argc; i++)
    {
        switch (argv[i][0])
        {
        case 'X':
            theta[0] = atof(&argv[i][1]);
            shouldMove &= 1;
            break;
        case 'Y':
            theta[1] = atof(&argv[i][1]);
            shouldMove &= 1 << 1;
            break;
        case 'Z':
            theta[2] = atof(&argv[i][1]);
            shouldMove &= 1 << 2;
            break;
        case 'U':
            theta[3] = atof(&argv[i][1]);
            shouldMove &= 1 << 3;
            break;
        case 'V':
            theta[4] = atof(&argv[i][1]);
            shouldMove &= 1 << 4;
            break;
        case 'W':
            theta[5] = atof(&argv[i][1]);
            shouldMove &= 1 << 5;
            break;
        case 'F':
            speed = atof(&argv[i][1]);
            break;
        default:
            Logger::warn("Unrecognized parameter '%c'", argv[i][0]);
            break;
        }
    }
    bool success = false;
    if (shouldMove) //at least one parameter was provided
    {
        switch (movement_mode)
        {
        case RELATIVE:
            success = move_relative(shouldMove, theta, speed);
            break;
        default:
            Logger::warn("movementMode is unrecognized. Using ABSOLUTE.");
        case ABSOLUTE:
            success = move_absolute(shouldMove, theta, speed);
            break;
        }

        if (success)
        {
            // Serial.println("ok"); //TODO: Better response string.
            return SHELL_RET_SUCCESS;
        }
        else
        {
            Serial.print("// Error moving: ");
            Serial.println(errorNames[joint_movement_error]);
            return SHELL_RET_FAILURE;
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
    return handleMove(argc, argv);
}

/**
 * Move in an Clockwise arc, starting from the current position, according to the provided parameters:
 * 
 * Parameters:
 *    Xnnn the X coordinate of the arc centerpoint
 *    Ynnn the Y coordinate of the arc centerpoint
 *    Znnn the Z coordinate of the arc centerpoint
 *    Innn the relative angle, in degrees, of the arc. (0-360).
 *    Fnnn (optional) the relative speed (from 0.0 to 0.1) of the movement.
 * 
 **/

//NOTE: This is a very system level movement, and may require an IK Solution to implement.
// Might be worth moving this up to the RPI layer.
int handleCircleMoveCW(int argc, char **argv) // G02
{
    //TODO: Prerequisite: Support waypoint queueing in motor controller.
    Serial.println("ok // G02 Not yet implemented.");
    return SHELL_RET_SUCCESS;
}

/**
 * Move in an Counter-Clockwise arc, starting from the current position, according to the provided parameters:
 * 
 * Parameters:
 *    Xnnn the X coordinate of the arc centerpoint
 *    Ynnn the Y coordinate of the arc centerpoint
 *    Znnn the Z coordinate of the arc centerpoint
 *    Innn the relative angle, in degrees, of the arc. (0-360).
 *    Fnnn (optional) the relative speed (from 0.0 to 0.1) of the movement.
 * 
 **/

//NOTE: This is a very system level movement, and may require an IK Solution to implement.
// Might be worth moving this up to the RPI layer.
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
            delay_ms = atof(&argv[i][1]);
            break;
        }
        else if (argv[i][0] == 'S')
        {
            delay_ms = atof(&argv[i][1]);
            delay_ms *= 1000;
            break;
        }
    }

    if (delay_ms == -1)
    {
        Serial.println("//G04: Missing required parameter: Pnnn or Snnn");
        return SHELL_RET_FAILURE;
    }

    if (delay_ms == 0)
    {
        //wait until any asynchronous movement completes.
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

/**
 * The G28 code treats XYZ/ABC coordinates slightly differently than other "Move" codes. 
 * Normally (such as in G0 or G1) these coordinates would refer to the pose of the end 
 * effector's frame origin, relative to the world frame origin. In the case of G28, we 
 * don't care about the location of the end effector, as much as we care about the 
 * location of individual joints, so X refers to J1, Y refers to J2, Z refers to 
 * J3, and so on.
 * 
 * Parameters:
 *  This command can be used without any additional parameters. 
 *    X Flag to move J1 to its minimum end-stop position.
 *    Y Flag to move J2 to its minimum end-stop position.
 *    Z Flag to move J3 to its minimum end-stop position.
 *    A Flag to move J4 to its minimum end-stop position.
 *    B Flag to move J5 to its minimum end-stop position.
 *    C Flag to move J6 to its minimum end-stop position.
 **/
int handleHome(int argc, char **argv) // G28
{
    bool x, y, z, a, b, c;
    if (argc > 1)
    {
        //parse parameters
        for (int i = 0; i < argc; i++)
        {
            switch (argv[i][0])
            {
            case 'X':
                x = true;
                break;
            case 'Y':
                y = true;
                break;
            case 'Z':
                z = true;
                break;
            case 'A':
                a = true;
                break;
            case 'B':
                b = true;
                break;
            case 'C':
                c = true;
                break;
            default:
                //ignore.
                break;
            }
        }
    }
#ifdef LIMIT_SWITCHES_SUPPORTED
    safe_zero(x, y, z, a, b, c);
#else
    unsafe_zero(x, y, z, a, b, c);
#endif
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
    if (argc == 1)
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
    return SHELL_RET_SUCCESS;
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
    Serial.printf("ok X:%8.4f Y:%8.4f Z:%8.4f U:%8.4f V:%8.4f W:%8.4f F:%8.4f\n", px, py, pz, pu, pv, pw, feed_rate);
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
    //ee_frame
    //units
    stack_entry_t e = {
        movement_mode, px, py, pz, pu, pv, pw};

    if (stack.count() < CONFIG_MAX_STACK_DEPTH)
    {

        stack.push(e);
        Serial.println("ok //M120 Stack push complete.");
        return SHELL_RET_SUCCESS;
    }
    else
    {
        Serial.println("!! //M120 Maximum stack depth exceeded.");
        return SHELL_RET_FAILURE;
    }
}

int handlePop(int argc, char **argv) // M121
{
    //pop state off the stack, and move the robot.
    //movementMode
    //ee_frame
    //units
    if (stack.count() > 0)
    {
        while (controller.isRunning())
        {
            delay(1); //wait for motors to stop moving.
        }
        stack_entry_t e = stack.pop();
        movement_mode = e.movement_mode;
        double cur_pos[MOTOR_COUNT] = {e.px, e.py, e.pz, e.pu, e.pv, e.pw};
        move_absolute(cur_pos);

        Serial.println("ok //M121 State pop complete.");
        return SHELL_RET_SUCCESS;
    }
    Serial.println("!! //M121 State stack is empty.");
    return SHELL_RET_FAILURE;
}

int handleSetHome(int argc, char **argv) // M306
{
    if (argc > 1)
    {
        //parse parameters
        for (int i = 0; i < argc; i++)
        {
            if (argv[i][0] == 'J') //we only handle the first J parameter we come across.
            {
                int joint_num = atoi(&argv[1][1]);
                jointConfig[joint_num].minPosition = motors[joint_num]->getPosition(); // min = home.
                Serial.printf("ok //M306: New home set for joint #%d.\n", joint_num);
                return SHELL_RET_SUCCESS;
            }
        }
    }

    //no J parameters.
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        jointConfig[i].minPosition = motors[i]->getPosition(); // min = home.
    }
    Serial.println("ok //M306: New home set.");
    return SHELL_RET_SUCCESS;
}

/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input buf the buffer to search.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char *buf, char code, float val)
{
    char *ptr = buf; // start at the beginning of buffer
    while ((*ptr))   //until we hit the end of the string (NULL char)
    {                // walk to the end
        if (*ptr == code)
        {                         // if you find code on your walk,
            return atof(ptr + 1); // convert the digits that follow into a float and return it
        }
        ptr = strchr(ptr, ' ') + 1; // take a step from here to the letter after the next space
    }
    return val; // end reached, nothing found, return default val.
}

/**
 * print the current position, feedrate, and absolute mode.
 */
void where()
{
    Serial.printf("X%6.3f", px);
    Serial.printf("Y%6.3f", py);
    Serial.printf("Z%6.3f", pz);
    Serial.printf("U%6.3f", pu);
    Serial.printf("V%6.3f", pv);
    Serial.printf("W%6.3f", pw);
    Serial.printf("F%6.3f", feed_rate);
    Serial.println(movement_mode == ABSOLUTE ? "ABS" : "REL");
}

/**
 * display helpful information
 */
void help()
{
    Serial.print(F("Flexo v"));
    Serial.println(FIRMWARE_VERSION);
    Serial.println(F("Commands:"));
    Serial.println(F("\tG00/G01 [X/Y/Z/U/V/W(steps)] [F(feedrate)]; - linear move"));
    Serial.println(F("\tG02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
    Serial.println(F("\tG03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
    Serial.println(F("\tG04 P[seconds]; - delay"));
    Serial.println(F("\tG90; - absolute mode"));
    Serial.println(F("\tG91; - relative mode"));
    Serial.println(F("\tG92 [X/Y/Z/U/V/W(steps)]; - change logical position"));
    Serial.println(F("\tM18; - disable motors"));
    Serial.println(F("\tM100; - this help message"));
    Serial.println(F("\tM114; - report position and feedrate"));
    Serial.println(F("\tAll commands must end with a newline."));
}
