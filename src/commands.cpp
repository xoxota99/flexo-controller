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

#include "commands.h"

int cCount = 0;

void setup_shell(int bps /* = SERIAL_BAUD */)
{
  if (!Serial)
  {
    Serial.begin(bps);
    delay(50);
  }

  Serial.printf("Flexo, v%s\n", FLEXO_VERSION);
  Serial.printf("Built %s, %s\n", __DATE__, __TIME__);
  Serial.println("=======================\n");

  Logger::debug("Command-line is ENABLED.");

  shell_init(shell_reader, shell_writer, 0);

  const int c1 = sizeof(commands);
  if (c1 > 0)
  {
    const int c2 = sizeof(commands[0]);
    const int ccount = c1 / c2;

    for (int i = 0; i < ccount; i++)
    {
      Logger::info("Registering command: %s", commands[i].shell_command_string);
      if (shell_register(commands[i].shell_program, commands[i].shell_command_string))
      {
        cCount++;
      }
    }
  }
  delay(100);
}

void loop_shell()
{
  // This should always be called to process user input
  if (shellMode == INTERACTIVE)
  {
    shell_task();
  }
}

/**
   Function to read data from serial port
   Functions to read from physical media should use this prototype:
   int my_reader_function(char * data)
 */
int shell_reader(char *data)
{
  // Wrapper for Serial.read() method
  if (Serial.available())
  {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

/**
   Function to write data to serial port
   Functions to write to physical media should use this prototype
   void my_writer_function(char data)
 */
void shell_writer(char data)
{
  Serial.write(data);
}

int handleHelp(int argc, char **argv)
{
  int size = *(&commands + 1) - commands;
  Logger::info("%d commands suported\n", cCount);

  for (int i = 0; i < size; i++)
  {
    Serial.printf("%s\t\t%s\n", commands[i].shell_command_string, commands[i].shell_help_string);
  }
  return SHELL_RET_SUCCESS;
}

int handleReset(int argc, char **argv)
{

  CPU_RESTART;
  return SHELL_RET_SUCCESS; //unreachable, but the compiler complains if I leave it out.
}

int handleLog(int argc, char **argv)
{
  if (argc != 2)
  {
    Serial.println(USAGE_LOG);
    return SHELL_RET_FAILURE;
  }

  String token(argv[1]);
  token = token.toUpperCase();

  if (token == "TRACE")
  {
    Logger::level = Logger::TRACE;
  }
  else if (token == "DEBUG")
  {
    Logger::level = Logger::DEBUG;
  }
  else if (token == "INFO")
  {
    Logger::level = Logger::INFO;
  }
  else if (token == "WARN")
  {
    Logger::level = Logger::WARN;
  }
  else if (token == "ERROR")
  {
    Logger::level = Logger::ERROR;
  }
  else if (token == "FATAL")
  {
    Logger::level = Logger::FATAL;
  }
  else
  {
    Serial.println(USAGE_LOG);
    return SHELL_RET_FAILURE;
  }

  return SHELL_RET_SUCCESS;
}

/**
 * Move the entire robot to the specified set of absolute angles (in radians). 
 * Angles are provided relative to the joint's home position.
 **/
int handleMove(int argc, char **argv)
{
  if (argc != MOTOR_COUNT + 1)
  {
    Serial.println(USAGE_MOVE);
  }
  else
  {
    if (systemState == RUNNING)
    {
      switch (moveState)
      {
      case MOVING:
        Serial.println("Movement aborted. Robot is already moving.");
        return SHELL_RET_FAILURE;
        break;
      case STOPPED:
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
          String token(argv[i + 1]);
          float degrees = token.toFloat();
          float steps = (double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio * degrees / 360;
          //convert absolute radians to relative steps
          motors[i]->setTargetAbs(steps);
        }
        moveState = MOVING;
        controller.moveAsync(motors);
        break;
      }
    }
    else
    {
      Serial.printf("Movement aborted. System is in state: %s\n", systemStateNames[systemState]);
      return SHELL_RET_FAILURE;
    }
  }
  return SHELL_RET_SUCCESS;
}

/**
 * Move the given joint, by a given number of steps. 
 * e.g.: move 3 -20 -o
 **/
int handleJog(int argc, char **argv)
{
  if (argc < 3)
  {
    Serial.println(USAGE_JOG);
  }
  else

      if (systemState == RUNNING)
  {

    {
      switch (moveState)
      {
      case MOVING:
        Serial.println("Movement aborted. Controller is already running.");
        return SHELL_RET_FAILURE;
        break;
      case STOPPED:
        String token(argv[1]);
        long jointNum = token.toInt();
        if (jointNum <= MOTOR_COUNT - 1)
        {
          token = argv[2];
          long degrees = token.toInt();
          long steps = degrees / 360.0 * jointConfig[jointNum].stepsPerRev / jointConfig[jointNum].gearRatio;
          // Logger::info("%d degrees = %d steps\n", degrees, steps);
          bool continuous = (jointConfig[jointNum].minPosition == jointConfig[jointNum].maxPosition);
          bool ignoreLimits = false;

          if (argc > 3 && !continuous)
          {
            String o(argv[3]);

            o = o.toUpperCase();

            if (o == "-O")
            {
              ignoreLimits = true;
            }
          }

          if (!ignoreLimits && !continuous)
          {
            //determine if this movement would put us outside the motor's limits.
            int32_t pos = motors[jointNum]->getPosition();
            Serial.printf("pos=%d, steps=%d\n", pos, steps);
            long targetPos = pos + steps;
            if (jointConfig[jointNum].minPosition != jointConfig[jointNum].maxPosition)
            { //check extents
              if (targetPos < jointConfig[jointNum].minPosition)
              {
                Serial.printf("ERROR: The specified movement (%d steps) would cause joint #%d to exceed it's minimum position limit [%d]. Use the -o switch to override.\n", targetPos, jointNum, jointConfig[jointNum].minPosition);
                return SHELL_RET_FAILURE;
              }
              else if (targetPos > jointConfig[jointNum].maxPosition)
              {
                Serial.printf("ERROR: The specified movement (%d steps) would cause joint #%d to exceed it's maximum position limit [%d]. Use the -o switch to override.\n", targetPos, jointNum, jointConfig[jointNum].maxPosition);
                return SHELL_RET_FAILURE;
              }
            }
          }
          motors[jointNum]->setTargetRel(steps);
          moveState = MOVING;
          controller.moveAsync(motors);
        }
        else
        {
          Serial.printf("ERROR: Invalid joint #%d. Valid joints are 0 thru %d\n", jointNum, MOTOR_COUNT - 1);
          return SHELL_RET_FAILURE;
        }
        break;
      }
    }
    return SHELL_RET_SUCCESS;
  }
  else
  {
    Serial.printf("Movement aborted. System is in state: %s\n", systemStateNames[systemState]);
    return SHELL_RET_FAILURE;
  }
}

void dumpJoint(int i)
{
  double pos = motors[i]->getPosition();
  Serial.printf("=================================\n");
  Serial.printf("Joint %d:\n", i);
  Serial.printf("\tCurrent Position: %d\n\n", pos);
  Serial.printf("\tmin: %d\n", jointConfig[i].minPosition);
  Serial.printf("\tmax: %d\n", jointConfig[i].maxPosition);
  Serial.printf("\thome: %d\n", jointConfig[i].homePosition);
  Serial.printf("\tacceleration: %d\n", jointConfig[i].acceleration);
  Serial.printf("\tinverseRotation: %s\n", jointConfig[i].inverseRotation ? "true" : "false");
  Serial.printf("\tmaxSpeed: %d\n", jointConfig[i].maxSpeed);
  Serial.printf("\tpullInFreq: %d\n", jointConfig[i].pullInFreq);
  Serial.printf("\tstepPinPolarity: %d\n", jointConfig[i].stepPinPolarity);
  Serial.printf("\tgearRatio: %.2f\n", jointConfig[i].gearRatio);
  Serial.printf("\tdirPin: %d\n", jointConfig[i].dirPin);
  Serial.printf("\tstepPin: %d\n", jointConfig[i].stepPin);
  Serial.printf("\tcsPin: %d\n", jointConfig[i].csPin);
  Serial.printf("\tstepsPerRev: %d\n", jointConfig[i].stepsPerRev);
  Serial.printf("\tunsafeStartup: %s\n", jointConfig[i].unsafeStartup ? "true" : "false");
}

/**
 * Dump known status of each joint / motor.
 **/
int handleStatus(int argc, char **argv)
{
  String utime = uptime();
  unsigned int l = utime.length();
  char buf[l + 1];
  utime.toCharArray(buf, l);

  Serial.printf("Flexo version:  %s\n", FLEXO_VERSION);
  Serial.printf("Built:          %s, %s\n", __DATE__, __TIME__);
  Serial.printf("Uptime:         %s\n", buf);
  Serial.printf("Shell Mode:     %s\n", shellModeNames[shellMode]);
  if (argc > 1)
  {
    String token = argv[1];
    long idx = token.toInt();
    dumpJoint((int)idx);
  }
  else
  {
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      dumpJoint(i);
    }
  }
  return SHELL_RET_SUCCESS;
}

/**
 * Take a given (zero-based) joint's current position, and use it as the minimum | maximum | zero point for that joint.
 * format:
 *        - set [min|max|zero] [0-5] (e.g. "set min 3")
 **/
int handleSet(int argc, char **argv)
{
  if (argc <= 1)
  {
    Serial.println(USAGE_SET);
  }
  else
  {
    String extent(argv[1]);
    extent = extent.toUpperCase();

    if (argc > 2)
    {
      String token(argv[2]);
      long jointNum = token.toInt();
      if (extent.equals("MIN"))
      {
        jointConfig[jointNum].minPosition = motors[jointNum]->getPosition();
      }
      else if (extent.equals("MAX"))
      {
        jointConfig[jointNum].maxPosition = motors[jointNum]->getPosition();
      }
      else if (extent.equals("HOME"))
      {
        motors[jointNum]->setPosition(0);
      }
      else
      {
        Serial.println(USAGE_SET);
      }
    }
    else
    {
      // set extent for all joints
      if (extent.equals("MIN"))
      {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
          jointConfig[i].minPosition = motors[i]->getPosition();
        }
      }
      else if (extent.equals("MAX"))
      {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
          jointConfig[i].maxPosition = motors[i]->getPosition();
        }
      }
      else if (extent.equals("ZERO"))
      {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
          motors[i]->setPosition(0);
        }
      }
      else
      {
        Serial.println(USAGE_SET);
      }
    }
  }
  return 0;
}
