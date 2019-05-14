/*
  joint.cpp - Methods for commanding joints.
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

#include "joint.h"

//TODO: Determine correct "home" values.
//TODO: Emergency stop when joints hit limit switches.
const frame_t home_frame = {
  x : 0,
  y : 0,
  z : 0,
  yaw : 0,
  pitch : 0,
  roll : 0
};

const char *errorNames[] = {FOREACH_ERROR(GENERATE_STRING)};

frame_t current_frame;
double joint_position[6];
error_t joint_movement_error;
movementMode_t movement_mode = ABSOLUTE;

/**
 * Stepper Motors that make up the robot.
 **/
Stepper *motors[MOTOR_COUNT] = {
    new Stepper(PIN_STEP_1, PIN_DIR_1),
    new Stepper(PIN_STEP_2, PIN_DIR_2),
    new Stepper(PIN_STEP_3, PIN_DIR_3),
    new Stepper(PIN_STEP_4, PIN_DIR_4),
    new Stepper(PIN_STEP_5, PIN_DIR_5),
    new Stepper(PIN_STEP_6, PIN_DIR_6)};

/**
 * The Stepper motor controller.
 **/
StepControl controller;

/**
 * Configuration of joints in the robot.
 **/
jointConfig_t jointConfig[] = {
    {minPosition : -3600,
     maxPosition : 3600,
     //  homePosition : 0,
     acceleration : 7500,
     inverseRotation : false,
     maxSpeed : 5000, //Trying for 1 rotation per second (accounting for gearRatio)
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.2250f, //19:80
     dirPin : PIN_DIR_1,
     stepPin : PIN_STEP_1,
     limitPin : PIN_CS_1,
     limitPinActive : LOW,
     stepsPerRev : 1600},
    {minPosition : 0,
     maxPosition : 0,
     //  homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.02127659574f, //1:47
     dirPin : PIN_DIR_2,
     stepPin : PIN_STEP_2,
     limitPin : PIN_CS_2,
     limitPinActive : LOW,
     stepsPerRev : 1600},
    {minPosition : -50,
     maxPosition : 50,
     //  homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.02127659574f, //1:47
     dirPin : PIN_DIR_3,
     stepPin : PIN_STEP_3,
     limitPin : PIN_CS_3,
     limitPinActive : LOW,
     stepsPerRev : 1600},
    {minPosition : -50,
     maxPosition : 50,
     //  homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.266666666667f, //16:60
     dirPin : PIN_DIR_4,
     stepPin : PIN_STEP_4,
     limitPin : PIN_CS_4,
     limitPinActive : LOW,
     stepsPerRev : 1600},
    {minPosition : -50,
     maxPosition : 50,
     //  homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 0.6f, //30:50
     dirPin : PIN_DIR_5,
     stepPin : PIN_STEP_5,
     limitPin : PIN_CS_5,
     limitPinActive : LOW,
     stepsPerRev : 1600},
    {minPosition : -50,
     maxPosition : 50,
     //  homePosition : 0,
     acceleration : 50000,
     inverseRotation : true,
     maxSpeed : 15000,
     pullInFreq : 100,
     stepPinPolarity : HIGH,
     gearRatio : 1.0f,
     dirPin : PIN_DIR_6,
     stepPin : PIN_STEP_6,
     limitPin : PIN_CS_6,
     limitPinActive : LOW,
     stepsPerRev : 1600}};

bool move_linear(frame_t position, float speed)
{
  speed = min(max(0.01, speed), 1.0);
  return move_linear(position.x, position.y, position.z, position.yaw, position.pitch, position.roll, speed);
}

bool move_linear(double x_pos, double y_pos, double z_pos, double yaw_theta, double pitch_theta, double roll_theta, float speed)
{
  speed = min(max(0.01, speed), 1.0);
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      joint_movement_error = ERR_SUCCESS; //clear the error field.
      double solution[MOTOR_COUNT];
      double x, y, z, a, b, c;
      switch (movement_mode)
      {
      case RELATIVE:
        x = current_frame.x + x_pos;
        y = current_frame.y + y_pos;
        z = current_frame.z + z_pos;
        a = current_frame.yaw + yaw_theta;
        b = current_frame.pitch + pitch_theta;
        c = current_frame.roll + roll_theta;
        break;
      case ABSOLUTE:
      default:
        x = x_pos;
        y = y_pos;
        z = z_pos;
        a = yaw_theta;
        b = pitch_theta;
        c = roll_theta;
      }
      inverse(solution, z, y, z, a, b, c);
      int32_t target[MOTOR_COUNT];

      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        //TODO: Need to map motor position (in steps) to angles (in DH params)
        target[i] = jointConfig[i].minPosition + ((solution[i] / 360 * jointConfig[i].stepsPerRev) / jointConfig[i].gearRatio);
        if (jointConfig[i].minPosition > target[i] ||
            jointConfig[i].maxPosition < target[i])
        {
          joint_movement_error = ERR_RESULT_OUTSIDE_WORKSPACE;
          return false;
        }
      }

      //still good.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setTargetAbs(target[i]);
        joint_position[i] = target[i]; //TODO: Non-atomic here. It's possible to fail mid-move, and have incorrect data in joint_position. Move this to joint_loop.
      }
      controller.moveAsync(motors);
      // controller.moveAsync(motors, speed);

      //TODO: Non-atomic here. It's possible to fail mid-move, and have incorrect data in current_frame. Move this to joint_loop.
      current_frame.x = x;
      current_frame.y = y;
      current_frame.z = z;
      current_frame.yaw = a;
      current_frame.pitch = b;
      current_frame.roll = c;

      return true;
    }
    else
    {
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
    }
  }
  else
  {
    joint_movement_error = ERR_BAD_RUN_STATE;
  }
  return false;
}

/**
* Jog a single joint to the provided angle, at the specified speed. The angle can be relative or absolute, depending on the current movement mode.
*
* Return: True if the movement was successful. False if the movement was not successful.
**/
bool move_joints(int idx, double theta, float speed)
{
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      int steps_per_rad = (int)((double)jointConfig[idx].stepsPerRev / jointConfig[idx].gearRatio / TWO_PI);
      int steps = steps_per_rad * theta;

      Logger::trace("Steps_per_rad=%d", steps_per_rad);

      int absTarget = steps;
      if (movement_mode == RELATIVE)
      {
        absTarget += motors[idx]->getPosition();
      }

      if (absTarget > jointConfig[idx].maxPosition)
      {
        Logger::error("Movement aborted. Joint J%d target position is > maxPosition (%d).", idx + 1, jointConfig[idx].maxPosition);

        joint_movement_error = ERR_RESULT_OUTSIDE_WORKSPACE;
        return false;
      }
      else if (absTarget < jointConfig[idx].minPosition)
      {
        Logger::error("Movement aborted. Joint J%d target position is < min>Position (%d).", idx + 1, jointConfig[idx].minPosition);

        joint_movement_error = ERR_RESULT_OUTSIDE_WORKSPACE;
        return false;
      }
      else
      {
        switch (movement_mode)
        {
        case ABSOLUTE:
          motors[idx]->setTargetAbs(steps);
          break;
        case RELATIVE:
        default:
          motors[idx]->setTargetRel(steps);
        }

        controller.moveAsync(*(motors[idx]));
        return true;
      }
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
    }
  }
  else
  { //systemState!=READY
  }
  return false;
}

/**
 * Move joints, either relatively or absolutely.
 * Parameters:
 * shouldMove: An array of booleans, indicating whether a particular joint should be moved at all.
 * theta: An array of doubles, indicating the angle (in degrees) to apply to the joint.
 *  If movementMode is RELATIVE, the joint will be moved relative to its current position.
 *  If movementMode is ABSOLUTE, the joint will be moved to the absolute position indicated by theta.
 * 
 **/
bool move_joints(bool *shouldMove, double *theta, float speed)
{
  bool bMove = false;
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      int steps[MOTOR_COUNT];

      //TODO: Define a work area concept, that holistically validates the target position of the end-effector is within the work space of the entire arm.

      //check all potential targets, ensure that they fall within the min/max values for each joint.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        if (shouldMove[i])
        {
          bMove = true;
          int steps_per_deg = (int)((double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio / 360);
          Logger::trace("steps_per_deg[%d]=%d", i, steps_per_deg);
          steps[i] = steps_per_deg * theta[i];
          int absTarget = steps[i];
          if (movement_mode == RELATIVE)
          {
            absTarget += motors[i]->getPosition();
          }

          if (absTarget > jointConfig[i].maxPosition)
          {
            Logger::error("Movement aborted. Joint J%d target position (%d) is > maxPosition (%d).", i + 1, absTarget, jointConfig[i].maxPosition);

            joint_movement_error = ERR_RESULT_OUTSIDE_WORKSPACE;
            return false;
          }
          else if (absTarget < jointConfig[i].minPosition)
          {
            Logger::error("Movement aborted. Joint J%d target position (%d) is < minPosition (%d).", i + 1, absTarget, jointConfig[i].minPosition);

            joint_movement_error = ERR_RESULT_OUTSIDE_WORKSPACE;
            return false;
          }
        }
      }

      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        if (shouldMove[i])
        {
          switch (movement_mode)
          {
          case ABSOLUTE:
            motors[i]->setTargetAbs(steps[i]);
            break;
          case RELATIVE:
          default:
            motors[i]->setTargetRel(steps[i]);
          }
        }
      }
      if (bMove)
      {
        controller.moveAsync(motors);
      }
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
    }
  }
  else
  { //systemState!=READY
  }
  return false;
}

// bool move_home(float speed)
// {
//   speed = min(max(0.01, speed), 1.0);
//   if (runState == READY)
//   {
//     if (!controller.isRunning())
//     {
//       for (int i = 0; i < MOTOR_COUNT; i++)
//       {
//         motors[i]->setTargetAbs(jointConfig[i].homePosition);
//       }
//       controller.moveAsync(motors, speed);
//       return true;
//     }
//     else
//     {
//       Logger::error("Movement aborted: Robot is already moving!");
//     }
//   }
//   else
//   {
//     Logger::error("Movement aborted: Robot is not ready.");
//   }
//   return false;
// }

void setup_joints()
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i]->setAcceleration(jointConfig[i].acceleration);
    motors[i]->setInverseRotation(jointConfig[i].inverseRotation);
    motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
    // motors[i]->setPullInSpeed(jointConfig[i].pullInFreq);
    motors[i]->setStepPinPolarity(jointConfig[i].stepPinPolarity);
    motors[i]->setPosition(0);

    pinMode(jointConfig[i].limitPin, (jointConfig[i].limitPinActive ? INPUT_PULLDOWN : INPUT_PULLUP));
  }
  safe_zero();
}

void safe_zero(bool j1, bool j2, bool j3, bool j4, bool j5, bool j6, float speed)
{
  Logger::trace("Initializing joints to end stops");
  run_state_t oldRunState = runState;
  runState = CALIBRATING;
  controller.emergencyStop();            //just in case.
  speed = fmax(fmin(1.0f, speed), 0.0f); //clamp speed.

  //home from j6 down to j1.
  for (int i = MOTOR_COUNT; i > 0; i--)
  {
    if ((i == 0 && j1) ||
        (i == 1 && j2) ||
        (i == 2 && j3) ||
        (i == 3 && j4) ||
        (i == 4 && j5) ||
        (i == 5 && j6))
    {
      motors[i]->setTargetRel(INT32_MIN); // go forever.
      //TODO: What direction is each motor moving in? Is it deterministic? Same every time? Is there some internal state?
      motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * speed);
    }

    controller.moveAsync(motors);
    // controller.moveAsync(motors, speed); // 1/10th speed.
    while (true)
    {
      delay(1);
      //check if this joint has hit its limit switch.
      if (digitalReadFast(jointConfig[i].limitPin) == jointConfig[i].limitPinActive)
      {
        uint32_t startMillis = millis();
        while (millis() - startMillis < 20) //debounce 20 ms. (too long?)
        {
          if (digitalReadFast(jointConfig[i].limitPin) != jointConfig[i].limitPinActive)
          {
            startMillis = millis(); //start over. (Risk of infinite loop).
            // TODO: Deal with false positives (timeout?)
          }
          delay(1);
        }
        Logger::trace("Motor %d hit limit.", i);
        controller.emergencyStop();
        // motors[i]->setTargetAbs(motors[i]->getPosition()); //so the motor doesn't startup again when we restart the controller. Test this.
      }
      break; //while(true)
    }
  }

  //all motors should be at limits now.
  Logger::trace("All motors at limits.");

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if ((i == 0 && j1) ||
        (i == 1 && j2) ||
        (i == 2 && j3) ||
        (i == 3 && j4) ||
        (i == 4 && j5) ||
        (i == 5 && j6))
    {
      motors[i]->setTargetRel(jointConfig[i].stepsPerRev); // one revolution forward.
      //TODO: What direction is each motor moving in? Is it deterministic? Same every time? Is there some internal state?

      controller.moveAsync(motors);
      // controller.moveAsync(motors, HOMING_SPEED_REL); // 1/10th speed.
      while (true)
      {
        delay(1);
        if (digitalReadFast(jointConfig[i].limitPin) != jointConfig[i].limitPinActive)
        {
          uint32_t startMillis = millis();
          while (millis() - startMillis < 20) //debounce 20 ms. (too long?)
          {
            if (digitalReadFast(jointConfig[i].limitPin) == jointConfig[i].limitPinActive)
            {
              startMillis = millis(); //start over. (Risk of infinite loop).
              // TODO: Deal with false positives (timeout?)
            }
            delay(1);
          }
          controller.emergencyStop();
          motors[i]->setPosition(jointConfig[i].minPosition); //We're officially at minimum position now.
          Logger::trace("Motor %d hit minimum position %d.", i, jointConfig[i].minPosition);
        }
        break; //while(true)
      }
    }
  }
  Logger::trace("Homing complete.");
  runState = oldRunState;
}

void loop_joints()
{
  //If a joint has hit it's end stop, we will e-stop the machine and require a restart / recalibration.
  if (runState != CALIBRATING)
  {
    //TODO: We could do this faster here by reading a PORT register or something, since we only care if at least ONE pin is low.
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      if (digitalReadFast(jointConfig[i].limitPin) == jointConfig[i].limitPinActive)
      {
        //limit switch has been hit. panic and stop.
        controller.emergencyStop();
        Logger::fatal("At least one joint has hit its limits (J%d). This should never happen outside calibration. The robot is now HALTED, and must be reset / recalibrated.", i);
        runState = HALTED;
        return;
      }
    }
  }
}