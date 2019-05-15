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

frame_t ee_frame; //end effector frame.
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
     limitPin : PIN_LIMIT_1,
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
     limitPin : PIN_LIMIT_2,
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
     limitPin : PIN_LIMIT_3,
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
     limitPin : PIN_LIMIT_4,
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
     limitPin : PIN_LIMIT_5,
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
     limitPin : PIN_LIMIT_6,
     limitPinActive : LOW,
     stepsPerRev : 1600}};

bool move_linear(frame_t position, float speed)
{
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
        x = ee_frame.x + x_pos;
        y = ee_frame.y + y_pos;
        z = ee_frame.z + z_pos;
        a = ee_frame.yaw + yaw_theta;
        b = ee_frame.pitch + pitch_theta;
        c = ee_frame.roll + roll_theta;
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
        motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * speed);
        joint_position[i] = target[i]; //TODO: Non-atomic here. It's possible to fail mid-move, and have incorrect data in joint_position. Move this to joint_loop.
      }
      controller.move(motors); //blocking...

      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setMaxSpeed(jointConfig[i].maxSpeed); //set speed back to normal.
      }
      // controller.move(motors, speed);

      //TODO: Non-atomic here. It's possible to fail mid-move, and have incorrect data in ee_frame. Move this to joint_loop.
      ee_frame.x = x;
      ee_frame.y = y;
      ee_frame.z = z;
      ee_frame.yaw = a;
      ee_frame.pitch = b;
      ee_frame.roll = c;

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
        motors[idx]->setMaxSpeed(jointConfig[idx].maxSpeed * speed);
        controller.move(*(motors[idx])); //blocking.
        motors[idx]->setMaxSpeed(jointConfig[idx].maxSpeed);
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
  speed = fmax(fmin(speed, 1.0f), 0.0f); //clamp speed
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
          motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * speed);
        }
      }
      if (bMove)
      {
        controller.move(motors); //blocking.
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
          motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
        }
      }
      joint_movement_error = ERR_SUCCESS;
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
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
//       controller.move(motors, speed);    //blocking.
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
#ifdef LIMIT_SWITCHES_SUPPORTED
  safe_zero();
#endif
}

#ifdef LIMIT_SWITCHES_SUPPORTED
/**
 * Zero each motor, ONE AT A TIME, starting with j6, and ending with j1. Each 
 * motor will move to its end-stop, then jog forward until the stop disengages, 
 * and its position will be set to the minimum for the joint.
 * 
 * This method will block until all motors are zeroed.
 **/
bool safe_zero(bool j1, bool j2, bool j3, bool j4, bool j5, bool j6, float speed)
{
  Logger::trace("Initializing joints to end stops");
  run_state_t oldRunState = runState;

  runState = CALIBRATING;
  controller.emergencyStop();            //just in case.
  speed = fmax(fmin(1.0f, speed), 0.0f); //clamp speed.

  for (int i = MOTOR_COUNT; i > 0; i--)
  {
    //for each motor
    //are we homing this motor?
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

      controller.moveAsync(motors); //remember, we're homing motors one at a time. So this only ever moves one motor. We move Asynchronously so we can cancel movement based on some external stimuli (the limit switch).
      while (true)
      {
        delay(1);
        uint8_t state = digitalReadFast(jointConfig[i].limitPin);

        //check if this joint has hit its limit switch.
        if (state == jointConfig[i].limitPinActive)
        {
          uint32_t startMillis = millis();
          uint32_t t0 = startMillis;
          while (millis() - startMillis < 5 && millis() - t0 < 300) //debounce 5 ms. (too short? too long?). Timeout at 300ms.
          {
            if (digitalReadFast(jointConfig[i].limitPin) != state)
            {
              Logger::trace("debounce flap.");
              state = digitalReadFast(jointConfig[i].limitPin);
              startMillis = millis(); //start over.
            }
            delay(1);
          }
          if (millis() - t0 >= 300)
          {
            Logger::error("Timeout debouncing Motor limit switch %d. Emergency stop.", i);
            controller.emergencyStop();
            runState = SHUTDOWN;
            joint_movement_error = ERR_IO_TIMEOUT;
            return false;
          }
          else if (state == jointConfig[i].limitPinActive)
          {
            //stop.
            controller.emergencyStop();
            Logger::trace("Motor %d hit limit. Moving forward.", i);

            motors[i]->setTargetRel(jointConfig[i].stepsPerRev); //one revolution.
            while (true)
            {
              //advance until the limit switch is no longer pressed for this motor.
              delay(1);
              uint8_t state = digitalReadFast(jointConfig[i].limitPin);

              //check if this joint has released its limit switch.
              if (state != jointConfig[i].limitPinActive)
              {
                uint32_t startMillis = millis();
                uint32_t t0 = startMillis;
                while (millis() - startMillis < 5 && millis() - t0 < 300) //debounce 5 ms. (too short? too long?). Timeout at 300ms.
                {
                  if (digitalReadFast(jointConfig[i].limitPin) != state)
                  {
                    Logger::trace("debounce flap.");
                    state = digitalReadFast(jointConfig[i].limitPin);
                    startMillis = millis(); //start over.
                  }
                  delay(1);
                }
                if (millis() - t0 >= 300)
                {
                  Logger::error("Timeout debouncing Motor limit switch %d. Emergency stop.", i);
                  controller.emergencyStop();
                  runState = SHUTDOWN;
                  joint_movement_error = ERR_IO_TIMEOUT;
                  return false;
                }
                else
                {
                  if (state != jointConfig[i].limitPinActive)
                  {
                    //stop.
                    Logger::trace("Motor %d advanced. Limit switch released.", i);
                    controller.emergencyStop();

                    //set motor position.
                    motors[i]->setPosition(jointConfig[i].minPosition);
                    //finally, set the motor's max speed back to its intended value.
                    motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
                    break;
                  }
                  else
                  {
                    Logger::trace("Motor %d false alarm (no limit hit). Continuing.");
                  }
                }
              }
            } //while true
          }   // no timeout, but we somehow exited the debounce loop without activating the limit switch, so just keep going...
        }
      } //while true
    }
  } //foreach motor [6..1]
  runState = oldRunState;
  joint_movement_error = ERR_SUCCESS;
  return true;
}

#endif //LIMIT_SWITCHES_SUPPORTED

void loop_joints()
{
  //If a joint has hit its end stop, we will e-stop the machine and require a restart / recalibration.
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