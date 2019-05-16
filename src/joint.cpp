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

const char *errorNames[] = {FOREACH_ERROR(GENERATE_STRING)};

double joint_position[6];
error_t joint_movement_error;

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

double px, py, pz, pu, pv, pw, feed_rate;

//TODO: There is way too much duplicate code in move_relative and move_absolute. Refactor these to reduce duplication.
bool move_relative(double *theta, float relspeed)
{
  relspeed = fmax(fmin(relspeed, 1.0f), 0.01f); //clamp speed
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      int steps[MOTOR_COUNT];

      //TODO: Define a work area concept, that holistically validates the target position of the end-effector is within the work space of the entire arm.

      //check all potential targets, ensure that they fall within the min/max values for each joint.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        int steps_per_deg = (int)((double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio / 360);
        Logger::trace("steps_per_deg[%d]=%d", i, steps_per_deg);
        steps[i] = steps_per_deg * theta[i];

        int absTarget = steps[i] + motors[i]->getPosition();

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
        else
        {
          motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * relspeed);
          motors[i]->setTargetRel(steps[i]);
        }
      }
      controller.move(motors); //blocking...
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
      }
      joint_movement_error = ERR_SUCCESS;
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
      return false;
    }
  }
  else
  {
    Logger::error("Movement aborted: Robot is already moving!");
    joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
    return false;
  }
}

bool move_relative(uint8_t shouldMove, double *theta, float relspeed)
{
  //shortcut.
  if (shouldMove == 0)
  {
    Logger::trace("Movement aborted. parameter 'shouldMove' is zero.");
  }
  relspeed = fmax(fmin(relspeed, 1.0f), 0.01f); //clamp speed
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      int steps[MOTOR_COUNT];

      //TODO: Define a work area concept, that holistically validates the target position of the end-effector is within the work space of the entire arm.

      //check all potential targets, ensure that they fall within the min/max values for each joint.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        if (shouldMove & 1 << i)
        {
          int steps_per_deg = (int)((double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio / 360);
          Logger::trace("steps_per_deg[%d]=%d", i, steps_per_deg);
          steps[i] = steps_per_deg * theta[i];

          int absTarget = steps[i] + motors[i]->getPosition();

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
          else
          {
            motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * relspeed);
            motors[i]->setTargetRel(steps[i]);
          }
        }
        else
        {
          //bit is not set for this motor, so ignore it.
          motors[i]->setTargetRel(0);
        }
      }
      controller.move(motors); //blocking...
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
      }
      joint_movement_error = ERR_SUCCESS;
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
      return false;
    }
  }
  else
  {
    Logger::error("Movement aborted: Robot is already moving!");
    joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
    return false;
  }
}

bool move_absolute(double *theta, float relspeed)
{
  relspeed = fmax(fmin(relspeed, 1.0f), 0.01f); //clamp speed
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      int steps[MOTOR_COUNT];

      //TODO: Define a work area concept, that holistically validates the target position of the end-effector is within the work space of the entire arm.

      //check all potential targets, ensure that they fall within the min/max values for each joint.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        int steps_per_deg = (int)((double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio / 360);
        Logger::trace("steps_per_deg[%d]=%d", i, steps_per_deg);
        steps[i] = steps_per_deg * theta[i];

        int absTarget = steps[i];

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
        else
        {
          motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * relspeed);
          motors[i]->setTargetAbs(steps[i]);
        }
      }
      controller.move(motors); //blocking...
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
      }
      joint_movement_error = ERR_SUCCESS;
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
      return false;
    }
  }
  else
  {
    Logger::error("Movement aborted: Robot is already moving!");
    joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
    return false;
  }
}

bool move_absolute(uint8_t shouldMove, double *theta, float relspeed)
{
  //shortcut.
  if (shouldMove == 0)
  {
    Logger::trace("Movement aborted. parameter 'shouldMove' is zero.");
  }
  relspeed = fmax(fmin(relspeed, 1.0f), 0.01f); //clamp speed
  if (runState == READY)
  {
    if (!controller.isRunning())
    {
      int steps[MOTOR_COUNT];

      //TODO: Define a work area concept, that holistically validates the target position of the end-effector is within the work space of the entire arm.

      //check all potential targets, ensure that they fall within the min/max values for each joint.
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        if (shouldMove & 1 << i)
        {
          int steps_per_deg = (int)((double)jointConfig[i].stepsPerRev / jointConfig[i].gearRatio / 360);
          Logger::trace("steps_per_deg[%d]=%d", i, steps_per_deg);
          steps[i] = steps_per_deg * theta[i];

          int absTarget = steps[i];

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
          else
          {
            motors[i]->setMaxSpeed(jointConfig[i].maxSpeed * relspeed);
            motors[i]->setTargetRel(steps[i]);
          }
        }
        else
        {
          //bit is not set for this motor, so ignore it.
          motors[i]->setTargetAbs(0);
        }
      }
      controller.move(motors); //blocking...
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motors[i]->setMaxSpeed(jointConfig[i].maxSpeed);
      }
      joint_movement_error = ERR_SUCCESS;
      return true;
    }
    else
    {
      Logger::error("Movement aborted: Robot is already moving!");
      joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
      return false;
    }
  }
  else
  {
    Logger::error("Movement aborted: Robot is already moving!");
    joint_movement_error = ERR_MOVEMENT_IN_PROGRESS;
    return false;
  }
}

bool move_path(int argc, double **theta, float relspeed)
{
  for (int i = 0; i < argc; i++)
  {
    double *waypoint = theta[i];
    for (int j = 0; j < MOTOR_COUNT; j++)
    {
      motors[j]->setTargetAbs(waypoint[j]);
    }
  }
  joint_movement_error = ERR_UNSUPPORTED_FUNCTION;
  return false;
}

void moveCompleteCallback()
{
  Logger::trace("move completed.");

  //TODO: Is this super dumb? Do I even know how to use pointers?
  double *ptrs[6] = {&px, &py, &pz, &pu, &pv, &pw};

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    *ptrs[i] = (double)motors[i]->getPosition() * jointConfig[i].gearRatio / jointConfig[i].stepsPerRev * 360;
  }
}

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
  controller.setCallback(moveCompleteCallback);
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
#else

bool unsafe_zero(bool j1, bool j2, bool j3, bool j4, bool j5, bool j6, float speed)
{
  return false;
}

#endif //LIMIT_SWITCHES_SUPPORTED

void loop_joints()
{
  //If a joint has hit its end stop, we will e-stop the machine and require a restart / recalibration.
  if (runState == READY && controller.isRunning())
  {
    //TODO: We could do this faster here by reading a PORT register or setting an ISR for limit pins,
    // since we only care if at least ONE pin is low.
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