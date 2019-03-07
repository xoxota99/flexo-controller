#include <Arduino.h>
#include <AS5048A.h>
#include "config.h"
#include <StepControl.h>

#define ENC_SCALE 0x3fff
#define STEPSILON 10

// this is the magic trick for printf to support float
asm(".global _printf_float");
// this is the magic trick for scanf to support float
asm(".global _scanf_float");

AS5048A *angleSensor[MOTOR_COUNT] = {
    new AS5048A(PIN_CS_1)};

Stepper *motors[MOTOR_COUNT] = {
    new Stepper(PIN_STEP_1, PIN_DIR_1)};

StepControl<> controller;

int pos[MOTOR_COUNT];    // position as reported by the encoder (0 to 2^14-1)
int target[MOTOR_COUNT]; // target (0 to 2^14-1)
bool bArrived[MOTOR_COUNT];

void setup_motors()
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i]->setAcceleration(kMotorConfig[i].acceleration);
    motors[i]->setInverseRotation(kMotorConfig[i].inverseRotation);
    motors[i]->setMaxSpeed(kMotorConfig[i].maxSpeed);
    motors[i]->setPullInSpeed(kMotorConfig[i].pullInFreq);
    motors[i]->setStepPinPolarity(kMotorConfig[i].stepPinPolarity);

    target[i] = ENC_SCALE / 2; //keep it simple for now.

    motors[i]->setPosition(target[i]);
    motors[i]->setTargetAbs(target[i]);
  }
}

void loop_motors()
{
  // TODO: Try using controller.rotateAsync() and controller.emergencyStop()
  bool bGo = false;
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    int mPos = motors[i]->getPosition();
    int mPosDelta = mPos - (kMotorConfig[i].stepsPerRev * (double)target[i] / ENC_SCALE);

    if (abs(mPosDelta) < STEPSILON)
    {
      // we're where we're supposed to be.
      bArrived[i] = true;
      // motors[i]->setPosition(target[i]);
      // Serial.println("Arrived.");
    }

    int delta = pos[i] - target[i];          // Delta (in encoder units)
    float fDelta = (float)delta / ENC_SCALE; // Delta (in revolutions)

    int stepDelta = kMotorConfig[i].stepsPerRev * fDelta; // Delta (in steps).

    // if(fDelta>1){
    //   stepDelta *= -1;
    // }

    int halfRev = (int)(kMotorConfig[i].stepsPerRev / 2.0);

    // Serial.printf("stepDelta=%d\n", stepDelta);

    if (abs(stepDelta) > STEPSILON)
    {
      if (stepDelta > halfRev)
      // {
      //   stepDelta = -stepDelta;
      // }
      // else
      {
        stepDelta = kMotorConfig[i].stepsPerRev - stepDelta; //
      }
      else
      {
        stepDelta = -stepDelta;
      }

      // Serial.printf("delta=%d, fDelta=%.3f, stepDelta=%d\n", delta, fDelta, stepDelta);
      // while (!Serial.available())
      //   ;

      // while (Serial.available())
      // {
      //   Serial.read();
      // }

      motors[i]->setPosition(target[i] - stepDelta);
      motors[i]->setTargetAbs(target[i]);
      bArrived[i] = false;
      bGo = true;
    }
    // target[i] = stepDelta;
    // motors[i]->setTargetAbs(target[i]);
  }
  if (bGo)
  {
    controller.move(motors, 1.0f);
  }
}

void setup_encoders()
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    angleSensor[i]->init();
  }
}

void loop_encoders()
{
  //read position from encoders, set postion in motor controller.
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    pos[i] = ENC_SCALE - angleSensor[i]->getRawRotation(); // 0 to 2 ^ 14-1
    // Serial.printf("Theta: %d\n", pos[i]);
  }
}

void setup()
{

  while (!Serial && millis() < 1000)
    ;
  // pinMode(LED_BUILTIN, OUTPUT);

  setup_encoders();
  setup_motors();
}

void loop()
{
  // delay(10);

  loop_encoders();
  loop_motors();
  // Serial.printf("%d\t%d\n", motors[0]->getPosition(), angleSensor[0]->getRawRotation());
}
