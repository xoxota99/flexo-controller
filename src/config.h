#include <Arduino.h>
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define MOTOR_COUNT 1

#if defined(__MK20DX256__)

#define TEENSY_3_2
#define PIN_DIR_1 2
#define PIN_STEP_1 3
#define PIN_CS_1 17

#define PIN_DIR_2 7
#define PIN_STEP_2 4
#define PIN_CS_2 18

#define PIN_DIR_3 8
#define PIN_STEP_3 5
#define PIN_CS_3 19

#define PIN_DIR_4 14
#define PIN_STEP_4 6
#define PIN_CS_4 20

#define PIN_DIR_5 15
#define PIN_STEP_5 9
#define PIN_CS_5 21

#define PIN_DIR_6 16
#define PIN_STEP_6 10
#define PIN_CS_6 22

#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_SCK 13
#define PIN_ENABLE 23

// Teensy 3.5 / 3.6
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)

#define TEENSY_3_5
#define PIN_DIR_1 2
#define PIN_STEP_1 3
#define PIN_CS_1 17

#define PIN_DIR_2 7
#define PIN_STEP_2 4
#define PIN_CS_2 18

#define PIN_DIR_3 8
#define PIN_STEP_3 5
#define PIN_CS_3 19

#define PIN_DIR_4 14
#define PIN_STEP_4 6
#define PIN_CS_4 20

#define PIN_DIR_5 15
#define PIN_STEP_5 9
#define PIN_CS_5 21

#define PIN_DIR_6 16
#define PIN_STEP_6 10
#define PIN_CS_6 22

#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_SCK 13
#define PIN_ENABLE 23

#endif

typedef struct motorConfig_t
{
    unsigned int acceleration;
    bool inverseRotation;
    int maxSpeed;
    unsigned int pullInFreq;
    int stepPinPolarity;
    float gearRatio;
    byte dirPin;
    byte stepPin;
    byte csPin;
    unsigned int stepsPerRev;
    bool unsafeStartup;
} motorConfig_t;

const motorConfig_t kMotorConfig[] = {
    {.acceleration = 500000,
     .inverseRotation = true,
     .maxSpeed = 50000,
     .pullInFreq = 100,
     .stepPinPolarity = HIGH,
     .gearRatio = 1.0f,
     .dirPin = PIN_DIR_1,
     .stepPin = PIN_STEP_1,
     .csPin = PIN_CS_1,
     .stepsPerRev = 1600,
     .unsafeStartup = false}};

#endif