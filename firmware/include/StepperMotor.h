#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <AccelStepper.h>

class StepperMotor {

public:
  StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, int stepsPerRev, int microstepping);

  void enable();
  void disable();
  void run();
  void stop();

  void setUpEnablePin();
  void setSpeedRad(float radPerSec);
  void setAccelerationRad(float radPerSec2);

private:
  AccelStepper stepper;

  // Motor Pins
  const uint8_t stepPin;
  const uint8_t dirPin;
  const uint8_t enablePin;

  // Motor Parameters
  const int stepsPerRev;         
  const int microstepping;
  float accelerationRad;

  // Conversion constant
  float radToMicrosteps;
  
  float targetSpeedRad;
  float maxSpeedMicrosteps;
  
  void computeConversionFactor();
  float toMicrostep(float radValue);
  
};

#endif