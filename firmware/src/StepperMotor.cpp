#include "../include/StepperMotor.h"
#include <Arduino.h>
#include <math.h>

StepperMotor::StepperMotor(uint8_t stepPin_, uint8_t dirPin_, uint8_t enablePin_, int stepsPerRev_, int microstepping_)
  : stepPin(stepPin_), dirPin(dirPin_), enablePin(enablePin_), stepsPerRev(stepsPerRev_), microstepping(microstepping_) {

    stepper = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
    computeConversionFactor();

  }

void StepperMotor::computeConversionFactor(){
  radToMicrosteps = stepsPerRev * microstepping / (2 * PI);
}

void StepperMotor::setUpEnablePin(){
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); 
}

float StepperMotor::toMicrostep(float radValue){
  return radValue * radToMicrosteps;
}

void StepperMotor::setAccelerationRad(float radPerSec2){
  accelerationRad = radPerSec2;
  stepper.setAcceleration(this->toMicrostep(accelerationRad));
}

void StepperMotor::setSpeedRad(float radPerSec){
  targetSpeedRad = radPerSec;
  maxSpeedMicrosteps = toMicrostep(targetSpeedRad);
  stepper.setMaxSpeed(maxSpeedMicrosteps);
}

void StepperMotor::enable(){
  digitalWrite(enablePin, LOW); 
}

void StepperMotor::disable(){
  digitalWrite(enablePin, HIGH); 
}

void StepperMotor::run(){
  stepper.moveTo(stepper.currentPosition() + 1e6);
  stepper.run();
}

void StepperMotor::stop(){
  stepper.stop();  // decelerate
  while (stepper.isRunning()) stepper.run();
}