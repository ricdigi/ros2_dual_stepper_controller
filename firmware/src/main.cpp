#include <Arduino.h>
#include <Wire.h>

#include "StepperMotor.h"
#include "SerialComm.h"
#include "MagneticEncoder.h"

#define STEP_PIN_A       15
#define STEP_DIR_PIN_A   21

#define STEP_PIN_B       22
#define STEP_DIR_PIN_B   23

#define ENABLE_PIN_AB    14

#define MUX_PIN_A        27
#define MUX_PIN_B        28

#define DIR_PIN_A        29
#define DIR_PIN_B        30

StepperMotor stepper_A(STEP_PIN_A, STEP_DIR_PIN_A, ENABLE_PIN_AB, 200, 16);
StepperMotor stepper_B(STEP_PIN_B, STEP_DIR_PIN_B, ENABLE_PIN_AB, 200, 16);

MagneticEncoder encoders;
SerialComm comm;

void setup() {
  comm.begin(115200);
  Wire.begin();

  stepper_A.setRotationDir(true);
  stepper_B.setRotationDir(false);

  stepper_A.setUpEnablePin();
  stepper_B.setUpEnablePin();

  stepper_A.setSpeedRad(0);
  stepper_B.setSpeedRad(0);

  stepper_A.setAccelerationRad(100);
  stepper_B.setAccelerationRad(100);

  encoders.encoderInit(MUX_PIN_A, MUX_PIN_B, DIR_PIN_A, DIR_PIN_B);

}

void loop() {
  
  comm.receive();

  if (comm.hasCommand()) {
    uint8_t command = comm.getCommand();
    if (command == SerialComm::ENABLE_CMD) {
      stepper_A.enable();
      stepper_B.enable();
    } else if (command == SerialComm::DISABLE_CMD) {
      stepper_A.disable();
      stepper_B.disable();
    } else if (command == SerialComm::VEL_CMD) {
      float speed_A = comm.getSpeedA();
      float speed_B = comm.getSpeedB();

      stepper_A.setSpeedRad(speed_A);
      stepper_B.setSpeedRad(speed_B);
    }
    comm.clearCommand();
  }

  if (encoders.readSensors()) {
    comm.sendEncoderData(encoders.getEncoderAData(), encoders.getEncoderBData());
  }

  stepper_A.run();
  stepper_B.run();

}
