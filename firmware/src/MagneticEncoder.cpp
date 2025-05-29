
#include <Arduino.h>
#include "../include/MagneticEncoder.h"

MagneticEncoder::MagneticEncoder(){}

void MagneticEncoder::encoderInit(uint8_t MUX_A_, uint8_t MUX_B_, uint8_t DIR_A, uint8_t DIR_B) {

  MUX_A = MUX_A_;
  MUX_B = MUX_B_;

  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);

  delay(200);  // Wait for AS5600 to fully power up and stabilize

  // Calibrate the encoders in zero position at the start
  selectMuxChannel(1);
  delayMicroseconds(50);
  as5600.begin();  
  as5600.setDirection(AS5600_CLOCK_WISE);
  offset_A = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;

  selectMuxChannel(2);
  delayMicroseconds(50);
  offset_B = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;

  // Leave first channel selected
  selectMuxChannel(1);
  delayMicroseconds(50);

  // Successfully initialized
  Serial.println("Magnetic Encoder initialized successfully.");

}

void MagneticEncoder::readSensors() {

  if (!waiting) {

    current_reading = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;

    if (currentChannel == 1) {
      enc_A_data = current_reading - offset_A;
      if (enc_A_data < 0.0) { enc_A_data += 360.0; }
      currentChannel = 2;
    } 
    
    else { 
      enc_B_data = current_reading - offset_B;
      if (enc_B_data < 0.0) { enc_B_data += 360.0; }
      currentChannel = 1; 
    }

    selectMuxChannel(currentChannel);
    waiting = true;
    muxSwitchTime = micros();

  }

  if ((micros() - muxSwitchTime) >= 50 ) {
    waiting = false;
  }

}

void MagneticEncoder::selectMuxChannel(uint8_t channel) {
  digitalWrite(MUX_A, channel & 0x01);
  digitalWrite(MUX_B, (channel >> 1) & 0x01);
}

