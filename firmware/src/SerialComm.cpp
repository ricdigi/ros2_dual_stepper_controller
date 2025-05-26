#include "../include/SerialComm.h"
#include <Arduino.h>

SerialComm::SerialComm(){}

void SerialComm::begin(long baud){
  Serial.begin(baud);
  state = WAIT_HEADER;
  index = 0;
  commandReady = false;
}

bool SerialComm::receive(){

  if (!Serial.available()) return false;
    uint8_t byteIn = Serial.read();

    switch (state) {
      case WAIT_HEADER:
        if (byteIn == HEADER) {
          checksum = byteIn;
          state = READ_CMD;
        }
        break;

      case READ_CMD:
        cmd = byteIn;
        checksum ^= byteIn;
        state = READ_LEN;
        break;

      case READ_LEN:
        len = byteIn;
        if (len > VEL_DATA_LEN) {
          state = WAIT_HEADER;  // reset on invalid length
        } else {
          checksum ^= byteIn;
          index = 0;
          state = READ_DATA;
        }
        break;

      case READ_DATA:
        data[index++] = byteIn;
        checksum ^= byteIn;
        if (index >= len) {
          state = READ_CRC;
        }
        break;

      case READ_CRC:
        if (checksum == byteIn) {
          commandReady = true;
        }
        state = WAIT_HEADER; 
        return commandReady;

      default:
        state = WAIT_HEADER;
        break;
  }

  return false;
}

void SerialComm::sendEncoderData(float enc_a_data, float enc_b_data) {

  uint8_t checksum = HEADER ^ ENC_CMD ^ ENC_DATA_LEN;
  uint8_t payload[8];

  memcpy(payload, &enc_a_data, 4);
  memcpy(payload + 4, &enc_b_data, 4);

  Serial.write(HEADER);
  Serial.write(ENC_CMD);
  Serial.write(ENC_DATA_LEN);
  Serial.write(payload, 8);

  for (int i = 0; i < 8; i++) {
    checksum ^= payload[i];
  }

  Serial.write(checksum);
}

bool SerialComm::hasCommand() {
  return commandReady;
}

float SerialComm::getSpeedA() {
  float val;
  memcpy(&val, data, 4);
  return val;
}

float SerialComm::getSpeedB() {
  float val;
  memcpy(&val, data + 4, 4);
  return val;
}

void SerialComm::clearCommand() {
  commandReady = false;
}

