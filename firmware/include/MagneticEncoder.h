
#ifndef MAGNETICENCODER_H
#define MAGNETICENCODER_H

#include <AS5600.h>  

class MagneticEncoder {

  public:

    MagneticEncoder();

    void encoderInit(uint8_t MUX_A, uint8_t MUX_B, uint8_t DIR_A, uint8_t DIR_B);
    bool readSensors();

    float getEncoderAData() { return enc_A_data; }
    float getEncoderBData() { return enc_B_data; }
    float getEncoderUpdateFrequency() { return encoderUpdateFrequency; }

  private:

    AS5600 as5600;

    float current_reading;
    float enc_A_data = 0.0;
    float enc_B_data = 0.0;
    float offset_A = 0.0;
    float offset_B = 0.0;

    uint8_t MUX_A;
    uint8_t MUX_B;

    unsigned long muxSwitchTime = 0;
    uint8_t currentChannel = 1;
    bool waiting = false;

    float encoderUpdateFrequency = 20.0;
    float lastUpdateTime = 0.0;

    void selectMuxChannel(uint8_t channel);

};

#endif 